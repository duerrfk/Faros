/**
 * This file is part of Faros.
 * 
 * Copyright 2015 Frank Duerr
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
// Need to include EEPROM.h here although it is not reference from this file
// (at least Arduino 1.5.8 IDE fails to include it automatically). 
#include <EEPROM.h>
#include <SPI.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include "services.h"
#include "url_schemes.h"
#include "commands.h"

//// Define your hardware platform to set pin mappings, etc. 
// The following definition is suitable for the Faros board. 
//#define HARDWARE_ATMEGA328P_1MHz
// The following definition is suitable for Arduino Pro Micro.
#define HARDWARE_ATMEGA32U4_8MHz

// Putting the MCU into power-down mode is essential to save 
// energy and to achieve runtimes of several years. However, 
// be careful if you use an Arduino flashed via serial port
// like Arduino Pro Micro. If the serial port is not 
// discovered by your PC until the device falls asleep, you 
// might not be able to flash it anymore, via serial port. 
// To recover, you have two options: 
//
// 1. Connect a cable between PIN_RDYN and GND of the Arduino. 
//    This will prevent the the activation of power-down 
//    mode since INT0 is constantly active (the device thinks,
//    that the nRF8001 wants to send events).
// 2. Flash via ISP
//
// Since the Faros board is flashed via ISP anyway, it is
// save to use this setting with the Faros board.
#ifdef HARDWARE_ATMEGA328P_1MHz
#define DO_SLEEP
#endif

// Disabling brown-out detecting while sleeping saves about 25 uA.
// Not all MCUs support this. ATmega328P used by the Faros board 
// does. ATmega32U4 used by Arduino Pro Micro doesn't.
#ifdef HARDWARE_ATMEGA328P_1MHz
#define DISABLE_BOD_WHILE_SLEEPING
#endif

// 8.8 fixed point arithmetic
#define FIXED_8_8_FBITS 8
#define FIXED_8_8_ONE (1<<FIXED_8_8_FBITS)
#define FIXED_8_8_FROM_FLOAT(x) ((int16_t) ((x)*FIXED_8_8_ONE))

//// Arduino pins connected to nRF8001.
#ifdef HARDWARE_ATMEGA32U4_8MHz
// The following values are valid for Arduino Pro Micro. 
#define PIN_MOSI 16
#define PIN_MISO 14
#define PIN_SCK 15
// nRF8001 uses two "slave select" pins, one from Arduino (master) to nRF8001
// (slave) called REQN, and one from slave to master called RDYN for signaling 
// events from nRF8001 to Arduino. For RDYN select a pin with attached 
// hardware interrupt to enable wake-up from sleep mode. 
#define PIN_REQN 7
#define PIN_RDYN 3  
#define PIN_RST 4

// We use this interrupt to react to RDYN events. nRF8001 pulls
// RDYN low, when it has some data to send to the MCU.
// pin 3 = RDYN is linked to INT0 on Arduino Pro Micro.
#define RDYN_INTR_NO 0

// nRF8001 supports max. 3 MHz SPI clock frequency.
// Arduino Pro Micro runs at 8 MHz. By dividing by 4, we
// set the SPI clock frequency to 2 MHz.    
#define SPI_CLOCK_DIV SPI_CLOCK_DIV4
#endif

#ifdef HARDWARE_ATMEGA328P_1MHz
// The following values are valid for the Faros board. 
#define PIN_MOSI 11
#define PIN_MISO 12
#define PIN_SCK 13
// nRF8001 uses two "slave select" pins, one from Arduino (master) to nRF8001
// (slave) called REQN, and one from slave to master called RDYN for signaling 
// events from nRF8001 to Arduino. For RDYN select a pin with attached 
// hardware interrupt to enable wake-up from sleep mode. 
#define PIN_REQN 10
#define PIN_RDYN 2  
#define PIN_RST 3

// We use this interrupt to react to RDYN events. nRF8001 pulls
// RDYN low, when it has some data to send to the MCU.
// pin 2 = RDYN is linked to INT0 on the Faros board.
#define RDYN_INTR_NO 0

// nRF8001 supports max. 3 MHz SPI clock frequency.
// The Faros board runs at 1 MHz. Two is the smallest clock
// divider that we can set:    
#define SPI_CLOCK_DIV SPI_CLOCK_DIV2
#endif

// Advertisement interval in 0.625 ms, min 0x0020 (20 ms), max 0x4000 (10.240 s). 
// 1600 -> 1 s
#define ADV_INTERVAL 1600

// Maximum sizes of the different Eddystone frames 
#define EDDYSTONE_UID_DATA_SIZE 20
#define EDDYSTONE_URL_DATA_SIZE 20
#define EDDYSTONE_TLM_DATA_SIZE 14

//// nRF8001 data structures
#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    services_pipe_type_mapping_t services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    services_pipe_type_mapping_t *services_pipe_type_mapping = NULL;
#endif

// Store the setup for the nRF8001 in the flash of the AVR (PROGMEM) to save on RAM
const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

// Selection of Eddystone frame types that should be sent
const boolean is_enabled_uid = true;
const boolean is_enabled_url = true;
const boolean is_enabled_tlm = true;

//// Definition of data for the different Eddystone frame types.
// Note that multi-byte Eddystone data is in Big-Endian format.
// Frame type ids
const uint8_t eddystone_frame_type_uid = 0x00;
const uint8_t eddystone_frame_type_url = 0x10;
const uint8_t eddystone_frame_type_tlm = 0x20;

// 10 byte namespace id. Google suggests different methods to create this:
// - Truncated hash: first 10 bytes of your SHA1 hash of your FQDN.
// - Elided Version 4 UUID: version 4 UUID with bytes 5 - 10 (inclusive) removed 
const uint8_t eddystone_namespace_id[] = {0x21, 0x83, 0xa8, 0x77, 0xa7, 0xb7, 0x5d, 0x07, 0x15, 0x13};
// 6 byte instance id (any scheme you like).
const uint8_t eddystone_instance_id[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
// TX power in dBm, calibrated at 0 m distance.
// Range: -100 dBm to +20 dBm; resolution: 1 dBm.
// Can be found out empirically by measuring the signal strength at 1 m.
// distance and adding 41 dBm.
const int8_t eddystone_txpwr = -17;
// Scheme of the encoded URL.
const url_schemes eddystone_url_scheme = http_www_dot;
// Encoded URL (max. 17 bytes)
// The following bytes expand to a sequence of characters:
// 0x00  .com/
// 0x01  .org/
// 0x02  .edu/
// 0x03  .net/
// 0x04  .info/
// 0x05  .biz/
// 0x06  .gov/
// 0x07  .com
// 0x08  .org
// 0x09  .edu
// 0x0a  .net
// 0x0b  .info
// 0x0c  .biz
// 0x0d  .gov
// 14..32  0x0e..0x20  Reserved for Future Use
// 127..255  0x7F..0xFF  Reserved for Future Use
// The following example encodes the URL frank-duerr.de
// ("http://www." is added by the schema definition)
const uint8_t eddystone_enc_url[] = {0x66, 0x72, 0x61, 0x6e, 0x6b, 0x2d, 0x64, 0x75, 0x72, 0x72, 0x2e, 0x64, 0x65};
const unsigned int eddystone_enc_url_size = sizeof(eddystone_enc_url);
// TLM version
const uint8_t eddystone_tlm_version = 0x00;

// Complete Eddystone UID frame:
// - 1 byte frame type (0x00 for UID frame)
// - 1 byte TX power
// - 10 byte namespace id
// - 6 byte instance id
// - 2 bytes reserved (must be set to 0x00 0x00)
uint8_t eddystone_uid_data[EDDYSTONE_UID_DATA_SIZE];
unsigned int eddystone_uid_data_size;
 
// Complete Eddystone URL frame:
// - 1 byte frame type (0x10 for UID frame)
// - 1 byte TX power
// - 1 byte URL scheme
// - 0-17 bytes encoded URL
uint8_t eddystone_url_data[EDDYSTONE_URL_DATA_SIZE];
unsigned int eddystone_url_data_size;

// Complete Eddystone TLM frame:
// - 1 byte frame type (0x20 for TLM frame)
// - 1 byte TLM version (currently 0x00)
// - 2 byte battery level in millivolt. 
// - 2 byte temperature (degree Celsius; signed 8.8 fixed point notation)
// - 4 bytes adv. PDU count (total count of advertisement frames of 
//   all types emitted by the beacon since (re-)boot)
// - 4 bytes time since (re-)boot (0.1 sec resolution)
uint8_t eddystone_tlm_data[EDDYSTONE_TLM_DATA_SIZE];
unsigned int eddystone_tlm_data_size;

// Temperature [degree Celsius]
float temperature = 12.34;
// Battery level [volt]
float voltage = 1.23;
// Total number of advertisement frames sent since re-boot
uint32_t adv_pdu_cnt = 0;
// Uptime in tens of seconds
uint32_t uptime = 0;

struct aci_state_t aci_state;
hal_aci_evt_t  aci_data;
hal_aci_data_t aci_cmd;

volatile boolean ble_ready = false;
volatile boolean timer_fired = false;
boolean is_broadcasting = false;

// Eddystone frame types are sent round robin. 
// Variable next_frame_type tells, which frame type to send next.
enum frame_types {uid, url, tlm};
enum frame_types next_frame_type;

// nRF8001 can only process one system command at a time.
// After sending a request, we need to wait for the response
// before sending the next command. In order to simplify implementation,
// we queue pending commands in a command queue. Whenever a response
// is received, the next command is taken out of the queue and sent
// to nRF8001. The command queue is implemented as a ring buffer.
#define CMD_QUEUE_SIZE 16
#define CMD_QUEUE_SIZE_MOD_MASK 0x0f 
enum commands cmd_queue[CMD_QUEUE_SIZE];
unsigned int cmd_queue_head = 0;
unsigned int cmd_queue_tail = 0;
unsigned int cmd_queue_free = CMD_QUEUE_SIZE;
boolean cmd_queue_pending = false;

/**
 * Called after a fatal error happened. 
 */
void die()
{  
    // Enable watchdog system reset mode to reboot system with next
    // watch dog event. 
    //WDTCSR |= _BV(WDE);
    
    // Go into endless loop. User will notice error through TLM frames
    // (either missing or counters don't increase anymore).
    while (true);
}

/**
 * Enqueue a command into the command queue. 
 * 
 * @param command the command to be enqueued
 * 
 * @returns true if the command fits into the queue; false if
 *          there is no space in the queue anymore.
 */
boolean cmd_queue_enqueue(enum commands command)
{
    if (cmd_queue_free == 0)
        return false;

    cmd_queue[cmd_queue_head] = command;
    cmd_queue_head++;
    cmd_queue_head &= CMD_QUEUE_SIZE_MOD_MASK;
    cmd_queue_free--;
    
    // Try to send command
    cmd_queue_send_cmd();

    return true;
}

/**
 * If no other command is pending and there is at least
 * one queued command, send next command to nRF8001.
 */
void cmd_queue_send_cmd() 
{
    // Cannot send a command if either a command is pending 
    // or queue is empty
    if (cmd_queue_pending || cmd_queue_free == CMD_QUEUE_SIZE)
        return;
        
    enum commands cmd = cmd_queue[cmd_queue_tail];
    switch (cmd) {
    case get_temperature :
        lib_aci_get_temperature();
        break;
    case get_battery_level :
        lib_aci_get_battery_level();
        break;
    case send_uid_frame :
        lib_aci_set_local_data(&aci_state, 
            PIPE_EDDYSTONE_EDDYSTONEDATA_BROADCAST,
            eddystone_uid_data, EDDYSTONE_UID_DATA_SIZE);
        break;
    case send_url_frame :
        lib_aci_set_local_data(&aci_state, 
            PIPE_EDDYSTONE_EDDYSTONEDATA_BROADCAST,
            eddystone_url_data, eddystone_url_data_size);
        break;
    case send_tlm_frame :
        eddystone_tlm_data_size = setup_eddystone_TLM_frame(eddystone_tlm_data, 
            eddystone_tlm_version, voltage, temperature, adv_pdu_cnt, uptime);
        lib_aci_set_local_data(&aci_state, 
            PIPE_EDDYSTONE_EDDYSTONEDATA_BROADCAST,
            eddystone_tlm_data, eddystone_tlm_data_size);
        break;
    case open_adv_pipe :
        lib_aci_open_adv_pipe(PIPE_EDDYSTONE_EDDYSTONEDATA_BROADCAST);
        break;
    case start_broadcasting :
        // Start advertising in non-connectable (broadcast) mode. 
        // First parameter defines the adv. timeout (how long to
        // advertise; 0 = forever). Second parameter is advertisement 
        // interval in milli-seconds.
        lib_aci_broadcast(0, ADV_INTERVAL);
        break;
    case send_hello :
        break;
    }
    
    cmd_queue_pending = true;
    
    cmd_queue_tail++;
    cmd_queue_tail &= CMD_QUEUE_SIZE_MOD_MASK;
    
    cmd_queue_free++;
}

/**
 * Received response for pending command.
 */
void cmd_queue_response_recvd()
{
    cmd_queue_pending = false;

    // Can send next enqueued command now (if any is available)
    cmd_queue_send_cmd();
}

/**
 * Setup an Eddystone TLM frame.
 * 
 * @param data array storing the frame data
 * @param version TLM version
 * @param battery_voltage voltage of the battery in Volts; 
 *        set to zero if not supported
 * @param beacon_temperature temperature of the beacon in degree Celcius
 *        set to -128 if not supported
 * @param adv_pdu_cnt total number of adv. frames sent since (re-)boot
 * @param uptime uptime in tens of seconds since (re-)boot
 * 
 * @return the size of the frame in bytes
 */
unsigned int setup_eddystone_TLM_frame(uint8_t data[EDDYSTONE_TLM_DATA_SIZE], 
    uint8_t version, float battery_voltage, float beacon_temperature, 
    uint32_t adv_pdu_cnt, uint32_t uptime)
{
    unsigned int offset = 0;
    
    data[offset] = eddystone_frame_type_tlm;
    offset++;
 
    data[offset] = version;
    offset++;

    // Convert battery level to millivolts
    uint16_t voltage = (uint16_t ) (battery_voltage*1000.0);
    // Store in Big Endian format
    data[offset] = (voltage>>8)&0xff;
    offset++;
    data[offset] = voltage&0xff;
    offset++;

    // Convert temperature to 8.8 fixed point format
    int16_t temperature = FIXED_8_8_FROM_FLOAT(beacon_temperature);
    // Store in Big Endian format
    data[offset] = (temperature>>8)&0xff;
    offset++;
    data[offset] = temperature&0xff;
    offset++;

    data[offset] = (adv_pdu_cnt>>24)&0xff;
    offset++;
    data[offset] = (adv_pdu_cnt>>16)&0xff;
    offset++;
    data[offset] = (adv_pdu_cnt>>8)&0xff;
    offset++;
    data[offset] = adv_pdu_cnt&0xff;
    offset++;

    data[offset] = (uptime>>24)&0xff;
    offset++;
    data[offset] = (uptime>>16)&0xff;
    offset++;
    data[offset] = (uptime>>8)&0xff;
    offset++;
    data[offset] = uptime&0xff;
    offset++;

    return offset;
}

/**
 * Setup an Eddystone URL frame.
 * 
 * @param data array storing the frame data
 * @param txpwr transmission power
 * @param scheme URL scheme
 * @param enc_url encoded URL 
 * @param enc_url_size length of the enc_url array
 * 
 * @return the size of the frame in bytes
 */
unsigned int setup_eddystone_URL_frame(uint8_t data[EDDYSTONE_URL_DATA_SIZE], 
    int8_t txpwr, enum url_schemes scheme, const uint8_t *enc_url, 
    unsigned int enc_url_size)
{
    unsigned int offset = 0;
    
    data[offset] = eddystone_frame_type_url;
    offset++;

    data[offset] = txpwr;
    offset++;

    uint8_t scheme_byte;
    switch (scheme) {
    case http_www_dot:
        scheme_byte = 0x00;
        break;
    case https_www_dot:
        scheme_byte = 0x01;
        break;
    case http: 
        scheme_byte = 0x02;
        break;
    case https:
        scheme_byte = 0x03;
        break;
    }
    data[offset] = scheme_byte;
    offset++;

    for (unsigned int i = 0; i < enc_url_size; i++) {
        data[offset] = enc_url[i];
        offset++;
    }

    return offset;
}

/**
 * Setup an Eddystone UID frame.
 * 
 * @param data array storing the frame data
 * @param txpwr transmission power
 * @param namespace_id namespace identifier
 * @param instance_id instance identifier
 * 
 * @return the size of the frame in bytes
 */
 
/**
 * Defines the data of an Eddystone UID frame.
 */
unsigned int setup_eddystone_UID_frame(uint8_t data[EDDYSTONE_UID_DATA_SIZE], 
    int8_t txpwr, const uint8_t namespace_id[10], const uint8_t instance_id[6])
{
    unsigned int offset = 0;
    
    data[offset] = eddystone_frame_type_uid;
    offset++;
    
    data[offset] = txpwr;
    offset++;
    
    for (unsigned int i = 0; i < 10; i++) {
        data[offset] = namespace_id[i];
        offset++;
    }
    
    for (unsigned int i = 0; i < 6; i++) {
        data[offset] = instance_id[i];
        offset++;
    }
    
    data[offset] = 0x00;
    offset++;
    data[offset] = 0x00;
    offset++;
    
    return offset;
}
  
/**
 * This function is used by BLE library to signal failed assertions.
 */
void __ble_assert(const char *file, uint16_t line)
{
    while (true);
}

/**
 * ISR for RDYN low events
 */
void rdyn_isr()
{
    // This is a level interrupt that would fire again while the
    // signal is low. Thus, we need to detach the interrupt.
    detachInterrupt(RDYN_INTR_NO);
    ble_ready = true;  
}

/**
 * Watch dog interrupt. Will be fired every second.
 * 
 * The watch dog will keep runing even in power-down mode.
 * When it fires, it will wake-up the Arduino.
 */
ISR(WDT_vect)
{  
    timer_fired = true;
}

/**
 * Setup watch dog to trigger interrupts every 1 s.
 */
void start_watchdog()
{
    // WDTCSR: Watch Dog Control Status Register
    // WDIE: Watchdog Interrupt Enable
    // WDCE: Watchdog Change Enabled
    // WDE: Watchdog System Reset Enabled
    // WDRF: Watchdog Reset Flag
    /* Watch Dog Prescaler (WDP3-0)
    0         0      0        0             2K (2048) cycles      16 ms
    0         0      0        1             4K (4096) cycles      32 ms
    0         0      1        0             8K (8192) cycles      64 ms
    0         0      1        1            16K (16384) cycles     0.125 s
    0         1      0        0            32K (32768) cycles     0.25 s
    0         1      0        1            64K (65536) cycles     0.5 s
    0         1      1        0            128K (131072) cycles   1.0 s
    0         1      1        1            256K (262144) cycles   2.0 s
    1         0      0        0            512K (524288) cycles   4.0 s
    1         0      0        1            1024K (1048576) cycles 8.0 s
    */
    // Clear WDRF or it will overwrite cleared WDE.
    MCUSR &= ~_BV(WDRF);
    byte sreg_saved;
    sreg_saved = SREG;
    cli();
    // Watchdog modus:
    //  WDE, ~WDIE: system reset mode
    // ~WDE,  WDIE: interrupt mode
    //  WDE,  WDIE: interrupt mode; after first interrupt go to system reset mode 
    // Changing timing and clearing WDE require a timed sequence:
    // Set WDCE bit; then change settings within 4 clock cycles.
    WDTCSR |= _BV(WDCE) | _BV(WDE);
    // Disable watchdog reset, enable interrupt mode, interval 1 s
    WDTCSR = _BV(WDP2) | _BV(WDP1) | _BV(WDIE);
    SREG = sreg_saved;
    wdt_reset();
}

/**
 * One-time setup.
 */
void setup()
{     
    wdt_disable();
      
    if (is_enabled_uid)
        next_frame_type = uid;
    else if (is_enabled_url)
        next_frame_type = url;
    else if (is_enabled_tlm)
        next_frame_type = tlm;
    
    // Disable ADC (saves about 300 uA)
    ADCSRA = 0;

    eddystone_uid_data_size = setup_eddystone_UID_frame(eddystone_uid_data, 
        eddystone_txpwr, eddystone_namespace_id, eddystone_instance_id);

    eddystone_url_data_size = setup_eddystone_URL_frame(eddystone_url_data, 
        eddystone_txpwr, eddystone_url_scheme, eddystone_enc_url, 
        eddystone_enc_url_size);
    
    // Setup nRF800 using the definitions from services.h created by nRFgo Studio
    if (services_pipe_type_mapping != NULL) {
        aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
    } else {
        aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
    }
    aci_state.aci_setup_info.number_of_pipes = NUMBER_OF_PIPES;
    aci_state.aci_setup_info.setup_msgs = (hal_aci_data_t *) setup_msgs;
    aci_state.aci_setup_info.num_setup_msgs = NB_SETUP_MESSAGES;
    
    // Pin mapping from nRF8001 to Arduino. Not connected pins are defined as UNUSED.
    aci_state.aci_pins.board_name = BOARD_DEFAULT;
    aci_state.aci_pins.reqn_pin = PIN_REQN;
    aci_state.aci_pins.rdyn_pin = PIN_RDYN;
    aci_state.aci_pins.mosi_pin = PIN_MOSI;
    aci_state.aci_pins.miso_pin = PIN_MISO;
    aci_state.aci_pins.sck_pin = PIN_SCK;
    aci_state.aci_pins.reset_pin = PIN_RST;
    aci_state.aci_pins.active_pin = UNUSED;
    aci_state.aci_pins.optional_chip_sel_pin = UNUSED;
    
    aci_state.aci_pins.spi_clock_divider = SPI_CLOCK_DIV;
    
    // We implement our own interrupt handling to better control
    // when we spend time for nRF8001 event processing and when for 
    // other things like sensor readings. Moreover, this will help
    // us to implement a correct sleep/wake-up procedure that
    // is not missing events.
    // Therefore, we turn off interrupts here and poll nRF8001
    // in the event loop whenever we think it's suitable.
    aci_state.aci_pins.interface_is_interrupt = false;
    aci_state.aci_pins.interrupt_number = 0;
    
    // Reset nRF8001. 
    // Set second parameter to true for debug output.
    lib_aci_init(&aci_state, false);
    
    // Install interrupt for RDYN line of nRF8001 for event handling.
    // We use a level-interrupt that can also fire in sleep mode to
    // wake up the Arduino when an event is received.
    attachInterrupt(RDYN_INTR_NO, rdyn_isr, LOW);
}

/**
 * Send the next Eddystone frame to be advertised to nRF8001. 
 * 
 * We cycle through all enabled frame types. Every frame get an
 * equal share of "air time".
 */
void set_next_frame()
{        
    switch (next_frame_type) {
    case uid :
        cmd_queue_enqueue(send_uid_frame);
        if (is_enabled_url)
            next_frame_type = url;
        else if (is_enabled_tlm)
            next_frame_type = tlm;
        break;
    case url :
        cmd_queue_enqueue(send_url_frame);
        if (is_enabled_tlm)
            next_frame_type = tlm;
        else if (is_enabled_uid)
            next_frame_type = uid;
        break;
    case tlm :
        // Before sending a TLM frame, we first need to get current 
        // temperature and voltage readings.
        cmd_queue_enqueue(get_temperature);
        cmd_queue_enqueue(get_battery_level);
        cmd_queue_enqueue(send_tlm_frame);
        if (is_enabled_uid)
            next_frame_type = uid;
        else if (is_enabled_url)
            next_frame_type = url;
        break;    
    }
}

/**
 * BLE event loop. Exits, if no more events are there to be processed at 
 * the moment.
 */
void aci_loop()
{  
    while (lib_aci_event_get(&aci_state, &aci_data)) {
        // Only entered if there is an event to be processed.
        aci_evt_t *aci_evt;
        aci_evt = &aci_data.evt;
        switch(aci_evt->evt_opcode) {
        case ACI_EVT_DEVICE_STARTED:
            aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
            switch(aci_evt->params.device_started.device_mode) {
            case ACI_DEVICE_SETUP:
                aci_state.device_state = ACI_DEVICE_SETUP;
                if (do_aci_setup(&aci_state) == SETUP_SUCCESS) {
                    // Setup OK
                } else {
                    // Setup failed.
                    // This is a fatal error. We bail out.
                    die();
                }
                break;
            case ACI_DEVICE_STANDBY:
                aci_state.device_state = ACI_DEVICE_STANDBY;
                // In order to setup the beacon, we need to go through the 
                // following event/action sequence:
                // On entering standby state -> open advertisement pipe
                // On adv. pipe is open -> set local data (Eddystone frame data)
                // On data is set -> start advertising
                cmd_queue_enqueue(open_adv_pipe);
                // We can assume that at least one of these frame types is enabled
                // or the beacon is pretty useless.
                if (is_enabled_uid)
                    cmd_queue_enqueue(send_uid_frame);
                else
                    cmd_queue_enqueue(send_url_frame);
                cmd_queue_enqueue(start_broadcasting);
                break;
            }
            break;
        case ACI_EVT_CMD_RSP:
            if (aci_evt->params.cmd_rsp.cmd_opcode == ACI_CMD_SET_LOCAL_DATA) {
                if (aci_evt->params.cmd_rsp.cmd_status == ACI_STATUS_SUCCESS) {
                    // Eddystone data is set.
                } else {
                    // Failed to set data.
                    // This is a fatal error. We bail out.
                    die();  
                }
            }
            
            if (aci_evt->params.cmd_rsp.cmd_opcode == ACI_CMD_BROADCAST) {
                if (aci_evt->params.cmd_rsp.cmd_status == ACI_STATUS_SUCCESS) {
                    // Start firing timer events
                    start_watchdog();
                } else {
                    // Failed to start broadcasting
                    // This is a fatal error. We bail out.
                    die();    
                }
            }  

            if (aci_evt->params.cmd_rsp.cmd_opcode ==  ACI_CMD_OPEN_ADV_PIPE) {
                if (aci_evt->params.cmd_rsp.cmd_status == ACI_STATUS_SUCCESS) {
                    // Adv. pipe is open now
                } else {
                    // Failed to open adv. pipe
                    // This is a fatal error. We bail out.
                    die();   
                }
            }

            if (aci_evt->params.cmd_rsp.cmd_opcode == ACI_CMD_GET_BATTERY_LEVEL) {
                if (aci_evt->params.cmd_rsp.cmd_status == ACI_STATUS_SUCCESS) {
                    // According to nRF8001 manual, format is LSB/MSB. 
                    // Not sure whether the Arduino SDK already does some conversion on
                    // Big Endian machines. Anyway, ATMega is Little Endian, so no 
                    // conversion is needed here. However, if we ever run this code
                    // on Big Endian machines, we should check the result. 
                    uint16_t level = aci_evt->params.cmd_rsp.params.get_battery_level.battery_level;
                    // Resolution is 3.52 mV
                    voltage = 0.0035*level;
                } else {
                    // Failed to read battery level
                }
            }

            if (aci_evt->params.cmd_rsp.cmd_opcode == ACI_CMD_GET_TEMPERATURE) {
                if (aci_evt->params.cmd_rsp.cmd_status == ACI_STATUS_SUCCESS) {
                    // According to nRF8001 manual, format is LSB/MSB. 
                    // Not sure whether the Arduino SDK already does some conversion on
                    // Big Endian machines. Anyway, ATMega is Little Endian, so no 
                    // conversion is needed here. However, if we ever run this code
                    // on Big Endian machines, we should check the result. 
                    int16_t temp = aci_evt->params.cmd_rsp.params.get_temperature.temperature_value;
                    // Resolution is 0.25 deg
                    temperature = 0.25*temp;
                } else {
                    // Failed to read temperature
                }
            }
            // Signal to the command queue that no command is pending anymore
            // and the next enqueued command can be sent now.
            cmd_queue_response_recvd();           
            break;
        case ACI_EVT_DATA_ACK:
            // Data acknowledged.
            // Not relevant for connection-less beacon.
            break;    
        case ACI_EVT_PIPE_STATUS:
            // Status of broadcast pipes is not announced here.
            // -> Use command response event instead. 
            break;
        case ACI_EVT_TIMING:
            // Timing of connection changed.
            // Not relevant for connection-less beacon.
            break;
        case ACI_EVT_CONNECTED:
            // Connection established. 
            // Not relevant for connection-less beacon.
            break;
        case ACI_EVT_DATA_CREDIT:
            // Bluetooth radio ack received from the peer radio for the data packet sent.
            // Not relevant for connection-less beacon.
            break;
        case ACI_EVT_PIPE_ERROR: 
            break;
        case ACI_EVT_DISCONNECTED:
            // Connection closed.
            // Not relevant for connection-less beacon.
            break;
        case ACI_EVT_HW_ERROR:
            // This is a fatal error. We bail out.
            die();
            break;
        }
    } 
}

/**
 * Put device into power-down mode to save as much energy as possible.
 * 
 * The device will wake up again by interrupt, either the watch dog timer 
 * or RDYN going low (= nRF8001 sends an event to device).
 */
void do_sleep()
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    // Disable interrupts until we sleep to avoid race conditions
    // (interrupt firing before going to sleep would prevent MCU from
    // waking by interrupt).
    cli();
    if (ble_ready || timer_fired) {
        // Last chance to stay awake.
        sei();
    } else {
        sleep_enable();
        #ifdef DISABLE_BOD_WHILE_SLEEPING
        // Disabling brown-out detection while sleeping 
        // Saves about 25 uA.
        // BODS: Brown-out Detection Sleep
        // BODSE: Brown-out Detection Sleep Enable
        // This is a timed sequence:
        // First, BODS and BODSE must me set to one.
        // Then, BODS must be set to one and BODSE to zero
        // within four clock cycles. Then, BODS stays active three
        // clock cycles, so sleep_cpu() must be called within
        // three cycles after setting BODS.
        MCUCR = bit(BODS) | bit(BODSE);
        MCUCR = bit(BODS);
        #endif 
        // Enable interrupts again. It is guranteed that the next
        // command (entering sleep mode) is executed *before* an 
        // interrupt is fired (no race condition). From the data sheet:
        // "When using the SEI instruction to enable interrupts, 
        // the instruction following SEI will be executed
        // before any pending interrupts."
        sei();
        sleep_cpu();
        // Wake again after interrupt.
        sleep_disable();
    }
}

/**
 * Main loop
 */
void loop()
{             
    if (ble_ready) {
        // Process all pending ACI events from nRF8001.
        aci_loop();
        ble_ready = false;
        // nRF8001 will cause an interrupt when more events are available
        attachInterrupt(RDYN_INTR_NO, rdyn_isr, LOW);
    }

    if (timer_fired) {
        // For Eddystone TLM frames, we need the number of sent
        // frames since (re-)boot. nRF8001 does not tell us when 
        // it sends an advertisement. However, knowing the advertisment
        // interval and watch dog timer interval, we can estimate the 
        // number of transmitted frames. For 1 s avd. interval and 1 s
        // watch dog timer, we simply have to increase the avd. counter
        // with every timer event.
        adv_pdu_cnt++;

        // Eddystone TLM frames contain the uptime. Although other timers 
        // are stopped in power-down mode, the watch dog timer keeps running. 
        // The watch dog fires every second. Eddystone wants the uptime in
        // tens of seconds.
        uptime += 10;

        // Switch to next frame type
        set_next_frame();

        timer_fired = false;
    }

    #ifdef DO_SLEEP
    do_sleep();
    #endif
}

