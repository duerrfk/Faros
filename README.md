Faros is a Bluetooth Low Energy (BLE) beacon supporting Google's open beacon 
format Eddystone [1]. As hardware platform, Faros uses Arduino and the nRF8001 
BLE chip by Nordic Semiconductors [2].

The main features are:

* Full support for all Eddystone frame types (UID, URL, TLM)
* Energy efficiency allowing for runtimes of several years
* Using popular and powerful hardware platforms: Arduino and nRF8001 BLE chip
* Simplicity of hardware: easy to build using a commodity Arduino or our
  Faros board together with the BLE module MOD-nRF8001 from Olimex [3]
* Liberal licensing: Apache License 2.0 for software, CERN Open Hardware 
  Licence v1.2 for hardware 

The following images show a self-etched Faros board and a Faros board
manufactured by a PCB manufacturer.

![Self-etched Faros board](/img/faros_board_50x70_v1_0.jpg)

![Faros PCB manufactured by Seeed Studio](/img/faros_board_50x50_v1_0.jpg)

This project includes:

* Source code for Arduino
* Faros board schematics and board layouts

# Source Code

## Prerequisites

To compile the code, you need the Arduino IDE (tested with version 1.6.5).

Moreover, you need the Arduino BLE SDK from Nordic Semiconductors available 
from GitHub:

https://github.com/NordicSemiconductor/ble-sdk-arduino

1. Download SDK: `git clone https://github.com/NordicSemiconductor/ble-sdk-arduino.git`
2. Start the Arduino IDE (tested with v1.6.5)
3. Import the library: select `Sketch/Import library/Add` library and 
   then choose folder `libraries/BLE` from the downloaded SDK.
4. Check the availability of the library. There should be an entry `BLE`
   in menu `Sketch/Import library`.

The BLE services of nRF8001 are configured by the freely available nRFgo 
Studio tool (Windows application) [5]. This tool generates a file 
(`services.h`), which is also included in the repository. Thus, you  
only need to install this tool if you want to modify the BLE service 
description for some reason. The data of Eddystone frames is defined in the 
source code.

You also need the BLE module MOD-nRF8001 from Olimex [3]. Earlier versions 
of this module used the "C" revision of the nRF8001, which does not support
broadcasting data! So you need a module with the "D" revision of the nRF8001.
According to Olimex, all modules sold now have the "D" revision of the
nRF8001.

Since the nRF8001 module is specified for a voltage range of 1.9 V - 3.6 V, 
you should use an Arduino with suitable voltage levels. For instance, 
Arduino UNO uses 5 V, so you would have to shift levels when connecting to 
nRF8001. We tested with a 3.3 V Arduino Pro Micro and the Faros board
whose design is also included in this repository.
 
## Adapting the Source Code

If you use the Faros board or Arduino Pro Micro, you can simply
set the following definitions at the beginning of the sketch.

For the Faros board:

    #define HARDWARE_ATMEGA328P_1MHz

For Arduino Pro Micro:

    #define HARDWARE_ATMEGA32U4_8MHz

For those who are interested in the technical details: The Arduino communicates
with the nRF8001 through SPI. Beyond the standard SPI pins (MISO, MOSI, SCK, 
SS), the nRF8001 uses another select pin (called RDYN) to signal that the 
nRF8001 has events to be sent to the Arduino. Moreover, the reset pin (RST) of 
the nRF8001 has to be connected. Thus, we need 6 pins to connect the Arduino 
to the nRF8001:

1. MISO
2. MOSI
3. SCK
4. REQN/SS
5. RDYN
7. RST

For the standard SPI pins, select the usual pins depending on your Arduino.
RDYN should be assigned to a pin with a hardware interrupt. We use this 
interrupt to wake up the Arduino from power-down mode when the nRF8001 wants
to send an event to the Arduino.

For the Arduino Pro Micro, we used the following pins according to the
Arduino numbering scheme:

* MISO = 14
* MOSI = 16
* SCK = 15
* REQN/SS = 7
* RDYN = 3 -> INT0
* RST = 4

For the Faros board based on ATmega328P, we used the following pins (Arduino 
numbering scheme; ATmega names are given in brackets):

* MISO = 12 (PB4)
* MOSI = 11 (PB3)
* SCK = 13 (PB5)
* REQN/SS = 10 (PB2)
* RDYN = 2 (PD2) -> INT0
* RST = 3 (PD3)

Moreover, nRF8001 supports a maximum SPI clock frequency of 3 MHz. To adjust the
SPI clock frequency, set the definition SPI_CLOCK_DIV according to your 
platform. For instance, the Arduino Pro Micro is running at 8 MHz, so we need 
to divide it by 4 (SPI_CLOCK_DIV4) to get 2 MHz. The Faros board uses an 
ATmega328P at 1 MHz, so theoretically we could divide by 1. However, the 
smallest divider available for ATmega is 2 (SPI_CLOCK_DIV2), so we will run at 
500 kHz SPI clock.

Finally, you need to set your specific data to be broadcasted by the beacon.
Eddystone supports the following data:

* UID frames:
    * beacon ID consisting of namespace id and instance id
    * TX power level
* URL frame: 
    * short URL 
    * TX power level
* TLM frame: 
    * battery level
    * temperature
    * uptime
    * number of broadcasted frames since boot

You need to set the data for UID and URL frames through variables at the 
beginning of the sketch. The data for TLD frames is detected automatically 
(nRF8001 features a temperature sensor and voltage sensor).

Moreover, you can select whether you want to send UID and/or URL frames 
and/or TLM frames by setting the variables `is_enabled_uid`, `is_enabled_url`, 
and `is_enabled_tlm`, respectively. The beacon will cycle through all 
enabled frames, so it can send UID frames, URL frames, and TLM frames 
(sequentially).

# Hardware: The Faros Board

If you just want to experiment, you can simply use a commodity Arduino 
running at 3.3 V like the Arduino Pro Micro. Just directly connect the pins
of the Arduino Pro Micro and the BLE module like this:

* MISO = 14
* MOSI = 16
* SCK = 15
* REQN/SS = 7
* RDYN = 3 -> INT0
* RST = 4

However, if you want to deploy beacons in the field, two requirements become 
essential:

* Energy efficiency: the beacon must run from battery for several years 
* Cost: if you want to deploy many beacons, a commodity Arduino board
  is too expensive

The Faros board tries to fulfill these requirements plus one more: keep
things as simple as possible so virtually everyone can build a Faros
beacon.

The Faros board uses an ATmega328P together with the BLE module MOD-nRF8001 
from Olimex [3] based on the nRF8001 chip from Nordic Semiconductors. We 
selected the nRF8001 for several reasons:

* Easy to connect to Arduino through SPI
* Open Arduino SDK [4]
* Includes temperature sensor (required for TLM frames)
* Includes voltage sensor (required for TLM frames)

The BLE module MOD-nRF8001 from Olimex provides a ready-to-use solution to use
the nRF8001 together with the Arduino.

The Faros board is kept as simple as possible (through-hole design, no SMD 
components). It comes in two variants: (1) a single-sided 50 mm x 70 mm layout 
that is well-suited for self-etching (a PDF of the layout can be found in 
folder `pcb`); (2) a double-sided 50 mm x 50 mm layout that can be sent to a 
PCB manufacturer (the Gerber files can be found in folder `gerber`). For both 
variants, the ATmega is programmed using ISP, so no USB is required which 
again saves cost and energy. Moreover, we use the ATmega's internal RC 
oscillator, so no external crystal is required. 

Here is the bill of material for the Faros board:

* C1, C2, C3: 100 nF (ceramic capacitor, X7R, 5.08 pitch)
* LED1: 3 mm
* R1: 10 k Ohm
* R2: 100 Ohm
* R3, R4, R5: 4.7 k Ohm
* IC1: ATmega328P-PU
* MOD1: Olimex MOD-nRF8001
* SV1: 3x2 pin header (2.54 pitch)
* X1: screw terminal (5.08 pitch)

## Programming the Faros Board

The Faros board is programmed via ISP. The 6 pin ISP6 connector has the usual 
layout:

    MISO <-- 1 2 --> VCC
     SCK <-- 3 4 --> MOSI
     RST <-- 5 6 --> GND

![Faros board connected to ISP](/img/faros_isp.jpg)

You can leave the BLE module connected while programming the ATmega (the 
4.7 k resistors shield the SPI pins of the nRF8001 during programming). However,
you should use a 3.3 V programmer since the maximum voltage of the nRF8001
is 3.6 V.

To preprare the ATmega328P, program the following fuses (note that "0" means 
that the fuse is programmed):

* CKSEL = 0010: use internal 8 MHz RC oscillator
* CKDIV8 = 0: 1 MHz system clock (dividing the 8 MHz internal clock by 8)
* SPIEN = 0: enable serial programming
* BODLEVEL = 110: brown-out detection set to 1.8 V
* SUT = 00: lowest possible startup time (6 CK from power-down; since we use
  BOD, we do not need additional waiting time for the power source to come up)
    
Using avrdude, the command looks like this (note: under Linux you might need
to execute this command as root depending on your programmer):

    $> avrdude -c usbasp -p m328p -U lfuse:w:0x42:m -U hfuse:w:0xd9:m -U efuse:w:0x06:m

Then, we compile and write the program to the ATmega. Before compiling the 
program, make sure you have adapted the pins (see above), SPI clock divider 
(see above), and to generate code for an ATmega at 1 MHz. In the repository, 
you will find a suitable board definition for the Arduino IDE (see folder 
`board_definition`). Copy the directory `faros-atmega` into the folder 
`hardware` in your Arduino sketchbook. Then you should find and select the 
board  called `Faros ATMega 328P @ 1 MHz` under the menu item `Tools/Board`. 

Compiling generates a hex file that we need to program the ATmega. This hex 
file is a little bit hidden in the temporary build directory of the Arduino
IDE. If you use Linux  and Arduino IDE 1.6, have a look at the `/tmp` 
directory. After hitting the  compile button in the Arduino IDE, search for the
latest hex file called  `faros.cpp.hex` in a temporary directory named 
`/tmp/build...`. If you have found the hex file, you can write it using avrdude
(again, you might need root right):

    $> avrdude -p m328p -c usbasp -v -U flash:w:faros.cpp.hex
 
## Saving Energy

The basic method to save energy is to minimize the duty cycle. Therefore,
we put the ATmega into power-down mode (deepest power-saving mode) as long as
possible, and only wake it up, when some work has to be done. This can either
be an event from the nRF8001 module , which triggers a hardware interrupt on
the RDYN pin. Or it can be a timer event from the watch dog timer (which
keeps running in power-down mode in contrast to the other timers). By switching
of the ADC and disabling brown-out detecting while sleeping (only possible
with the ATmega328P, not with the ATmega328), the ATmega328P only consumes 
a few uA (actually, the "P" in 328P means "pico-power"). 

Another important design choice is the type of battery. The nRF8001 
can run down to 1.9 V; the ATmega down to 1.8 V at frequencies <= 4 MHz. The 
maximum voltage for the nRF8001 is 3.6 V. Thus, one good option is to use two 
AA or AAA size alkaline batteries. They are cheap. They provide > 1800 mAh, 
which should suffice for several years of runtime. They are discharged at about 
1.0 V (then the voltage drops rapidly), which fits nicely our desired voltage 
range of 1.9 - 3.0 V. And at runtimes of several years, no re-charging is
required (you rather replace the device than changing the batteries). Of 
course, you can also try out other options like coin cells (e.g., one 
CR 2450 @ 3.0 V, 560 mAh), or one battery (1.5 V) plus a step-up converter 
(which wastes maybe more than 20 % energy for conversion and is probably more
expensive than a second AA or AAA battery).

The board also has a 3 mm low-current LED, which can be switched on by digital 
pin 4 of the Arduino. With a resistor of 100 Ohm, it draws 10 mA at 3 V, which 
is a lot if we target runtimes of several years! So if you aim at maximum 
battery lifetime, only send short pulses in long intervals (e.g., one 100 ms 
pulse every 30 s amounts to about 33 uA average current), or even better: rely 
on TLM frames.

# Testing

In order to test that your beacon works, you need a client, e.g., a mobile
phone or tablet with BLE and an application that understands Eddystone
beacons. There are (at least) two nice apps for Android that show all the
details of the three types of Eddystone frames:

* nRFMaster Control Panel by Nordic Semiconductors [6]
* Eddystone Validator by Google (requires Android 5 aka. Lollipop) [7]

Faros is cycling through all frame types (if you have enabled all of them)
and switches to a new frame type every second. So after about 3 seconds you 
should see the data of all three frame types (most interesting is maybe
the TLM type since its data changes dynamically).

# Licensing

Faros uses two licenses for open hardware and software, respectively:

* The software (source code) is licensed under the Apache License 2.0 [8]
* The hardware (schematic diagrams, circuit board layouts, hardware
  documentation) is licensed under the CERN Open Hardware Licence v1.2 [9]

Both licenses are also included in the repository.

# References

* [1] https://developers.google.com/beacons/
* [2] https://www.nordicsemi.com/eng/Products/Bluetooth-Smart-Bluetooth-low-energy/nRF8001 
* [3] https://www.olimex.com/Products/Modules/RF/MOD-nRF8001/
* [4] https://github.com/NordicSemiconductor/ble-sdk-arduino
* [5] https://www.nordicsemi.com/chi/node_176/2.4GHz-RF/nRFgo-Studio
* [6] https://www.nordicsemi.com/eng/Products/nRFready-Demo-Apps/nRF-Master-Control-Panel-application
* [7] https://github.com/google/eddystone
* [8] http://www.apache.org/licenses/LICENSE-2.0
* [9] http://www.ohwr.org/attachments/2388/cern_ohl_v_1_2.txt
