# SUNDIAL


## An Euclidian Drum Machine focused on MIDI (instead of CV) 

### Originally inspired by the [Qu-Bit Pulsar](https://www.qubitelectronix.com/shop/pulsar). 


![drum machine top](./media/sundial_top.jpg)

![drum machines](./media/sundial.jpg)


The goal of the Sundial was to create a working proof-of-concept of a feature-rich MIDI device from scratch, and to design my first PCB around this concept. With the success of the intended goals of the project, the active development has ended with the version 3 breadboarded version. Multiple versions of the software and hardware are being shared here for reference and to assist anyone taking on a similar project. 

 
The Sundial itself is an USB+DIN MIDI drum machine with GM2 (general MIDI 2) co-processor for audio output. The audio chip allows for standalone use with a wide range of instruments and drum sounds available, while the MIDI connectivity allows for controlling external music gear. The drum machine is driven by an ATmega32u4, and the audio chip used is an VS1053B. Communication between the two processors is over an MIDI bus internal to the system. Sundial supports external MIDI-in for GM2 instrument playback by other equipment, and also MIDI-out over both USB and DIN jacks with emphasis placed on flexibility. 16 separate 16-note Tracks (running simultaneously) are available for each Patch (song), with each Track able to choose any MIDI channel independently and send the note, pitch, velocity, etc. Saving and loading of Patches is done through an 24LC256 EEPROM over I2C. This EEPROM allows saving of 256 Patches, viewed on the LED Ring as 16 banks of 16 Patches each. In version 3, saving and restoring backups were extended to be possible using SD cards, in order to share Patches between Sundial units as well as transferring saves to a PC. The LED Ring is made using RGB LEDs and was originally driven by HC595 shift registers and 4-pin 'dumb' LEDs, with version 3 changing to addressable LEDs. Adafruit Neopixel libraries were used in operating the addressable LEDs, with some of their comments brought into the code for ease of use.


## The LED Ring and Rotary Encoder Interface


Using the LED Ring display, users are quickly able to set note quantity per track, change spacing/clustering between notes, pitch, velocity, note length, track length, instrument, MIDI channel, overall tempo, LED brightness, as well as accessing the save/load and backup menus. These functions and their settings are divided into different Pages being displayed on the LED Ring, one at a time, and quickly changeable. By pressing one of the 4 Page buttons along the side of the LED Ring, or a combination of 2 Page buttons, or by clicking the rotary encoder, users are able to access these different settings Pages for each Track. Using RGB LEDs, different Tracks and settings Pages are color-coded for ease of use. 


The bulk of creating a song is done on a default/idle page, 4 main Pages, and 4 alternate Pages. Spinning the rotary encoder cycles between the 16 different Tracks available per Patch and also changes the values of the current Page's settings. This allows for rapid use after gaining some familiarity with the 4 Page buttons (and a click of the encoder accessing the alternate function for each Page). One Push of a button and/or encoder click brings the user to the most commonly used settings, and a spinning of the encoder is used to increase or decrease the setting. Less commonly used settings (such as LED brightness) are accessed by pressing 2 Page buttons at once. For options with more than 16 values to choose between, such as Pitch and most other MIDI commands having 256 available values, a 3-color system uses the first 10 LEDs with a different color for each the ones place, tens place, and hundreds place simultaneously shown to display the setting's value. Seeing the current value is intuitive using this 3-color system, and is accessing the full range of values is made easy: rapidly spinning the encoder faster changes these by a variably greater amount quickly while slower spinning allows for fine-tuning values, all in the same motion. A reference sheet explaining the controls is below.

![control reference sheet](./media/control_sheet.png)


## Firmware Versions


The firmware spans 3 versions:
* Version 1: This was made for the original handwired prototype and subsequent 'Euclidimidi' prototype PCB.
* Version 2: This version was the next breadboarded iteration of improvements, included only for reference.
* Version 3: This latest version made a jump away from HC595 shift registers into driving addressable LEDs and also implemented an addon bus for 'hats' (for SD card saves and future purposes). This version was intended to be ported to a SAMD21 for more processing power and to allow for more features.


### Version 1

![version 1 original breadboard](./media/v1_breadboard.jpeg)
Version 1 was originally breadboarded using an Arduino UNO. It was then handwired using an Arduino Pro Micro to utilize built-in USB support for MIDI usage using protoboard. A 6N138 optoisolater protects the MIDI connection (as per MIDI specifications) and 6x 74HC595 shift registers control the red, blue, and green signals to the LED Ring. A 74HC4051 multiplexer was used to sweep the button connections, allowing for only 4 pins to be used to monitor the 8 buttons (3 select pins and one data return pin). The VS1053B circuit was made following Adafruit designs to provide clean audio, with its own combination 3.3v and 1.8v power supply. A 24LC256 EEPROM was used to save Patches over an I2C connection.

![v1 handwired exposed circuits](./media/v1_handwire.jpg)

Once the handwired design was proven it was used to create the schematic and layout for the version 1 PCB. This was done in EasyEDA, an online tool provided by the PCB fabricator, JLCPCB. Unfortunately, the online backups were corrupted and only earlier incomplete .png image versions of the schematic and layout were able to be recovered for this collection. A photograph of the hardcopy final schematic is preserved [here](./media/v1_hardcopy_schematic).

**V1 Schematic**
![v1 schematic](./media/schematic_v1.png)


**V1 Layout**
![v1 layout](./media/layout_v1.png)


**V1 PCB**
![v1 circuit board](./media/v1_pcb.jpg)


**V1 Prototypes**
![exposed version 1 circuitry](./media/version1.jpg)


**V1 Design Corrections**
The ICSP header (used to program the ATmega32u4) did not include pulldown resistors and these were manually added after PCB production, as was a wire to fix an unconnected data pin to read back from the multiplexer. An additional ground and power wire were also required to increase the amount of copper supporting the power delivery to the LEDs.   


Silkscreen artwork depicting the spacejunk floating in orbit around earth was added to the area around the circle of 74HC595's, however this silkscreen test did not turn out as appealing as intended and was scrapped in subsequent designs.

![spacejunk artwork test](./media/spacejunk.png)


### Version 2

Version 2 included minor hardware and firmware updates that were incompatible with the original PCB. The firmware has been included in the collection for completeness. Some of the updates included changing the sizes of components to be consistent. Version 1 included many components of different sizes for testing purposes, and some were too small to comfortably hand soldered. Version 2 updated these components to 0603 or 0805 SMD sizes. Some original parts that were swapped are pictured below.

![small components from version 1](./media/v1_small_parts.jpg)

### Version 3

Version 3 made significant changes to the design. The 74HC595's were removed and the shift-register-based design was changed to an addressable LED design. These addressable LEDs require more power to operate, but offer simpler wiring connections, as well as many more available colors. The shift registers offered only on/off operation for each of the 3 color channels (R, G, and B), however the entire #000000-#FFFFFF hex color spectrum is possible with addressable LEDs through serial communication. Extra-large 8mm LEDs were chosen for aesthetics. The controls were also redesigned to use surface mount pushbuttons, which would have caps placed on top for a more professional look.  


![version 3 breadboard](./media/v3_breadboard.jpg)


Version 3 also added many different expansion ports to the design. Numerous breakout ports were added to unused microcontroller pins and some data pins, which would easily allow them to be manually soldered and repurposed after production. Solderable ports were added to be able to interrupt the power delivery system in order to optionally add a rechargeable battery in the future. Additionally, two SPI based expansion buses was added, which allowed for adding removable 'shield' daughterboards for increased functionality. A FATcat daughterboard to save/load backups to SD cards was designed, breadboarded, and successfully tested using this SPI bus. 

Though running on overall version 3 firmware, the most current hardware design is separately considered the version 4 PCB. This github will continue the convention of referring to the firmware's version instead of the PCB's label. This circuit design has been breadboarded, tested for operation, and designed in EasyEDA, however it has not been produced as a PCB as the design was intended to be ported next to a SAMD21 processor. The exported EasyEDA .json file for the version 3 design as well as the BOM for all the components have been provided in [the assets section](./assets/) of this github.


**V3 Schematic**
![v3 schematic](./media/schematic_v3.png)


**V3 Layout**
![v3 layout](./media/layout_v3.png)


**V3 PCB Render**
![v3 circuit board render front](./media/3d_front_v3.png)

![v3 circuit board render back](./media/3d_rear_v3.png)


**V3 Printed Renders for Test Fitting**
![v3 circuit board testfit front](./media/v3_testfit1.jpg)

![v3 circuit board testfit back](./media/v3_testfit2.jpg)


## Shield

The FATcat is designed to work as an SD card backup device. After successfully breadboarding with a premade SD module, the FATcat schematics and footprint were made using EasyEDA, however the board was never produced.


**FATcat Schematic**
![FATcat shield schematic](./media/FATcat_schematic.png)


**FATcat Renders**
![FATcat shield render front](./media/FATcat_shield_top.png) | ![FATcat shield render rear](./media/FATcat_shield_bottom.png)


Thanks for taking a look!
