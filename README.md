dcc_gen
=======

Simple DCC generator for ATMega8 (V2, schematics and board) and ATMega32 (V3-5, only protoypes) to control small isolated layouts.

Features:
- RS232 Interface (9600 baud) for debugging and simple control (use an normal USB to RS232 converter)
- Heartbeat LED should blink roundabout every 2 sek, fast blinking if waiting for second key
- Connect simple switches (ATMega8 10 inputs | ATMega32 4 groups, 20 Inputs, 80 switches in total)
- Support start-end logic and searches for routes in the internal memory
- Setup for USBProg ISP-Interface adapter by www.embeded-projecs.de (for any other modify platformio.ini)

Toolchain (PlatformIo)
- install Visual Studio Code
- install PlatformIO Extension via Extension Manager (left toolbar). Wait til its installed completely (Icon appears in left toolbar)!
- clone, pull and open workspace dcc_gen
- Build it (bottom bar tick icon)Toolchain
- install driver for USBasp with Zadig (https://zadig.akeo.ie/)
  == USBasp (HK version with fischl.de FW): more or less working with: WinUSB v6.1.7600 and libusb_win32, somehow unreliable
  == USBProg 3 running with modified config (check for platformio.ini)
- use "Upload" in Tasks by click on the PlatformIo Icon (Note: returns an error even if the upload was succesful), check avrdude output!

Toolchain Linux (_Archive)
- sudo aptitude install avr-libc avrdude gcc-avr simulavr
- check/use the Makefile

Kown issues: 
- it is NOT a central unit, just a simple generator
- a lot of hardcoded stuff, no SW to init the memory
- bare metal (no Ardunio Code)
- explizit build for either ATMega8 oder ATMega32
- tested only with a few turnout decoder (Claus Illchmann, LDT Datentechnik) some decoder are not working (QDecoder). 
- like almost all motorshields based on this hardware the generatet DCC signal is not recommended for train control. 
- the ATMega8 is very limited in memory and far less verbose (no help etc.)

Helpers
- read fuses with avrdude: .\avrdude -c avrispmkii -p atmega8 -U lfuse:r:-:i -v
- http://www.engbedded.com/fusecalc
- https://hendrich.org/blogs/mikrocontroller/atmega-644-fuses-setzen/
- programm size: C:/Users/EMM/.platformio/packages/toolchain-atmelavr/bin/avr-size --mcu=atmega8 -C .\firmware.elf