# Name: Makefile
# Author: henniek 18-06-2018

# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE     = atmega328p 
CLOCK      = 8000000
PROGRAMMER = -c avrispmkII -P usb
OBJECTS    = main.o
#::::::::::::::::::::http://www.engbedded.com/fusecalc/
FUSES      = -U lfuse:w:0xe2:m -U hfuse:w:0xd5:m -U efuse:w:0xff:m #--ERASE EEPROM
#FUSES      = -U lfuse:w:0xe2:m -U hfuse:w:0xd5:m -U efuse:w:0x07:m #--ERASE EEPROM
#FUSES      = -U lfuse:w:0xe2:m -U hfuse:w:0xd1:m -U efuse:w:0x07:m #--PRESERVE EEPROM


######################################################################
######################################################################

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE) -v #-vvv #-B10
#CXX = avr-gcc -print-file-name=include -v -Wall -g -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) 
CXX = avr-gcc -Wall -g -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)  

# symbolic targets:
all:	main.hex

.c.o:
	$(CXX) -c  $< -o $@

.S.o:
	$(CXX) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(CXX) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

install: flash fuse

# if you use a bootloader, change the command below appropriately:
#load: all
#	bootloadHID main.hex

upload: flash fuse

clean:
	rm -f main.hex main.elf $(OBJECTS)

# file targets:
main.elf: $(OBJECTS)
	$(CXX) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(CXX) -E main.c