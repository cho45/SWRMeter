DEVICE     = atmega328p
CLOCK      = 8000000
PROGRAMMER = -c avrispmkII -P usb
OBJECTS    = main.o
# Use http://www.engbedded.com/fusecalc
FUSES      = -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
HEX = .pioenvs/nanoatmega328/firmware.hex

all:
	platformio run

flash:	all
	$(AVRDUDE) -U flash:w:$(HEX):i

fuse:
	$(AVRDUDE) $(FUSES)

