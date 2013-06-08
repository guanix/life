PROJECT		= life
F_CPU 		= 8000000
PART		= attiny24a
DUDEPART	= t24
CFLAGS		= -Wall -mmcu=$(PART) -DF_CPU=$(F_CPU) -Os -g
OBJECTS		= $(PROJECT).o
CC		= avr-g++
P 		= $(PROJECT).hex
OBJCOPY 	= avr-objcopy
TOOL = dragon_isp
SPEED = 16

all: $(P) binary size

%.hex: %.o
	$(OBJCOPY) -Oihex $< $@

%.o: %.c %.h
	$(CC) $(CFLAGS) -o $@ $<

fi:
	avrdude -c $(TOOL) -P usb -p $(DUDEPART) -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m -U flash:w:$(P) -B $(SPEED)

install:
	avrdude -c $(TOOL) -P usb -p $(DUDEPART) -U flash:w:$(P) -B $(SPEED)

fuses:
	avrdude -c $(TOOL) -P usb -p $(DUDEPART) -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

binary: $(PROJECT).bin

$(PROJECT).bin: $(OBJECTS)
	$(OBJCOPY) -Obinary $(OBJECTS) $(PROJECT).bin

das: $(PROJECT).as

$(PROJECT).as: $(PROJECT).o
	avr-objdump -S $(PROJECT).o > $(PROJECT).as

size: $(OBJECTS)
	avr-size $(OBJECTS)

clean:
	rm *.bin *.hex *.o
