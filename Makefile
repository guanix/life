PROJECT		= life
F_CPU 		= 8000000
PART		= attiny24a
DUDEPART	= t24
CFLAGS		= -Wall -mmcu=$(PART) -DF_CPU=$(F_CPU) -Os -g
OBJECTS		= $(PROJECT).o
CC		= avr-g++
P 		= $(PROJECT).hex
OBJCOPY 	= avr-objcopy

all: $(P) binary size

%.hex: %.o
	$(OBJCOPY) -Oihex $< $@

%.o: %.c %.h
	$(CC) $(CFLAGS) -o $@ $<

install:
	avrdude -c dragon_isp -P usb -p $(DUDEPART) -U flash:w:$(P) -B 16

fuses:
	avrdude -c dragon_isp -P usb -p $(DUDEPART) -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

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
