#  $@ is the name of the current target, $< is the name of the first prerequisite, and $^ is the list of all the prerequisites
#
BIN=sumo
OBJS=sumo.o serial.o sonar.o driver.o analog.o reflectance.o sharpdistance.o timer.o

BAUD=9600

CC=avr-gcc
OBJCOPY=avr-objcopy
MCU=atmega328p
CFLAGS=-Os -DF_CPU=16000000UL -DBAUD=${BAUD}UL -mmcu=${MCU} -Wall
PORT=/dev/ttyACM0

.PHONY: install clean backup disassemble monitor

DEBUG ?= 0
ifeq ($(DEBUG), 1)
	CFLAGS += -DDEBUG -g
endif

${BIN}.hex: ${BIN}.elf
	    ${OBJCOPY} -O ihex -R .eeprom $< $@

${BIN}.elf: ${OBJS}
	    ${CC} ${CFLAGS} -o $@ $^ 

${BIN}.lst: ${OBJS}
	avr-objdump -d $^ > $@

#install: backup ${BIN}.hex
install: ${BIN}.hex
	    avrdude -v -c arduino -p ${MCU} -P ${PORT} -b 115200 -U flash:w:$<

clean:
	    rm -f ${BIN}.elf ${BIN}.hex ${OBJS} ${BIN}.lst

disassemble: ${BIN}.lst

backup:
	avrdude -F -V -c arduino -p ${MCU} -P ${PORT} -b 115200 -U flash:r:flash_backup.hex:i

monitor:
	screen ${PORT} ${BAUD}
