#  $@ is the name of the current target, $< is the name of the first prerequisite, and $^ is the list of all the prerequisites
#
BIN=hello
OBJS=hello.o serial.o

CC=avr-gcc
OBJCOPY=avr-objcopy
MCU=atmega328p
CFLAGS=-Os -DF_CPU=18000000UL -DBAUD=115200 -mmcu=${MCU} -Wall
PORT=/dev/ttyACM0

.PHONY: install clean backup disassemble

${BIN}.hex: ${BIN}.elf
	    ${OBJCOPY} -O ihex -R .eeprom $< $@

${BIN}.elf: ${OBJS}
	    ${CC} ${CFLAGS} -o $@ $^ 

${BIN}.lst: ${BIN}
	avr-objdump -d $^ > $@

#install: backup ${BIN}.hex
install: ${BIN}.hex
	    avrdude -v -c arduino -p ${MCU} -P ${PORT} -b 115200 -U flash:w:$<

clean:
	    rm -f ${BIN}.elf ${BIN}.hex ${OBJS} ${BIN}.lst

disassemble: ${BIN}.lst

backup:
	avrdude -F -V -c arduino -p ${MCU} -P ${PORT} -b 115200 -U flash:r:flash_backup.hex:i
