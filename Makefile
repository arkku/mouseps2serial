# Makefile
# Copyright (c) 2020 Kimmo Kulovesi, https://arkku.dev/
###############################################################################
# Put your local settings in "local.mk", it is ignored by Git.
-include local.mk

### AVR MCU ###################################################################
# Only tested with ATmega328P.

MCU ?= atmega328p
F_CPU ?= 16000000UL
#F_CPU ?= 14745600UL

# Bootloader rate only, the device itself is configured by DIP switches
BAUD ?= 115200UL

LFUSE ?= BF
HFUSE ?= C7
EFUSE ?= FD

### CONFIGURATION #############################################################
# Define these to hard-code settings instead of supporting DIP configuration;
# this reduces the size of the binary by about half when the debug output
# support is removed.
#FORCE_PROTOCOL ?= PROTOCOL_MICROSOFT
#FORCE_BAUD ?= 1200UL

FORCE_PROTOCOL ?=
FORCE_BAUD ?=
MOUSE_DIVISOR ?=
MOUSE_SCALING ?=

SERIAL_STATE_INVERTED ?= 1

PS2_BUFFER_SIZE ?= 256

#### BURNER ###################################################################
# Specify the burner on the command-line if you wish, e.g.,
#	make burn BURNER=avrisp2 PORT=/dev/ttyUSB0 BPS=115200

# Burner device
BURNER ?= dragon_isp
# Burner port
#PORT ?= /dev/ttyUSB0
# Burner speed
#BPS ?= 115200
###############################################################################
CC=avr-gcc
CFLAGS=-Wall -std=c11 -pedantic -Wextra -Wno-unused-parameter -Os $(AVR_FLAGS) $(CONFIG_FLAGS)
LDFLAGS=-Os $(AVR_FLAGS)
AR=avr-ar
ARFLAGS=rcs
OBJCOPY=avr-objcopy
AVRDUDE=avrdude

AVR_FLAGS=-mmcu=$(MCU) -DF_CPU=$(F_CPU)

CONFIG_FLAGS=-DKK_UART_TRANSMIT_BUFFER_SIZE=0 -DKK_UART_RECEIVE_BUFFER_SIZE=32 -DKK_PS2_BUFFER_SIZE=$(PS2_BUFFER_SIZE) -DBAUD=$(BAUD) -DFORCE_SERIAL_PROTOCOL=$(FORCE_PROTOCOL) -DFORCE_BAUD=$(FORCE_BAUD) -DSERIAL_STATE_INVERTED=$(SERIAL_STATE_INVERTED) -DMOUSE_DIVISOR=$(MOUSE_DIVISOR) -DMOUSE_SCALING=$(MOUSE_SCALING)

HEX=ps2serial.hex
BIN=ps2serial.elf
OBJS=ps2serial.o kk_uart.o kk_ps2.o
BOOTLOADER=optiboot_atmega328.hex

all: $(HEX)

ps2serial.o: kk_uart.h kk_ps2.h flowctl.h led.h dip.h timer.h
kk_uart.o: kk_uart.h
kk_ps2.o: kk_ps2.h
$(OBJS): Makefile

$(HEX): $(BIN)

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $+

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

burn: $(HEX)
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b $(BPS) ,)-p $(MCU) -U flash:w:$< -v

upload: $(HEX)
	$(AVRDUDE) -c arduino $(if $(PORT),-P $(PORT),-P /dev/ttyUSB0) $(if $(BPS),-b $(BPS),-b $(subst L,,$(subst U,,$(BAUD)))) -p $(MCU) -U flash:w:$< -v

fuses:
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b (BPS) ,)-p $(MCU) -U efuse:w:0x$(EFUSE):m -U hfuse:w:0x$(HFUSE):m -U lfuse:w:0x$(LFUSE):m

bootloader: $(BOOTLOADER)
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b $(BPS) ,)-p $(MCU) -e -U flash:w:$< -v

unlock:
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b $(BPS) ,)-p $(MCU) -U lock:w:0x3F:m -v

lock:
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b $(BPS) ,)-p $(MCU) -U lock:w:0x0F:m -v

.ccls: Makefile
	@echo $(CC) >$@
	@echo --target=avr >>$@
	@echo $(CFLAGS) | awk '{ for (i=1; i<=NF; i++) { print $$i } }' >>$@
	@echo -nostdinc >>$@
	@[ -d /usr/local/avr/include ] && echo -I/usr/local/avr/include >>$@
	@echo | $(CC) $(CFLAGS) -E -Wp,-v - 2>&1 | awk '/#include .* search starts here:/ { output=1; next } !output { next } /^End/ || /^#/ { output=0 } output && $$1 ~ /^\// { sub(/^[ ]*/, ""); print "-I" $$0 }' >>$@
	@cat $@

clean:
	rm -f $(OBJS)

distclean: | clean
	rm -f $(HEX) $(BIN) .ccls

.PHONY: all clean distclean burn fuses upload lock unlock bootloader
