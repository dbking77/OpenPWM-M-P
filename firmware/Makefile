##
## This file is part of the avr-gcc-examples project.
##
## Copyright (C) 2008 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
##

PROGRAM = openpwm-m-p
MCU = attiny85
AVRDUDE_MCU = -p t85
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
CFLAGS += -Wall -ggdb -Os -mmcu=$(MCU)
LDFLAGS +=
OBJS = $(PROGRAM).o

# Programming device for AVRDUDE
AVRDUDE_PROG = -c avrispv2 -P /dev/ttyACM0

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

all: $(PROGRAM).hex $(PROGRAM).lst


$(PROGRAM).lst: $(PROGRAM).elf
	@printf "  OBJDUMP  $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(OBJDUMP) -S -t -d $^ > $@

$(PROGRAM).elf: $(PROGRAM).o
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(PROGRAM).hex: $(PROGRAM).elf
	@printf "  OBJCOPY $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(OBJCOPY) -O ihex $< $@

%.o: %.c
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

flash: $(PROGRAM).hex
	@printf "  FLASH   $(PROGRAM).hex\n"
	$(Q)avrdude $(AVRDUDE_PROG) $(AVRDUDE_MCU) -U flash:w:$(PROGRAM).hex

write_fuse:
	@printf "  FUSES\n"
	$(Q)avrdude $(AVRDUDE_PROG) $(AVRDUDE_MCU) -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

read_fuse:
	@printf "  READING FUSES\n"
	$(Q)avrdude $(AVRDUDE_PROG) $(AVRDUDE_MCU) -U lfuse:r:lfuse.txt:h -U hfuse:r:hfuse.txt:h -U efuse:r:efuse.txt:h

clean:
	@printf "  CLEAN   $(subst $(shell pwd)/,,$(OBJS))\n"
	$(Q)rm -f $(OBJS)
	@printf "  CLEAN   $(PROGRAM).elf\n"
	$(Q)rm -f *.elf
	@printf "  CLEAN   $(PROGRAM).hex\n"
	$(Q)rm -f *.hex

