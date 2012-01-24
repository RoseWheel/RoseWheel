# This makefile is part of the RoseWheel project.
# Copyright (C) 2011 RoseWheel Team <rosewheel@googlegroups.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


# This file will be included in project's Makefile
#
# The Makefile has to define:
#   DENSITY: md, hd or cl
#   MODE:    flash or ram
#   OBJS:    the list of object files
#   PROGRAM: the final program name

MKDIR_P = mkdir -p

ASSERT_HEADER_INC = -include assert_param.h

CC = arm-none-eabi-gcc
CFLAGS += -std=c99 -Wall -Werror -fasm -fdata-sections -ffunction-sections 	\
	$(INCCFLAGS) $(ARCHFLAGS) $(OPTFLAGS) $(DEBUGFLAGS)
INCCFLAGS = -DGCC_ARMCM3 -I. -I$(LIBGLOBAL_DIR)					\
	-I$(FREERTOS_INCDIR) -I$(FW_INCDIR)					\
	-I$(STM32_DIR) -I$(FIRMWARE_DIR) -I$(ROSEWHEEL_DIR) -I$(SOURCE_DIR) 	\
	$(PROGRAM_MODE)
ARCHFLAGS = -mcpu=cortex-m3 -mthumb
OPTFLAGS = -g -Os -fmerge-all-constants -fsee
RANLIB = arm-none-eabi-ranlib
ARCHIVE = ar rc

ifeq ($(MODE),flash)
PROGRAM_MODE=-DPROGRAM_MODE_FLASH
else
PROGRAM_MODE=-DPROGRAM_MODE_RAM
endif

ifeq ($(DENSITY),hd)
INCCFLAGS += -DSTM32F10X_HD
else ifeq ($(DENSITY),md)
INCCFLAGS += -DSTM32F10X_MD
else
INCCFLAGS += -DSTM32F10X_CL
endif

STARTUP_OBJECT = startup_stm32f10x_$(DENSITY).o
LINKER_SCRIPT = stm32f10x_$(MODE)_$(DENSITY).ld

OBJS += $(STARTUP_OBJECT)

AS = arm-none-eabi-as
ASFLAGS = $(ARCHFLAGS)

LD = arm-none-eabi-gcc
LDFLAGS += -L$(STM32_DIR) 	\
	-T $(LINKER_SCRIPT) -g 	\
	-Wl,-Map=$(PROGRAM).map \
	-Wl,--gc-sections 	\
	$(ARCHFLAGS)

OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

SCRIPTS_DIR = ../../../scripts
FLASH = $(SCRIPTS_DIR)/flash

SOURCE_DIR = ../../..
PIDVALUES_DIR = $(SOURCE_DIR)/pidvalues
ROSEWHEEL_DIR = ../../RoseWheel
LIBGLOBAL_DIR = $(ROSEWHEEL_DIR)/libglobal
FREERTOS_DIR = ../../Source
FREERTOS_INCDIR = $(FREERTOS_DIR)/include
FREERTOS_PLATDIR = $(FREERTOS_DIR)/portable/GCC/ARM_CM3
FREERTOS_MEMDIR = $(FREERTOS_DIR)/portable/MemMang

FIRMWARE_DIR = ../firmware

FW_DIR = $(STM32_DIR)/STM32_USB-FS-Device_Lib_V3.1.0/Libraries
FW_CORE_DIR = $(FW_DIR)/CMSIS/Core/CM3
FW_STARTUP_DIR = $(FW_CORE_DIR)/startup/gcc
FW_STDDRIVER_DIR = $(FW_DIR)/STM32F10x_StdPeriph_Driver
FW_STDDRIVER_SRCDIR = $(FW_STDDRIVER_DIR)/src
FW_STDDRIVER_INCDIR = $(FW_STDDRIVER_DIR)/inc
FW_USB_DIR = $(FW_DIR)/Libraries/STM32_USB-FS-Device_Driver
FW_USB_SRCDIR = $(FW_USB_DIR)/src
FW_USB_INCDIR = $(FW_USB_DIR)/inc
INCCFLAGS += -I$(FW_CORE_DIR) -I$(FW_STARTUP_DIR) -I$(FW_STDDRIVER_INCDIR) \
	-I$(FW_USB_INCDIR)
VPATH += $(FW_CORE_DIR) $(FW_STARTUP_DIR) $(FW_STDDRIVER_SRCDIR) \
	 $(FW_USB_SRCDIR) $(FREERTOS_DIR) $(FREERTOS_PLATDIR) $(FREERTOS_MEMDIR) \
	 $(STM32_DIR) $(FIRMWARE_DIR) $(FW_STDDRIVER_SRCDIR) $(PIDVALUES_DIR)

FW_SRCDIR = $(FW_DIR)/src
FW_INCDIR = $(FW_DIR)/inc

STM32_DIR = ../../STM32

LIB_STDDRIVER = libstddriver.a
LIB_STDDRIVER_SRC = $(wildcard $(FW_STDDRIVER_SRCDIR)/*.c)
LIB_STDDRIVER_OBJ = $(notdir $(LIB_STDDRIVER_SRC:.c=.o))

all:: $(PROGRAM) $(LIB)

clean::
	$(RM) $(PROGRAM) *.o $(PROGRAM).map $(PROGRAM).bin *~ \#*\#
	$(RM) $(LIB)
	@$(RM) -r .deps

$(PROGRAM): $(OBJS)
	$(LD) $(LDFLAGS) -o $(PROGRAM) $(OBJS) $(LIBS)
	@$(MAKE) info

$(LIB_STDDRIVER): $(LIB_STDDRIVER_OBJ)
	$(ARCHIVE) $@ $(LIB_STDDRIVER_OBJ)
	$(RANLIB) $@

$(LIB): $(LIBOBJS)
	$(RM) $@
	$(ARCHIVE) $@ $(filter %.o,$^)
	$(RANLIB) $@

info::
	$(OBJDUMP) --section-headers $(PROGRAM)

%.bin: %
	$(OBJCOPY) -O binary $< $@

$(LIB_STDDRIVER_OBJ): %.o: %.c
	$(CC) $(CFLAGS) $(ASSERT_HEADER_INC) -c -o $@ -MMD -MP $<
	@$(MKDIR_P) .deps
	@mv $(patsubst %.o,%.d,$@) .deps/

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ -MMD -MP $<
	@$(MKDIR_P) .deps
	@mv $(patsubst %.o,%.d,$@) .deps/

%.o: %.raw
	$(OBJCOPY) -B arm -I binary -O elf32-littlearm \
	--rename-section .data=.rodata,alloc,load,readonly,data,contents $< $@

*.o: Makefile ../Makefile.inc

flash:: all
	openocd -f ../openocd_flash.cfg

check-syntax:
	$(CC) -o /dev/null $(CFLAGS) -S -Wno-error ${CHK_SOURCES}

LDFLAGS += -L$(ROSEWHEEL_DIR)/libperiph
libperiph::
	$(MAKE) DENSITY=$(DENSITY) -C $(ROSEWHEEL_DIR)/libperiph

LDFLAGS += -L$(ROSEWHEEL_DIR)/libfilter
libfilter::
	$(MAKE) DENSITY=$(DENSITY) -C $(ROSEWHEEL_DIR)/libfilter

LDFLAGS += -L$(ROSEWHEEL_DIR)/libcontrol
libcontrol::
	$(MAKE) DENSITY=$(DENSITY) -C $(ROSEWHEEL_DIR)/libcontrol

LDFLAGS += -L$(ROSEWHEEL_DIR)/libglobal
libglobal::
	$(MAKE) DENSITY=$(DENSITY) -C $(ROSEWHEEL_DIR)/libglobal

#version.c::
#	echo    "const char* version = " > $@
#	echo    "\"$(shell git log --format='%h' -n1)\";" >> $@

DEPS := $(patsubst %.o,.deps/%.d,$(OBJS))

-include .deps/*.d
