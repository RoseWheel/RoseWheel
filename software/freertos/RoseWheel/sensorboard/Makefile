PROGRAM = main
LIBS = -lperiph -lfilter -lglobal -lm
CFLAGS = -DUSE_UART4

# One of "ram" or "flash"
MODE = flash

# One of "hd" or "md"
DENSITY = hd

# Object files
OBJS = main.o leds.o can.o uart.o version.o		\
       port.o queue.o tasks.o list.o heap_1.o

# Uncomment to allow debugging
# CFLAGS += -DDEBUG

include ../Makefile.inc

$(PROGRAM): libperiph libfilter libglobal
