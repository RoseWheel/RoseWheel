PROGRAM = main
LIBS = -lperiph -lcontrol -lglobal
CFLAGS = -DUSE_USART1

# One of "ram" or "flash"
MODE = flash

# One of "hd" or "md"
DENSITY = hd

# Object files
OBJS = main.o leds.o can.o uart.o control.o motors.o steering.o 	\
	port.o queue.o tasks.o list.o heap_1.o pidvalues.o battery.o 	\
	version.o sonar.o

# Uncomment to allow debugging
# CFLAGS += -DDEBUG

include ../Makefile.inc

$(PROGRAM): libperiph libcontrol libglobal
