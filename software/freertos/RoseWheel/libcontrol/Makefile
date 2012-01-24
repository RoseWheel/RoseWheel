LIB = libcontrol.a
LIBS = -lm

# One of "ram" or "flash"
MODE = flash

# One of "hd" or "md"
DENSITY = hd

# Object files
LIBOBJS = lqr.o pid.o
# Uncomment to allow debugging
# CFLAGS += -DDEBUG

include ../Makefile.inc

$(LIB): libglobal
