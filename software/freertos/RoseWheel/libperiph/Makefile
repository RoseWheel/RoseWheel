LIB = libperiph.a

# One of "ram" or "flash"
MODE = flash

# One of "hd" or "md"
DENSITY = hd

# Object files
LIBOBJS = can.o leds.o sharps.o buzzer.o hardware.o adc.o sonar.o 		\
	accelero.o gyroscope.o safety-switch.o bluetooth.o			\
	stm32f10x_adc.o stm32f10x_can.o stm32f10x_dma.o stm32f10x_exti.o 	\
	stm32f10x_flash.o stm32f10x_gpio.o stm32f10x_i2c.o stm32f10x_rcc.o 	\
	stm32f10x_spi.o stm32f10x_tim.o stm32f10x_usart.o misc.o

# Uncomment to allow debugging
# CFLAGS += -DDEBUG

include ../Makefile.inc

$(LIB): libglobal
