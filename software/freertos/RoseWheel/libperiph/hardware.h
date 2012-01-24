/*
  hardware.h is part of the RoseWheel project.
  Copyright (C) 2011 RoseWheel Team <rosewheel@googlegroups.com>
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LIBPERIPH_HARDWARE_H
# define LIBPERIPH_HARDWARE_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"

#define MS_TO_TICKS(time_ms) ((portTickType)((time_ms) / portTICK_RATE_MS))

#define GPIO_TO_EXTI_LINE(GPIO_Pin) (GPIO_Pin)

void hardware_init();
void gpio_clock_init(GPIO_TypeDef* GPIOx);
void timer_clock_init(TIM_TypeDef* TIMx);
void dma_clock_init(DMA_TypeDef* DMAx);
void adc_clock_init(ADC_TypeDef* ADCx);
void spi_clock_init(SPI_TypeDef* SPIx);
void can_clock_init(CAN_TypeDef* CANx);
uint8_t gpio_pin_to_source(uint16_t GPIO_Pin_x);
void wait_us(int x);

#endif
