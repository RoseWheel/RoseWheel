/*
  adc.h is part of the RoseWheel project.
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

#ifndef LIBPERIPH_ADC_H
# define LIBPERIPH_ADC_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
  uint8_t ADC_Channel;
} adc_t;

void adc_init(adc_t* adc);
unsigned short adc_read(adc_t* adc);

#endif
