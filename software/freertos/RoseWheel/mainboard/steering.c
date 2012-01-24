/*
  steering.c is part of the RoseWheel project.
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

#include "steering.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "libperiph/adc.h"

#include "libperiph/uart.h"
#include "strutils.h"

#define STEERING_MIN 1710
#define STEERING_MAX 2236

static adc_t steer_pot;

void vSteeringInit()
{
  steer_pot.GPIOx       = GPIOC;
  steer_pot.GPIO_Pin_x  = GPIO_Pin_4;
  steer_pot.ADC_Channel = ADC_Channel_14;
  adc_init(&steer_pot);
}

int iSteeringGetValue()
{
  uint16_t x = adc_read(&steer_pot);

  if (x < STEERING_MIN)
    x = STEERING_MIN;

  if (x > STEERING_MAX)
    x = STEERING_MAX;

  return 200 * (x - STEERING_MIN) / (STEERING_MAX - STEERING_MIN) - 100;
}

