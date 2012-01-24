/*
  leds.h is part of the RoseWheel project.
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

#ifndef LIBPERIPH_LEDS_H
# define LIBPERIPH_LEDS_H

# include "stm32f10x_gpio.h"
# include "stm32f10x_tim.h"

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
} led_switch_t;

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
  TIM_TypeDef* TIMx;
  uint16_t TIM_Channel;
} led_pwm_t;

void led_pwm_init(led_pwm_t* led);
void led_pwm_intensity(led_pwm_t* led, uint8_t value);

void led_switch_init(led_switch_t* led);
void led_switch_on(led_switch_t* led);
void led_switch_off(led_switch_t* led);
void led_switch_toggle(led_switch_t* led);

#endif
