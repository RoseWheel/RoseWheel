/*
  buzzer.h is part of the RoseWheel project.
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

#ifndef LIBPERIPH_BUZZER_H
# define LIBPERIPH_BUZZER_H

# define BUZZER_VOLUME_STEPS 1024

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
  TIM_TypeDef* TIMx;
  uint16_t TIM_Channel;
} buzzer_t;

void buzzer_init(buzzer_t* buzzer);
void buzzer_on(buzzer_t* buzzer);
void buzzer_off(buzzer_t* buzzer);
void buzzer_volume_set(buzzer_t* buzzer, int v);
void buzzer_frequency_set(buzzer_t* buzzer, int f);

#endif
