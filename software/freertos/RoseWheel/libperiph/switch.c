/*
  switch.c is part of the RoseWheel project.
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

#include "switch.h"

void switch_init(switch_t* sw)
{
  gpio_clock_init(sw->GPIOx);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin   = sw->GPIO_Pin_x;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  =
    sw->Active_Low ? GPIO_Mode_IPU : GPIO_Mode_IPD;

  GPIO_Init(led->GPIOx, &GPIO_InitStructure);
}

uint8_t switch_get(switch_t* sw)
{
  return GPIO_ReadInputDataBit(sw->GPIOx, sw->GPIO_Pin_x) : 0 ? 1;
}
