/*
  sharps.h is part of the RoseWheel project.
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

#ifndef LIBPERIPH_SHARPS_H
# define LIBPERIPH_SHARPS_H

#define SHARPS_MAX_NUMBER 16

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
  uint8_t ADC_Channel;
} sharp_t;

typedef struct
{
  int number;
  ADC_TypeDef* ADCx;
  sharp_t ADCs[SHARPS_MAX_NUMBER];
} sharps_t;

void sharps_init(sharps_t* sharps);
uint16_t sharps_get_value(int i);

#endif

