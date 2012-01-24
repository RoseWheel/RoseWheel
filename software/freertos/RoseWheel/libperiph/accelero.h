/*
  accelero.h is part of the RoseWheel project.
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

#ifndef LIBPERIPH_ACCELERO_H
# define LIBPERIPH_ACCELERO_H

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"

#include "direction.h"

#define ACCELERO_SENSITIVITY_TYP_6G 340 /*LSB/g*/

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} accelero_values_t;

void accelero_init(unsigned portBASE_TYPE daemon_priority);
int16_t accelero_get_value(direction_t dir);
void accelero_get_values(accelero_values_t* values);

#endif
