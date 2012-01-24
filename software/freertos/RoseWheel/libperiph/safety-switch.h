/*
  safety-switch.h is part of the RoseWheel project.
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

#ifndef SAFETY_SWITCH_H
# define SAFETY_SWITCH_H

#include "FreeRTOS.h"

typedef void (*safety_handler_t)();

typedef enum
{
  PRESS,
  RELEASE,
  LONG_RELEASE,
} event_t;

void safety_switch_init(unsigned portBASE_TYPE daemon_priority);
void safety_switch_set_handler(event_t event, safety_handler_t handler);
bool safety_switch_set_long_release_time(int ms);

#endif
