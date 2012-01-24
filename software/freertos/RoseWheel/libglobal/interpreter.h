/*
  interpreter.h is part of the RoseWheel project.
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

#ifndef INTERPRETER_H
# define INTERPRETER_H

#include "FreeRTOS.h"

typedef void (*pfunTokenHandle) (char*);

typedef struct
{
  char command;
  pfunTokenHandle handler;
} token_t;

void vInterpreterInit(const char* pr, token_t* tok, int n,
                      unsigned portBASE_TYPE daemon_priority);
void vInterpreterStart();

#endif
