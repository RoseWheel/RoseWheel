/*
  motors.h is part of the RoseWheel project.
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

#ifndef MOTORS_H
#define MOTORS_H

#include "FreeRTOS.h"

#define COMMAND_VOLT_RATE 20

void vMotorsInit(unsigned portBASE_TYPE motors_daemon_priority,
                 unsigned portBASE_TYPE beeping_daemon_priority);
void vMotorsEnable();
void vMotorsDisable();
void vMotorsCommand(int left, int right);
void vMotorsLeftCommand(int left);
void vMotorsRightCommand(int right);
int iMotorsGetLeftCommand();
int iMotorsGetRightCommand();
float fMotorsGetSteeringFactor(int iAngleDeg);
void vSetBeepFreq(int f);
void vSetBeepCont(int dt_ms);
void vSetBeep(unsigned int frequency, unsigned int duration_ms);
void vBeepOn();
void vBeepOff();

#endif

