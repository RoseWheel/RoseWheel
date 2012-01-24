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

#ifndef LEDS_H
# define LEDS_H

typedef enum
{
  LED_RED,
  LED_GREEN,
  LED_YELLOW,
} xLED;

typedef enum
{
  LED_COLOR_RED,
  LED_COLOR_GREEN,
  LED_COLOR_BLUE,
} xColorLED;

void vLEDsInit();
void vLEDOn(xLED led);
void vLEDOff(xLED led);
void vLEDToggle(xLED led);
void vLEDIntensity(xColorLED led, uint8_t intensity);
void vLEDColor(uint32_t color);
void vLEDColorOff();

#endif
