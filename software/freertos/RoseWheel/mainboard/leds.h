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

enum eColorLED {
  COLOR_LED_RED   = 0,
  COLOR_LED_GREEN = 1,
  COLOR_LED_BLUE  = 2,
};

enum eColor {
  BLACK   = 0b000,
  RED     = 0b001,
  GREEN   = 0b010,
  BLUE    = 0b100,
  YELLOW  = 0b011,
  MAGENTA = 0b101,
  CYAN    = 0b110,
  WHITE   = 0b111,
};

void vLEDsInit();
void vLEDOn(enum eColorLED eLED);
void vLEDOff(enum eColorLED eLED);
void vLEDToggle(enum eColorLED eLED);
void vLEDSetColor(enum eColor eLEDColor);

#endif
