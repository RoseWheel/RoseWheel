/*
  leds.c is part of the RoseWheel project.
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

#include "leds.h"
#include "libperiph/leds.h"

static led_switch_t leds[3] = {
  {.GPIOx = GPIOC, .GPIO_Pin_x = GPIO_Pin_10}, // LED_Rn
  {.GPIOx = GPIOC, .GPIO_Pin_x = GPIO_Pin_11}, // LED_Gn
  {.GPIOx = GPIOC, .GPIO_Pin_x = GPIO_Pin_12}  // LED_Bn
};

void vLEDsInit()
{
  for (int i = 0; i < 3; i++)
    led_switch_init(&leds[i]);
}

void vLEDOn(enum eColorLED eLED)
{
  led_switch_on(&leds[eLED]);
}

void vLEDOff(enum eColorLED eLED)
{
   led_switch_off(&leds[eLED]);
}

void vLEDToggle(enum eColorLED eLED)
{
  led_switch_toggle(&leds[eLED]);
}

void vLEDSetColor(enum eColor eLEDColor)
{
  for (int i = 0; i < 3; i++) {
    if (eLEDColor & (1 << i))
        led_switch_on(&leds[i]);
    else
      led_switch_off(&leds[i]);
  }
}


