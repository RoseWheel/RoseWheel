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

#include "libperiph/leds.h"

#include "leds.h"

static led_pwm_t led_rgb_r = {.GPIOx = GPIOA, .GPIO_Pin_x = GPIO_Pin_8,
                              .TIMx = TIM1,   .TIM_Channel = TIM_Channel_1};
static led_pwm_t led_rgb_g = {.GPIOx = GPIOA, .GPIO_Pin_x = GPIO_Pin_9,
                              .TIMx = TIM1,   .TIM_Channel = TIM_Channel_2};
static led_pwm_t led_rgb_b = {.GPIOx = GPIOA, .GPIO_Pin_x = GPIO_Pin_10,
                              .TIMx = TIM1,   .TIM_Channel = TIM_Channel_3};
static led_switch_t led_r = {.GPIOx = GPIOC, .GPIO_Pin_x = GPIO_Pin_14};
static led_switch_t led_g = {.GPIOx = GPIOC, .GPIO_Pin_x = GPIO_Pin_13};
static led_switch_t led_y = {.GPIOx = GPIOC, .GPIO_Pin_x = GPIO_Pin_15};

void vLEDsInit()
{
  led_pwm_init(&led_rgb_r);
  led_pwm_init(&led_rgb_g);
  led_pwm_init(&led_rgb_b);
  led_switch_init(&led_r);
  led_switch_init(&led_g);
  led_switch_init(&led_y);
}

void vLEDOn(xLED led)
{
  switch (led)
  {
    case LED_RED:
      led_switch_on(&led_r);
      break;
    case LED_GREEN:
      led_switch_on(&led_g);
      break;
    case LED_YELLOW:
      led_switch_on(&led_y);
      break;
  }
}

void vLEDOff(xLED led)
{
  switch (led)
  {
    case LED_RED:
      led_switch_off(&led_r);
      break;
    case LED_GREEN:
      led_switch_off(&led_g);
      break;
    case LED_YELLOW:
      led_switch_off(&led_y);
      break;
  }
}

void vLEDToggle(xLED led)
{
  switch (led)
  {
    case LED_RED:
      led_switch_toggle(&led_r);
      break;
    case LED_GREEN:
      led_switch_toggle(&led_g);
      break;
    case LED_YELLOW:
      led_switch_toggle(&led_y);
      break;
  }
}

void vLEDColorIntensity(xColorLED led, uint8_t intensity)
{
  switch (led)
  {
    case LED_COLOR_RED:
      led_pwm_intensity(&led_rgb_r, intensity);
      break;
    case LED_COLOR_GREEN:
      led_pwm_intensity(&led_rgb_g, intensity);
      break;
    case LED_COLOR_BLUE:
      led_pwm_intensity(&led_rgb_b, intensity);
      break;
  }
}

void vLEDColor(uint32_t color)
{
  led_pwm_intensity(&led_rgb_r, (uint8_t)((color << 0)  & 0xff));
  led_pwm_intensity(&led_rgb_g, (uint8_t)((color << 8)  & 0xff));
  led_pwm_intensity(&led_rgb_b, (uint8_t)((color << 16) & 0xff));
}

void vLEDColorOff()
{
  led_pwm_intensity(&led_rgb_r, 0);
  led_pwm_intensity(&led_rgb_g, 0);
  led_pwm_intensity(&led_rgb_b, 0);
}
