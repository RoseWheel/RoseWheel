/*
  battery.c is part of the RoseWheel project.
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

#include "FreeRTOS.h"
#include "task.h"

#include "libglobal/strutils.h"
#include "libperiph/uart.h"
#include "libperiph/adc.h"

#include "battery.h"
#include "can.h"
#include "leds.h"

#define V_MAX 24000
#define V_MIN 20000
#define V_EMPTY 18000
#define ALPHA 0.1

#ifdef NDEBUG
# define DEBUG 0
#else
# define DEBUG 1
#endif

// R1 = 47k, R2 = 3k, VADC = 3.3V
#define VOLTS_TO_LSB (4096 * 3000 / ((47000 + 3000) * 3.3))
#define ADC_TO_MILIVOLTS(Adc) (((Adc) * 1000) / VOLTS_TO_LSB)

static adc_t bat_voltage;
static uint32_t level = V_MAX;
static void vBatteryDaemon(void* pvParameters);

void vBatteryInit(unsigned portBASE_TYPE daemon_priority)
{
  bat_voltage.GPIOx       = GPIOC;
  bat_voltage.GPIO_Pin_x  = GPIO_Pin_0;
  bat_voltage.ADC_Channel = ADC_Channel_10;

  adc_init(&bat_voltage);

  // Create the daemon
  xTaskCreate(vBatteryDaemon, (const signed char * const)"batteryd",
	      configMINIMAL_STACK_SIZE, NULL, daemon_priority, NULL);
}

int iBatteryGetLevel()
{
  return level;
}

static void vBatteryDaemon(void* pvParameters)
{
  xCANMsg xMsg = {
    .eID = MSG_BATTERYLVL,
  };

  int n_low = 0;

  vLEDOff(COLOR_LED_RED);

  for (;;)
  {
    uint16_t adc = adc_read(&bat_voltage);
    uint32_t level_measured = ADC_TO_MILIVOLTS(adc);

    if (level_measured > V_MAX)
      level_measured = V_MAX;

    level = ALPHA * level_measured + (1 - ALPHA) * level;
    
    if (level <= V_MIN)
      n_low++;
    else if (n_low > 0)
      n_low--;

    xMsg.xData.words.low = ((level - V_EMPTY) * 100) / (V_MAX - V_EMPTY);

    if (n_low >= 2)
    {
      xMsg.xData.words.high = 0x1;
      vCANSendMsg(&xMsg);
      for (int i = 0; i < 6; i++)
      {
        uart_puts("LOW BATTERY !!!\r\n");
        vLEDToggle(COLOR_LED_RED);
        vTaskDelay(500 / portTICK_RATE_MS);
      }
    }
    else
    {
      xMsg.xData.words.high = 0x0;
      vCANSendMsg(&xMsg);
      vTaskDelay(3000 / portTICK_RATE_MS);
    }
  }
}
