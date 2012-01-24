/*
  accelero.c is part of the RoseWheel project.
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

#include "assert_param.h"
#include "FreeRTOS.h"
#include "misc.h"
#include "semphr.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"

#include "accelero.h"
#include "hardware.h"
#include "libglobal/strutils.h"
#include "uart.h"

#ifdef NDEBUG
# define DEBUG 0
#else
# define DEBUG 1
#endif

#define IAMX 0x3a
#define READ 0x80
#define MULTIPLE 0x40
#define POWER_ON 0xC0
#define XEN 0x01
#define YEN 0x02
#define ZEN 0x04
#define ST 0x04
#define DECIMATE128 0x10
#define DECIMATE32 0x20
#define DECIMATE8 0x30
#define DRDY 0x04
#define FS 0x80
#define DAS 0x01
#define BDU 0x40

typedef enum
{
  WHO_AM_I = 0x0f,
  OFFSET_X = 0x16,
  OFFSET_Y = 0x17,
  OFFSET_Z = 0x18,
  GAIN_X = 0x19,
  GAIN_Y = 0x1a,
  GAIN_Z = 0x1b,
  CTRL_REG1 = 0x20,
  CTRL_REG2 = 0x21,
  CTRL_REG3 = 0x22,
  HP_FILTER_RESET = 0x23,
  STATUS_REG = 0x27,
  OUTX_L = 0x28,
  OUTX_H = 0x29,
  OUTY_L = 0x2a,
  OUTY_H = 0x2b,
  OUTZ_L = 0x2c,
  OUTZ_H = 0x2d,
  FF_WU_CFG = 0x30,
  FF_WU_SRC = 0x31,
  FF_WU_ACK = 0x32,
  FF_WU_THS_L = 0x34,
  FF_WU_THS_H = 0x35,
  FF_WU_DURATION = 0x36,
  DD_CFG = 0x38,
  DD_SRC = 0x39,
  DD_ACK = 0x3a,
  DD_THSI_L = 0x3c,
  DD_THSI_H = 0x3d,
  DD_THSE_L = 0x3e,
  DD_THSE_H = 0x3f,
} LIS_register;

#define SPI_NSS_PIN GPIO_Pin_4
#define SPI_SCK_PIN GPIO_Pin_5
#define SPI_MISO_PIN GPIO_Pin_6
#define SPI_MOSI_PIN GPIO_Pin_7
#define SPI_RDY_PIN GPIO_Pin_4

static xSemaphoreHandle xSPIRxSemphr;
static xSemaphoreHandle xValuesMutex;
static uint8_t response;
static int16_t accelero_values[3];
static void accelero_daemon(void* pvParameters);

void accelero_init(unsigned portBASE_TYPE daemon_priority)
{
  gpio_clock_init(GPIOA);
  gpio_clock_init(GPIOC);
  spi_clock_init(SPI1);

  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin   = SPI_NSS_PIN,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_Out_PP,
    };
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = SPI_RDY_PIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn,
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7,
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0,
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE,
  NVIC_Init(&NVIC_InitStructure);

  SPI_InitTypeDef SPI_InitStructure;
  SPI_StructInit(&SPI_InitStructure);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
  SPI_SSOutputCmd(SPI1, ENABLE);

  vSemaphoreCreateBinary(xSPIRxSemphr);
  xValuesMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(xSPIRxSemphr, portMAX_DELAY);

  // Start communication
  SPI_Cmd(SPI1, ENABLE);

  // Create the daemon
  xTaskCreate(accelero_daemon, (const signed char * const)"accelerod",
	      configMINIMAL_STACK_SIZE, NULL, daemon_priority, NULL);
}

void SPI1_IRQHandler()
{
  portBASE_TYPE resch = pdFALSE;
  if (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE))
  {
    response = SPI1->DR;
    xSemaphoreGiveFromISR(xSPIRxSemphr, &resch);
  }
  portEND_SWITCHING_ISR(resch);
}

static uint8_t accelero_command(uint8_t cmd)
{
  SPI_I2S_SendData(SPI1, cmd);
  xSemaphoreTake(xSPIRxSemphr, portMAX_DELAY);
  return response;
}

static void accelero_start()
{
  uint8_t buffer;

  vTaskDelay(1);

  // Enable Chip Select
  GPIO_ResetBits(GPIOA, SPI_NSS_PIN);

  accelero_command(READ | WHO_AM_I);
  buffer = accelero_command(READ);
  assert_param(buffer == IAMX);

  // Disable Chip Select
  GPIO_SetBits(GPIOA, SPI_NSS_PIN);

  vTaskDelay(1);

  // Enable Chip Select
  GPIO_ResetBits(GPIOA, SPI_NSS_PIN);

  accelero_command(MULTIPLE | CTRL_REG1);
  accelero_command(POWER_ON | DECIMATE32 | XEN | YEN | ZEN);
  accelero_command(BDU | FS);

  // Disable Chip Select
  GPIO_SetBits(GPIOA, SPI_NSS_PIN);

  if (DEBUG)
  {
    // Enable Chip Select
    GPIO_ResetBits(GPIOA, SPI_NSS_PIN);

    char buf[32];
    accelero_command(READ | MULTIPLE | CTRL_REG1);
    buffer = accelero_command(CTRL_REG2);
    itoa(buffer, buf);
    uart_puts("ACCEL: CTRL_REG1: ");
    uart_puts(buf);
    uart_puts("\r\n");

    buffer = accelero_command(READ);
    itoa(buffer, buf);
    uart_puts("ACCEL: CTRL_REG2: ");
    uart_puts(buf);
    uart_puts("\r\n");

    // Disable Chip Select
    GPIO_SetBits(GPIOA, SPI_NSS_PIN);
  }
}

int16_t accelero_get_value(direction_t dir)
{
  switch (dir)
  {
    case X:
      return accelero_values[1];
    case Y:
      return -accelero_values[0];
    case Z:
      return -accelero_values[2];
  }
  assert_param(0);
}

void accelero_get_values(accelero_values_t* values)
{
  xSemaphoreTake(xValuesMutex, portMAX_DELAY);
  values->x =  accelero_values[1];
  values->y = -accelero_values[0];
  values->z = -accelero_values[2];
  xSemaphoreGive(xValuesMutex);
}

void accelero_update_values()
{
  // Enable Chip Select
  GPIO_ResetBits(GPIOA, SPI_NSS_PIN);

  accelero_command(READ | MULTIPLE | OUTX_L);

  xSemaphoreTake(xValuesMutex, portMAX_DELAY);
  for (int i = 0; i < 3; i++)
  {
    uint16_t accelero_value;
    accelero_command(READ);
    accelero_value = response;
    accelero_command(READ);
    accelero_value |= response << 8;
    accelero_values[i] = accelero_value;
  }
  xSemaphoreGive(xValuesMutex);

  // Disable Chip Select
  GPIO_SetBits(GPIOA, SPI_NSS_PIN);
}

void accelero_daemon(void* pvParameters)
{
  accelero_start();
  portTickType time = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&time, 5 / portTICK_RATE_MS);
    accelero_update_values();
  }
}
