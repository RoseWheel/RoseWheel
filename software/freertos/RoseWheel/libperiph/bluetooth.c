/*
  bluetooth.c is part of the RoseWheel project.
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

#include <string.h>

#include "assert_param.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

#include "bluetooth.h"

#define BT_RSTn_Pin GPIO_Pin_1
#define BT_EN_Pin   GPIO_Pin_2
#define BT_ACT_Pin  GPIO_Pin_3

static xQueueHandle xBtRxQueue;
static xQueueHandle xBtTxQueue;
static xSemaphoreHandle xBtTxMutex;

static void bluetooth_reset();
static void send_bluetooth_command(uint8_t cmd, uint8_t len, const uint8_t params[]);
static int bluetooth_config();

static void uart_init()
{
  xBtTxMutex = xSemaphoreCreateMutex();
  xBtRxQueue = xQueueCreate(16, sizeof(char));
  xBtTxQueue = xQueueCreate(16, sizeof(char));

  // Enable interrupt UART:
  NVIC_InitTypeDef NVIC_InitStructure =
  {
    .NVIC_IRQChannel = USART2_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 7,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE,
  };
  NVIC_Init(&NVIC_InitStructure);

  // Enable clock for AFIO, PA and PC
  RCC_APB2PeriphClockCmd((RCC_APB2Periph_AFIO
                          | RCC_APB2Periph_GPIOA
                          | RCC_APB2Periph_GPIOC), ENABLE);
  // Enable clock for USART2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  // Initialize Rx and CTS
  GPIO_InitTypeDef GPIO_RxCTSInitStruct = {
    .GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_3,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
  };
  GPIO_Init(GPIOA, &GPIO_RxCTSInitStruct);

  // Initialize RTS
  GPIO_InitTypeDef GPIO_RTSInitStruct = {
    .GPIO_Pin   = GPIO_Pin_1,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_Mode  = GPIO_Mode_AF_PP,
  };
  GPIO_Init(GPIOA, &GPIO_RTSInitStruct);

  // Initialize UART parameters
  USART_InitTypeDef USART_InitStruct = {
    .USART_BaudRate            = 38400,
    .USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS,
    .USART_Mode                = USART_Mode_Rx | USART_Mode_Tx,
    .USART_Parity              = USART_Parity_No,
    .USART_StopBits            = USART_StopBits_1,
    .USART_WordLength          = USART_WordLength_8b,
  };
  USART_Init(USART2, &USART_InitStruct);

  // Configure RXNE interrupt
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  // Activate UART
  USART_Cmd(USART2, ENABLE);
}

void bluetooth_init()
{
  uart_init();

  // Initialize RSTn
  GPIO_InitTypeDef GPIO_RSTnInitStruct = {
    .GPIO_Pin   = BT_RSTn_Pin,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_Mode  = GPIO_Mode_Out_OD,
  };
  GPIO_SetBits(GPIOC, GPIO_Pin_1); // Reset is active low, so set it
  GPIO_Init(GPIOC, &GPIO_RSTnInitStruct);

  // Initialize EN
  GPIO_InitTypeDef GPIO_ENInitStruct = {
    .GPIO_Pin   = BT_EN_Pin,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_Mode  = GPIO_Mode_Out_PP,
  };
  GPIO_SetBits(GPIOC, BT_EN_Pin);
  GPIO_Init(GPIOC, &GPIO_ENInitStruct);

  // Initialize ACT
  GPIO_InitTypeDef GPIO_ACTInitStruct = {
    .GPIO_Pin   = BT_ACT_Pin,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
  };
  GPIO_Init(GPIOC, &GPIO_ACTInitStruct);

  bluetooth_reset();
  vTaskDelay(800 / portTICK_RATE_MS);

  bluetooth_config();

  GPIO_ResetBits(GPIOC, BT_EN_Pin);
}

char bluetooth_getc()
{
  char c;
  xQueueReceive(xBtRxQueue, &c, portMAX_DELAY);
  return c;
}

void bluetooth_putc(char c)
{
  xQueueSend(xBtTxQueue, &c, portMAX_DELAY);
  // Enable TXE interrupt
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

void bluetooth_gets(char *s, char eol)
{
  while ((*s = bluetooth_getc()) != eol)
    s++;

  *s = '\0';
}

void bluetooth_puts(const char *s)
{
  xSemaphoreTake(xBtTxMutex, portMAX_DELAY);

  while (*s)
    bluetooth_putc(*s++);

  xSemaphoreGive(xBtTxMutex);
}

void USART2_IRQHandler()
{
  static bool initializedTxPin = FALSE;
  portBASE_TYPE reschedNeeded = pdFALSE;
  char data;

  if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) {
    data = USART_ReceiveData(USART2);
    xQueueSendFromISR(xBtRxQueue, &data, &reschedNeeded);
  } else if (USART_GetFlagStatus(USART2, USART_FLAG_TXE)) {
    if (!initializedTxPin) {
      // Initialize TX
      GPIO_InitTypeDef GPIO_TXInitStruct = {
        .GPIO_Pin   = GPIO_Pin_2,
        .GPIO_Speed = GPIO_Speed_2MHz,
        .GPIO_Mode  = GPIO_Mode_AF_PP,
      };
      GPIO_Init(GPIOA, &GPIO_TXInitStruct);
      initializedTxPin = TRUE;
    }

    if (xQueueReceiveFromISR(xBtTxQueue, &data, &reschedNeeded))
      USART_SendData(USART2, data);
    else
      // Disable TXE interrupt
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  }

  portEND_SWITCHING_ISR(reschedNeeded);
}

static void bluetooth_reset()
{
  GPIO_ResetBits(GPIOC, BT_RSTn_Pin);
  vTaskDelay(100 / portTICK_RATE_MS);

  GPIO_SetBits(GPIOC, BT_RSTn_Pin);
  vTaskDelay(100 / portTICK_RATE_MS);
}

static void bluetooth_flush()
{
  char c;

  while (xQueueReceive(xBtRxQueue, &c, 0));
}

static void send_bluetooth_command(uint8_t cmd, uint8_t len, const uint8_t params[])
{
  bluetooth_flush();
  bluetooth_putc(cmd);
  bluetooth_putc(len);

  for (int i = 0; i < len; i++)
    bluetooth_putc(params[i]);
}

static void receive_bluetooth_answer(uint8_t cmd, uint8_t *len, uint8_t params[])
{
  char maxlen = *len;

  while (cmd != bluetooth_getc());
  *len = bluetooth_getc();

  for (int i = 0; i < *len; i++) {
    char c = bluetooth_getc();

    if (i < maxlen)
      params[i] = c;
  }

  if (*len > maxlen)
    *len = maxlen;
}

static int enter_hcm() {
  uint8_t params[4];
  params[0] = 0xff;
  params[1] = 0x00;
  params[2] = 0x55;
  params[3] = 0xaa;
  send_bluetooth_command(0x01, 4, params);
  uint8_t len = 1;
  receive_bluetooth_answer(0x01, &len, params);
  return params[0];
}

static int leave_hcm() {
  send_bluetooth_command(0x50, 0, NULL);
  uint8_t answer = 0, len = 1;
  receive_bluetooth_answer(0x50, &len, &answer);
  return answer;
}

static int bluetooth_config() {
  char name[] = "Rosewheel";
  uint8_t len = strlen(name) + 1;
  uint8_t answer;

  if (len > 32)
    len = 32;

  if (enter_hcm() != 1)
    return 1;

  send_bluetooth_command(0x1d, strlen(name) + 1, (const uint8_t *) name);
  receive_bluetooth_answer(0x1d, &len, &answer);

  uint8_t params[4];
  params[0] = 2;
  params[1] = 0;
  send_bluetooth_command(0x63, 2, params);

  len = 1;
  receive_bluetooth_answer(0x63, &len, &answer);

  params[0] = '1';
  params[1] = '9';
  params[2] = '5';
  params[3] = '1';
  send_bluetooth_command(0x65, 4, params);

  len = 1;
  receive_bluetooth_answer(0x65, &len, &answer);

  if (leave_hcm() != 1)
    return 1;

  return answer != 1;
}

void get_bluetooth_name(char name[], uint8_t len) {
  if (enter_hcm() != 1) {
    name[0] = 0;
    return;
  }

  send_bluetooth_command(0x1c, 0, NULL);
  receive_bluetooth_answer(0x1c, &len, (uint8_t *) name);

  if (leave_hcm() != 1)
    name[0] = 0;
}

