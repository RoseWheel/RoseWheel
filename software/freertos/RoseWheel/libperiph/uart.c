/*
  uart.c is part of the RoseWheel project.
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
#include "queue.h"
#include "semphr.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "task.h"

#include "libperiph/uart.h"

#ifdef USE_USART1
# define UART USART1
#elif defined USE_UART4
# define UART UART4
# ifndef STM32F10X_HD
#  error "UART4 only available in HD density"
# endif
#else
# error "No UART chosen"
#endif

static xQueueHandle xUartRxQueue;
static xQueueHandle xUartTxQueue;
static xSemaphoreHandle xUartTxMutex;

void uart_init()
{
  xUartTxMutex = xSemaphoreCreateMutex();
  xUartRxQueue = xQueueCreate(16, sizeof(char));
  xUartTxQueue = xQueueCreate(16, sizeof(char));

  // Enable interrupt UART:
  NVIC_InitTypeDef NVIC_InitStructure =
  {
#ifdef USE_USART1
    .NVIC_IRQChannel = USART1_IRQn,
#elif defined USE_UART4
    .NVIC_IRQChannel = UART4_IRQn,
#endif
    .NVIC_IRQChannelPreemptionPriority = 7,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE,
  };
  NVIC_Init(&NVIC_InitStructure);

#ifdef USE_USART1
  // Enable clock for PC:
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  // Enable clock for UART:
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#elif defined USE_UART4
  // Enable clock for PC:
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  // Enable clock for UART:
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
#endif
  // Enable clock for AFIO:
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);

  // Rx pin:
  GPIO_InitTypeDef GPIO_InitStruct =
    {
#ifdef USE_USART1
      .GPIO_Pin = GPIO_Pin_10,
#elif defined USE_UART4
      .GPIO_Pin = GPIO_Pin_11,
#endif
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode = GPIO_Mode_IN_FLOATING,
    };
#ifdef USE_USART1
  GPIO_Init(GPIOA, &GPIO_InitStruct);
#elif defined USE_UART4
  GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

  USART_InitTypeDef UART_InitStructure;
  USART_StructInit(&UART_InitStructure);
  UART_InitStructure.USART_BaudRate = 115200,
  UART_InitStructure.USART_WordLength = USART_WordLength_8b,
#ifndef USE_UART4
  UART_InitStructure.USART_StopBits = USART_StopBits_1,
  UART_InitStructure.USART_Parity = USART_Parity_No,
  UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
#endif
  UART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
  USART_Init(UART, &UART_InitStructure);
  USART_Cmd(UART, ENABLE);

  UART->CR1 |= USART_CR1_RXNEIE;
}

char uart_getc()
{
  char c;
  xQueueReceive(xUartRxQueue, &c, portMAX_DELAY);
  return c;
}

void uart_gets(char* s, int size)
{
  char c;

  for (int i = 0; i < size; i++)
  {
    if (i == size - 1)
    {
      s[i] = 0;
      break;
    }

    c = uart_getc();

    if (c == '\r')
    {
      s[i] = 0;
      break;
    }

    s[i] = c;
  }
}

void uart_putc(char c)
{
  xQueueSend(xUartTxQueue, &c, portMAX_DELAY);
  UART->CR1 |= USART_CR1_TXEIE;
}

void uart_puts(const char* s)
{
  while (*s)
    uart_putc(*s++);
}

void uart_send(const char* s)
{
  xSemaphoreTake(xUartTxMutex, portMAX_DELAY);
  uart_puts(s);
  uart_putc('\r');
  xSemaphoreGive(xUartTxMutex);
}

#ifdef USE_USART1
void USART1_IRQHandler()
#elif defined USE_UART4
void UART4_IRQHandler()
#endif
{
  static bool initializedTxPin = FALSE;
  portBASE_TYPE reschedNeeded = pdFALSE;
  char c;

  if (UART->SR & USART_SR_RXNE) {
    c = UART->DR;
    xQueueSendFromISR(xUartRxQueue, &c, &reschedNeeded);
  }
  else if (UART->SR & USART_SR_TXE) {
    if (!initializedTxPin) {
      // We use registers instead of stm32 lib because we want to save
      // time inside an interrupt:
#ifdef USE_USART1
      // Tx (PA9) output push-pull 2MHz:
      GPIOA->CRH &= ~GPIO_CRH_MODE9_0;
      GPIOA->CRH |= GPIO_CRH_MODE9_1;
      GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
      GPIOA->CRH |= GPIO_CRH_CNF9_1;
#elif defined USE_UART4
      // Tx (PC10) output push-pull 2MHz:
      GPIOC->CRH &= ~GPIO_CRH_MODE10_0;
      GPIOC->CRH |= GPIO_CRH_MODE10_1;
      GPIOC->CRH &= ~GPIO_CRH_CNF10_0;
      GPIOC->CRH |= GPIO_CRH_CNF10_1;
#endif

      initializedTxPin = TRUE;
    }

    if (xQueueReceiveFromISR(xUartTxQueue, &c, &reschedNeeded))
      UART->DR = c;
    else
      UART->CR1 &= ~USART_CR1_TXEIE;
  }
  portEND_SWITCHING_ISR(reschedNeeded);
}
