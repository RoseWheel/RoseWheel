/*
  interpreter.c is part of the RoseWheel project.
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
#include "FreeRTOS.h"
#include "task.h"
#include "interpreter.h"
#include "libglobal/strutils.h"
#include "libperiph/uart.h"

static char prompt[32];
static token_t tokens[32];
static int n_tokens;
static unsigned portBASE_TYPE priority;

static void prvInterpreterDaemon(void* pvParameters);

void vInterpreterInit(const char* pr, token_t* tok, int n,
                      unsigned portBASE_TYPE daemon_priority)
{
  memcpy(prompt, pr, strlen(pr) * sizeof (char));
  n_tokens = n;
  for (int i = 0; i < n; i++)
  {
    tokens[i].command = tok[i].command;
    tokens[i].handler = tok[i].handler;
  }
  priority = daemon_priority;
}

void vInterpreterStart()
{
  xTaskCreate(prvInterpreterDaemon,
              (signed portCHAR*)"Interpreter",
              configMINIMAL_STACK_SIZE, NULL,
              priority, NULL);
}

static void prvInterpreterDaemon(void* pvParameters)
{
  char c;
  char buffer[32];
  char* cmd;
  int abort, size;
  vTaskDelay(1000);
  uart_puts("\r\n");
  for (;;)
  {
    abort = 0;
    size = 0;

    uart_puts(prompt);
    uart_puts(" # ");

    buffer[0] = 0;
    while ((c = uart_getc()))
    {
      if (size == 32)
      {
        abort = 1;
        uart_puts("\r\nerror: command too long...\r\n");
        break;
      }

      if (c == '\r' || c == '\n')
      {
        uart_puts("\r\n");
        buffer[size] = 0;
        break;
      }
      else if (c == 0x03)
      {
        abort = 1;
        uart_puts("\r\n");
        buffer[size] = 0;
        uart_puts(buffer);
        uart_puts(": abort\r\n");
        break;
      }
      else if (c == 0x08)
      {
        if (size > 0)
        {
          size--;
          uart_putc(c);
          uart_putc(' ');
          uart_putc(c);
        }
        continue;
      }
      else if (!is_letter(c) && !is_number(c)
               && !is_space(c) && c != '-'
               && c != ':'     && c != '.')
        continue;

      uart_putc(c);
      buffer[size++] = c;
    }

    if (abort)
      continue;

    cmd = trim_in_place(buffer);

    // Skip empty command
    if (cmd[0] == 0)
      continue;

    abort = 1;
    for (int i = 0; i < n_tokens; i++)
    {
      if (cmd[0] == tokens[i].command)
      {
        (*tokens[i].handler)(cmd + 1);
        abort = 0;
        break;
      }
    }

    if (abort)
    {
      uart_puts("error: undefined command '");
      uart_putc(cmd[0]);
      uart_puts("'\r\n");
    }
  }
}
