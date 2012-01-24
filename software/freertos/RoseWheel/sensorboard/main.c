/*
  main.c is part of the RoseWheel project.
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

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "libfilter/light_kalman.h"

#include "libglobal/interpreter.h"
#include "libglobal/uid.h"

#include "libperiph/accelero.h"
#include "libperiph/gyroscope.h"
#include "libperiph/hardware.h"
#include "libperiph/uart.h"
#include "libperiph/bluetooth.h"

#include "strutils.h"
#include "mathutils.h"

#include "can.h"
#include "leds.h"

#define DBGLOG(DBG_ENABLE, DBG_MESSAGE)         \
  if ((DBG_ENABLE)) uart_puts((DBG_MESSAGE))

#define BT_MSG_BUFFER_SIZE 32

extern const char* version;

static double dAlpha = 0.6;
static float Vleft = 0, Vright = 0;
static xSemaphoreHandle xVMutex;
static int stopped = 0;

static bool bBtInited = FALSE;

static bool bDebugKalman = DISABLE;
static bool bDebugComm   = DISABLE;

static void prvCanVitalRxTask(void* pvParameters);
static void prvCanRxTask(void* pvParameters);
static void prvComputeTask(void* pvParameters);
static void prvBtTask(void* pvParameters);

static void prvBtSendMsg(char cHeader, int xParameters[], int iNParameters);

void process_control_cmd(char* cmd);
void process_debug_cmd(char* cmd);

int main(void)
{
  // Check we are on the sensorboard:
  if (UID != SENSORBOARD_UID)
    return EXIT_FAILURE;

  hardware_init();
  gyroscope_init(tskIDLE_PRIORITY + 4);
  accelero_init(tskIDLE_PRIORITY + 4);
  vLEDsInit();
  uart_init();
  vCANInit();

  token_t tokens[2];
  tokens[0].command = 'c';
  tokens[0].handler = &process_control_cmd;
  tokens[1].command = 'd';
  tokens[1].handler = &process_debug_cmd;
  vInterpreterInit("sensorboard", tokens, 2, tskIDLE_PRIORITY + 4);

  xVMutex = xSemaphoreCreateMutex();

  xTaskCreate(prvComputeTask,
              (signed portCHAR*)"Compute Task",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(prvCanVitalRxTask,
              (signed portCHAR*)"Can Vital Rx Task",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(prvCanRxTask,
              (signed portCHAR*)"Can Rx Task",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(prvBtTask,
              (signed portCHAR*)"Bluetooth Task",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);

  vInterpreterStart();

  vTaskStartScheduler();

  return EXIT_SUCCESS;
}

static void prvCanVitalRxTask(void* pvParameters)
{
  xCANMsg msg;
  int iBtMsgCountdown = 10;

  for (;;)
    {
      vLEDToggle(LED_RED);

      vCANReceiveVitalMsg(&msg);
      switch (msg.eID)
        {
        default:
          break;
        case MSG_STOP:
          DBGLOG(bDebugComm, "STOP received\r\n");
          stopped = 1;
          break;
        case MSG_RESET:
          DBGLOG(bDebugComm, "RESET received\r\n");
          vTaskDelay(1);
          NVIC_SystemReset();
          break;
        case MSG_ANGLE_ANGVEL:
          break;
        case MSG_ENCINFO:
          break;
        case MSG_MOTORCMD:
          xSemaphoreTake(xVMutex, portMAX_DELAY);
          Vleft = msg.xData.values.first;
          Vright = msg.xData.values.second;
          xSemaphoreGive(xVMutex);

          if (bBtInited && !(iBtMsgCountdown--)) {
            int xParameters[] = {-msg.xData.values.first * 100 / 480,
                                 -msg.xData.values.second * 100 / 480};

            prvBtSendMsg('m', xParameters, 2);

            iBtMsgCountdown = 10;
          }
          break;
        case MSG_DANGER:
          break;
        }
    }
}

static void prvCanRxTask(void* pvParameters)
{
  xCANMsg msg;

  for (;;)
    {
      vLEDToggle(LED_GREEN);

      vCANReceiveNonVitalMsg(&msg);
      switch (msg.eID)
        {
        default:
          break;
        case MSG_REMOTECMD:
          break;
        case MSG_USERCMD:
          break;
        case MSG_SPEED:
          break;
        case MSG_BATTERYLVL:
          if (bBtInited) {
            int xParameters[] = {msg.xData.words.low};

            prvBtSendMsg('b', xParameters, 1);
          }
          break;
        case MSG_CTRLSTATE:
          if (bBtInited) {
            int xParameters[] = {msg.xData.words.low, msg.xData.words.high};
          
            prvBtSendMsg('S', xParameters, 2);
          }
        }
    }
}

static void prvComputeTask(void* pvParameters)
{
  xCANMsg msg;
  accelero_values_t accelero_values;
  double gyroscope_y = 0;
  double accelero_x = 0;
  double accelero_z = 0;
  portTickType time = xTaskGetTickCount();
  int i = 0;
  static char buf[80];
  static double previous_angle = 0;
  double d = 0;
  int iBtMsgCountdown = 10;

  uart_puts("\r\nSensorboard started...\r\n");
  uart_puts("Processor UID: ");
  uart_puts(itoa(UID, buf));
  uart_puts("\r\n");
  uart_puts("Version: ");
  uart_puts(version);
  uart_puts("\r\n");

  for (;;)
    {
      if (i++ == 100)
        {
          vLEDToggle(LED_YELLOW);
          i = 0;
        }

      if (stopped)
        vTaskDelay(portMAX_DELAY);

      vTaskDelayUntil(&time, 5 / portTICK_RATE_MS);

      accelero_get_values(&accelero_values);

      gyroscope_y = (double)gyroscope_get_value(Y) * PI / 180 / GYROSCOPE_SENSITIVITY;
      accelero_x  = (double)accelero_values.x  / ACCELERO_SENSITIVITY_TYP_6G;
      accelero_z  = (double)accelero_values.z  / ACCELERO_SENSITIVITY_TYP_6G;

      DBGLOG(bDebugKalman, itoa(atan2(accelero_x, -accelero_z) * 1000, buf));
      DBGLOG(bDebugKalman, " ");
      DBGLOG(bDebugKalman, itoa(gyroscope_y * 1000, buf));
      DBGLOG(bDebugKalman, " ");

      xSemaphoreTake(xVMutex, portMAX_DELAY);
      kalman_state_update(gyroscope_y, 0.005);
      xSemaphoreGive(xVMutex);

      kalman_cov_update(accelero_x, accelero_z);

      DBGLOG(bDebugKalman, itoa(kalman_get_angle() * 1000, buf));
      DBGLOG(bDebugKalman, " ");
      DBGLOG(bDebugKalman, itoa(kalman_get_rate() * 1000, buf));
      DBGLOG(bDebugKalman, " ");
      d = dAlpha * d + (1 - dAlpha) * kalman_get_rate();
      DBGLOG(bDebugKalman, itoa(d * 1000, buf));
      DBGLOG(bDebugKalman, "\r\n");
      previous_angle = kalman_get_angle();

      msg.eID = MSG_ANGLE_ANGVEL;
      msg.xData.values.first = kalman_get_angle();
      msg.xData.values.second = d;

      vCANSendMsg(&msg);

      if (bBtInited && !(iBtMsgCountdown--)) {
        int xParameters[1] = {-RAD2DEG(kalman_get_angle())};

        prvBtSendMsg('a', xParameters, 1);

        iBtMsgCountdown = 10;
      }
    }
}

static void prvBtSendMsg(char cHeader, int xParameters[], int iNParameters)
{
  char xBtMsgBuffer[BT_MSG_BUFFER_SIZE] = "#";
  int i = 1;

  for (int j = 0; j < iNParameters; j++) {
    xBtMsgBuffer[i++] = ' ';

    itoa(xParameters[j], &xBtMsgBuffer[i++]);

    for (; xBtMsgBuffer[i]; i++);
  }

  xBtMsgBuffer[1] = cHeader;

  for (; xBtMsgBuffer[i]; i++);

  xBtMsgBuffer[i++] = '\r';
  xBtMsgBuffer[i++] = '\n';
  xBtMsgBuffer[i] = '\0';

  bluetooth_puts(xBtMsgBuffer);
  
}

static void prvBtTask(void *pvParameters)
{
  xCANMsg xMsgBuffer = {
    .eID = MSG_REMOTECMD,
  };

  bluetooth_init();

  bBtInited = TRUE;

  for (;;) {
    char xBtMsgBuffer[BT_MSG_BUFFER_SIZE];
    int iValue;
    int i;

    // Wait for the next message
    while (bluetooth_getc() != BT_MSG_START);

    // Read data
    for (i = 0; i < BT_MSG_BUFFER_SIZE; i++) {
      xBtMsgBuffer[i] = bluetooth_getc();
      // Stop if we encounter BG_MSG_END
      if (xBtMsgBuffer[i] == BT_MSG_END)
        break;
      // Start over if a BT_MSG_START is received
      else if (xBtMsgBuffer[i] == BT_MSG_START)
        i = 0;
    }

    // Buffer overflow: discard the current message
    if (i == BT_MSG_BUFFER_SIZE)
      continue;

    // Remove BT_MSG_END from the end of the message
    xBtMsgBuffer[i] = '\0';

    uart_puts(xBtMsgBuffer);

    // p - ping
    if (xBtMsgBuffer[0] == 'p') {
      bluetooth_puts("pong\r\n");
      continue;
    }

    // Fill the CAN message header
    xMsgBuffer.xData.octets[0] = xBtMsgBuffer[0];

    // Parse the message
    switch (xBtMsgBuffer[0]) {
      // c - control
    case 'c':
      // Find the beginning of the steering value
      while (xBtMsgBuffer[i] != ' ')
        i--;

      // Convert the steering value and fill in the CAN message field
      iValue = atoi(&xBtMsgBuffer[i + 1]);
      xMsgBuffer.xData.octets[2] = (int8_t)iValue;

      // Trim the steering value out
      xBtMsgBuffer[i] = '\0';

      // Convert the equilibrium angle and fill in the CAN message field
      iValue = atoi(&xBtMsgBuffer[1]);
      xMsgBuffer.xData.octets[1] = (int8_t)iValue;

      break;
    case 's':
      // s - stop remote control

      // Nothing to do - the CAN message contains only the header

      break;
    case 'k':
      // k - klaxon

      // Convert the argument and fill in the CAN message field
      xMsgBuffer.xData.octets[1] = atoi(&xBtMsgBuffer[1]);

      break;
    case 'w':
      // w - wake up

      // Nothing to do - the CAN message contains only the header

      break;

    case 'S':
      // S - sleep

      // Nothing to do - the CAN message contains only the header
      break;

    case 'd':
      // d - danger detection

      // Convert the argument and fill in the CAN message field
      xMsgBuffer.xData.octets[1] = atoi(&xBtMsgBuffer[1]);

    default:
      break;
    }

    // Send the CAN message
    vCANSendMsg(&xMsgBuffer);

  }
}

void process_control_cmd(char* str)
{
  char buf[32];
  float value;
  char cmd = str[0];
  char* args = trim_in_place(str + 1);

  if (cmd == 'a')
    {
      value = atofl(args);
      dAlpha = value;
      uart_puts("changed alpha: ");
      uart_puts(fltoa(value, buf));
      uart_puts("\r\n");
    }
  else
    {
      uart_puts("error: undefined subcommand '");
      uart_putc(cmd);
      uart_puts("' for control command\r\n");
      uart_puts("\r\n");
    }
}

static inline void toggle_debug_param(bool* bFlag, char* pDescription) {
  *bFlag = *bFlag ? DISABLE : ENABLE;
  uart_puts(pDescription);
  uart_puts(" is now ");
  if (*bFlag)
    uart_puts("ENABLED\r\n");
  else
    uart_puts("DISABLED\r\n");
}

void process_debug_cmd(char* cmd)
{
  switch (cmd[0]) {
  case 'c':
    toggle_debug_param(&bDebugComm, "communication debug log");
    break;
  case 'k':
    toggle_debug_param(&bDebugKalman, "Kalman filter debug log");
    break;
  default:
    uart_puts("error: undefined subcommand '");
    uart_putc(cmd[0]);
    uart_puts("' for control command\r\n");
  }
}

