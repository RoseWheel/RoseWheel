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

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "libcontrol/pid.h"

#include "libglobal/interpreter.h"
#include "libglobal/strutils.h"
#include "libglobal/mathutils.h"
#include "libglobal/uid.h"

#include "libperiph/hardware.h"
#include "libperiph/safety-switch.h"
#include "libperiph/uart.h"

#include "leds.h"
#include "can.h"
#include "control.h"
#include "motors.h"
#include "steering.h"
#include "sonar.h"
#include "battery.h"

#define SECURITY_ANGLE DEG2RAD(90)

#ifdef NDEBUG
# define DEBUG 0
#else
# define DEBUG 1
#endif

#define DBGLOG(DBG_ENABLE, DBG_MESSAGE)         \
  if ((DBG_ENABLE)) uart_puts((DBG_MESSAGE))

extern const char* version;

static int  iSteeringValue = 0;
static bool bRemoteActive = TRUE;

static enum eControlState eControlState = REMOTE;

static enum eControlMode eControlMode = SELF_DRIVEN;
static bool bDebugSteering  = DISABLE;
static bool bDebugMotors    = DISABLE;
static bool bDebugComm      = DISABLE;
static bool bSteeringEnable = DISABLE;
static bool bControlEnable  = ENABLE;
static bool bMotorsEnable   = ENABLE;
static bool bRemoteEnable   = ENABLE;

static xSemaphoreHandle xStateMutex;
static double xCurrentStateData[4] = {0};
static vector_t xCurrentState;

static xSemaphoreHandle xSetPIDValuesSemphr;

static void prvFlashLEDTask(void* pvParameters);
static void prvVitalMsgTask(void* pvParameters);
static void prvNonVitalMsgTask(void* pvParameters);
static void prvControlTask(void* pvParameters);
static void prvSetFinalPIDValuesTask(void* pvParameters);
static void prvSendCtrlStateTask(void* pvParameters);
static void prvPress();
static void prvRelease();
static void prvLongRelease();
void process_motor_cmd(char* cmd);
void process_control_cmd(char* cmd);
void process_debug_cmd(char* cmd);

int main(void)
{
  // Check we are on the mainboard:
  if (UID != MAINBOARD_UID)
    return EXIT_FAILURE;

  hardware_init();
  vLEDsInit();
  vCANInit();
  uart_init();
  vSteeringInit();
  vMotorsInit(tskIDLE_PRIORITY + 3, tskIDLE_PRIORITY + 1);
  vBatteryInit(tskIDLE_PRIORITY + 1);
  vControlInit(tskIDLE_PRIORITY + 3);
  vSonarInit(tskIDLE_PRIORITY + 2);

  safety_switch_set_handler(LONG_RELEASE, prvLongRelease);
  safety_switch_set_handler(RELEASE, prvRelease);
  safety_switch_set_handler(PRESS, prvPress);
  safety_switch_set_long_release_time(1000);
  safety_switch_init(tskIDLE_PRIORITY + 5);

  token_t tokens[3];
  tokens[0].command = 'm';
  tokens[0].handler = &process_motor_cmd;
  tokens[1].command = 'c';
  tokens[1].handler = &process_control_cmd;
  tokens[2].command = 'd';
  tokens[2].handler = &process_debug_cmd;
  vInterpreterInit("mainboard", &tokens[0], 3, tskIDLE_PRIORITY + 4);

  xStateMutex = xSemaphoreCreateMutex();
  vSemaphoreCreateBinary(xSetPIDValuesSemphr);

  vector_init(&xCurrentState, xCurrentStateData, 4);

  xTaskCreate(prvFlashLEDTask,
              (signed portCHAR*)"Flash LED",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 1, NULL);

  xTaskCreate(prvVitalMsgTask,
              (signed portCHAR*)"Vital messages",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);

  xTaskCreate(prvNonVitalMsgTask,
              (signed portCHAR*)"Non-vital messages",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 1, NULL);

  xTaskCreate(prvControlTask,
              (signed portCHAR*)"Control",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 3, NULL);

  xTaskCreate(prvSetFinalPIDValuesTask,
              (signed portCHAR*)"Set Final PID values",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 4, NULL);

  xTaskCreate(prvSendCtrlStateTask,
              (signed portCHAR*)"Send control state",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 1, NULL);

  vInterpreterStart();

  if (bMotorsEnable)
    vMotorsEnable();

  xSemaphoreGive(xSetPIDValuesSemphr);

  vTaskStartScheduler();

  return EXIT_SUCCESS;
}

static void prvFlashLEDTask(void* pvParameters)
{
  char buf[32];
  uart_puts("\r\nMainboard started...\r\n");
  uart_puts("Processor UID: ");
  uart_puts(itoa(UID, buf));
  uart_puts("\r\n");
  uart_puts("Version: ");
  uart_puts(version);
  uart_puts("\r\n");
  for (;;) {
    vLEDToggle(COLOR_LED_GREEN);
    vTaskDelay(50 / portTICK_RATE_MS);
  }
}

static void prvVitalMsgTask(void* pvParameters)
{
  short sMsgCount = 0;
  char xCmdStr[8];

  for (;;) {
    xCANMsg xMsgBuffer;

    vCANReceiveVitalMsg(&xMsgBuffer);

    if (!(sMsgCount++ % 100))
      vLEDToggle(COLOR_LED_BLUE);

    switch (xMsgBuffer.eID) {
    case MSG_STOP:
      DBGLOG(bDebugComm, "RECEIVED STOP\r\n");
      break;
    case MSG_RESET:
      DBGLOG(bDebugComm, "Received RESET\r\n");
      DBGLOG(bDebugComm, "Doing nothing\r\n");
      break;
    case MSG_ANGLE_ANGVEL:
      if (bDebugComm) {
        DBGLOG(ENABLE, "Received ANGLE_ANGVEL\r\n");
        DBGLOG(ENABLE, "Angle = ");
        fltoa(xMsgBuffer.xData.values.first * 180 / PI, xCmdStr);
        DBGLOG(ENABLE, xCmdStr);
        DBGLOG(ENABLE, ", angular velocity = ");
        fltoa(xMsgBuffer.xData.values.second * 180 / PI, xCmdStr);
        DBGLOG(ENABLE, xCmdStr);
        DBGLOG(ENABLE, "\r\n");
      }

      xSemaphoreTake(xStateMutex, portMAX_DELAY);
      xCurrentStateData[PHI]     = xMsgBuffer.xData.values.first;
      xCurrentStateData[PHI_DOT] = xMsgBuffer.xData.values.second;
      xSemaphoreGive(xStateMutex);

      break;
    case MSG_DANGER:
      DBGLOG(bDebugComm, "Received DANGER\r\n");
      break;
    default:
      break;
    }
  }
}

static void prvNonVitalMsgTask(void* pvParameters)
{
  static char buf[32];

  for (;;) {
    xCANMsg xMsgBuffer;

    vCANReceiveNonVitalMsg(&xMsgBuffer);

    if ((xMsgBuffer.eID == MSG_REMOTECMD)
        && (xMsgBuffer.xData.octets[0] == 'd')) {
      if (xMsgBuffer.xData.octets[1])
        vSonarBeepOn();
      else
        vSonarBeepOff();
    } else {
      switch (eControlState) {
        case HUMAN:
          break;
        case REMOTE:

          switch (xMsgBuffer.eID) {
            case MSG_REMOTECMD:
              DBGLOG(bDebugComm, "Received REMOTECMD\r\n");

              if (bRemoteEnable) {
                switch (xMsgBuffer.xData.octets[0]) {
                  case 'c':
                    bRemoteActive = TRUE;
                    bSteeringEnable = TRUE;
                    vControlSetEqAngle(DEG2RAD(-(int8_t)xMsgBuffer.xData.octets[1]
                                               * 10.0 / 100)
                                       + EQ_ANGLE[eControlMode]);
                    DBGLOG(bDebugComm, itoa((int8_t)xMsgBuffer.xData.octets[1],
                                            buf));
                    DBGLOG(bDebugComm, " ");
                    iSteeringValue = -((int8_t)xMsgBuffer.xData.octets[2] * 35 / 100);
                    DBGLOG(bDebugComm, itoa((int8_t)xMsgBuffer.xData.octets[2],
                                            buf));
                    DBGLOG(bDebugComm, " ");
                    DBGLOG(bDebugComm, itoa(iSteeringValue,
                                            buf));
                    DBGLOG(bDebugComm, "\r\n");
                    break;
                  case 'k':
                    if (xMsgBuffer.xData.octets[1])
                      vBeepOn();
                    else {
                      vBeepOff();
                      vControlSetEqAngleOffset(0);
                    }
                    break;
                  case 's':
                    iSteeringValue = 0;
                    switch(eControlMode) {
                      case HUMAN_DRIVEN:
                        vControlSetEqAngle(HUMAN_DRIVEN_EQ_ANGLE);
                        bSteeringEnable = TRUE;
                        break;
                      case SELF_DRIVEN:
                        vControlSetEqAngle(SELF_DRIVEN_EQ_ANGLE);
                        bSteeringEnable = FALSE;
                        break;
                    }
                    bRemoteActive = FALSE;
                    break;
                  case 'w':
                    if (!bMotorsEnable) {
                      vMotorsEnable();
                      uart_puts("start motors");
                      uart_puts("\r\n");
                      bMotorsEnable = ENABLE;
                    }

                    bControlEnable = ENABLE;
                    uart_puts("control algorithm is now ENABLED.\r\n");

                    pid_set_kp(-30);
                    pid_set_kd(0);
                    pid_set_ki(0);

                    xSemaphoreGive(xSetPIDValuesSemphr);

                    break;
                  case 'S':
                    vMotorsDisable();
                    bMotorsEnable = DISABLE;
                    uart_puts("stop motors");
                    uart_puts("\r\n");
                    bControlEnable = DISABLE;
                    bRemoteActive = FALSE;
                    eControlMode = SLEEPING;
                    break;
                }
              }
              break;
            case MSG_USERCMD:
              DBGLOG(bDebugComm, "Received USERCMD\r\n");
              break;
            default:
              break;
          }
        case SLEEPING:
          if (xMsgBuffer.xData.octets[0] == 'w') {
            if (!bMotorsEnable) {
              vMotorsEnable();
              uart_puts("start motors");
              uart_puts("\r\n");
              bMotorsEnable = ENABLE;
            }

            bControlEnable = ENABLE;
            uart_puts("control algorithm is now ENABLED.\r\n");

            pid_set_kp(-30);
            pid_set_kd(0);
            pid_set_ki(0);

            xSemaphoreGive(xSetPIDValuesSemphr);

            eControlMode = REMOTE;
          }
          break;
      }
    }
  }
}

static void prvControlTask(void* pvParameters)
{
  static const portTickType xPeriod = 5;
  portTickType xLastWakeTime;
  static xCANMsg xMsgBuffer = {
    .eID = MSG_MOTORCMD,
  };

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vector_t* pxCommand;
    char xCmdStr[16];

    int iCommandLeft  = 0;
    int iCommandRight = 0;

    vTaskDelayUntil(&xLastWakeTime, xPeriod / portTICK_RATE_MS);
    xSemaphoreTake(xStateMutex, portMAX_DELAY);
    if ((ABS(xCurrentState.data[PHI]) > SECURITY_ANGLE)
        && (bControlEnable
            || bMotorsEnable)) {
      vMotorsDisable();
      bMotorsEnable = DISABLE;
      uart_puts("stop motors");
      uart_puts("\r\n");
      bControlEnable = DISABLE;
    } else {
      vControlUpdate(&xCurrentState, (double)xPeriod / 1000);
    }
    xSemaphoreGive(xStateMutex);

    pxCommand = xControlGetCommand();

    if (!bRemoteActive)
      iSteeringValue = iSteeringGetValue();

    if (bControlEnable) {
      iCommandLeft = pxCommand->data[LEFT] * COMMAND_VOLT_RATE;
      iCommandRight = pxCommand->data[RIGHT] * COMMAND_VOLT_RATE;
      if (bSteeringEnable)
      {
        iCommandLeft  += iSteeringValue *
          fMotorsGetSteeringFactor(RAD2DEG(xCurrentState.data[PHI]
                                           - EQ_ANGLE[eControlMode])) / 100;
        iCommandRight -= iSteeringValue *
          fMotorsGetSteeringFactor(RAD2DEG(xCurrentState.data[PHI]
                                           - EQ_ANGLE[eControlMode])) / 100;
      }
      vMotorsCommand(iCommandLeft, iCommandRight);
    }

    DBGLOG(bDebugSteering, "STEERING: ");
    DBGLOG(bDebugSteering, itoa(iSteeringValue, xCmdStr));
    DBGLOG(bDebugSteering, "\r\n");

    xMsgBuffer.xData.values.first  = iCommandLeft;
    xMsgBuffer.xData.values.second = iCommandRight;

    if (bDebugMotors) {
      DBGLOG(ENABLE, "LEFT_CMD: ");
      itoa(iCommandLeft, xCmdStr);
      DBGLOG(ENABLE, xCmdStr);

      DBGLOG(ENABLE, "RIGHT_CMD: ");
      itoa(iCommandRight, xCmdStr);
      DBGLOG(ENABLE, xCmdStr);
      DBGLOG(ENABLE, "\r\n");
    }

    vCANSendMsg(&xMsgBuffer);
  }
}

void process_motor_cmd(char* str)
{
  int value;
  char buffer[32];
  char cmd = str[0];
  char* args = trim_in_place(str + 1);

  if (cmd == 's') // start/stop
  {
    if (bMotorsEnable)
    {
      vMotorsDisable();
      uart_puts("stop motors");
      uart_puts("\r\n");
      bMotorsEnable = DISABLE;
    }
    else
    {
      vMotorsEnable();
      uart_puts("start motors");
      uart_puts("\r\n");
      bMotorsEnable = ENABLE;
    }
  }
  else if (cmd == 'l')
  {
    value = atoi(args);
    vMotorsLeftCommand(value);
    uart_puts("setting LEFT motor voltage: ");
    itoa(value, buffer);
    uart_puts(buffer);
    uart_puts("\r\n");
  }
  else if (cmd == 'r')
  {
    value = atoi(args);
    vMotorsRightCommand(value);
    uart_puts("setting RIGHT motor voltage: ");
    itoa(value, buffer);
    uart_puts(buffer);
    uart_puts("\r\n");
  }
  else if (cmd == 'b')
  {
    int sep = 1;
    while (args[sep++] != ':');
    value = atoi_eol(args, ':');
    vMotorsLeftCommand(value);
    uart_puts("setting LEFT motor voltage: ");
    itoa(value, buffer);
    uart_puts(buffer);
    uart_puts("\r\n");

    value = atoi(args + sep);
    vMotorsRightCommand(value);
    uart_puts("setting RIGHT motor voltage: ");
    itoa(value, buffer);
    uart_puts(buffer);
    uart_puts("\r\n");
  }
  else
  {
    uart_puts("error: undefined subcommand '");
    uart_putc(cmd);
    uart_puts("' for motor command\r\n");
  }
}

void process_control_cmd(char* str)
{
  char buf[32];
  float value;
  char cmd = str[0];
  char* args = trim_in_place(str + 1);

  if (cmd == 'p')
  {
    value = atofl(args);
    pid_set_kp(value);
    uart_puts("changed pid factor kp: ");
    uart_puts(fltoa(value, buf));
    uart_puts("\r\n");
  }
  else if (cmd == 'i')
  {
    value = atofl(args);
    pid_set_ki(value);
    uart_puts("changed pid factor ki: ");
    uart_puts(fltoa(value, buf));
    uart_puts("\r\n");
  }
  else if (cmd == 'd')
  {
    value = atofl(args);
    pid_set_kd(value);
    uart_puts("changed pid factor kd: ");
    uart_puts(fltoa(value, buf));
    uart_puts("\r\n");
  }
  else if (cmd == 'e')
  {
    value = atofl(args);
    vControlSetEqAngle(DEG2RAD(value));
    uart_puts("changed equilibrium angle: ");
    uart_puts(fltoa(value, buf));
    uart_puts(" deg\r\n");
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
  char buf[64];
  switch (cmd[0]) {
  case 'm':
    toggle_debug_param(&bDebugMotors, "motor debug log");
    break;
  case 'c':
    toggle_debug_param(&bDebugComm, "communication debug log");
    break;
  case 'a':
    toggle_debug_param(&bControlEnable, "control algorithm");
    if (!bControlEnable) {
      vMotorsDisable();
      bMotorsEnable = DISABLE;
    }
    break;
  case 'd':
    toggle_debug_param(&bDebugSteering, "steering debug log");
    break;
  case 's':
    toggle_debug_param(&bSteeringEnable, "steering");
    break;
  case 'b':
    uart_puts("battery level: ");
    uart_puts(itoa(iBatteryGetLevel(), buf));
    uart_puts("V\r\n");
    break;
  case 'r':
    toggle_debug_param(&bRemoteEnable, "remote control");
    if (!bRemoteEnable) {
      iSteeringValue = 0;
      switch(eControlMode) {
      case HUMAN_DRIVEN:
        vControlSetEqAngle(HUMAN_DRIVEN_EQ_ANGLE);
        bSteeringEnable = TRUE;
        break;
      case SELF_DRIVEN:
        vControlSetEqAngle(SELF_DRIVEN_EQ_ANGLE);
        bSteeringEnable = FALSE;
        break;
      }
      bRemoteActive = FALSE;
    }
    break;
  default:
    uart_puts("error: undefined subcommand '");
    uart_putc(cmd[0]);
    uart_puts("' for control command\r\n");
  }
}

static void prvPress()
{
  vSetBeepCont(100);

  switch (eControlState) {
  case REMOTE:
    vControlSetEqAngle(0);
    bSteeringEnable = TRUE;
    eControlMode = HUMAN_DRIVEN;
    bRemoteEnable = DISABLE;
    bRemoteActive = FALSE;
    eControlState = HUMAN;
    break;
  case HUMAN:
  case SLEEPING:
    break;
  }
}

static void prvRelease()
{
  vSetBeepCont(150);
}

static void prvLongRelease()
{
  vSetBeepCont(500);

  switch (eControlState) {
  case HUMAN:
    vControlSetEqAngle(SELF_DRIVEN_EQ_ANGLE);
    bSteeringEnable = FALSE;
    eControlMode = SELF_DRIVEN;
    bRemoteEnable = ENABLE;
    bRemoteActive = TRUE;
    eControlState = REMOTE;
    break;
  case REMOTE:
  case SLEEPING:
    break;
  }
}

static void prvSetFinalPIDValuesTask(void* pvParameters)
{
  for (;;) {
    xSemaphoreTake(xSetPIDValuesSemphr, portMAX_DELAY);
    vTaskDelay(5000 / portTICK_RATE_MS);
    pid_set_kp(-100);
    pid_set_ki(0);
    pid_set_kd(0);
  }
}

static void prvSendCtrlStateTask(void* pvParameters)
{
  static const portTickType xPeriod = 500;
  portTickType xLastWakeTime;
  static xCANMsg xMsgBuffer = {
    .eID = MSG_CTRLSTATE,
  };

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    xMsgBuffer.xData.words.low = eControlState;
    xMsgBuffer.xData.words.high = bGetSonarState();
    vCANSendMsg(&xMsgBuffer);

    vTaskDelayUntil(&xLastWakeTime, xPeriod / portTICK_RATE_MS);
  }
}
