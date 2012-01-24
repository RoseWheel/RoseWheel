/*
  control.c is part of the RoseWheel project.
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

#include "libcontrol/lqr.h"
#include "libcontrol/pid.h"
#include "pidvalues/pidvalues.h"
#include "libglobal/mathutils.h"

#include "control.h"
#include "libperiph/uart.h"
#include "strutils.h"

#define MAX_SET_POINT_STEP DEG2RAD(1)

const double EQ_ANGLE[] = {SELF_DRIVEN_EQ_ANGLE, HUMAN_DRIVEN_EQ_ANGLE};

static double xSystemData[4] = {0};
static vector_t xSystemState = {0};

static double xCommandData[2] = {0};
static vector_t xCommand = {0};

static double dSetPoint      = SELF_DRIVEN_EQ_ANGLE;
static double dOffset        = 0;
static double dFinalSetPoint = SELF_DRIVEN_EQ_ANGLE;
static double dCurrentAngle  = 0;

static void prvControlDaemon(void *pvParameters);

void vControlInit(unsigned portBASE_TYPE xDaemonPriority)
{
  vector_init(&xSystemState, xSystemData, 4);
  vector_init(&xCommand, xCommandData, 2);

  pid_init(pid_kp, pid_ki, pid_kd);

  xTaskCreate(prvControlDaemon,
              (signed portCHAR*)"controld",
              configMINIMAL_STACK_SIZE, NULL,
              xDaemonPriority, NULL);
}

void vControlUpdate(vector_t* pxCurrentState, double dTimeElapsed)
{
  dCurrentAngle = pxCurrentState->data[PHI];

  xCommandData[LEFT] = pid_compute_command(
    dSetPoint - pxCurrentState->data[PHI],
    - pxCurrentState->data[PHI_DOT],
    dTimeElapsed );
  xCommandData[RIGHT] = xCommandData[LEFT];
}

vector_t* xControlGetCommand()
{
  return &xCommand;
}

void vControlSetEqAngle(double dEqAngle)
{
  dFinalSetPoint = dEqAngle;
}

void vControlSetEqAngleOffset(double dEqAngleOffset)
{
  dOffset = dEqAngleOffset;
}

static void prvControlDaemon(void *pvParameters)
{
  static const portTickType xPeriod = 25 / portTICK_RATE_MS;
  portTickType xTime = xTaskGetTickCount();

  for (;;) {
    dSetPoint += ((ABS(dFinalSetPoint + dOffset - dSetPoint) > MAX_SET_POINT_STEP)
                  ? SIGN(dFinalSetPoint + dOffset - dSetPoint) * MAX_SET_POINT_STEP
                  : dFinalSetPoint + dOffset - dSetPoint);

    vTaskDelayUntil(&xTime, xPeriod);
  }
}

double vControlGetEqAngle()
{
  return dFinalSetPoint;
}

double vControlGetCurrentAngleOffset()
{
  return ABS(dFinalSetPoint - dCurrentAngle);
}
