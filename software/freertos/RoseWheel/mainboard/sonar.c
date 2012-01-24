/*
  sonar.c is part of the RoseWheel project.
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
#include "stm32f10x.h"

#include "libperiph/sonar.h"
#include "mathutils.h"

#include "motors.h"
#include "sonar.h"
#include "control.h"

#define MIN_DISTANCE_CM 0
#define MAX_DISTANCE_CM 200
#define DANGER 10
static int iDistanceCm;
static int iDanger = 0;

static void vSonarDaemon(void* pvParameters);

static bool bBeepEnable = FALSE;

void vSonarInit(unsigned portBASE_TYPE daemon_priority)
{
  sonar_init();

  // Create the daemon
  xTaskCreate(vSonarDaemon, (const signed char * const)"sonard",
	      configMINIMAL_STACK_SIZE, NULL, daemon_priority, NULL);
}

static void vSonarDaemon(void* pvParameters)
{
  int i = 10;
  double error = 0;
  double integral = 0;
  const double kp = -0.0005;
  const double ki = 0.0000;
  const double kd = 0.0005;
  static const portTickType xPeriod = 51 / portTICK_RATE_MS;
  portTickType xTime = xTaskGetTickCount();

  for (;;)
  {
    iDistanceCm = sonar_measure_dist_cm();

    if (iDistanceCm <= MAX_DISTANCE_CM
        && iDistanceCm >= MIN_DISTANCE_CM)
      iDanger++;
    else
      iDanger--;

    if (iDanger < 0)
      iDanger = 0;
    if (iDanger > DANGER)
      iDanger = DANGER;

    if (bBeepEnable) {
      if (bGetSonarDanger() &&
          (vControlGetCurrentAngleOffset() > DEG2RAD(1))
        ) {
        double current_error, derivative;
        vector_t* pxCurrentCommand = xControlGetCommand();

        current_error = pxCurrentCommand->data[LEFT] * (MAX_DISTANCE_CM - iDistanceCm);

        derivative = current_error - error;

        integral += current_error;

        vControlSetEqAngleOffset(kp * current_error
                                 + ki * integral
                                 + kd * derivative);

        error = current_error;

      } else {
        vControlSetEqAngleOffset(0);
        error = 0;
        integral = 0;
      }

      if (!(i--)) {
        if (bGetSonarDanger())
          vSetBeepFreq(11 - iDistanceCm/15);
        else
          vBeepOff();
        i = 10;
      }
    }

    vTaskDelayUntil(&xTime, xPeriod);
  }
}

int iGetSonarDistanceCm()
{
  return iDistanceCm;
}

bool bGetSonarDanger()
{
  return iDanger == DANGER;
}

void vSonarBeepOn()
{
  bBeepEnable = TRUE;
}

void vSonarBeepOff()
{
  bBeepEnable = FALSE;
  vBeepOff();
}

bool bGetSonarState()
{
  return bBeepEnable;
}
