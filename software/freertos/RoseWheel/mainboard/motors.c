/*
  motors.c is part of the RoseWheel project.
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

#include "motors.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "libperiph/hardware.h"

#define PERIOD          960
#define PWM_STOP        (PERIOD / 2)
#define MIN_PWM         (3.7 * PERIOD / 100)
#define MAX_PWM         (PERIOD - MIN_PWM)
#define MAX_VAL         (PERIOD / 2)
#define LIMIT_VAL       (220)
#define MAX_DIFF        10
#define STEERING_FACTOR 5
#define DEFAULT_PSC 2
#define BEEP_PSC 25

#define ABS(X)            (((X)<0)? -(X) : (X))
#define LIMIT(X,Min,Max)  (((X)<(Min))? (Min) : (((X)>(Max))? (Max) : (X)))
#define COMMAND_TO_PWM(C) (((C) + MAX_VAL) * PERIOD / (2 * MAX_VAL))

typedef enum{
  L_MOTOR,
  R_MOTOR
} motorID_t;

typedef union{
  struct {
    int16_t left;
    int16_t right;
  } motor;
  uint32_t motors;
} motors_command_t;

struct {
  unsigned int frequency;
  unsigned int duration_ms;
} beep_data = {0};

static xSemaphoreHandle xBeepMutex;

static volatile motors_command_t target_command;
static motors_command_t current_command;
static motors_command_t previous_command;
static void vMotorsApplyCommands(motors_command_t command);
static int iMotorsLimitCommand(int value, int previous);
static motors_command_t iMotorsLimitCommands(motors_command_t current,
                                             motors_command_t previous);
static void vMotorsTask(void* pvParameters);
static void vMotorsReset();
static void beeping_daemon(void* pvParameters);

void vMotorsInit(unsigned portBASE_TYPE motors_daemon_priority,
                 unsigned portBASE_TYPE beeping_daemon_priority)
{
  // Enable GPIOA & GPIOC clock
  gpio_clock_init(GPIOA);
  gpio_clock_init(GPIOC);
  timer_clock_init(TIM5);

  // Motors PWM: MOTOR1=left, MOTOR2=right ; A and B have opposed polarity
  GPIO_InitTypeDef GPIO_InitStructure1 =
  {
    .GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | // MOTOR2_B, MOTOR2_A
                  GPIO_Pin_2 | GPIO_Pin_3 , // MOTOR1_B, MOTOR1_A
    .GPIO_Mode  = GPIO_Mode_AF_PP,          // alternate function push pull
    .GPIO_Speed = GPIO_Speed_2MHz
  };
  GPIO_Init(GPIOA, &GPIO_InitStructure1);

  // Motors enable pin
  GPIO_InitTypeDef GPIO_InitStructure2 =
  {
    .GPIO_Pin   = GPIO_Pin_3,               // MOTORS_ENABLE
    .GPIO_Mode  = GPIO_Mode_Out_PP,         // push pull
    .GPIO_Speed = GPIO_Speed_2MHz
  };
  GPIO_Init(GPIOC, &GPIO_InitStructure2);

  // Set output compare interrupt flags of channels configured in output
  // (CCxS=00 in TIMx_CCMRx register) when counting up or down
  TIM_CounterModeConfig(TIM5, TIM_CounterMode_CenterAligned3);

  //  robots.freehostia.com/SpeedControl/SpeedControllersBody.html#2.2
  // f = 72MHz / 960 / 4 =~ 18kHz (P=5%, R=0.6ohms, L=100uH)
  TIM_TimeBaseInitTypeDef Timer_InitStructure =
  {
    .TIM_ClockDivision      = TIM_CKD_DIV1,
    .TIM_Prescaler          = DEFAULT_PSC,
    .TIM_Period             = PERIOD,
    .TIM_CounterMode        = TIM_CounterMode_Up
  };
  TIM_TimeBaseInit(TIM5, &Timer_InitStructure);


  // Output Compare Init :
  TIM_OCInitTypeDef OC_InitStructure;
  TIM_OCStructInit(&OC_InitStructure);
  OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM2;


  // Channel 1 & 2, left motor
  TIM_OC1Init(TIM5, &OC_InitStructure);
  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC1PolarityConfig(TIM5, TIM_OCPolarity_High); // pos pwm

  TIM_OC2Init(TIM5, &OC_InitStructure);
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC2PolarityConfig(TIM5, TIM_OCPolarity_Low);  // neg pwm

  // Channel 3 & 4, right motor
  TIM_OC3Init(TIM5, &OC_InitStructure);
  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC3PolarityConfig(TIM5, TIM_OCPolarity_High); // pos pwm

  TIM_OC4Init(TIM5, &OC_InitStructure);
  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC4PolarityConfig(TIM5, TIM_OCPolarity_Low);  // neg pwm


  // Enables the TIM Capture Compare Channels
  TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Enable);
  TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Enable);
  TIM_CCxCmd(TIM5, TIM_Channel_3, TIM_CCx_Enable);
  TIM_CCxCmd(TIM5, TIM_Channel_4, TIM_CCx_Enable);

  // Set default value: motor stopped
  vMotorsDisable();

  // Enables TIM5 peripheral Preload register on ARR
  TIM_ARRPreloadConfig(TIM5, ENABLE);

  TIM_Cmd(TIM5, ENABLE); // enable timer 5

  // Create the beep mutex
  xBeepMutex = xSemaphoreCreateMutex();

  // Create the daemon
  xTaskCreate(vMotorsTask, (const signed char * const)"motorsd",
              configMINIMAL_STACK_SIZE, NULL, motors_daemon_priority, NULL);
  xTaskCreate(beeping_daemon, (const signed char * const)"motorsd",
              configMINIMAL_STACK_SIZE, NULL, beeping_daemon_priority, NULL);
}

static void vMotorsReset()
{
  motors_command_t command;
  command.motors = 0;
  vMotorsApplyCommands(command);
  previous_command.motors = 0;
}

void vMotorsEnable()
{
  // We stop motors because speed was zero
  vMotorsReset();
  GPIO_SetBits(GPIOC, GPIO_Pin_3);
}

void vMotorsDisable()
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_3);
}

static void vMotorsApplyCommands(motors_command_t command)
{
  const int PWML = LIMIT(COMMAND_TO_PWM(command.motor.left), MIN_PWM, MAX_PWM);
  const int PWMR = LIMIT(COMMAND_TO_PWM(command.motor.right), MIN_PWM, MAX_PWM);

  // Here we disable interrupts to ensure that the same command will
  // be applied to the two motors at the same time. Moreover for a
  // single motor we want the PWMs to be fully synchronized.
  portDISABLE_INTERRUPTS();
  TIM_SetCompare1(TIM5, PWML);
  TIM_SetCompare2(TIM5, PWML);

  TIM_SetCompare3(TIM5, PWMR);
  TIM_SetCompare4(TIM5, PWMR);
  portENABLE_INTERRUPTS();
}

static int iMotorsLimitCommand(int value, int previous)
{
  // saturate if we don't respect the voltage limits:
  value = LIMIT(value, -LIMIT_VAL, LIMIT_VAL);

  // saturate if we change the direction too violently
  int diff = value - previous;

  if (ABS(diff) > MAX_DIFF)
    value = previous + ((diff > 0) ? MAX_DIFF : -MAX_DIFF);

  return value;
}

static motors_command_t iMotorsLimitCommands(motors_command_t current,
                                             motors_command_t previous)
{
  motors_command_t command;
  command.motor.left  = iMotorsLimitCommand(current.motor.left,
                                            previous.motor.left);
  command.motor.right = iMotorsLimitCommand(current.motor.right,
                                            previous.motor.right);
  return command;
}

// use pointers to get a feedback after the securisation
void vMotorsCommand(int left, int right)
{
   motors_command_t command = {
      .motor.left = left,
      .motor.right = right
   };
   target_command = command;
}

void vMotorsLeftCommand(int left)
{
  target_command.motor.left = left;
}

void vMotorsRightCommand(int right)
{
  target_command.motor.right = right;
}

int iMotorsGetLeftCommand()
{
  return current_command.motor.left;
}

int iMotorsGetRightCommand()
{
  return current_command.motor.right;
}

static void vMotorsTask(void* pvParameters)
{
  portTickType time = xTaskGetTickCount();

  for (;;)
  {
    // Sample the value at this moment:
    current_command = iMotorsLimitCommands(target_command, previous_command);
    vMotorsApplyCommands(current_command);
    previous_command = current_command;

    vTaskDelayUntil(&time, 5 / portTICK_RATE_MS);
  }
}

float fMotorsGetSteeringFactor(int iAngleDeg)
{
  iAngleDeg = (ABS(iAngleDeg) - 1) / 3 + 1;
  return MAX_VAL / (STEERING_FACTOR * iAngleDeg);
}

static void beep_on_internal()
{
  TIM_PrescalerConfig(TIM5, BEEP_PSC, TIM_PSCReloadMode_Update);

}

static void beep_off_internal()
{
  TIM_PrescalerConfig(TIM5, DEFAULT_PSC, TIM_PSCReloadMode_Update);
}

void vBeepOn()
{
  xSemaphoreTake(xBeepMutex, portMAX_DELAY);

  beep_data.duration_ms = 0xffffff;
  beep_data.frequency = 0;
  beep_on_internal();

  xSemaphoreGive(xBeepMutex);
}

void vBeepOff()
{
  xSemaphoreTake(xBeepMutex, portMAX_DELAY);

  beep_data.duration_ms = 0;
  beep_data.frequency = 0xffffffff;
  beep_off_internal();

  xSemaphoreGive(xBeepMutex);
}

void vSetBeep(unsigned int frequency, unsigned int duration_ms)
{
  xSemaphoreTake(xBeepMutex, portMAX_DELAY);

  beep_data.frequency = frequency;
  beep_data.duration_ms = duration_ms;

  xSemaphoreGive(xBeepMutex);
}

void vSetBeepFreq(int f)
{
  if ((1 <= f) && (f <= 11))
    vSetBeep(f, 0xffffffff);
}

void vSetBeepCont(int dt_ms)
{
  vSetBeep(0, dt_ms);
}

static void beeping_daemon(void* pvParameters)
{
  bool beeping = FALSE;
  portTickType time = xTaskGetTickCount();
  portTickType last_transition = 0;

  for (;;)
  {
    xSemaphoreTake(xBeepMutex, portMAX_DELAY);
    if (beep_data.duration_ms) {
      portTickType now = xTaskGetTickCount();

      if (beep_data.frequency > 0) {
        unsigned int period_ms = (1000 / beep_data.frequency);

        if (((now - last_transition) / portTICK_RATE_MS) > (period_ms / 2)) {
          if (beeping) {
            beep_off_internal();
            beeping = FALSE;
          } else {
            beep_on_internal();
            beeping = TRUE;
          }

          last_transition = now;
        }
      } else {
        beep_on_internal();
        last_transition = now;
      }
      beep_data.duration_ms = ((beep_data.duration_ms > 5)
                               ? beep_data.duration_ms - 5
                               : 0);
    } else {
      beep_off_internal();
    }

    xSemaphoreGive(xBeepMutex);
    vTaskDelayUntil(&time, 5 / portTICK_RATE_MS);
  }
}
