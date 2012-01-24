/*
  gyroscope.c is part of the RoseWheel project.
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
#include "queue.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "task.h"

#include "gyroscope.h"

#define I2C_GPIOx   GPIOB
#define I2C_SCL_Pin GPIO_Pin_6
#define I2C_SDA_Pin GPIO_Pin_7

#define IMU3000_ADDR           0x69
#define IMU3000_WHO_AM_I_REG   0x00
#define IMU3000_GYRO_OUT_BASE  0x1d
#define IMU3000_SMPLRT_DIV_REG 0x15
#define IMU3000_DLPF_REG       0x16
#define IMU3000_PWR_MGM_REG    0x3e

typedef struct {
  uint8_t direction;
  uint8_t byte_count;
} transfer_request_t;

static int16_t gyroscope_values[3] = {0};
static xQueueHandle xI2CTransferQueue;
static xQueueHandle xI2COutQueue;
static xQueueHandle xI2CInQueue;

static void i2c_clock_init();
static void i2c_write(const uint8_t* data, uint8_t byte_count);
static void i2c_read(uint8_t* buffer, uint8_t byte_count);
static void i2c_reset();
static void i2c_enable();
static void gyroscope_daemon(void *pvParameters);

void gyroscope_init(unsigned portBASE_TYPE daemon_priority)
{
  // Create queues
  xI2CTransferQueue = xQueueCreate(10, sizeof (transfer_request_t));
  assert_param(xI2CTransferQueue);

  xI2COutQueue = xQueueCreate(10, sizeof (uint8_t));
  assert_param(xI2COutQueue);

  xI2CInQueue = xQueueCreate(10, sizeof (uint8_t));
  assert_param(xI2CInQueue);

  // Configure clocks
  i2c_clock_init();

  // I2C initialization
  I2C_InitTypeDef I2C_InitStruct =
    {
      .I2C_ClockSpeed = 200000,
      .I2C_Mode = I2C_Mode_I2C,
      .I2C_DutyCycle = I2C_DutyCycle_2,
      .I2C_OwnAddress1 = 0x08,
      .I2C_Ack = I2C_Ack_Disable,
      .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
    };
  I2C_Init(I2C1, &I2C_InitStruct);

  // Enable interrupts
  I2C_ITConfig(I2C1, (I2C_IT_BUF | I2C_IT_EVT), ENABLE);
  NVIC_InitTypeDef NVIC_InitStruct =
    {
      .NVIC_IRQChannel                   = I2C1_EV_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 6,  // I2C MUST HAVE THE MAXIMUM
      .NVIC_IRQChannelSubPriority        = 0,  // PRIORITY AVAILABLE!
      .NVIC_IRQChannelCmd                = ENABLE,
    };
  NVIC_Init(&NVIC_InitStruct);

  //enable clock stretching
  I2C_StretchClockCmd(I2C1, ENABLE);

  // Create the daemon
  xTaskCreate(gyroscope_daemon, (const signed char * const)"gyrod",
	      configMINIMAL_STACK_SIZE, NULL, daemon_priority, NULL);
}

int16_t gyroscope_get_value(direction_t dir)
{
  switch (dir)
  {
    case X:
      return -gyroscope_values[0];
    case Y:
      return -gyroscope_values[1];
    case Z:
      return gyroscope_values[2];
  }
  assert_param(0);
}

void I2C1_EV_IRQHandler()
{
  portBASE_TYPE xNeedsRescheduling = pdFALSE;
  static transfer_request_t current_transfer = {0};

  if (I2C_GetFlagStatus(I2C1, I2C_FLAG_SB)) {
    // Start bit

    if (xQueueReceiveFromISR(xI2CTransferQueue, &current_transfer,
			     &xNeedsRescheduling))
      I2C_Send7bitAddress(I2C1, IMU3000_ADDR << 1, current_transfer.direction);
    else
      assert_param(0);


  } else if (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR)) {
    // Address sent

    current_transfer.byte_count--;

    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_TRA)) {
      // Transmitter mode
      uint8_t data_out;

      if (xQueueReceiveFromISR(xI2COutQueue, &data_out, &xNeedsRescheduling))
	I2C_SendData(I2C1, data_out);
      else
	assert_param(0);

      if ((current_transfer.byte_count == 0)
    	&& xQueueIsQueueEmptyFromISR(xI2CTransferQueue))
      I2C_GenerateSTOP(I2C1, ENABLE);
    } else {
      // Receiver mode

      if (current_transfer.byte_count == 0) {
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	if (xQueueIsQueueEmptyFromISR(xI2CTransferQueue))
	  I2C_GenerateSTOP(I2C1, ENABLE);
      } else {
	I2C_AcknowledgeConfig(I2C1, ENABLE);
      }
    }

  } else if (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE)) {
    // Data register empty (transmitters)

    if (current_transfer.byte_count > 0) {
      uint8_t data_out;

      if (xQueueReceiveFromISR(xI2COutQueue, &data_out, &xNeedsRescheduling))
	I2C_SendData(I2C1, data_out);
      else
	assert_param(0);

      current_transfer.byte_count--;

      if ((current_transfer.byte_count == 0)
	  && xQueueIsQueueEmptyFromISR(xI2CTransferQueue))
	I2C_GenerateSTOP(I2C1, ENABLE);
    } else {
      if (!xQueueIsQueueEmptyFromISR(xI2CTransferQueue))
	I2C_GenerateSTART(I2C1, ENABLE);
    }
  } else if (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE)) {
    // Data register not empty (receivers)

    uint8_t data_in = I2C1->DR;

    xQueueSendToBackFromISR(xI2CInQueue, &data_in, &xNeedsRescheduling);

    if (current_transfer.byte_count > 0) {
      current_transfer.byte_count--;

      if (current_transfer.byte_count == 0) {
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	if (xQueueIsQueueEmptyFromISR(xI2CTransferQueue))
	  I2C_GenerateSTOP(I2C1, ENABLE);
      }
    } else {
      if (!xQueueIsQueueEmptyFromISR(xI2CTransferQueue))
	I2C_GenerateSTART(I2C1, ENABLE);
    }

  }

  portEND_SWITCHING_ISR(xNeedsRescheduling);
}

static void i2c_clock_init()
{
  // Enable GPIOB clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  // Enable alternate function clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  // Enable I2C clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
}

static void i2c_reset()
{
  // Configure SCL and SDA as general purpose open-drain outputs
  GPIO_InitTypeDef GPIO_InitStruct =
    {
      .GPIO_Pin   = I2C_SCL_Pin | I2C_SDA_Pin,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_Out_OD,
    };
  GPIO_Init(I2C_GPIOx, &GPIO_InitStruct);

  /* First, we need to have SDA = 1, so that we can impose
     our values on the bus */

  // Try to set SDA
  GPIO_SetBits(I2C_GPIOx, I2C_SDA_Pin);

  // While SDA is not set...
  while (!GPIO_ReadInputDataBit(I2C_GPIOx, I2C_SDA_Pin)) {
    // ...generate a pulse on SCL
    GPIO_SetBits(I2C_GPIOx, I2C_SCL_Pin);
    vTaskDelay(1 / portTICK_RATE_MS);
    GPIO_ResetBits(I2C_GPIOx, I2C_SCL_Pin);
    vTaskDelay(1 / portTICK_RATE_MS);
  }

  /* Now we are going to generate a STOP condition */

  // First reset SDA and SCL
  GPIO_ResetBits(I2C_GPIOx, I2C_SDA_Pin | I2C_SCL_Pin);
  vTaskDelay(1 / portTICK_RATE_MS);

  // Rise SCL
  GPIO_SetBits(I2C_GPIOx, I2C_SCL_Pin);
  vTaskDelay(1 / portTICK_RATE_MS);

  // Wait until SCL is unequivocally set
  while (!GPIO_ReadInputDataBit(I2C_GPIOx, I2C_SCL_Pin));

  // Generate a rising edge on SDA
  GPIO_SetBits(I2C_GPIOx, I2C_SDA_Pin);

  // Wait until both SDA and SCL are unequivocally set
  while (!GPIO_ReadInputDataBit(I2C_GPIOx, I2C_SDA_Pin)
	 || !GPIO_ReadInputDataBit(I2C_GPIOx, I2C_SCL_Pin));

  // Bus idle time
  vTaskDelay(1 / portTICK_RATE_MS);

  /* At this point, we should have successfully reset the bus. */
}

static void i2c_enable()
{
  // Configure SCL and SDA as alternate function open-drain outputs
  GPIO_InitTypeDef GPIO_InitStruct =
    {
      .GPIO_Pin   = I2C_SCL_Pin | I2C_SDA_Pin,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_AF_OD,
    };
  GPIO_Init(I2C_GPIOx, &GPIO_InitStruct);

  // Enable I2C
  I2C_Cmd(I2C1, ENABLE);
}

static void i2c_write(const uint8_t* data, uint8_t byte_count)
{
  // Put a new transmit request in the transfer queue
  transfer_request_t request =
    {
      .direction = I2C_Direction_Transmitter,
      .byte_count = byte_count,
    };
  xQueueSendToBack(xI2CTransferQueue, &request, portMAX_DELAY);

  // Put data into the queue
  while (byte_count--)
    xQueueSendToBack(xI2COutQueue, data++, portMAX_DELAY);

  if (!I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL))
    // Generate start condition to enter I2C master mode
    I2C_GenerateSTART(I2C1, ENABLE);
}

static void i2c_read(uint8_t* buffer, uint8_t byte_count)
{
  // Put a new receive request in the transfer queue
  transfer_request_t request =
    {
      .direction = I2C_Direction_Receiver,
      .byte_count = byte_count,
    };
  xQueueSendToBack(xI2CTransferQueue, &request, portMAX_DELAY);

  if (!I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL))
    // Generate start condition to enter I2C master mode
    I2C_GenerateSTART(I2C1, ENABLE);

  // Wait for the data
  while (byte_count--)
    xQueueReceive(xI2CInQueue, buffer++, portMAX_DELAY);
}

static void gyroscope_write_register(uint8_t register_addr, uint8_t data)
{
  uint8_t i2c_data[2] = {register_addr, data};

  i2c_write(i2c_data, 2);
}

#if 0 // To eliminate warnings. This function might be useful for debugging.
static uint8_t gyroscope_read_register(uint8_t register_addr)
{
  uint8_t i2c_data;

  i2c_write(&register_addr, 1);

  i2c_read(&i2c_data, 1);

  return i2c_data;
}
#endif

static void gyroscope_read_register_burst(uint8_t base_addr, uint8_t* buffer,
					  uint8_t byte_count)
{
  i2c_write(&base_addr, 1);

  i2c_read(buffer, byte_count);
}

static void gyroscope_config()
{
  // Register read/write start-up time
  vTaskDelay(100 / portTICK_RATE_MS);

  // Set internal clock to PLL with Y Gyro reference
  gyroscope_write_register(IMU3000_PWR_MGM_REG, (2 << 0));

  // Set range to 1000°/s, analog sample rate to 1kHz
  // and low-pass filter bandwidth to 98 Hz
  gyroscope_write_register(IMU3000_DLPF_REG, (2 << 3) | (2 << 0));

  // Set sampling rate to 200 Hz
  gyroscope_write_register(IMU3000_SMPLRT_DIV_REG, 0x04);

  // Gyro start-up time
  vTaskDelay(50 / portTICK_RATE_MS);
}

static void gyroscope_daemon(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xPeriod = 5 / portTICK_RATE_MS;

  i2c_reset();

  i2c_enable();

  // Configure gyroscope registers
  gyroscope_config();

  // Initialize the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    uint8_t register_values[6];

    // Wait for the next cycle.
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    // Read the gyroscope values
    gyroscope_read_register_burst(IMU3000_GYRO_OUT_BASE, register_values, 6);

    // Aggregate high and low bytes
    for (int i = 0; i < 3; i++) {
      uint8_t high = register_values[2 * i];
      uint8_t low  = register_values[2 * i + 1];

      // Write the result to the gyroscope value table
      gyroscope_values[i] = (high << 8) + low;
    }
  }
}

