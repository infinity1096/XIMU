/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "ms5611.h"
#include "mpu9250.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"


#include "dataProcessing.h"

#include "usb_device.h"

#include "math.h"
#include "gnss.h"
#include "string.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP_READ_INTERVAL 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern int send_imu_data;
extern int send_mag_data;
extern int send_gps_data;
extern int send_pt_data;

extern double a[3], w[3], q[4], m[3], lla[3], pt[2];

int initialize = 0;

short timer_index = 0;
int ms5611_read_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  led_update();
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	//this timer runs on 200Hz.
	if (timer_index % 1 == 0){
		//code here runs on 200Hz, the data is updating at 100Hz.
		//Over-sampling ensures the data be updated at 100Hz.
		  short gyro[3] = {0,0,0}, accel[3] = {0,0,0}, sensors;
		  unsigned char more;
		  unsigned long timestamp;//unused
		  long quat[4];

		  do{
		  int status = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors,&more);
		  if (status == 0){

			  //quat is in q30 format, need to divide by 2^30.
			  q[0] = ((double)quat[0]) / 1073741824.0;
			  q[1] = ((double)quat[1]) / 1073741824.0;
			  q[2] = ((double)quat[2]) / 1073741824.0;
			  q[3] = ((double)quat[3]) / 1073741824.0;

			  //32768 = 2.0g
			  a[0] = ((double)accel[0]) * 9.81 / 16384.0;
			  a[1] = ((double)accel[1]) * 9.81 / 16384.0;
			  a[2] = ((double)accel[2]) * 9.81 / 16384.0;

			  w[0] = ((double)gyro[0]) * M_PI * 2000.0 / 180.0 / 32768.0; //32768 = 2000DEG/s
			  w[1] = ((double)gyro[1]) * M_PI * 2000.0 / 180.0 / 32768.0;
			  w[2] = ((double)gyro[2]) * M_PI * 2000.0 / 180.0 / 32768.0;
			  send_imu_data = 1;
		  }
		  }while(more != 0 && initialize == 0);
		  initialize = 1;
	}

	if (timer_index % 10 == 1){//avoid IMU update

		if (ms5611_read_index == 0){
			ms5611_retrieve_temperature();
			pt[1] = ms5611_get_temperature();
			ms5611_request_pressure();
			ms5611_read_index++;
		}else if (ms5611_read_index == TEMP_READ_INTERVAL - 1){
			ms5611_retrieve_pressure();
			pt[0] = ms5611_get_pressure();
			ms5611_request_temperature();
			ms5611_read_index = 0;
		}else{
			ms5611_retrieve_pressure();
			pt[0] = ms5611_get_pressure();
			ms5611_request_pressure();
			ms5611_read_index++;
		}
		send_pt_data = 1;
	}

	if (timer_index % 20 == 1){//avoid IMU update
		//code here runs on 10Hz
		short data[3];

		int status = mpu_get_compass_reg(data,NULL);

		if (status == 0){
			double raw_mag[3];
			raw_mag[0] = (double)data[0];
			raw_mag[1] = (double)data[1];
			raw_mag[2] = (double)data[2];
			calibrate_and_convert_mag_reading(raw_mag,m);
			send_mag_data = 1;
		}

		//poll DMA buffer
		GNSS_RX_Update();

		//Record GPS data
		if (GNSS.GNGGA.status != 0){//data is valid
			lla[0] = GNSS.GNGGA.lat;
			lla[1] = GNSS.GNGGA.lon;
			lla[2] = GNSS.GNGGA.MSL_alt;
			send_gps_data = 1;
		}
	}

	timer_index++;
	if (timer_index == 200){
		timer_index = 0;
	}
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
