/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	float Kp, TI;										//Controller parameters	
	float Ks, D;										//Ks: Anti-windup Gain; D = 1-Ts/TI
	float Umax, Umin, Usk, Usk_1, Uk, Uk_1;	//Control signal
	float Ek, Ek_1, Esk, Esk_1;			//Error
	float Ts;												//Sampling time
}cPI;

typedef struct
{
	float Ts;							//Sampling time
	float Fc; 						//Conrner frequency (Hz)
	float a1, a2, b1, b2;	//Parameters of the filter
	float alpha;
	float yk, yk_1, yk_2, uk, uk_1;
}LPF;

typedef struct
{
	int16_t ppr;								//Pulse per round of the Encoder
	float Ts;										//Sampling time (sec)
	int32_t Cnt_k, Cnt_k_1;			//Value of counter at time instance k and k_1
	uint16_t Cnt_Ovf; 					//Overflow counter
	uint16_t rpm;								//Round per minute of the motor - need to be calculated
	int16_t Fb_rpm;
	int8_t Direction;						//Direction of the motor
	int8_t PU_rpm;
	int8_t FirstCnt;	
  int64_t Cnt_pulse;
	int16_t Cnt_rot;
}QEI;

typedef struct
{
	uint16_t data[2];
	float rawVolIa;
	float filterVolIa;
	float Ia;
}adc_Objt;

typedef struct
{
	int16_t SP_position;
	float SP_position_ramp;
	int16_t SP_speed;
	float SP_current;
	float Ua;
	float Uref;
  float Duty,Duty1, Duty2;
  uint16_t Cnt,Cnt_1, Cnt_2;	
}controller_objt;

typedef enum
{
	ACC, 
	RUN, 
	DEC
}RAMP_STATE;

typedef struct
{
	int16_t SPk;												//Current and past value of setpoint
	float inputMax, inputMin;
	uint16_t T_Acc, T_Dec;						//Ramp up and down time (s)
	float delta_Acc, delta_Dec;				//step size in ACC and DEC mode
	uint16_t set_AccCnt, set_DecCnt;	//Set count - 10ms Resolution
	RAMP_STATE State;
	float Output;
}RAMP;

typedef struct{
	uint8_t Rx_buff[5];
	uint8_t Rx_data;
	uint8_t Tx_data[3];
	uint8_t Rx_flag;
	uint8_t Rx_flag_err;
	uint8_t check_sum,dir,round;
	uint8_t pointer,cnt,state;
	
} Uart_Obj;
#define Frame_start 0x7A
#define Frame_end   0x7F
#define Uart_Wait   0x02
#define Uart_Reading 0x03
#define Success     0x00
#define Fail        0x01

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void vLPF_Init(LPF *pLPF, uint16_t ConnerFrqHz, float samplingTime);
float fLPF_Run(LPF *pLPF, float input);
void vAdc_DataInit(adc_Objt *pObjt);
void vAdc_Data_Handle(adc_Objt *pObjt);
void PI_Init(cPI *pPI, float TI, float KP, float Ts,double Umax,double Umin);
float fPI_Run(cPI *pPI, float Ref, float Fb);
void Position_controller(controller_objt *pcon,QEI *pQEI);
void Controller_Init(controller_objt *pcon);
void Speed_controller(controller_objt *pcon,QEI *pQEI);
void Current_controller(controller_objt *pcon,adc_Objt *padc);
void UART(controller_objt *pcon,QEI *pQEI);
void vQEI_Config(QEI *pQEI,float Ts);
void vRPM_Cal(QEI *pQEI);
void vRamp_Config(RAMP *pRamp);
float fRampFcnGen(int16_t SP, RAMP *pRamp);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void UART_SP(void);
void Uart_Init(Uart_Obj *uart);
void Uart_read(Uart_Obj *uart);
_ARMABI int fputc(int c, FILE *stream) ;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define PI 3.1416
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
