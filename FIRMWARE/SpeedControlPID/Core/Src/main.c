/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct MotorHandle_t
{
  uint32_t* ADC_raw;
  float ADC_Voltage_mV;
  uint8_t conversionRatio_mVpA;
  float ADC_Current_A;
  float ADC_Current_mA;

  int32_t Encoder_Counter_Tick;
  int32_t Encoder_Counter_preTick;
  const uint16_t conversionRatio_TickpRotation;
  int32_t Encoder_Speed_TickpSec;
  int32_t Encoder_Speed_RPM;

  float Duty;
  uint32_t Vcc_mV;

} MotorHandle_t;

typedef struct LPF_t
{
  //Conner Frequency in Hz
	float Fc;
  //Sampling Time in second
  float Ts;

  //Filter Parameters
	float a1, a2, b1, b2;
	float alpha;
	float yk, yk_1, yk_2, uk, uk_1;

} LPF_t;

typedef struct PIHanlde_t
{
  MotorHandle_t MotorHandle;

  float Kp_Rotation;
  float Ki_Rotation;

  float Kp_Current;
  float Ki_Current;

  int32_t Ref_Speed_RPM;

  //error of rotation speed with time (0 - now)
  int32_t e_wt[2];
  //error of current with time (0 - now)
  float e_It[2];
  //value of gain part RPM & Current
  float vP_w;
  float vP_I;
  //value of integral part RPM & Current with time (0 - now)
  float vI_wt[2];
  float vI_It[2];
  //Output value of each PI
  float vOut_w;
  float vOut_I;

  //Sampling Time in millisecond
  uint32_t Ts;
  uint32_t timeKeeper;
  LPF_t Rotation_LPFHandle;
  LPF_t Current_LPFHandle;

  TIM_HandleTypeDef* PWM_Timer;
  uint32_t PWM_Channel;

  TIM_HandleTypeDef* FB_Encoder_Timer;

  bool Enable;

} PIHanlde_t;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI                (3.1415926535)
#define V25               (0.76)
#define AVG_SLOPE         (0.0025)
#define VREFINT           (1.21)
#define ADC_RESOLUTION    (4095.0)
#define COUNTER_PERIOD    (8399)

#define DEBUG             (1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char* InitSentence = "---\tHELLO! WELCOME TO P-I SPEED CONTROL PROGRAM!\t---\r\n";
char USB_RX_BUFFER[100] = {0};
volatile bool isNewUSBPacket = false;
volatile bool isUSBFree = true;

uint32_t ADC_raw_value[11][5] = {0};
uint8_t ADC_raw_value_index = 0;
uint32_t ADC_VREF_mV = 3300;
float chipTemperature = 0.0;

uint8_t LogBuffer[9] = {0};

PIHanlde_t Motor_1 = {
  .MotorHandle.ADC_raw = &ADC_raw_value[10][0],
  .MotorHandle.conversionRatio_mVpA = 185,
  .MotorHandle.conversionRatio_TickpRotation = 374,
  .MotorHandle.Vcc_mV = 12000,
  .Kp_Rotation = 0.04,
  .Ki_Rotation = 0.08,
  .Kp_Current = 1,
  .Ki_Current = 0.01,
  .Ts = 100,
  .PWM_Timer = &htim2,
  .PWM_Channel = TIM_CHANNEL_1,
  .FB_Encoder_Timer = &htim3,
  .Enable = false
};

PIHanlde_t Motor_2 = {
  .MotorHandle.ADC_raw = &ADC_raw_value[10][1],
  .MotorHandle.conversionRatio_mVpA = 185,
  .MotorHandle.conversionRatio_TickpRotation = 374,
  .MotorHandle.Vcc_mV = 12000,
  .Kp_Rotation = 0.04,
  .Ki_Rotation = 0.08,
  .Kp_Current = 1,
  .Ki_Current = 0.01,
  .Ts = 100,
  .PWM_Timer = &htim2,
  .PWM_Channel = TIM_CHANNEL_2,
  .FB_Encoder_Timer = &htim4,
  .Enable = false
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void LPF_Init(LPF_t *pLPF, uint16_t ConnerFrqHz, float samplingTime);

int32_t getRotationSpeed(PIHanlde_t* motor);
float getCurrent(PIHanlde_t* motor);
int32_t runRotation_LPF(LPF_t *pLPF, int32_t input);
float runCurrent_LPF(LPF_t *pLPF, float input);
float runPIControl(PIHanlde_t* motor);


float getChipTemperature(uint32_t rawADCtemp);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1500);
  HAL_IWDG_Refresh(&hiwdg);
  CDC_Transmit_FS((uint8_t*)InitSentence, strlen(InitSentence));

  LPF_Init(&Motor_1.Current_LPFHandle, 10, 0.1);
  LPF_Init(&Motor_1.Rotation_LPFHandle, 10, 0.1);
  LPF_Init(&Motor_2.Current_LPFHandle, 10, 0.1);
  LPF_Init(&Motor_2.Rotation_LPFHandle, 10, 0.1);

  HAL_TIM_Base_Start_IT(&htim5);

  HAL_TIM_PWM_Start(Motor_1.PWM_Timer, Motor_1.PWM_Channel);
  HAL_TIM_PWM_Start(Motor_2.PWM_Timer, Motor_2.PWM_Channel);

  HAL_ADC_Start_DMA(&hadc1, ADC_raw_value[0], 4);

  char tempbuf[100] = {0};

  uint32_t timeKeeper = HAL_GetTick();
  uint32_t timeKeeper1 = HAL_GetTick();
  Motor_1.timeKeeper = HAL_GetTick();
  Motor_2.timeKeeper = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    HAL_Delay(1);
    HAL_IWDG_Refresh(&hiwdg);
    
    if ((HAL_GetTick() - Motor_1.timeKeeper) >= Motor_1.Ts)
    {
      Motor_1.MotorHandle.ADC_Current_mA = runCurrent_LPF(&Motor_1.Current_LPFHandle, getCurrent(&Motor_1));
      Motor_1.MotorHandle.Encoder_Speed_RPM = runRotation_LPF(&Motor_1.Current_LPFHandle, getRotationSpeed(&Motor_1));

      if (Motor_1.Enable == true)
      {
        runPIControl(&Motor_1);

        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      }

      Motor_1.timeKeeper = HAL_GetTick();
    }

    if ((HAL_GetTick() - Motor_2.timeKeeper) >= Motor_2.Ts)
    {
      Motor_2.MotorHandle.ADC_Current_mA = runCurrent_LPF(&Motor_2.Current_LPFHandle, getCurrent(&Motor_2));
      Motor_2.MotorHandle.Encoder_Speed_RPM = runRotation_LPF(&Motor_2.Current_LPFHandle, getRotationSpeed(&Motor_2));

      if (Motor_2.Enable == true)
      {
        runPIControl(&Motor_2);

        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      }

      Motor_2.timeKeeper = HAL_GetTick();
    }

    if (((HAL_GetTick() - timeKeeper1) >= 5000) && DEBUG)
    {
      memset(tempbuf, '\0', sizeof(tempbuf));
      sprintf(tempbuf, "ADC raw value:\t[0] = [%li]\t[1] = [%li]\t[2] = [%li]\t[3] = [%li]\r\n", ADC_raw_value[10][0], ADC_raw_value[10][1], ADC_raw_value[10][2], ADC_raw_value[10][3]);
      CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      timeKeeper = HAL_GetTick();
      while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= 200));

      ADC_VREF_mV = (uint32_t)(VREFINT * ADC_RESOLUTION * 1000 / ADC_raw_value[10][3]);

      memset(tempbuf, '\0', sizeof(tempbuf));
      sprintf(tempbuf, "VDDA Voltage Calibrated:\t[%li]mV\r\n", ADC_VREF_mV);
      CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      timeKeeper = HAL_GetTick();
      while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= 200));

      chipTemperature = getChipTemperature(ADC_raw_value[10][2]);

      memset(tempbuf, '\0', sizeof(tempbuf));
      sprintf(tempbuf, "Chip temperature:\t[%2.2f]oC\r\n", chipTemperature);
      CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      timeKeeper = HAL_GetTick();
      while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= 200));

      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      timeKeeper1 = HAL_GetTick();
    }
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int32_t getRotationSpeed(PIHanlde_t* motor)
{
  motor->MotorHandle.Encoder_Speed_RPM = (int32_t)(motor->MotorHandle.Encoder_Speed_TickpSec / (60 * motor->MotorHandle.conversionRatio_TickpRotation));
  return (int32_t)(motor->MotorHandle.Encoder_Speed_RPM);
}

float getCurrent(PIHanlde_t* motor)
{
  motor->MotorHandle.ADC_Voltage_mV = (float)(*motor->MotorHandle.ADC_raw * ADC_VREF_mV / 4095.0);
  motor->MotorHandle.ADC_Current_A = (float)(motor->MotorHandle.ADC_Voltage_mV / motor->MotorHandle.conversionRatio_mVpA);
  motor->MotorHandle.ADC_Current_mA = (float)(motor->MotorHandle.ADC_Current_A * 1000.0);
  return (float)(motor->MotorHandle.ADC_Current_mA);
}

float runCurrent_LPF(LPF_t *pLPF, float input)
{
	//Read LPF input
	pLPF->uk = input;	
	//Compute LPF output
	pLPF->yk = -pLPF->b1 * pLPF->yk_1 - pLPF->b2 * pLPF->yk_2 + pLPF->a1 * pLPF->uk + pLPF->a2 * pLPF->uk_1; 
	//Save LPF past data
	pLPF->yk_2 = pLPF->yk_1; 
	pLPF->yk_1 = pLPF->yk; 
	pLPF->uk_1 = pLPF->uk;
	//Return LPF output
	return pLPF->yk;
}

int32_t runRotation_LPF(LPF_t *pLPF, int32_t input)
{
	//Read LPF input
	pLPF->uk = (float)input;	
	//Compute LPF output
	pLPF->yk = -pLPF->b1*pLPF->yk_1 - pLPF->b2*pLPF->yk_2 + pLPF->a1*pLPF->uk + pLPF->a2*pLPF->uk_1; 
	//Save LPF past data
	pLPF->yk_2 = pLPF->yk_1; 
	pLPF->yk_1 = pLPF->yk; 
	pLPF->uk_1 = pLPF->uk;
	//Return LPF output
	return (int32_t)(pLPF->yk);
}

float runPIControl(PIHanlde_t* motor)
{
  //error of rotation speed RPM
  motor->e_wt[0] = motor->Ref_Speed_RPM - motor->MotorHandle.Encoder_Speed_RPM;
  motor->vP_w = motor->Kp_Rotation * motor->e_wt[0];
  motor->vI_wt[0] = (motor->Ki_Rotation * motor->Ts * (motor->e_wt[0] - motor->e_wt[1]) / 2.0) + motor->vI_wt[1];
  //current set point mA
  motor->vOut_w = motor->vP_w + motor->vI_wt[0];

  //error of motor's current mA
  motor->e_It[0] = motor->vOut_w - motor->MotorHandle.ADC_Current_mA;
  motor->vP_I = motor->Kp_Current * motor->e_It[0];
  motor->vI_It[0] = (motor->Ki_Current * motor->Ts * (motor->e_It[0] - motor->e_It[1]) / 2.0) + motor->vI_It[1];
  //voltage set point mV
  motor->vOut_I = motor->vP_I + motor->vI_It[0];

  //Duty calculation
  motor->MotorHandle.Duty = ((float)(motor->vOut_I / motor->MotorHandle.Vcc_mV) + 1) / 2;

  //Duty saturation
  if (motor->MotorHandle.Duty > 1.0)        motor->MotorHandle.Duty = 1.0;
  else if (motor->MotorHandle.Duty < 0.0)   motor->MotorHandle.Duty = 0.0;

  //Update time variable
  motor->e_wt[1] = motor->e_wt[0];
  motor->vI_wt[1] = motor->vI_wt[0];
  motor->e_It[1] = motor->e_It[0];
  motor->vI_It[1] = motor->vI_It[0];

  //Set PWM Duty
  __HAL_TIM_SetCompare(motor->PWM_Timer, motor->PWM_Channel, (uint32_t)(motor->MotorHandle.Duty * COUNTER_PERIOD));

  return (float)(motor->MotorHandle.Duty);
}

void LPF_Init(LPF_t *pLPF, uint16_t ConnerFrqHz, float samplingTime)
{
	/*Low pass filter*/
	float temp1, temp2;
	
	pLPF->Fc = ConnerFrqHz;
	pLPF->Ts = samplingTime;
	pLPF->alpha = 1 / (2 * PI * pLPF->Ts * pLPF->Fc);
	
	temp1 = (1 + pLPF->alpha); 
	temp2 = temp1 * temp1;
	pLPF->a1 = (1 + 2 * pLPF->alpha) / temp2; 
	pLPF->a2 = (-2) * pLPF->alpha / temp2 ;
	pLPF->b1 = (-2) * pLPF->alpha / temp1; 
	pLPF->b2 = pLPF->alpha * pLPF->alpha / temp2;
	
	pLPF->yk = 0;
	pLPF->yk_1 = 0; 
	pLPF->yk_2 = 0;
	pLPF->uk = 0;
	pLPF->uk_1 = 0;
}

float getChipTemperature(uint32_t rawADCtemp)
{
  return (float)((((((float)rawADCtemp/ADC_RESOLUTION) * (float)(ADC_VREF_mV/1000.0)) - V25) / AVG_SLOPE) + 25.0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
    Motor_1.MotorHandle.Encoder_Counter_Tick = __HAL_TIM_GET_COUNTER(Motor_1.FB_Encoder_Timer);
    Motor_1.MotorHandle.Encoder_Speed_TickpSec = (int32_t)((int32_t)(Motor_1.MotorHandle.Encoder_Counter_Tick - Motor_1.MotorHandle.Encoder_Counter_preTick) * 1000 / Motor_1.Ts);
    Motor_1.MotorHandle.Encoder_Counter_preTick = Motor_1.MotorHandle.Encoder_Counter_Tick;
    Motor_2.MotorHandle.Encoder_Counter_Tick = __HAL_TIM_GET_COUNTER(Motor_2.FB_Encoder_Timer);
    Motor_2.MotorHandle.Encoder_Speed_TickpSec = (int32_t)((int32_t)(Motor_2.MotorHandle.Encoder_Counter_Tick - Motor_2.MotorHandle.Encoder_Counter_preTick) * 1000 / Motor_2.Ts);
    Motor_2.MotorHandle.Encoder_Counter_preTick = Motor_2.MotorHandle.Encoder_Counter_Tick;

    uint32_t temp = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
      temp = 0;
      for (uint8_t j = 0; j < 10; j++)
      {
        temp += ADC_raw_value[j][i];
      }
      ADC_raw_value[10][i] = (uint32_t)(temp / 10);
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (((ADC_raw_value_index % 4) == 3) && (ADC_raw_value_index < 40))
  {
    HAL_ADC_Stop_DMA(&hadc1);

    HAL_ADC_Start_DMA(&hadc1, ADC_raw_value[(ADC_raw_value_index + 1) / 4], 4);
  }
  ADC_raw_value_index++;
  if (ADC_raw_value_index >= 40)  ADC_raw_value_index = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
