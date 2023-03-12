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

#include "log_codec.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct MotorHandle_t
{
  uint32_t* ADC_raw;
  uint32_t ADC_Voltage_mV;
  uint32_t ACS_MidPointVoltage_mV;
  uint8_t conversionRatio_mVpA;
  float ADC_Current_A;
  int32_t ADC_Current_mA;

  int32_t encoderValue;
  uint16_t Encoder_Counter_Tick;
  uint16_t Encoder_Counter_preTick;
  uint16_t conversionRatio_TickpRotation;
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
#define PI                      (3.1415926535)
#define V25                     (0.76)
#define AVG_SLOPE               (0.0025)
#define VREFINT                 (1.21)
#define ADC_RESOLUTION          (4095.0)
#define COUNTER_PERIOD          (8399)
#define NUMBER_OF_ADC_CHANNEL   (5)
#define ADC_BUFFER_LENGTH       (100)
#define LOG_DELAY_MS            (20)
#define ACS_ADC_RATIO           (0.6416)
#define ALPHA                   (0.0001)

#define MOTOR_1_FLASH_ADDR      (0x08020000)
#define MOTOR_2_FLASH_ADDR      (0x08020500)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static char* InitSentence = "---\tHELLO! WELCOME TO P-I SPEED CONTROL PROGRAM!\t---\r\n";
uint8_t USB_RX_BUFFER[100] = {0};
volatile bool isNewUSBPacket = false;
volatile bool isUSBFree = true;

volatile bool isDEBUG = false;
volatile bool isAppConnected = false;
volatile bool isUpdating = false;
volatile bool isLogSetting = false;

static uint32_t ADC_raw_value[ADC_BUFFER_LENGTH + 1][NUMBER_OF_ADC_CHANNEL] = {0};
static uint32_t ADC_raw_Inverted[NUMBER_OF_ADC_CHANNEL][ADC_BUFFER_LENGTH + 1] = {0};
uint16_t ADC_raw_value_index = 0;
static uint32_t ADC_VREF_mV = 3325;
static uint32_t ADC_VIN = 12000;
float chipTemperature = 0.0;

uint32_t encoderIndex = 0;

static PIHanlde_t Motor_1 = {
  // .MotorHandle.ADC_raw = &ADC_raw_value[ADC_BUFFER_LENGTH][0],
  .MotorHandle.conversionRatio_mVpA = 185,
  // .MotorHandle.ACS_MidPointVoltage_mV = 2500,
  .MotorHandle.conversionRatio_TickpRotation = 334,
  .MotorHandle.Vcc_mV = 12000,
  .Ref_Speed_RPM = 0,
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

static PIHanlde_t Motor_2 = {
  // .MotorHandle.ADC_raw = &ADC_raw_value[ADC_BUFFER_LENGTH][1],
  .MotorHandle.conversionRatio_mVpA = 185,
  // .MotorHandle.ACS_MidPointVoltage_mV = 2500,
  .MotorHandle.conversionRatio_TickpRotation = 334,
  .MotorHandle.Vcc_mV = 12000,
  .Ref_Speed_RPM = 0,
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

static uint8_t dummy1[300] = {0};
static uint8_t dummy2[300] = {0};

PIHanlde_t* pMotor;

uint32_t timeKeeper = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void LPF_Init(LPF_t *pLPF, uint16_t ConnerFrqHz, float samplingTime);

int32_t getRotationSpeed(PIHanlde_t* motor);
uint32_t getCurrent(PIHanlde_t* motor);
int32_t runRotation_LPF(LPF_t *pLPF, int32_t input);
int32_t runCurrent_LPF(LPF_t *pLPF, int32_t input);
float runPIControl(PIHanlde_t* motor);
void runEncoderTickHandle(PIHanlde_t* motor);

bool logData(PIHanlde_t* motor, uint8_t codec);
bool updateData(void);

uint32_t prepareSaveSpace();
uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);

float getChipTemperature(uint32_t rawADCtemp);

uint16_t CheckSumCrc16(uint8_t *ucBufferTemp, uint8_t ucLength)
{
    uint32_t CRCFull = 0xFFFF;
    char CRCLSB;
    for (uint8_t i = 0; i < ucLength; i++)
    {
        CRCFull = (uint16_t) (CRCFull ^ ucBufferTemp[i]);
        for ( int j = 0; j < 8; j++ )
        {
            CRCLSB = (char) (CRCFull & 0x0001);
            CRCFull = (uint16_t) ((CRCFull >> 1) & 0x7FFFF);
            
            if ( CRCLSB == 1 )
            {
                CRCFull = (uint16_t) (CRCFull ^ 0xA001);
            }
        }
    }
    return CRCFull;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// void thisTestFunction(void)
// {
//   const char a[] = "this is a TEST, this text should be at the end of flash!!!!!";
//   CDC_Transmit_FS((uint8_t*)a, strlen(a));
// }

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
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(20);
  HAL_IWDG_Refresh(&hiwdg);
  CDC_Transmit_FS((uint8_t*)InitSentence, strlen(InitSentence));

  LPF_Init(&Motor_1.Current_LPFHandle, 10, 0.1);
  LPF_Init(&Motor_1.Rotation_LPFHandle, 10, 0.1);
  LPF_Init(&Motor_2.Current_LPFHandle, 10, 0.1);
  LPF_Init(&Motor_2.Rotation_LPFHandle, 10, 0.1);

  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim9);

  HAL_GPIO_WritePin(MOTOR_DIR1_GPIO_Port, MOTOR_DIR1_Pin, 0);
  HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, 0);
  HAL_GPIO_WritePin(MOTOR_DIR3_GPIO_Port, MOTOR_DIR3_Pin, 0);
  HAL_GPIO_WritePin(MOTOR_DIR4_GPIO_Port, MOTOR_DIR4_Pin, 0);

  HAL_ADC_Start_DMA(&hadc1, ADC_raw_value[0], NUMBER_OF_ADC_CHANNEL);

  char tempbuf[100] = {0};
  uint8_t checkLog1 = 0;
  uint8_t checkLog2 = 0;

  // Flash_write
  // Flash_Write_Data(MOTOR_1_FLASH_ADDR, (uint32_t*)&Motor_1, sizeof(Motor_1)/4);
  // Flash_Write_Data(MOTOR_2_FLASH_ADDR, (uint32_t*)&Motor_2, sizeof(Motor_2)/4);

  // Load data from flash
  memcpy(&Motor_1, (uint32_t*)MOTOR_1_FLASH_ADDR, sizeof(Motor_1));
  memcpy(&Motor_2, (uint32_t*)MOTOR_2_FLASH_ADDR, sizeof(Motor_2));

  // *(uint32_t*)&dummy1[(uint32_t)(&pMotor->Kp_Rotation) - MOTOR_1_FLASH_ADDR] = 0x12345678;
  HAL_IWDG_Refresh(&hiwdg);

  Motor_1.MotorHandle.ADC_raw = &ADC_raw_value[ADC_BUFFER_LENGTH][0];
  Motor_1.PWM_Timer = &htim2;
  Motor_1.PWM_Channel = TIM_CHANNEL_1;
  Motor_1.FB_Encoder_Timer = &htim3;

  Motor_2.MotorHandle.ADC_raw = &ADC_raw_value[ADC_BUFFER_LENGTH][1];
  Motor_2.PWM_Timer = &htim2;
  Motor_2.PWM_Channel = TIM_CHANNEL_2;
  Motor_2.FB_Encoder_Timer = &htim4;

  HAL_TIM_PWM_Start(Motor_1.PWM_Timer, Motor_1.PWM_Channel);
  HAL_TIM_PWM_Start(Motor_2.PWM_Timer, Motor_2.PWM_Channel);

  HAL_TIM_Encoder_Start(Motor_1.FB_Encoder_Timer, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(Motor_2.FB_Encoder_Timer, TIM_CHANNEL_ALL);

  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(500);
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(500);
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  HAL_IWDG_Refresh(&hiwdg);

  HAL_Delay(2000);
  Motor_1.MotorHandle.ACS_MidPointVoltage_mV = (uint32_t)((*Motor_1.MotorHandle.ADC_raw * ADC_VREF_mV / 4095.0) / ACS_ADC_RATIO);
  Motor_2.MotorHandle.ACS_MidPointVoltage_mV = (uint32_t)((*Motor_2.MotorHandle.ADC_raw * ADC_VREF_mV / 4095.0) / ACS_ADC_RATIO);
  ADC_VIN = (uint32_t)(ADC_raw_value[ADC_BUFFER_LENGTH][4] * 7.2686 * ADC_VREF_mV / ADC_RESOLUTION);

  HAL_IWDG_Refresh(&hiwdg);
  timeKeeper = HAL_GetTick();
  uint32_t timeKeeper1 = HAL_GetTick();
  uint32_t timeKeeper2 = HAL_GetTick();
  Motor_1.timeKeeper = HAL_GetTick();
  Motor_2.timeKeeper = HAL_GetTick();

  // thisTestFunction();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    HAL_Delay(1);
    // HAL_IWDG_Refresh(&hiwdg);
    
    if ((HAL_GetTick() - Motor_1.timeKeeper) >= Motor_1.Ts)
    {
      Motor_1.MotorHandle.ADC_Current_mA = runCurrent_LPF(&Motor_1.Current_LPFHandle, getCurrent(&Motor_1));
      Motor_1.MotorHandle.Encoder_Speed_RPM = runRotation_LPF(&Motor_1.Current_LPFHandle, getRotationSpeed(&Motor_1));

      // Motor_1.MotorHandle.ADC_Current_mA = getCurrent(&Motor_1);
      // Motor_1.MotorHandle.Encoder_Speed_RPM = getRotationSpeed(&Motor_1);
      if ((isAppConnected == true) && (isUpdating == false)) checkLog1++;

      if (Motor_1.Enable == true)
      {
        if (Motor_1.Ref_Speed_RPM > 0)
        {
          HAL_GPIO_WritePin(MOTOR_DIR1_GPIO_Port, MOTOR_DIR1_Pin, 1);
          HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, 0);
        }
        else if (Motor_1.Ref_Speed_RPM < 0)
        {
          HAL_GPIO_WritePin(MOTOR_DIR1_GPIO_Port, MOTOR_DIR1_Pin, 0);
          HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, 1);
        }
        else
        {
          HAL_GPIO_WritePin(MOTOR_DIR1_GPIO_Port, MOTOR_DIR1_Pin, 0);
          HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, 0);
        }
        
        // Motor_1.MotorHandle.Duty = 0.5;
        // __HAL_TIM_SetCompare(Motor_1.PWM_Timer, Motor_1.PWM_Channel, 4199);
        runPIControl(&Motor_1);
      }
      else
      {
        __HAL_TIM_SetCompare(Motor_1.PWM_Timer, Motor_1.PWM_Channel, (uint32_t)0);
        HAL_GPIO_WritePin(MOTOR_DIR1_GPIO_Port, MOTOR_DIR1_Pin, 0);
        HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, 0);
      }

      Motor_1.timeKeeper = HAL_GetTick();
    }

    if ((HAL_GetTick() - Motor_2.timeKeeper) >= Motor_2.Ts)
    {
      // Motor_2.MotorHandle.ADC_Current_mA = runCurrent_LPF(&Motor_2.Current_LPFHandle, getCurrent(&Motor_2));
      // Motor_2.MotorHandle.Encoder_Speed_RPM = runRotation_LPF(&Motor_2.Current_LPFHandle, getRotationSpeed(&Motor_2));

      Motor_2.MotorHandle.ADC_Current_mA = getCurrent(&Motor_2);
      Motor_2.MotorHandle.Encoder_Speed_RPM = getRotationSpeed(&Motor_2);
      if ((isAppConnected == true) && (isUpdating == false)) checkLog2++;

      if (Motor_2.Enable == true)
      {
        if (Motor_2.Ref_Speed_RPM > 0)
        {
          HAL_GPIO_WritePin(MOTOR_DIR3_GPIO_Port, MOTOR_DIR3_Pin, 1);
          HAL_GPIO_WritePin(MOTOR_DIR4_GPIO_Port, MOTOR_DIR4_Pin, 0);
        }
        else if (Motor_2.Ref_Speed_RPM < 0)
        {
          HAL_GPIO_WritePin(MOTOR_DIR3_GPIO_Port, MOTOR_DIR3_Pin, 0);
          HAL_GPIO_WritePin(MOTOR_DIR4_GPIO_Port, MOTOR_DIR4_Pin, 1);
        }
        else
        {
          HAL_GPIO_WritePin(MOTOR_DIR3_GPIO_Port, MOTOR_DIR3_Pin, 0);
          HAL_GPIO_WritePin(MOTOR_DIR4_GPIO_Port, MOTOR_DIR4_Pin, 0);
        }
        
        runPIControl(&Motor_2);
      }
      else
      {
        __HAL_TIM_SetCompare(Motor_2.PWM_Timer, Motor_2.PWM_Channel, (uint32_t)0);
        HAL_GPIO_WritePin(MOTOR_DIR3_GPIO_Port, MOTOR_DIR3_Pin, 0);
        HAL_GPIO_WritePin(MOTOR_DIR4_GPIO_Port, MOTOR_DIR4_Pin, 0);
      }

      Motor_2.timeKeeper = HAL_GetTick();
    }

    //---------------------------------------------------------------------------------

    if (((HAL_GetTick() - timeKeeper1) >= 5000) && (isDEBUG == true) && (isUpdating == false))
    {
      //Detail Log of ADC, VDDA and Temperature

      // memset(tempbuf, '\0', sizeof(tempbuf));
      // sprintf(tempbuf, "ADC raw value:\t[0] = [%li]\t[1] = [%li]\t[2] = [%li]\t[3] = [%li]\t[4] = [%li]\r\n", ADC_raw_value[10][0], ADC_raw_value[10][1], ADC_raw_value[10][2], ADC_raw_value[10][3], ADC_raw_value[10][4]);
      // CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      // timeKeeper = HAL_GetTick();
      // while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));

      memset(tempbuf, '\0', sizeof(tempbuf));
      sprintf(tempbuf, "\r\n");
      CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      timeKeeper = HAL_GetTick();
      while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));

      memset(tempbuf, '\0', sizeof(tempbuf));
      sprintf(tempbuf, "ADC raw value:\t[0] = [%li]\tACS1 Input Voltage:\t[%li]mV\r\n", *Motor_1.MotorHandle.ADC_raw, Motor_1.MotorHandle.ADC_Voltage_mV);
      CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      timeKeeper = HAL_GetTick();
      while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));

      memset(tempbuf, '\0', sizeof(tempbuf));
      sprintf(tempbuf, "ADC raw value:\t[1] = [%li]\tACS2 Input Voltage:\t[%li]mV\r\n", *Motor_2.MotorHandle.ADC_raw, Motor_2.MotorHandle.ADC_Voltage_mV);
      CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      timeKeeper = HAL_GetTick();
      while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));

      // // chipTemperature = getChipTemperature(ADC_raw_value[ADC_BUFFER_LENGTH][2]);

      // memset(tempbuf, '\0', sizeof(tempbuf));
      // sprintf(tempbuf, "ADC raw value:\t[2] = [%li]\tChip temperature:\t[%2.2f]oC\r\n", ADC_raw_value[ADC_BUFFER_LENGTH][2], chipTemperature);
      // CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      // timeKeeper = HAL_GetTick();
      // while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));

      // // ADC_VREF_mV = (uint32_t)(VREFINT * ADC_RESOLUTION * 1000 / ADC_raw_value[ADC_BUFFER_LENGTH][3]);

      // memset(tempbuf, '\0', sizeof(tempbuf));
      // sprintf(tempbuf, "ADC raw value:\t[3] = [%li]\tVDD Voltage Calibrated:\t[%li]mV\r\n", ADC_raw_value[ADC_BUFFER_LENGTH][3], ADC_VREF_mV);
      // CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      // timeKeeper = HAL_GetTick();
      // while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));

      // // ADC_VIN = (uint32_t)(ADC_raw_value[ADC_BUFFER_LENGTH][4] * 4 * ADC_VREF_mV / ADC_RESOLUTION);

      // memset(tempbuf, '\0', sizeof(tempbuf));
      // sprintf(tempbuf, "ADC raw value:\t[4] = [%li]\tVCC Voltage Calibrated:\t[%li]mV\r\n", ADC_raw_value[ADC_BUFFER_LENGTH][4], ADC_VIN);
      // CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      // timeKeeper = HAL_GetTick();
      // while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));

      // pMotor = (PIHanlde_t*) MOTOR_1_FLASH_ADDR;
      // memset(tempbuf, '\0', sizeof(tempbuf));
      // sprintf(tempbuf, "Test:\t[0x%08X]\r\n", &pMotor->MotorHandle.conversionRatio_TickpRotation);
      // CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      // timeKeeper = HAL_GetTick();
      // while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));

      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      timeKeeper1 = HAL_GetTick();
    }
    
    if ((checkLog1 == 5) && (isAppConnected == true) && (isUpdating == false))
    {
      checkLog1 = 0;

      logData(&Motor_1, R_CUR_mA);
      logData(&Motor_1, R_SPD_RPM);
      logData(&Motor_1, R_Duty);
      logData(&Motor_1, R_VCC_mV);
      logData(&Motor_1, R_ENA);

      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

      // if (DEBUG)
      // {
      //   memset(tempbuf, '\0', sizeof(tempbuf));
      //   sprintf(tempbuf, "Speed Motor 1:\t[%li] RPM\r\n", Motor_1.MotorHandle.Encoder_Speed_RPM);
      //   CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      //   timeKeeper = HAL_GetTick();
      //   while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));
      // }
    }

    if ((checkLog2 == 5) && (isAppConnected == true) && (isUpdating == false))
    {
      checkLog2 = 0;

      logData(&Motor_2, R_CUR_mA);
      logData(&Motor_2, R_SPD_RPM);
      logData(&Motor_2, R_Duty);
      logData(&Motor_2, R_VCC_mV);
      logData(&Motor_2, R_ENA);

      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

      // if (DEBUG)
      // {
      //   memset(tempbuf, '\0', sizeof(tempbuf));
      //   sprintf(tempbuf, "Speed Motor 2:\t[%li] RPM\r\n", Motor_2.MotorHandle.Encoder_Speed_RPM);
      //   CDC_Transmit_FS((uint8_t*)tempbuf, strlen(tempbuf));
      //   timeKeeper = HAL_GetTick();
      //   while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));
      // }
    }
  
    if (isNewUSBPacket == true)
    {
      uint16_t check = CheckSumCrc16(USB_RX_BUFFER, 6);
      if ((USB_RX_BUFFER[0] != 0xAB) && (USB_RX_BUFFER[8] != 0xBA))
      {
        //error in frame - request resend
        logData(NULL, W_req_retry);
      }
      else if ((USB_RX_BUFFER[6] != (uint8_t)((check & 0xFF00) >> 8)) || (USB_RX_BUFFER[7] != (uint8_t)(check & 0x00FF)))
      {
        //error in CRC - data corrupted - request resend
        logData(NULL, W_req_retry);
      }
      else
      {
        //no error - parse data
        updateData();
        logData(NULL, W_ack);
      }
      isNewUSBPacket = false;
    }
  
    if (isLogSetting == true)
    {
      // memcpy(dummy1, (uint8_t*)MOTOR_1_FLASH_ADDR, sizeof(Motor_1));
      // memcpy(dummy2, (uint8_t*)MOTOR_2_FLASH_ADDR, sizeof(Motor_2));

      pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
      logData(pMotor, R_Kp_R);
      logData(pMotor, R_Ki_R);
      logData(pMotor, R_Kp_C);
      logData(pMotor, R_Ki_C);
      logData(pMotor, R_REF_RPM);
      logData(pMotor, R_Ts);
      logData(pMotor, R_conv_mVpA);
      logData(pMotor, R_conv_TpR);

      pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
      logData(pMotor, R_Kp_R);
      logData(pMotor, R_Ki_R);
      logData(pMotor, R_Kp_C);
      logData(pMotor, R_Ki_C);
      logData(pMotor, R_REF_RPM);
      logData(pMotor, R_Ts);
      logData(pMotor, R_conv_mVpA);
      logData(pMotor, R_conv_TpR);

      isLogSetting = false;
      isAppConnected = true;
    }

    if ((HAL_GetTick() - timeKeeper2) >= 100)
    {
      // ADC_VIN = (uint32_t)(ADC_raw_value[ADC_BUFFER_LENGTH][4] * 7.2686 * ADC_VREF_mV / ADC_RESOLUTION);
      Motor_1.MotorHandle.Vcc_mV = ADC_VIN;
      Motor_2.MotorHandle.Vcc_mV = ADC_VIN;
      timeKeeper2 = HAL_GetTick();
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

int32_t runCurrent_LPF(LPF_t *pLPF, int32_t input)
{
	//Read LPF input
	pLPF->uk = (float)input;	
	//Compute LPF output
	pLPF->yk = -pLPF->b1 * pLPF->yk_1 - pLPF->b2 * pLPF->yk_2 + pLPF->a1 * pLPF->uk + pLPF->a2 * pLPF->uk_1; 
	//Save LPF past data
	pLPF->yk_2 = pLPF->yk_1; 
	pLPF->yk_1 = pLPF->yk; 
	pLPF->uk_1 = pLPF->uk;
	//Return LPF output
	return (int32_t)pLPF->yk;
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

uint16_t digitalFilter(uint32_t* buf, uint8_t len, double alpha)
{
  uint32_t y_now = 0;
  uint32_t y_pre = buf[0];
  for (uint8_t i = 1; i < len; i++)
  {
    y_now = alpha * buf[i] + (1.0-alpha) * y_pre;
    y_pre = buf[i];
  }
  return (uint16_t)(y_now);
}

void invertArray(void)
{
  for (uint8_t i = 0; i < ADC_BUFFER_LENGTH; i++)
  {
    ADC_raw_Inverted[0][i] = ADC_raw_value[i][0];
    ADC_raw_Inverted[1][i] = ADC_raw_value[i][1];
    ADC_raw_Inverted[2][i] = ADC_raw_value[i][2];
    ADC_raw_Inverted[3][i] = ADC_raw_value[i][3];
    ADC_raw_Inverted[4][i] = ADC_raw_value[i][4];
  }
}

//---------------------------------------------------------------------------------------------------

bool logData(PIHanlde_t* motor, uint8_t codec)
{
  uint8_t LogBuffer[9] = {0};

  LogBuffer[0] = 0xAB;

  if ((motor == &Motor_2) || (motor == (PIHanlde_t*)MOTOR_2_FLASH_ADDR))
  {
    //ID = 1
    LogBuffer[1] |= (0x01 << 7);
  }
  // READ operation = 0
  LogBuffer[1] &= ~(0x01 << 6);
  // No error (yet)
  LogBuffer[1] &= ~(0x01 << 5);

  // WRITE operation = 1
  if (motor == NULL)
  {
    LogBuffer[1] |= (0x01 << 6);
    if (codec == W_req_retry)
      LogBuffer[1] |= (0x01 << 5); 
    else
      LogBuffer[1] &= ~(0x01 << 5);
  }

  // Codec 4bits
  for (uint8_t i = 0; i < 4; i++)
  {
    if (codec & (0x01 << i))
    {
      LogBuffer[1] |= (0x01 << i);
    }
    else
    {
      LogBuffer[1] &= ~(0x01 << i);
    }
  }

  switch (codec)
  {
    case R_CUR_mA: {
      memcpy(&LogBuffer[2], &motor->MotorHandle.ADC_Current_mA, sizeof(int32_t));
      break;
    }
    case R_SPD_RPM: {
      memcpy(&LogBuffer[2], &motor->MotorHandle.Encoder_Speed_RPM, sizeof(int32_t));
      break;
    }
    case R_Duty: {
      memcpy(&LogBuffer[2], &motor->MotorHandle.Duty, sizeof(float));
      break;
    }
    case R_VCC_mV: {
      memcpy(&LogBuffer[2], &motor->MotorHandle.Vcc_mV, sizeof(uint32_t));
      break;
    }
    case R_ENA: {
      memcpy(&LogBuffer[2], &motor->Enable, sizeof(bool));
      break;
    }
    case W_req_retry: {
      // retry dont need data!
      break;
    }
    case W_ack: {
      // ack dont need data~
      break;
    }
    case R_Kp_R: {
      memcpy(&LogBuffer[2], &motor->Kp_Rotation, sizeof(float));
      break;
    }
    case R_Ki_R: {
      memcpy(&LogBuffer[2], &motor->Ki_Rotation, sizeof(float));
      break;
    }
    case R_Kp_C: {
      memcpy(&LogBuffer[2], &motor->Kp_Current, sizeof(float));
      break;
    }
    case R_Ki_C: {
      memcpy(&LogBuffer[2], &motor->Ki_Current, sizeof(float));
      break;
    }
    case R_REF_RPM: {
      memcpy(&LogBuffer[2], &motor->Ref_Speed_RPM, sizeof(int32_t));
      break;
    }
    case R_Ts: {
      memcpy(&LogBuffer[2], &motor->Ts, sizeof(uint32_t));
      break;
    }
    case R_conv_mVpA: {
      memcpy(&LogBuffer[2], &motor->MotorHandle.conversionRatio_mVpA, sizeof(uint8_t));
      break;
    }
    case R_conv_TpR: {
      memcpy(&LogBuffer[2], &motor->MotorHandle.conversionRatio_TickpRotation, sizeof(uint16_t));
      break;
    }
    default:  return false;
  }

  uint16_t checksum = CheckSumCrc16(LogBuffer, 6);

  LogBuffer[6] = (uint8_t)((checksum & 0xFF00) >> 8);
  LogBuffer[7] = (uint8_t)(checksum & 0x00FF);
  LogBuffer[8] = 0xBA;

  CDC_Transmit_FS((uint8_t*)LogBuffer, 9);
  timeKeeper = HAL_GetTick();
  while ((isUSBFree != true) && ((HAL_GetTick() - timeKeeper) <= LOG_DELAY_MS));

  return true;
}

bool updateData(void)
{
  uint8_t ID = (uint8_t)((USB_RX_BUFFER[1] & (0x01 << offset_ID)) >> offset_ID);
  // uint8_t RW = (uint8_t)((USB_RX_BUFFER[1] & (0x01 << offset_RW)) >> offset_RW);
  uint8_t codec = (uint8_t)(USB_RX_BUFFER[1] & 0x0F);

  switch (codec)
  {
    case(W_log): {
      if (USB_RX_BUFFER[2] == 0x01)
        isDEBUG = true;
      else
        isDEBUG = false;
      break;
    }
    case(W_APP_CON): {
      isLogSetting = true;
      break;
    }
    case(W_APP_DIS): {
      isAppConnected = false;
      isDEBUG = false;
      break;
    }
    case(W_Kp_R): {
      isUpdating = true;
      prepareSaveSpace();
      if (ID == 0)
      {
        pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
        memcpy(&dummy1[(uint32_t)(&pMotor->Kp_Rotation) - MOTOR_1_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(float));
      }
      else
      {
        pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
        memcpy(&dummy2[(uint32_t)(&pMotor->Kp_Rotation) - MOTOR_2_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(float));
      }
      break;
    }
    case(W_Ki_R): {
      if (ID == 0)
      {
        pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
        memcpy(&dummy1[(uint32_t)(&pMotor->Ki_Rotation) - MOTOR_1_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(float));
      }
      else
      {
        pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
        memcpy(&dummy2[(uint32_t)(&pMotor->Ki_Rotation) - MOTOR_2_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(float));
      }
      break;
    }
    case(W_Kp_C): {
      if (ID == 0)
      {
        pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
        memcpy(&dummy1[(uint32_t)(&pMotor->Kp_Current) - MOTOR_1_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(float));
      }
      else
      {
        pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
        memcpy(&dummy2[(uint32_t)(&pMotor->Kp_Current) - MOTOR_2_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(float));
      }
      break;
    }
    case(W_Ki_C): {
      if (ID == 0)
      {
        pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
        memcpy(&dummy1[(uint32_t)(&pMotor->Ki_Current) - MOTOR_1_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(float));
      }
      else
      {
        pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
        memcpy(&dummy2[(uint32_t)(&pMotor->Ki_Current) - MOTOR_2_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(float));
      }
      break;
    }
    case(W_REF_RPM): {
      if (ID == 0)
      {
        pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
        memcpy(&dummy1[(uint32_t)(&pMotor->Ref_Speed_RPM) - MOTOR_1_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(int32_t));
      }
      else
      {
        pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
        memcpy(&dummy2[(uint32_t)(&pMotor->Ref_Speed_RPM) - MOTOR_2_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(int32_t));
      }
      break;
    }
    case(W_Ts): {
      if (ID == 0)
      {
        pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
        memcpy(&dummy1[(uint32_t)(&pMotor->Ts) - MOTOR_1_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(uint32_t));
        uint32_t t = 0;
        memcpy((uint8_t*)&t, &USB_RX_BUFFER[2], sizeof(uint32_t));
        t = (uint32_t)(t / 1000);
        memcpy(&dummy1[(uint32_t)(&pMotor->Current_LPFHandle.Ts) - MOTOR_1_FLASH_ADDR], (uint8_t*)&t, sizeof(uint32_t));
        memcpy(&dummy1[(uint32_t)(&pMotor->Rotation_LPFHandle.Ts) - MOTOR_1_FLASH_ADDR], (uint8_t*)&t, sizeof(uint32_t));
      }
      else
      {
        pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
        memcpy(&dummy2[(uint32_t)(&pMotor->Ts) - MOTOR_2_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(uint32_t));
        uint32_t t = 0;
        memcpy((uint8_t*)&t, &USB_RX_BUFFER[2], sizeof(uint32_t));
        t = (uint32_t)(t / 1000);
        memcpy(&dummy2[(uint32_t)(&pMotor->Current_LPFHandle.Ts) - MOTOR_2_FLASH_ADDR], (uint8_t*)&t, sizeof(uint32_t));
        memcpy(&dummy2[(uint32_t)(&pMotor->Rotation_LPFHandle.Ts) - MOTOR_2_FLASH_ADDR], (uint8_t*)&t, sizeof(uint32_t));
      }
      break;
    }
    case(W_conv_mVpA): {
      if (ID == 0)
      {
        pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
        memcpy(&dummy1[(uint32_t)(&pMotor->MotorHandle.conversionRatio_mVpA) - MOTOR_1_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(uint8_t));
      }
      else
      {
        pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
        memcpy(&dummy2[(uint32_t)(&pMotor->MotorHandle.conversionRatio_mVpA) - MOTOR_2_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(uint8_t));
      }
      break;
    }
    case(W_conv_TpR): {
      if (ID == 0)
      {
        pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
        memcpy(&dummy1[(uint32_t)(&pMotor->MotorHandle.conversionRatio_TickpRotation) - MOTOR_1_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(uint16_t));
      }
      else
      {
        pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
        memcpy(&dummy2[(uint32_t)(&pMotor->MotorHandle.conversionRatio_TickpRotation) - MOTOR_2_FLASH_ADDR], &USB_RX_BUFFER[2], sizeof(uint16_t));
      }

      //save data
      pMotor = (PIHanlde_t*)MOTOR_1_FLASH_ADDR;
      for (uint8_t i = 0; i < 5; i++)
      {
        Flash_Write_Data(MOTOR_1_FLASH_ADDR, (uint32_t*)&dummy1, sizeof(Motor_1)/4);
      }
      memcpy(&dummy1[(uint32_t)(&pMotor->MotorHandle.ACS_MidPointVoltage_mV) - MOTOR_1_FLASH_ADDR], &Motor_1.MotorHandle.ACS_MidPointVoltage_mV, sizeof(uint16_t));
      pMotor = (PIHanlde_t*)MOTOR_2_FLASH_ADDR;
      for (uint8_t i = 0; i < 5; i++)
      {
        Flash_Write_Data(MOTOR_2_FLASH_ADDR, (uint32_t*)&dummy2, sizeof(Motor_2)/4);
      }
      memcpy(&dummy2[(uint32_t)(&pMotor->MotorHandle.ACS_MidPointVoltage_mV) - MOTOR_2_FLASH_ADDR], &Motor_2.MotorHandle.ACS_MidPointVoltage_mV, sizeof(uint16_t));

      //reload data
      memcpy(&Motor_1, &dummy1, sizeof(Motor_1));
      memcpy(&Motor_2, &dummy2, sizeof(Motor_2));

      Motor_1.MotorHandle.ADC_raw = &ADC_raw_value[ADC_BUFFER_LENGTH][0];
      Motor_1.PWM_Timer = &htim2;
      Motor_1.PWM_Channel = TIM_CHANNEL_1;
      Motor_1.FB_Encoder_Timer = &htim3;

      Motor_2.MotorHandle.ADC_raw = &ADC_raw_value[ADC_BUFFER_LENGTH][1];
      Motor_2.PWM_Timer = &htim2;
      Motor_2.PWM_Channel = TIM_CHANNEL_2;
      Motor_2.FB_Encoder_Timer = &htim4;

      isUpdating = false;
      break;
    }
    case(W_ENA): {
      if (ID == 0)
      {
        if (USB_RX_BUFFER[2] == 0x01)
          Motor_1.Enable = true;
        else
          Motor_1.Enable = false;
      }
      else
      {
        if (USB_RX_BUFFER[2] == 0x01)
          Motor_2.Enable = true;
        else
          Motor_2.Enable = false;
      }
      break;
    }
    default:  break;
  }
  return true;
}

//---------------------------------------------------------------------------------------------------

int32_t getRotationSpeed(PIHanlde_t* motor)
{
  motor->MotorHandle.Encoder_Speed_RPM = (int32_t)(motor->MotorHandle.Encoder_Speed_TickpSec * 60 / motor->MotorHandle.conversionRatio_TickpRotation);
  return (int32_t)(motor->MotorHandle.Encoder_Speed_RPM);
}

uint32_t getCurrent(PIHanlde_t* motor)
{
  motor->MotorHandle.ADC_Voltage_mV = (uint32_t)((*motor->MotorHandle.ADC_raw * ADC_VREF_mV / 4095.0) / ACS_ADC_RATIO);
  motor->MotorHandle.ADC_Current_A = (float)((float)((float)motor->MotorHandle.ADC_Voltage_mV - (float)motor->MotorHandle.ACS_MidPointVoltage_mV) / motor->MotorHandle.conversionRatio_mVpA);
  motor->MotorHandle.ADC_Current_mA = (int32_t)(motor->MotorHandle.ADC_Current_A * 1000.0);
  if (motor->MotorHandle.ADC_Current_mA > 5000)
    motor->MotorHandle.ADC_Current_mA = 0;
  return (uint32_t)(motor->MotorHandle.ADC_Current_mA);
}

float runPIControl(PIHanlde_t* motor)
{
  //error of rotation speed RPM
  motor->e_wt[0] = motor->Ref_Speed_RPM - motor->MotorHandle.Encoder_Speed_RPM;
  motor->vP_w = motor->Kp_Rotation * motor->e_wt[0];
  motor->vI_wt[0] = (motor->Ki_Rotation * motor->Ts * (motor->e_wt[0] + motor->e_wt[1]) / 2000.0) + motor->vI_wt[1];
  //current set point mA
  motor->vOut_w = motor->vP_w + motor->vI_wt[0];

  //error of motor's current mA
  // motor->e_It[0] = motor->vOut_w - motor->MotorHandle.ADC_Current_mA;
  // motor->vP_I = motor->Kp_Current * motor->e_It[0];
  // motor->vI_It[0] = (motor->Ki_Current * motor->Ts * (motor->e_It[0] - motor->e_It[1]) / 2.0) + motor->vI_It[1];
  // //voltage set point mV
  // motor->vOut_I = motor->vP_I + motor->vI_It[0];

  motor->vOut_I = motor->vOut_w;

  //Duty calculation
  motor->MotorHandle.Duty = (float)(motor->vOut_I / motor->MotorHandle.Vcc_mV);

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

//---------------------------------------------------------------------------------------------------

float getChipTemperature(uint32_t rawADCtemp)
{
  return (float)((((((float)rawADCtemp/ADC_RESOLUTION) * (float)(ADC_VREF_mV/1000.0)) - V25) / AVG_SLOPE) + 25.0);
}

//---------------------------------------------------------------------------------------------------

uint32_t prepareSaveSpace(void)
{

  // memset(dummy1, '\0', sizeof(dummy1));
  memcpy(dummy1, (uint8_t*)MOTOR_1_FLASH_ADDR, sizeof(Motor_1));
  // memset(dummy2, '\0', sizeof(dummy2));
  memcpy(dummy2, (uint8_t*)MOTOR_2_FLASH_ADDR, sizeof(Motor_2));

  /* Unlock the Flash to enable the flash control register access *************/
  while (HAL_FLASH_Unlock() != HAL_OK);

  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = 5;
  EraseInitStruct.NbSectors = 1;
  if(HAL_FLASHEx_Erase(&EraseInitStruct, NULL) != HAL_OK)
  { 
   /* FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    // Error_Handler();
    return 1;
  }
  
  while (HAL_FLASH_Lock() != HAL_OK);
  return 0;
}

uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{

	int count = 0;

  // while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);

  /* Unlock the Flash to enable the flash control register access *************/
  while (HAL_FLASH_Unlock() != HAL_OK);

  /* Program the user Flash area word by word*/
  for (uint8_t i = 0; i < 5; i++)
  {
    while (count < numberofwords)
    {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress, (uint64_t)Data[count]) == HAL_OK)
      {
        StartPageAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
        count++;
      }
      else
      {
        /* Error occurred while writing data in Flash memory*/
        return HAL_FLASH_GetError();
      }
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
  while(HAL_FLASH_Lock() != HAL_OK);

  return 0;
}

//---------------------------------------------------------------------------------------------------

void runEncoderTickHandle(PIHanlde_t* motor)
{
    motor->MotorHandle.Encoder_Counter_Tick = (uint16_t)(__HAL_TIM_GET_COUNTER(motor->FB_Encoder_Timer));
    
    if (motor->MotorHandle.Encoder_Counter_Tick <= 35 && motor->MotorHandle.Encoder_Counter_preTick >= 65500)
    {
      //overflow
      motor->MotorHandle.encoderValue += (65536 + motor->MotorHandle.Encoder_Counter_Tick - motor->MotorHandle.Encoder_Counter_preTick);
    }
    else if (motor->MotorHandle.Encoder_Counter_Tick >= 65500 && motor->MotorHandle.Encoder_Counter_preTick <= 35)
    {
      //underflow
      motor->MotorHandle.encoderValue -= (65536 - motor->MotorHandle.Encoder_Counter_Tick + motor->MotorHandle.Encoder_Counter_preTick);
    }
    else if (motor->MotorHandle.Encoder_Counter_Tick >= motor->MotorHandle.Encoder_Counter_preTick)
    {
      motor->MotorHandle.encoderValue += (int32_t)((motor->MotorHandle.Encoder_Counter_Tick - motor->MotorHandle.Encoder_Counter_preTick));
    }
    else if (motor->MotorHandle.Encoder_Counter_Tick <= motor->MotorHandle.Encoder_Counter_preTick)
    {
      motor->MotorHandle.encoderValue -= (int32_t)((motor->MotorHandle.Encoder_Counter_preTick - motor->MotorHandle.Encoder_Counter_Tick));
    }

    motor->MotorHandle.Encoder_Counter_preTick = motor->MotorHandle.Encoder_Counter_Tick;
}

//---------------------------------------------------------------------------------------------------

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
    HAL_IWDG_Refresh(&hiwdg);
    // invertArray();
    // ADC_raw_value[ADC_BUFFER_LENGTH][0] = digitalFilter(&ADC_raw_Inverted[0], ADC_BUFFER_LENGTH, ALPHA);
    // ADC_raw_value[ADC_BUFFER_LENGTH][1] = digitalFilter(&ADC_raw_Inverted[1], ADC_BUFFER_LENGTH, ALPHA);
    // ADC_raw_value[ADC_BUFFER_LENGTH][4] = digitalFilter(&ADC_raw_Inverted[4], ADC_BUFFER_LENGTH, ALPHA);
  }

  if (htim->Instance == TIM9)
  {
    encoderIndex++;

    runEncoderTickHandle(&Motor_1);
    runEncoderTickHandle(&Motor_2);

    if (encoderIndex >= 1000)
    {
      encoderIndex = 0;

      Motor_1.MotorHandle.Encoder_Speed_TickpSec = (int32_t)(Motor_1.MotorHandle.encoderValue * 10 / 4);
      Motor_1.MotorHandle.encoderValue = 0;

      Motor_2.MotorHandle.Encoder_Speed_TickpSec = (int32_t)(Motor_2.MotorHandle.encoderValue * 10 / 4);
      Motor_2.MotorHandle.encoderValue = 0;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  ADC_raw_value_index++;
  if (((ADC_raw_value_index % NUMBER_OF_ADC_CHANNEL) == 0) && (ADC_raw_value_index < (NUMBER_OF_ADC_CHANNEL * ADC_BUFFER_LENGTH)))
  {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, ADC_raw_value[(ADC_raw_value_index + 1) / NUMBER_OF_ADC_CHANNEL], NUMBER_OF_ADC_CHANNEL);
  }
  
  if (ADC_raw_value_index >= (NUMBER_OF_ADC_CHANNEL * ADC_BUFFER_LENGTH))
  {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, ADC_raw_value[0], NUMBER_OF_ADC_CHANNEL);
    ADC_raw_value_index = 0;
    invertArray();
    ADC_raw_value[ADC_BUFFER_LENGTH][0] = digitalFilter(&ADC_raw_Inverted[0], ADC_BUFFER_LENGTH, ALPHA) + 60;
    ADC_raw_value[ADC_BUFFER_LENGTH][1] = digitalFilter(&ADC_raw_Inverted[1], ADC_BUFFER_LENGTH, ALPHA) + 60;
    ADC_raw_value[ADC_BUFFER_LENGTH][4] = digitalFilter(&ADC_raw_Inverted[4], ADC_BUFFER_LENGTH, ALPHA) + 10;
  }
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
