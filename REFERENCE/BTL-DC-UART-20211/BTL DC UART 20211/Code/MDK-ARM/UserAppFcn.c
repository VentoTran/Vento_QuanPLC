#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;

LPF stLPF_Var;
QEI stQEI_Var,stQEI_Var1;
RAMP st_RAMP_Var;
cPI PI_Speed,PI_Current,PI_position;
controller_objt controller_var;
uint8_t u8_RxBuff[5];
adc_Objt adc_Objt_var;
Uart_Obj vUart;

uint16_t cnt=0;
uint8_t timtick100ms,timtick1s;
/**********************************************************************************************/
_ARMABI int fputc(int c, FILE *stream) 
{
	HAL_UART_Transmit(&huart2,(uint8_t *)&c,1,100);
	return 0;
}
/**********************************************************************************************/
void PI_Init(cPI *pPI, float TI, float KP, float Ts,double Umax,double Umin)
{
	pPI->Uk_1 = 0; pPI->Uk = 0; pPI->Usk_1 = 0;
	pPI->Ek = 0; pPI->Ek_1 = 0; 
	pPI->Esk = 0; pPI->Esk_1 = 0; 
	
	pPI->Umax = Umax; pPI->Umin = Umin;
	pPI->Ts = Ts;			//Sampling time (sec)
	pPI->TI = TI;			//Integral time (sec)
	pPI->D = 1 - pPI->Ts/pPI->TI;
	pPI->Kp = KP; 
	//pPI->Kp = 0.5*pPI->Umax/MaxSP;
	pPI->Ks = pPI->Kp;
}
/**********************************************************************************************/
float fPI_Run(cPI *pPI, float Ref, float Fb)
{
	pPI->Ek = Ref - Fb;
	pPI->Esk = pPI->Ek_1 + (1/pPI->Ks)*(pPI->Usk_1-pPI->Uk_1);
	pPI->Uk = pPI->Usk_1 +pPI->Kp*(pPI->Ek-pPI->D*pPI->Esk_1);
	
	if(pPI->Ek ==0 && pPI->Ek_1==0 && pPI->Usk_1<5 && pPI->Usk_1>-5)
	{
		pPI->Uk=0;
	}

	if(pPI->Uk > pPI->Umax)
			pPI->Usk = pPI->Umax;
	else if(pPI->Uk < pPI->Umin)
			pPI->Usk = pPI->Umin;
	else 
			pPI->Usk = pPI->Uk;
	//update data
	pPI->Ek_1 = pPI->Ek;
	pPI->Esk_1 = pPI->Esk;
	pPI->Uk_1 = pPI->Uk;
	pPI->Usk_1 = pPI->Usk;
	return pPI->Usk;
}
/**********************************************************************************************/
void Controller_Init(controller_objt *pcon)
{
	pcon->SP_position=0;
	pcon->SP_speed=0;
	pcon->SP_current=0;
	pcon->Cnt=64000;
	pcon->Cnt_1=0;
	pcon->Cnt_2=0;
	pcon->Duty=0;
	pcon->Duty1=0;
	pcon->Duty2=0;
	pcon->Ua=0;
	pcon->Uref=12.0;
}
/**********************************************************************************************/
void Position_controller(controller_objt *pcon,QEI *pQEI)
{
	pcon->SP_position_ramp = fRampFcnGen(pcon->SP_position,&st_RAMP_Var);
	pcon->SP_speed=fPI_Run(&PI_position, pcon->SP_position_ramp, pQEI->Cnt_rot);
}
/**********************************************************************************************/
void Speed_controller(controller_objt *pcon,QEI *pQEI)
{
	pcon->SP_current=fPI_Run(&PI_Speed, pcon->SP_speed, pQEI->Fb_rpm);
	pcon->Duty=pcon->SP_current/pcon->Uref;
	pcon->Duty1=(pcon->Duty+1)/2.0;

	pcon->Cnt_1=(pcon->Duty1*pcon->Cnt);
	pcon->Cnt_2=pcon->Cnt-pcon->Cnt_1;
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pcon->Cnt_1);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pcon->Cnt_2);
}
/**********************************************************************************************/
void Current_controller(controller_objt *pcon,adc_Objt *padc)
{
	pcon->Ua=fPI_Run(&PI_Current,pcon->SP_current , padc->Ia);
	pcon->Duty=pcon->Ua/pcon->Uref;
	pcon->Duty1=(pcon->Duty+1)/2.0;

	pcon->Cnt_1=(pcon->Duty1*pcon->Cnt);
	pcon->Cnt_2=pcon->Cnt-pcon->Cnt_1;
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pcon->Cnt_1);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,pcon->Cnt_2);
	
}
/**********************************************************************************************/
void vLPF_Init(LPF *pLPF, uint16_t ConnerFrqHz, float samplingTime)
{
	/*Low pass filter*/
	float temp1, temp2;
	
	pLPF->Fc = ConnerFrqHz;
	pLPF->Ts = samplingTime;
	pLPF->alpha = 1/(2*PI*pLPF->Ts*pLPF->Fc);
	
	temp1 = (1+pLPF->alpha); 
	temp2 = temp1*temp1;
	pLPF->a1 = (1+2*pLPF->alpha)/temp2; 
	pLPF->a2 = -2*pLPF->alpha/temp2;
	pLPF->b1 = -2*pLPF->alpha/temp1; 
	pLPF->b2 = pLPF->alpha*pLPF->alpha/temp2;
	
	pLPF->yk = 0;
	pLPF->yk_1 = 0; 
	pLPF->yk_2 = 0;
	pLPF->uk = 0;
	pLPF->uk_1 = 0;
}
/**********************************************************************************************/
float fLPF_Run(LPF *pLPF, float input)
{
	//Read LPF input
	pLPF->uk = input;	
	//Compute LPF output
	pLPF->yk = -pLPF->b1*pLPF->yk_1 - pLPF->b2*pLPF->yk_2 + pLPF->a1*pLPF->uk + pLPF->a2*pLPF->uk_1; 
	//Save LPF past data
	pLPF->yk_2 = pLPF->yk_1; 
	pLPF->yk_1 = pLPF->yk; 
	pLPF->uk_1 = pLPF->uk;
	//Return LPF output
	return pLPF->yk;
}
/**********************************************************************************************/
void vAdc_DataInit(adc_Objt *pObjt)			//Initialize adc data
{
	pObjt->data[0]=0;
	pObjt->Ia=0;
	pObjt->rawVolIa=0;
}
/**********************************************************************************************/
void vAdc_Data_Handle(adc_Objt *pObjt)
{
	pObjt->rawVolIa = 1.0*(3.3*pObjt->data[0]) /4095.0;	
	pObjt->filterVolIa = fLPF_Run(&stLPF_Var, pObjt->rawVolIa);
	pObjt->Ia=(pObjt->filterVolIa -2.495)/0.185;
}
/**********************************************************************************************/
void vQEI_Config(QEI *pQEI,float Ts)
{
	pQEI->FirstCnt = 0;			//To ignore the first resulst
	pQEI->Cnt_k = 0; 
	pQEI->Cnt_k_1 = 0;
	pQEI->Cnt_Ovf = 0;
	pQEI->Cnt_pulse=0;
	pQEI->ppr = 11*30*2;
	pQEI->PU_rpm = 0;
	pQEI->rpm = 0;
	pQEI->Ts = Ts;				//10ms
	pQEI->Direction = 0;		//Real Direction
	pQEI->Cnt_rot=0;
}
/**********************************************************************************************/
void vRPM_Cal(QEI *pQEI)
{
	//Detect the direction of the QEI
	pQEI->Direction = (TIM1->CR1>>4)&0x01;	
	//Read QEI Counter
	pQEI->Cnt_k = TIM1->CNT;
	//printf("dir=%d",pQEI->Direction);
	//Compute the rotation speed (rpm)
	if(pQEI->Direction == 0)	//Up counting - RUN Forward
	{
		if(pQEI->FirstCnt == 1)
		{
			if(pQEI->Cnt_Ovf == 0)
			{
				pQEI->rpm = ((float)(pQEI->Cnt_k-pQEI->Cnt_k_1)/pQEI->Ts)*(60.0/(float)(pQEI->ppr)); 
				
				pQEI->Cnt_pulse=pQEI->Cnt_pulse+(pQEI->Cnt_k-pQEI->Cnt_k_1);
			}
			else
			{
				//Note rpm must be equal to auto-reload value of QEI Timer
				pQEI->rpm =((float)(pQEI->Cnt_k+pQEI->Cnt_Ovf*65535-pQEI->Cnt_k_1)/pQEI->Ts)*(60.0/(float)(pQEI->ppr));
				
				pQEI->Cnt_pulse=pQEI->Cnt_pulse+(pQEI->Cnt_k+pQEI->Cnt_Ovf*65535-pQEI->Cnt_k_1);
				//Reset overflow counter
				pQEI->Cnt_Ovf  = 0;
			}
		  
		}
		else	//Ignore results in first cycle
		{
			pQEI->FirstCnt = 1;
			pQEI->rpm = 0;
			pQEI->PU_rpm = 0;
			pQEI->Cnt_Ovf  = 0;
		}
		pQEI->Fb_rpm= pQEI->rpm;
	}
	else if(pQEI->Direction == 1)	//Down-counting - RUN Reverse
	{
		if(pQEI->FirstCnt == 1)
		{
			if(pQEI->Cnt_Ovf == 0)
			{
				pQEI->rpm = ((float)(pQEI->Cnt_k_1-pQEI->Cnt_k)/pQEI->Ts)*(60.0/(float)(pQEI->ppr)); 
				pQEI->Cnt_pulse=pQEI->Cnt_pulse+(pQEI->Cnt_k-pQEI->Cnt_k_1);
			}
			else
			{
				//Note rpm must be equal to auto-reload value of QEI Timer
				pQEI->rpm = ((float)(pQEI->Cnt_k_1+pQEI->Cnt_Ovf*65535-pQEI->Cnt_k)/pQEI->Ts)*(60.0/(float)(pQEI->ppr)); 
				
				pQEI->Cnt_pulse=pQEI->Cnt_pulse-(pQEI->Cnt_k_1+pQEI->Cnt_Ovf*65535-pQEI->Cnt_k);
				//Reset overflow counter
				pQEI->Cnt_Ovf  = 0;
				
			}
		}
		else	//Ignore results in first cycle
		{
			pQEI->FirstCnt = 1;
			pQEI->rpm = 0;
			pQEI->PU_rpm = 0;
			pQEI->Cnt_Ovf  = 0;
		}
		pQEI->Fb_rpm = -pQEI->rpm;
	}
	pQEI->Cnt_rot=pQEI->Cnt_pulse/660;
	pQEI->Cnt_k_1 = pQEI->Cnt_k;
	
}

/**********************************************************************************************/
void vRamp_Config(RAMP *pRamp)
{
	pRamp->SPk = 0;
	pRamp->Output = 0;
	pRamp->delta_Acc=2.5;
	pRamp->delta_Dec=2.5;
}

/**********************************************************************************************/
float fRampFcnGen(int16_t SP, RAMP *pRamp)
{
	pRamp->SPk = SP;
	//Detect the state
	if(pRamp->SPk > pRamp->Output + pRamp->delta_Acc)
	{
		pRamp->State = ACC;
	}
	else if (pRamp->SPk < pRamp->Output - pRamp->delta_Dec)
	{
		pRamp->State = DEC;
	}
	else
	{
		pRamp->State = RUN;
	}
	//Generate output
	if(pRamp->State == ACC)
	{
		pRamp->Output = pRamp->Output + pRamp->delta_Acc;
		if(pRamp->Output >= pRamp->SPk)	//End the acceleration
		{
			pRamp->Output = pRamp->SPk;
			pRamp->State = RUN; 					
		}
	}
	else if(pRamp->State == DEC)
	{
		pRamp->Output = pRamp->Output - pRamp->delta_Dec;
		if(pRamp->Output <= pRamp->SPk)	//End the deceleration
		{
			pRamp->Output = pRamp->SPk;
			pRamp->State = RUN; 					
		}
	}
	else if(pRamp->State == RUN)
	{
		pRamp->Output = pRamp->SPk;
	}
	return pRamp->Output;
}
/**********************************************************************************************/
void UART(controller_objt *pcon,QEI *pQEI)
{
	printf("%.1f,%d,0,%d,%d,0,%.1f,0,0\r",pcon->SP_position_ramp,pQEI->Cnt_rot,pcon->SP_speed,pQEI->Fb_rpm,pcon->Duty);
}
/**********************************************************************************************/
void UART_SP(void)
{
	if(vUart.Rx_flag == 1){
			if(vUart.check_sum == (vUart.Rx_buff[1]+vUart.Rx_buff[2])){
			  vUart.dir = vUart.Rx_buff[1];
				vUart.round = vUart.Rx_buff[2];
				vUart.Tx_data[1] = Success;
				HAL_UART_Transmit(&huart2,vUart.Tx_data,3,100);
				for(int i =0 ; i < 5;i++){
					vUart.Rx_buff[i] = 0;
				}
				vUart.pointer = 0;
				vUart.Rx_flag = 0;
				vUart.state = Uart_Wait;
        controller_var.SP_position=	(1-2*vUart.dir)*vUart.round;		
		}
			else {
				vUart.Rx_flag_err =1;
				vUart.Rx_flag = 0;}	
  }
		if(vUart.Rx_flag_err ==1){
		  	vUart.Tx_data[1] = Fail;
				HAL_UART_Transmit(&huart2,vUart.Tx_data,3,100);
				for(int i =0 ; i < 5;i++){
					vUart.Rx_buff[i] = 0;
				}
				vUart.pointer = 0;
				vUart.Rx_flag_err = 0;
				vUart.state = Uart_Wait;	
			  vUart.cnt++;
		}
		if(vUart.cnt >5)
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
			// canh bao nguoi dung
			}
}
/**********************************************************************************************/
void Uart_Init(Uart_Obj *uart){
	uart->pointer =0;
	uart->state = Uart_Wait;
	uart->Rx_flag= 0;
	uart->Rx_flag_err =0;
	uart->Tx_data[0] = Frame_start;
	uart->Tx_data[2] = Frame_end;
	uart->cnt = 0;
	
	
}
/**********************************************************************************************/
void Uart_read(Uart_Obj *uart){
	switch(uart->state){
		case Uart_Wait:
			uart->Rx_buff[0] = uart->Rx_data;
			if(uart->Rx_buff[0] == 0X7A)
			{
				uart->state = Uart_Reading;
				uart->pointer = 1;
			}
			else {
				uart->Rx_flag_err =1;
			}
		 break;
		case Uart_Reading:
			uart->Rx_buff[uart->pointer] = uart->Rx_data;
			if( uart->pointer == 4){
				if(uart->Rx_data == 0x7f){
					uart->check_sum = uart->Rx_buff[3];
				  uart->Rx_flag =1;
				}
				else{
					uart->Rx_flag_err = 1;
		}
	}
			else{
		   	uart->pointer++;
				if(uart->pointer >4){
					uart->Rx_flag_err = 1;

				}
			}
			break;
}
	}
/**********************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);		
	/*USER CODE*/
	if(htim->Instance==TIM2)				//Timer 2 interrupt every 100ms
	{
		timtick100ms=1;
	}
	if(htim->Instance==TIM1)				
	{
		stQEI_Var.Cnt_Ovf = stQEI_Var.Cnt_Ovf+1;
		if(stQEI_Var.Cnt_Ovf > 65000)
		{
			stQEI_Var.Cnt_Ovf = 0;
		}
	}
	if(htim->Instance==TIM4)				//Timer 2 interrupt every 1s
	{
		timtick1s=1;

	}
}
/**********************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		if(huart->Instance == huart2.Instance){
			Uart_read(&vUart);
			HAL_UART_Receive_IT(&huart2,&vUart.Rx_data,1);
			
		}
		
	}