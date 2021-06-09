/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
//  TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

//void EXTI0_IRQHandler(void){
//	TIM_Cmd(TIM3, DISABLE);
//	TIM_SetCounter(TIM3, 0);
//	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) != Bit_RESET){ //channelB leads
//		if (mainMotor.direction == 1) mainMotor.channel_A = 0;
//		mainMotor.direction = -1;
//	}
//	else if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET){ //channelA leads
//		if (mainMotor.direction == -1) mainMotor.channel_A = 0;
//		mainMotor.direction = 1;
//	}
//	mainMotor.channel_A++;
//	EXTI->PR = EXTI_Line0;
//}
int diff=1;
void DMA1_Stream1_IRQHandler(void){
	char checksum_Rx = 0;
	DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	if(RXBuffer[0]=='$' && RXBuffer[1]=='S' && RXBuffer[2]=='P' && RXBuffer[3]=='E' && RXBuffer[4]=='E' && RXBuffer[5]=='D' && RXBuffer[6]==','){
		for(int k = 0; k < 4; k++){
			m_data.mybyte[3-k] = RXBuffer[7+k];
			checksum_Rx += RXBuffer[7+k];
		}
		if(checksum_Rx == RXBuffer[11])
			if(m_data.myfloat<=360 && m_data.myfloat>=-360)
				mainMotor.rr = m_data.myfloat;
	} else if(RXBuffer[0]=='$' && RXBuffer[1]=='S' && RXBuffer[2]=='e' && RXBuffer[3]=='t' && RXBuffer[4]=='_' && RXBuffer[6]==','){
		//set Kp, Ki, Kd, T //"$Set_P,"
		for(int i = 0; i < 4; i++){
			m_data.mybyte[3-i] = RXBuffer[7+i];
			checksum_Rx += RXBuffer[7+i];
		}
		if(checksum_Rx == RXBuffer[11]){
			switch(RXBuffer[5]){
				case 'P':
					PID.Kp = (m_data.myfloat >= 0)? m_data.myfloat:PID.Kp;
					mras_lyapunov.Kp = (m_data.myfloat >= 0)? m_data.myfloat:mras_lyapunov.Kp;
					mras_grad.Kp = (m_data.myfloat >= 0)? m_data.myfloat:mras_grad.Kp;
					Kp_set = (m_data.myfloat >= 0)? m_data.myfloat:Kp_set;
					break;
				case 'I':
					PID.Ki = (m_data.myfloat >= 0)? m_data.myfloat:PID.Ki;
					mras_lyapunov.Ki = (m_data.myfloat >= 0)? m_data.myfloat:mras_lyapunov.Ki;
					mras_grad.Ki = (m_data.myfloat >= 0)? m_data.myfloat:mras_grad.Ki;
					Ki_set = (m_data.myfloat >= 0)? m_data.myfloat:Ki_set;
					break;
				case 'D':
					PID.Kd = (m_data.myfloat >= 0)? m_data.myfloat:PID.Kd;
					mras_lyapunov.Kd = (m_data.myfloat >= 0)? m_data.myfloat:mras_lyapunov.Kd;
					mras_grad.Kd = (m_data.myfloat >= 0)? m_data.myfloat:mras_grad.Kd;
					Kd_set = (m_data.myfloat >= 0)? m_data.myfloat:Kd_set;
					break;
				case '1':
					mras_lyapunov.gamma1 = (m_data.myfloat >= 0)? m_data.myfloat:mras_lyapunov.gamma1;
					mras_grad.gamma1 = (m_data.myfloat >= 0)? m_data.myfloat:mras_grad.gamma1;
					break;
				case '2':
					mras_lyapunov.gamma2 = (m_data.myfloat >= 0)? m_data.myfloat:mras_lyapunov.gamma2;
					mras_grad.gamma2 = (m_data.myfloat >= 0)? m_data.myfloat:mras_grad.gamma2;
					break;
				case '3':
					mras_lyapunov.gamma3 = (m_data.myfloat >= 0)? m_data.myfloat:mras_lyapunov.gamma3;
					mras_grad.gamma3 = (m_data.myfloat >= 0)? m_data.myfloat:mras_grad.gamma3;
					break;
				case 'O':
					mainMotor.measure_position = 0.0;
					zeroValue = (int)TIM5->CNT;
					gain = 0;
					break;
				case 'p':
					mainMotor.measure_position = m_data.myfloat;
					diff = (int)TIM5->CNT - zeroValue;
					gain = m_data.myfloat/diff;
					break;
				default:
					break;
			}
		}
	} else if(RXBuffer[0]=='$' && RXBuffer[1]=='M' && RXBuffer[2]=='o' && RXBuffer[3]=='d' && RXBuffer[4]=='e' && RXBuffer[6]==','){
		//Set mode
		for(int i = 0; i < 4; i++){
			m_data.mybyte[3-i] = RXBuffer[7+i];
			checksum_Rx += RXBuffer[7+i];
		}
		if(checksum_Rx == RXBuffer[11]){
			switch(RXBuffer[5]){
				case 'B'://mras_lyapunov_posi
					mode = 3;
					break;
				case 'A'://mras_lyapunov_speed
					mode = 2;
					break;
				case 'P'://posi
					mode = 1;
					break;
				case 'S'://pid_speed
					mode = 0;
					break;
				default:
					break;
			}
		}
	}
	DMA_Cmd(DMA1_Stream1, ENABLE);
}
int loop = 0;
void TIM3_IRQHandler(void){
	if (TIM_GetITStatus(TIM3, TIM_FLAG_Update) == SET){
	switch(mode){
			case 3://mras_lyapunov_posi
				break;
			case 4://mras_lyapunov_speed
				full_mras_lyapunov();
				break;
			case 2://mras_grad_speed
				full_mras_grad();
				break;
			case 1://posi
				full_pid();
				break;
			case 0://pid_speed
				full_pid();
				break;
			default:
				break;
		}
		TIM_ClearITPendingBit (TIM3, TIM_FLAG_Update);
	}
}






