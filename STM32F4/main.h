/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
//#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void TimingDelay_Decrement(void);
extern void calculate_speed(void);
extern void My_GPIO_Init(void);
extern void My_TIMER_Init(void);
extern void My_PWM_Init(void);
extern void EXTILine_Config(void);
extern void USART_DMA_Configuration(unsigned int BaudRate);
extern void delay_ms(uint32_t milliSeconds);
extern void delay_us(uint32_t microSeconds);
extern void MyDelay(__IO uint32_t number);
extern int readEncoder(void);
extern void My_Encoder_Init(void);


#define MAIN_BAUDRATE 115200 //57600
#define MAIN_STOPBITS USART_StopBits_2
#define MAIN_PARITY USART_Parity_No //USART_Parity_Even
#define BUFF_SIZE_RX 14
#define BUFF_SIZE_TX 18
#define RPMmax 360
#define RPMmin -360

extern char RXBuffer[BUFF_SIZE_RX];
extern char TXBuffer[BUFF_SIZE_TX];
extern int preEncValue, encValue;
extern int mode;
extern float gain;
extern int zeroValue;
extern float Ts;
extern float Kp_set, Ki_set, Kd_set;
void setpoint_filter(void);
void speed_filter(int count);
void sendData(void);
void PID_control(void);
void reference_model(void);
void update_value(void);
void take_derivative_mras_lyapunov(void);
void pi_mras_lyapunov_control(void);
void full_mras_lyapunov(void);
void full_pid(void);
void calculate_del_mras_grad(void);
void pid_mras_grad_control(void);
void full_mras_grad(void);

struct motor_Values{
	float rr;
	float yr, yr_1, yr_2;
	float yt, yt_1, yt_2;
	float rt, rt_1, rt_2;
	float ym, ym_1, ym_2;
	float em, em_1, em_2;
	float et, et_1, et_2;
	float yt_dot, em_dot, et_dot;
	float yt_dot_1_order, em_dot_1_order, et_dot_1_order;
	float measure_position;
} extern mainMotor;
union ByteToFloat{
	float myfloat;
	char mybyte[4];
} extern m_data,m_datap;
struct pid_settings{
	float Kp, Ki, Kd, T, Output, Output_1, Error, Error_1, Error_2;
	float Kp_dot, Ki_dot1, Ki_dot2;
	double P_part, I_part, D_part;
} extern PID;

struct mras_lyapunov{
	float Kp, Ki, Kd;
	float gamma1, gamma2, gamma3;
	float Kp_dot, Ki_dot1, Ki_dot2;
	float Kp_dot_max, Ki_dot1_max, Ki_dot2_max;
	double P_part, I_part, D_part;
	float Output, Output_1;
} extern mras_lyapunov;

struct mras_grad{
	float Kp, Ki, Kd;
	float Kp_dot, Ki_dot, Kd_dot;
	float num_Kp_func[3], den_Kp_func[3];
	float num_Ki_func[3], den_Ki_func[3];
	float num_Kd_func[3], den_Kd_func[3];
	float del_Kp, del_Kp_1, del_Kp_2;
	float del_Ki, del_Ki_1, del_Ki_2;
	float del_Kd, del_Kd_1, del_Kd_2;
	float gamma1, gamma2, gamma3;
	double P_part, I_part, D_part;
	float Output, Output_1;
} extern mras_grad;


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
