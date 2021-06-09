#include "main.h"
USART_InitTypeDef				  USART_InitStruct;
GPIO_InitTypeDef 				  GPIO_InitStruct;
NVIC_InitTypeDef					NVIC_InitStruct;
EXTI_InitTypeDef					EXTI_InitStruct;
TIM_TimeBaseInitTypeDef 	TIM_BaseStruct;
TIM_OCInitTypeDef 				TIM_OCStruct;
DMA_InitTypeDef 					DMA_InitStructure;

int mode = 0;
char RXBuffer[BUFF_SIZE_RX];
char TXBuffer[BUFF_SIZE_TX] = {'$','$','D','L','C','N',',','S','P','E','D','P','O','S','I','0','\r','\n'};
union ByteToFloat m_data, m_datap;
struct motor_Values mainMotor = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
struct pid_settings PID = {15, 40, 0.05, 0.01, 4200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
struct mras_lyapunov mras_lyapunov = {15, 40, 0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 0, 0, 0, 0, 0};
struct mras_grad mras_grad = {15, 40, 0.05, 0, 0, 0,{0, 0.0800522, -0.0800522},{1, -1.9523332, 0.9531338},{0, 0.0004035, 0.0003971},{1, -1.9523332, 0.9531338},{8.2, -16.3966914, 8.1966914},{1, -1.9523332, 0.9531338},
															0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float speed = 0, pre_speed = 0, filtered_sp = 0, pre_filtered_sp = 0, gain = 0;
float Kp_set = 0, Ki_set = 0, Kd_set = 0;
int preEncValue = 0, encValue = 0;
int zeroValue = 0;
float Ts = 0.01;
float max = 0;
int main(void){
	SystemInit();
	USART_DMA_Configuration(MAIN_BAUDRATE);
	My_GPIO_Init();
	My_TIMER_Init();
	My_PWM_Init();
	My_Encoder_Init();
	TIM1->CCR1=4200;
	while(1){
		GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		delay_ms(100);
	}
}

int readEncoder(void){
	preEncValue = encValue;
	encValue = TIM5->CNT;
	if (encValue<1000&&preEncValue>65000)
		return encValue + (65536 - preEncValue);
	if (encValue>65000&&preEncValue<1000)
		return -(preEncValue + (65536 - encValue));
		return encValue - preEncValue; 
}

void PID_control(){
	PID.Output_1 = PID.Output;
	PID.Error_2 = PID.Error_1;
	PID.Error_1 = PID.Error;
	PID.Error =  mainMotor.ym - mainMotor.yt;
	PID.P_part = PID.Kp*(PID.Error - PID.Error_1);
	PID.I_part = PID.Ki*PID.T/2*(PID.Error + PID.Error_1);
	PID.D_part = PID.Kd/PID.T*(PID.Error - 2*PID.Error_1 + PID.Error_2);
	if (PID.Error<=1 && PID.Error>=-1){
		PID.Output = PID.Output;
	} else if(PID.Error>=1 || PID.Error<=-1){
		PID.Output = PID.Output_1 + PID.P_part + PID.I_part + PID.D_part;
	}
	if (mainMotor.yt<2 && PID.Output<4300 && mainMotor.ym>2)
		PID.Output = 4300;
	if (mainMotor.yt>-2 && PID.Output>4100 && mainMotor.ym<-2)
		PID.Output = 4100;
	if (PID.Output > 0.95*8400)
		PID.Output = 0.95*8400;
	else if (PID.Output < 0.05*8400)
		PID.Output = 0.05*8400;
	
	TIM1->CCR1 = (int)PID.Output;
}

void full_pid(void){
		update_value();
		int count = readEncoder();
		setpoint_filter();
		speed_filter(count);
		reference_model();
		if (mode != 1)
			mainMotor.measure_position += (float)count*360/(4*234);
		if (mode == 0 && gain != 0)
			mainMotor.measure_position += (float)count*gain;
		if (mainMotor.measure_position > 180.0)
			mainMotor.measure_position = -180.0 + (mainMotor.measure_position - 180.0);
		if (mainMotor.measure_position < -180.0)
			mainMotor.measure_position = 180.0 - (mainMotor.measure_position + 180.0);
		if (mode == 0 && mainMotor.ym != mainMotor.yt)
				PID_control();
		sendData();
}

void speed_filter(int count){
	mainMotor.yr = (float)count*60/(4*234)/Ts;
	//1-order
	mainMotor.yt = 0.0198*mainMotor.yr_1 + 0.9802*mainMotor.yt_1;
	//2-order
//	mainMotor.yt = 0.00008*mainMotor.yr_1 + 0.0008*mainMotor.yr_2 + 1.98237*mainMotor.yt_1 + 0.98253*mainMotor.yt_2;
	mainMotor.et = mainMotor.rt - mainMotor.yt;
}

void setpoint_filter(void){
	mainMotor.rt = 0.0952*mainMotor.rr + 0.9048*mainMotor.rt_1;
}

void sendData(){
	char checksum_Tx = 0;
	m_data.myfloat = mainMotor.yt;
	m_datap.myfloat = mainMotor.measure_position;
	for (int i = 0; i < 4; i++){
		TXBuffer[7+i] = m_data.mybyte[3-i];
		TXBuffer[11+i] = m_datap.mybyte[3-i];
		checksum_Tx += m_data.mybyte[3-i];
		checksum_Tx += m_datap.mybyte[3-i];
	}
	TXBuffer[15] = checksum_Tx;
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
  DMA1_Stream3->NDTR = BUFF_SIZE_TX;
  DMA_Cmd(DMA1_Stream3, ENABLE);
}

void reference_model(void){
	mainMotor.ym = 1.9523332*mainMotor.ym_1 - 0.9531338*mainMotor.ym_2 + 0.0004035*mainMotor.rt_1 + 0.0003971*mainMotor.rt_2;
	mainMotor.em = mainMotor.yt - mainMotor.ym;
}

void take_derivative_mras_lyapunov(void){
	mainMotor.yt_dot = (mainMotor.yt_2 - 4*mainMotor.yt_1 + 3*mainMotor.yt)/(2*Ts);
	mainMotor.em_dot = (mainMotor.em_2 - 4*mainMotor.em_1 + 3*mainMotor.em)/(2*Ts);
	mainMotor.et_dot = (mainMotor.et_2 - 4*mainMotor.et_1 + 3*mainMotor.et)/(2*Ts);
	mainMotor.yt_dot_1_order = (- mainMotor.yt_1 + mainMotor.yt)/Ts;
	mainMotor.em_dot_1_order = (- mainMotor.em_1 + mainMotor.em)/Ts;
	mainMotor.et_dot_1_order = (- mainMotor.et_1 + mainMotor.et)/Ts;
}

void pi_mras_lyapunov_control(void){
	mras_lyapunov.Ki_dot1 = - mras_lyapunov.gamma1 * mainMotor.em_dot * mainMotor.rt;
	mras_lyapunov.Ki_dot2 = - mras_lyapunov.gamma2 * mainMotor.em_dot * mainMotor.yt;
	mras_lyapunov.Kp_dot  = - mras_lyapunov.gamma3 * mainMotor.em_dot * mainMotor.yt_dot;
	
	if (mras_lyapunov.Ki_dot1<-mras_lyapunov.Ki_dot1_max) mras_lyapunov.Ki += Ts * (-mras_lyapunov.Ki_dot1_max);
		else if (mras_lyapunov.Ki_dot1>mras_lyapunov.Ki_dot1_max) mras_lyapunov.Ki += Ts * mras_lyapunov.Ki_dot1_max;
		else mras_lyapunov.Ki += Ts * mras_lyapunov.Ki_dot1;
	if (mras_lyapunov.Ki_dot2<-mras_lyapunov.Ki_dot2_max) mras_lyapunov.Ki += Ts * (-mras_lyapunov.Ki_dot2_max);
		else if (mras_lyapunov.Ki_dot1>mras_lyapunov.Ki_dot2_max) mras_lyapunov.Ki += Ts * mras_lyapunov.Ki_dot2_max;
		else mras_lyapunov.Ki += Ts * mras_lyapunov.Ki_dot2;
	if (mras_lyapunov.Kp_dot<-mras_lyapunov.Kp_dot_max) mras_lyapunov.Kp += Ts * (-mras_lyapunov.Kp_dot_max);
		else if (mras_lyapunov.Kp_dot>mras_lyapunov.Kp_dot_max) mras_lyapunov.Kp += Ts * mras_lyapunov.Kp_dot_max;
		else mras_lyapunov.Kp += Ts * mras_lyapunov.Kp_dot;
	
	if (mras_lyapunov.Ki<0) mras_lyapunov.Ki=0;
	if (mras_lyapunov.Kp<0) mras_lyapunov.Kp=0;
	
	mras_lyapunov.P_part = mras_lyapunov.Kp*(mainMotor.et - mainMotor.et_1);
	mras_lyapunov.I_part = mras_lyapunov.Ki*Ts/2*(mainMotor.et + mainMotor.et_1);
//	mras_lyapunov.D_part = mras_lyapunov.Kd/Ts*(mainMotor.et - 2*mainMotor.et_1 + mainMotor.et_2);
	
	if (mainMotor.et<=2.5 && mainMotor.et>=-2.5){
		mras_lyapunov.Output = mras_lyapunov.Output;
	} else if(mainMotor.et>1 || mainMotor.et<-1){
		mras_lyapunov.Output = mras_lyapunov.Output_1 + mras_lyapunov.P_part + mras_lyapunov.I_part;
//		+ mras_lyapunov.D_part;
	}
	if (mainMotor.yt<2 && mras_lyapunov.Output<4300 && mainMotor.rt>2)
		mras_lyapunov.Output = 4300;
	if (mainMotor.yt>-2 && mras_lyapunov.Output>4100 && mainMotor.rt<-2)
		mras_lyapunov.Output = 4100;
	if (mras_lyapunov.Output > 0.95*8400)
		mras_lyapunov.Output = 0.95*8400;
	else if (mras_lyapunov.Output < 0.05*8400)
		mras_lyapunov.Output = 0.05*8400;
	
	TIM1->CCR1 = (int)mras_lyapunov.Output;
}

void update_value(void){
	mainMotor.yr_2 = mainMotor.yr_1;
	mainMotor.yr_1 = mainMotor.yr;
	mainMotor.yt_2 = mainMotor.yt_1;
	mainMotor.yt_1 = mainMotor.yt;
	mainMotor.ym_2 = mainMotor.ym_1;
	mainMotor.ym_1 = mainMotor.ym;
	mainMotor.rt_2 = mainMotor.rt_1;
	mainMotor.rt_1 = mainMotor.rt;
	mainMotor.em_2 = mainMotor.em_1;
	mainMotor.em_1 = mainMotor.em;
	mainMotor.et_2 = mainMotor.et_1;
	mainMotor.et_1 = mainMotor.et;
	mras_lyapunov.Output_1 = mras_lyapunov.Output;
	mras_grad.del_Kp_2 = mras_grad.del_Kp_1;
	mras_grad.del_Kp_1 = mras_grad.del_Kp;
	mras_grad.del_Ki_2 = mras_grad.del_Ki_1;
	mras_grad.del_Ki_1 = mras_grad.del_Ki;
	mras_grad.del_Kd_2 = mras_grad.del_Kd_1;
	mras_grad.del_Kd_1 = mras_grad.del_Kd;
	mras_grad.Output_1 = mras_grad.Output;
}

void full_mras_lyapunov(void){
	update_value();
	int count = readEncoder();
	//calculate position
	mainMotor.measure_position += (float)count * 360 / (4 * 234);
	if (mainMotor.measure_position > 180.0)
		mainMotor.measure_position = - 180.0 + (mainMotor.measure_position - 180.0);
	if (mainMotor.measure_position < -180.0)
		mainMotor.measure_position = 180.0 - (mainMotor.measure_position + 180.0);
	setpoint_filter();
	speed_filter(count);
	reference_model();
	take_derivative_mras_lyapunov();
	if (mainMotor.rt != mainMotor.yt)
		pi_mras_lyapunov_control();
	
	sendData();
}

void calculate_del_mras_grad(void){
	mras_grad.del_Kp = mainMotor.et*mras_grad.num_Kp_func[0] + mainMotor.et_1*mras_grad.num_Kp_func[1] + mainMotor.et_2*mras_grad.num_Kp_func[2]
										- mras_grad.del_Kp_1*mras_grad.den_Kp_func[1] - mras_grad.del_Kp_2*mras_grad.den_Kp_func[2];
	mras_grad.del_Ki = mainMotor.et*mras_grad.num_Ki_func[0] + mainMotor.et_1*mras_grad.num_Ki_func[1] + mainMotor.et_2*mras_grad.num_Ki_func[2]
										- mras_grad.del_Ki_1*mras_grad.den_Ki_func[1] - mras_grad.del_Ki_2*mras_grad.den_Ki_func[2];
	mras_grad.del_Kd = mainMotor.et*mras_grad.num_Kd_func[0] + mainMotor.et_1*mras_grad.num_Kd_func[1] + mainMotor.et_2*mras_grad.num_Kd_func[2]
										- mras_grad.del_Kd_1*mras_grad.den_Kd_func[1] - mras_grad.del_Kd_2*mras_grad.den_Kd_func[2];
}

void pid_mras_grad_control(void){
	mras_grad.Kp_dot = - mras_grad.gamma1*mainMotor.em*mras_grad.del_Kp;
	mras_grad.Ki_dot = - mras_grad.gamma2*mainMotor.em*mras_grad.del_Ki;
	mras_grad.Kd_dot = - mras_grad.gamma3*mainMotor.em*mras_grad.del_Kd;
	
	mras_grad.Kp += Ts*mras_grad.Kp_dot;
	mras_grad.Ki += Ts*mras_grad.Ki_dot;
	mras_grad.Kd += Ts*mras_grad.Kd_dot;
	
	if (mras_grad.Ki<0) mras_grad.Ki=0;
	if (mras_grad.Kp<0) mras_grad.Kp=0;
	if (mras_grad.Kd<0) mras_grad.Kd=0; if (mras_grad.Kd>2.5) mras_grad.Kd=2.5;
	
	mras_grad.P_part = mras_grad.Kp*(mainMotor.et - mainMotor.et_1);
	mras_grad.I_part = mras_grad.Ki*Ts/2*(mainMotor.et + mainMotor.et_1);
	mras_grad.D_part = mras_grad.Kd/Ts*(mainMotor.et - 2*mainMotor.et_1 + mainMotor.et_2);
	
	if (mainMotor.et<=2.5 && mainMotor.et>=-2.5){
		mras_grad.Output = mras_grad.Output;
//		mras_grad.Kp = Kp_set;
//		mras_grad.Ki = Ki_set;
//		mras_grad.Kd = Kd_set;
	}	
	else if(mainMotor.et>2.5 || mainMotor.et<-2.5)
		mras_grad.Output = mras_grad.Output_1 + mras_grad.P_part + mras_grad.I_part + mras_grad.D_part;
	
	if (mainMotor.yt<2 && mras_grad.Output<4300 && mainMotor.rt>2)
		mras_grad.Output = 4300;
	if (mainMotor.yt>-2 && mras_grad.Output>4100 && mainMotor.rt<-2)
		mras_grad.Output = 4100;
	if (mras_grad.Output > 0.95*8400)
		mras_grad.Output = 0.95*8400;
	else if (mras_grad.Output < 0.05*8400)
		mras_grad.Output = 0.05*8400;
	
	TIM1->CCR1 = (int)mras_grad.Output;
}

void full_mras_grad(void){
	update_value();
	int count = readEncoder();
	//calculate position
	mainMotor.measure_position += (float)count * 360 / (4 * 234);
	if (mainMotor.measure_position > 180.0)
		mainMotor.measure_position = - 180.0 + (mainMotor.measure_position - 180.0);
	if (mainMotor.measure_position < -180.0)
		mainMotor.measure_position = 180.0 - (mainMotor.measure_position + 180.0);
	setpoint_filter();
	speed_filter(count);
	reference_model();
	calculate_del_mras_grad();
	if (mainMotor.rt != mainMotor.yt)
		pid_mras_grad_control();
	
	sendData();
}

void USART_DMA_Configuration(unsigned int BaudRate){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); 
	
	/* Khoi tao chân TX & RX Uart*/
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate = BaudRate;

	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = MAIN_STOPBITS;
	USART_InitStruct.USART_Parity = MAIN_PARITY;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStruct);

	USART_Cmd(USART3, ENABLE);
	/* Enable USART3 DMA */
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	/* Configure DMA Initialization Structure */
	
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull ;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART3->DR)) ;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;

	/* Configure TX DMA */
	DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_TX;
	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)TXBuffer ;
	DMA_Init(DMA1_Stream3,&DMA_InitStructure);
	DMA_Cmd(DMA1_Stream3, ENABLE);
	
	/* Configure RX DMA */
	DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_RX;
	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)&RXBuffer;
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);	
	DMA_Cmd(DMA1_Stream1, ENABLE);
	
	/* Enable DMA Interrupt to the highest priority */
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	/* Transfer complete interrupt mask */
	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
}

void delay_ms(uint32_t milliSeconds){
	while (milliSeconds--){
		TIM_SetCounter(TIM6, 0);
		TIM_Cmd(TIM6, ENABLE);
		while (TIM_GetFlagStatus(TIM6, TIM_FLAG_Update) != SET);
		TIM_Cmd(TIM6, DISABLE);
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	}
}

void delay_us(uint32_t microSeconds){
	TIM_SetCounter(TIM6, 0);
	TIM_Cmd(TIM6, ENABLE);
	while (TIM_GetCounter(TIM6) < microSeconds);
	TIM_Cmd(TIM6, DISABLE);
}

void MyDelay(__IO uint32_t number){
	while(number--){}
}

void My_GPIO_Init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void My_TIMER_Init(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_BaseStruct.TIM_Prescaler = 0;
	TIM_BaseStruct.TIM_Period = 8399;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);
	TIM_Cmd(TIM1, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_BaseStruct.TIM_Prescaler = 83;
	TIM_BaseStruct.TIM_Period = 999; //delay 1ms
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &TIM_BaseStruct);
	TIM_UpdateDisableConfig(TIM6, DISABLE);
	TIM_ARRPreloadConfig(TIM6, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_BaseStruct.TIM_Prescaler = 8399;
	TIM_BaseStruct.TIM_Period = 99999; //10kHz 10s
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_BaseStruct);
	TIM_UpdateDisableConfig(TIM7, DISABLE);
	TIM_ARRPreloadConfig(TIM7, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure; 
	TIM_TimeBaseStructure.TIM_ClockDivision 				= TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Prescaler 						= 8400-1;
	TIM_TimeBaseStructure.TIM_Period 								= (int)(Ts*10000)-1; //10ms
//	TIM_TimeBaseStructure.TIM_Period 								= 199; //20ms
	TIM_TimeBaseStructure.TIM_CounterMode 					= TIM_CounterMode_Up;
 	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	
	NVIC_InitTypeDef																			NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel 										= TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority					= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 								= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void My_PWM_Init(void){
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1; //clear on compare match
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	
	TIM_OCStruct.TIM_Pulse = 50*8400/100 - 1;
	TIM_OC1Init(TIM1, &TIM_OCStruct);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM1, &TIM_OCStruct);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_Cmd(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE); 
}

void My_Encoder_Init(void){
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	//Config GPIO for 2 channels of ENC
			//PA00		CHANNEL_A
			//PA01		CHANNEL_B
	GPIO_InitTypeDef	GPIO_ENC_InitStructure;
  GPIO_ENC_InitStructure.GPIO_Pin 					= GPIO_Pin_0| GPIO_Pin_1; 
	GPIO_ENC_InitStructure.GPIO_OType 				= GPIO_OType_PP;
	GPIO_ENC_InitStructure.GPIO_Mode 					= GPIO_Mode_AF;
	GPIO_ENC_InitStructure.GPIO_PuPd 					= GPIO_PuPd_NOPULL; 		
	GPIO_ENC_InitStructure.GPIO_Speed 				= GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_ENC_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
	
	//cau hinh input capture cho 2 kenh A, B
	TIM_TimeBaseInitTypeDef										TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Prescaler 			= 0;
  TIM_TimeBaseStructure.TIM_Period 					= 0xFFFF;	
  TIM_TimeBaseStructure.TIM_ClockDivision 	= TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_ICInitTypeDef 		TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel 					= TIM_Channel_1 | TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICFilter 					= 10; 
  TIM_ICInitStructure.TIM_ICPolarity 				= TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICPrescaler 			= TIM_ICPSC_DIV4;
  TIM_ICInitStructure.TIM_ICSelection 			= TIM_ICSelection_DirectTI;
  TIM_ICInit(TIM5,&TIM_ICInitStructure);
 
  TIM_EncoderInterfaceConfig(TIM5,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
  TIM_SetCounter(TIM5,0);
  TIM_Cmd(TIM5,ENABLE);
  TIM_ClearFlag(TIM5,TIM_FLAG_Update);
}