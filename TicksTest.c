#include <stm32f30x.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <stm32f30x_usart.h>
#include <stm32f30x_tim.h>
#include <stm32f30x_exti.h>
#include <stm32f30x_syscfg.h>
#include <stm32f30x_flash.h>
volatile int Ticks[2]={0,0},j=0,count=0,Enable_Flag=0,Update_Flag=0,Interrupt_Flag=0;     //Ticks Stores Number of Ticks ,count set number of times the 3.2ms timer will be called
volatile int Store_Ticks[2][100]={0},Channel1=0,Channel2=0;
volatile int Target[2]={0,0},ERR[2]={0,0},D_ERR[2]={0,0},P_ERR[2]={0,0},P_D_ERR[2]={0,0},Kp=0,Kd=0;				//SET the Target OCR send by uart



/* Pin Configuration
 * Board - STM32F3 Discovery
 * Current Processor Speed - 8MHz
 *
 * Pins for LEDs: PE8 - PE12
 *
 * Pins for Motor PWM - PB7 PB9 Motor 1
 * 						PB8 PB6 Motor 2
 *
 * PWM on Timer 4
 * Timer 4 Channels - 	PB6 Channel 1
 *						PB7 Channel 2
 * 						PB8 Channel 3
 * 						PB9 Channel 4
 *
 * 	UART on Channel 2
 * 	Pins for UART -  	PA3 Rx
 * 						PA2 Tx
 *
 * 	Timer Interrupt on Timer 2
 *
 * 	External Interrupt - PD5 Motor 1
 * 						 PC10 Motor 2
 *
 * 	Pins for Encoder - PE6 Motor 1
 * 					   PF9 Motor 2
 *
 */

void clockshift()
{
	RCC_ClocksTypeDef rccinit;  	//structure that contains aur peripherals and system  clock frequency as feild
	FLASH_PrefetchBufferCmd(ENABLE);
	RCC_HSEConfig(RCC_HSE_ON);                            // swutch on the external clock
	while((RCC_WaitForHSEStartUp())!=SUCCESS);            //wait untill HSE will activate
	                                                      //Disable the HSI clock
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_9);    //INPUT the HSE clock to PLL which is 8 Mhz and providing multiplying factor 9 making it 72 Mhz
	RCC_PLLCmd(ENABLE);                                   //ENABLING PLL
	RCC_HSEConfig(RCC_HSE_Bypass);                        //Bye passing external clockk
	FLASH_SetLatency( FLASH_Latency_2);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);            //Provide SYSTEM THE PLL clock

	RCC_GetClocksFreq(&rccinit);                          //must be after each configuration in any of the clock
	RCC_HSICmd(DISABLE);
	RCC_HCLKConfig(RCC_SYSCLK_Div1);                      //HCLK =36 MHZ  System clock by 2
	RCC_PCLK1Config(RCC_HCLK_Div1);                       //APB1 clock HCLK =36
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_GetClocksFreq(&rccinit);
}
/*void clockshift()
{
	uint8_t HSEStartUpStatus=0;
	//RCC_ClocksTypeDef rccinit;
	//FLASH_PrefetchBufferCmd(ENABLE);
	RCC_HSEConfig(RCC_HSE_ON);            //structure that contains aur peripherals and system  clock frequency as feild

	HSEStartUpStatus =RCC_WaitForHSEStartUp();// swutch on the external clock
	if(HSEStartUpStatus==SUCCESS)
	{
		FLASH_SetLatency(FLASH_Latency_3);
		FLASH_PrefetchBufferCmd(ENABLE);
		//FLASH_InstructionCacheCmd(ENABLE);
		//FLASH_DataCacheCmd(ENABLE);
		RCC_HCLKConfig(RCC_SYSCLK_Div4);
	}
	                                                      //Disable the HSI clock
	RCC_PLLConfig(RCC_PLLSource_PREDIV1,RCC_PLLMul_9);    //INPUT the HSE clock to PLL which is 8 Mhz and providing multiplying factor 9 making it 72 Mhz
	RCC_PLLCmd(ENABLE);                                   //ENABLING PLL
	RCC_HSEConfig(RCC_HSE_Bypass);                        //Bye passing external clockk
	FLASH_SetLatency( FLASH_Latency_2);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);            //Provide SYSTEM THE PLL clock

	RCC_GetClocksFreq(&rccinit);                          //must be after each configuration in any of the clock
	RCC_HSICmd(DISABLE);
	RCC_HCLKConfig(RCC_SYSCLK_Div4);                      //HCLK =36 MHZ  System clock by 2
	RCC_PCLK1Config(RCC_HCLK_Div1);                       //APB1 clock HCLK =36
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_GetClocksFreq(&rccinit);
}*/
void Initialise_Clock()
{
	/*!------------------------------------------- GPIO -------------------------------------------!*/

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF,ENABLE);

	/*!------------------------------------------- For GPIO -------------------------------------------!*/

	/*!------------------------------------ For External Interrupt ------------------------------------!*/

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*!------------------------------------ For External Interrupt ------------------------------------!*/

	/*!-------------------------------------------- For UART ------------------------------------------!*/

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/*!-------------------------------------------- For UART ------------------------------------------!*/

	/*!-------------------------------------------- For Timer ------------------------------------------!*/

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //For PWM
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //For Timer Interrupt

	/*!-------------------------------------------- For Timer ------------------------------------------!*/
}


void Initialise_UART()
{
	/*!--------------------------------------- Initialise Structure ----------------------------------------!*/

	USART_InitTypeDef UART2_DEF;

	UART2_DEF.USART_BaudRate = 38400;
	UART2_DEF.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART2_DEF.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	UART2_DEF.USART_Parity = USART_Parity_No;
	UART2_DEF.USART_StopBits = USART_StopBits_1;
	UART2_DEF.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART2,&UART2_DEF);

	/*!---------------------------------------- Initialise Structure ----------------------------------------!*/

	/*!---------------------------------------- Interrupt Controller ----------------------------------------!*/

	NVIC_InitTypeDef NVIC_UART;

	NVIC_UART.NVIC_IRQChannel =  USART2_IRQn;
	NVIC_UART.NVIC_IRQChannelCmd = ENABLE;
	NVIC_UART.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_UART.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_UART);

	/*!---------------------------------------- Interrupt Controller ----------------------------------------!*/

	/*!---------------------------------------- Enable UART Interrupt ----------------------------------------!*/

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

	/*!---------------------------------------- Enable UART Interrupt ----------------------------------------!*/

	/*!---------------------------------------- Enable UART  ----------------------------------------!*/

	USART_Cmd(USART2,ENABLE);

	/*!---------------------------------------- Enable UART  ----------------------------------------!*/
}



void Initialise_GPIO()

{
	/*!------------------------------------------- For LEDs -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_LED;

	GPIO_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_LED.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_LED.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_LED.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_LED.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOE,&GPIO_LED);

	/*!------------------------------------------- For LEDs -------------------------------------------!*/

	/*!------------------------------------------- For PWM -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_PWM;

	GPIO_PWM.GPIO_Mode = GPIO_Mode_AF;
	GPIO_PWM.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_PWM.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_PWM.GPIO_OType = GPIO_OType_PP;
	GPIO_PWM.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9, GPIO_AF_2);

	GPIO_Init(GPIOB,&GPIO_PWM);

	/*!------------------------------------------- For PWM -------------------------------------------!*/

	/*!------------------------------------------- For UART -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_UART;

	GPIO_UART.GPIO_Mode = GPIO_Mode_AF;
	GPIO_UART.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_UART.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_UART.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_UART.GPIO_OType = GPIO_OType_PP;

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_7);

	GPIO_Init(GPIOA,&GPIO_UART);

	/*!------------------------------------------- For UART -------------------------------------------!*/

	/*!------------------------------------ For External Interrupt ---------------------------------------!*/

	GPIO_InitTypeDef GPIO_C,GPIO_D;

	GPIO_C.GPIO_Mode = GPIO_Mode_IN;
	GPIO_C.GPIO_Pin = GPIO_Pin_10;
	GPIO_C.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_C.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_C.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOC,&GPIO_C);

	GPIO_D.GPIO_Mode = GPIO_Mode_IN;
	GPIO_D.GPIO_Pin = GPIO_Pin_5;
	GPIO_D.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_D.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_D.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOD,&GPIO_D);

	/*!------------------------------------ For External Interrupt ---------------------------------------!*/

	/*!------------------------------------ For Encoders ---------------------------------------!*/

	GPIO_InitTypeDef GPIO_E,GPIO_F;

	GPIO_E.GPIO_Mode = GPIO_Mode_IN;
	GPIO_E.GPIO_Pin = GPIO_Pin_6;
	GPIO_E.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_E.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_E.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOE,&GPIO_E);

	GPIO_F.GPIO_Mode = GPIO_Mode_IN;
	GPIO_F.GPIO_Pin = GPIO_Pin_9;
	GPIO_F.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_F.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_F.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOF,&GPIO_F);

	/*!------------------------------------ For Encoders ---------------------------------------!*/
}

void Initialise_TimerInterrupt()
{
	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	TIM_TimeBaseInitTypeDef TIM2_INIT;

	TIM2_INIT.TIM_Prescaler = 3600;
	TIM2_INIT.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM2_INIT.TIM_CounterMode = TIM_CounterMode_Up;
	TIM2_INIT.TIM_Period = 32;

	TIM_TimeBaseInit(TIM2,&TIM2_INIT);

	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	/*!----------------------------- Interrupt Controller ---------------------------------!*/

	NVIC_InitTypeDef NVIC_TIMER;

	NVIC_TIMER.NVIC_IRQChannel =  TIM2_IRQn;
	NVIC_TIMER.NVIC_IRQChannelCmd = ENABLE;
	NVIC_TIMER.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_TIMER.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_TIMER);

	/*!----------------------------- Interrupt Controller ---------------------------------!*/

	/*!----------------------------- Enable Interrupt ---------------------------------!*/

	//TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

	/*!----------------------------- Enable Interrupt ---------------------------------!*/

	/*!----------------------------- Enable Timer ---------------------------------!*/

	//TIM_Cmd(TIM2,ENABLE);

	/*!----------------------------- Enable Timer ---------------------------------!*/
}

void Initialise_ExternalInterrupt()
{
	/*!------------------------------------ Initialise Structures ---------------------------------------!*/

	EXTI_InitTypeDef EXTI_PD5,EXTI_PC10;

	EXTI_PC10.EXTI_Line = EXTI_Line10;
	EXTI_PC10.EXTI_LineCmd = ENABLE;
	EXTI_PC10.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_PC10.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_PD5.EXTI_Line = EXTI_Line5;
	EXTI_PD5.EXTI_LineCmd = ENABLE;
	EXTI_PD5.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_PD5.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_Init(&EXTI_PC10);
	EXTI_Init(&EXTI_PD5);

	/*!------------------------------------ Initialise Structures ---------------------------------------!*/

	/*!------------------------------------ Configure Lines ---------------------------------------!*/

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource10);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource5);

	/*!------------------------------------ Configure Lines ---------------------------------------!*/

	/*!---------------------------------- Interrupt Controller ---------------------------------------!*/

	NVIC_InitTypeDef NVIC_PC10,NVIC_PD5;

	NVIC_PD5.NVIC_IRQChannel =  EXTI9_5_IRQn;
	NVIC_PD5.NVIC_IRQChannelCmd = ENABLE;
	NVIC_PD5.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_PD5.NVIC_IRQChannelSubPriority = 1;

	NVIC_PC10.NVIC_IRQChannel =  EXTI15_10_IRQn;
	NVIC_PC10.NVIC_IRQChannelCmd = ENABLE;
	NVIC_PC10.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_PC10.NVIC_IRQChannelSubPriority = 2;

	NVIC_Init(&NVIC_PD5);
	NVIC_Init(&NVIC_PC10);

	/*!---------------------------------- Interrupt Controller ---------------------------------------!*/
}

void Initialise_TimerPWM()
{
	/*!----------------------------- Initialise Base Structures -----------------------------!*/

	TIM_TimeBaseInitTypeDef TIM4_INIT;
	TIM_OCInitTypeDef TIM4_OC;

	TIM4_INIT.TIM_Prescaler = 100;
	TIM4_INIT.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM4_INIT.TIM_CounterMode = TIM_CounterMode_Up;
	TIM4_INIT.TIM_Period = 255; //8 bit equivalent

	TIM_TimeBaseInit(TIM4,&TIM4_INIT);

	/*!----------------------------- Initialise Base Structures -----------------------------!*/

	/*!----------------------------- OC Mode -----------------------------!*/

	TIM4_OC.TIM_OCMode = TIM_OCMode_PWM1;
	TIM4_OC.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM4_OC.TIM_OutputState = TIM_OutputState_Enable ;

	TIM_OC1Init(TIM4,&TIM4_OC);
	TIM_OC2Init(TIM4,&TIM4_OC);
	TIM_OC3Init(TIM4,&TIM4_OC);
	TIM_OC4Init(TIM4,&TIM4_OC);

	/*!----------------------------- OC Mode -----------------------------!*/

	/*!----------------------------- Enable Timer -----------------------------!*/

	TIM_Cmd(TIM4, ENABLE);
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);

	/*!----------------------------- Enable Timer -----------------------------!*/
}

void GiveMotorVelocity(int Left1, int Right1, int Left2, int Right2)
{
	TIM_SetCompare1(TIM4,Left1); //Motor 2
	TIM_SetCompare3(TIM4,Right1);


	TIM_SetCompare2(TIM4,Left2); //Motor 1
	TIM_SetCompare4(TIM4,Right2);
}


void delay(int i)
{
	long time;
	time = 100000*i;
	for( ; time > 0; time -- )
	{

	}
}

void Enable()     //Enable Interrupt and Timers
{

	TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	Interrupt_Flag=1;
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);

}
void Disable()      //Disables Interrupt and Timers
{
	TIM_Cmd(TIM2,DISABLE);
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);

	TIM_Cmd(TIM4,DISABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	NVIC_DisableIRQ(TIM2_IRQn);
	Ticks[0]=0;
	Ticks[1]=0;
	count=0;
	j=0;
}
int PD(int OCR,int err,int D_err)
{
	return ((OCR+Kp*err+Kd*err)>255?255:(OCR+Kp*err+Kd*err));

}
int main()
{
	int k=0,i=0;
	Ticks[0]=0;
	Ticks[1]=0;

	clockshift();
	Initialise_Clock();
	Initialise_GPIO();
	Initialise_TimerPWM();
	Initialise_TimerInterrupt();
	Initialise_UART();
	Initialise_ExternalInterrupt();
	for(i=0;i<2;i++)
	{
		for(k=0;k<100;k++)
		{
			Store_Ticks[i][k]=0;
		}
	}
	//TIM_Cmd(TIM2,ENABLE);
	//TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	/*TIM_SetCompare1(TIM4,50);
	TIM_SetCompare2(TIM4,100);
	TIM_SetCompare3(TIM4,150);
	TIM_SetCompare4(TIM4,200);*/
	//USART_SendData(USART2,200);
	while(1)
	{
		/*GPIOE->ODR=0xf000;
		delay(10);
		uint32_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture4(TIM_TypeDef* TIMx);
		GPIOE->ODR=0x0f00;
		delay(10);*/
		      //Give Velocity

		if(Update_Flag)
		{
			Channel1=TIM_GetCapture1(TIM4);
			Channel2=TIM_GetCapture2(TIM4);
			GiveMotorVelocity(PD(TIM_GetCapture1(TIM4),ERR[1],D_ERR[1]),0,PD(TIM_GetCapture2(TIM4),ERR[0],D_ERR[0]),0);
			Update_Flag=0;
		}
		if(count>=100)                                            //After 300 loops send the Store Ticks
		{
			j==0;
			Disable();
			GiveMotorVelocity(0,0,0,0);
			for(i=0;i<2;i++)
			{
				for(k=0;k<100;k++)
				{
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
					USART_SendData(USART2,Store_Ticks[i][k]);
				}
			}
			count=0;
		}
		/*Test the following one by one before proceeding
		 * UART
		 * PWM
		 * GPIO
		 * External Interrupt
		 * Timer Interrupt
		 */
		//GPIO_SetBits(GPIOE,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
		//delay(2);
		//GPIO_ResetBits(GPIOE,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
		//delay(2);
	}

}


void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_11);
		Target[0]=USART_ReceiveData(USART2);
		Target[1]=Target[0];
		Enable();
		GiveMotorVelocity(Target[1],0,Target[0],0);
		Enable_Flag=1;
		//Ticks=0;
		//count=0;
		//j=0;


		//GPIO_SetBits(GPIOE,GPIO_Pin_9);
	//	USART_RequestCmd(USART2,USART_Request_RXFRQ,ENABLE); //used if we dont read the incoming data
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}
	//USART2_RXNE IT Pending Bit cleared on reading the USART Receive Register
}

void EXTI15_10_IRQHandler()
{
	if(EXTI_GetFlagStatus(EXTI_Line10)==!RESET)
	{


		//GPIO_SetBits(GPIOE,GPIO_Pin_8); // Pointless
		//Ticks[1]++;
		if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_9)==SET)
		{
			Ticks[1]--;

		}
		else
		{
			Ticks[1]++;

		}
//		GPIO_SetBits(GPIOE,GPIO_Pin_8);

		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}

void EXTI9_5_IRQHandler()
{
	if(EXTI_GetFlagStatus(EXTI_Line5)==!RESET)
	{
		//GPIO_SetBits(GPIOE,GPIO_Pin_9);
		if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)==SET)
		{
			Ticks[0]++;

		}
		else
		{
			Ticks[0]--;

		}
		//GPIO_SetBits(GPIOE,GPIO_Pin_9);

		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}
/*
void TIM2_IRQHandler()
{
	int i;
	if((TIM_GetITStatus(TIM2,TIM_IT_Update)==SET))
	{
		//&&(j<100);
		//GPIOE->ODR^=~0xaa00;
		if(Interrupt_Flag)
		{
			Interrupt_Flag=0;
			Ticks[0]=0;
			Ticks[1]=0;
		}
		else
		{
			count++;
			for(i=0;i<2;i++)
			{

				Store_Ticks[i][j]=Ticks[i];
				Ticks[i]=0;

			}
			j++;
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
			NVIC_ClearPendingIRQ(TIM2_IRQn);
		}
	}
}
*/
void TIM2_IRQHandler()
{

	if(Interrupt_Flag)
	{
		Interrupt_Flag=0;
		Ticks[0]=0;
		Ticks[1]=0;
	}
	else
	{
	//GPIOE->ODR^=0xaa00;
	//Cycle_Complete_Flag = 1;
		ERR[0] = Target[0] - Ticks[0] ;
		ERR[1] = Target[1] - Ticks[1] ;

		D_ERR[1] = ERR[1] - P_ERR[1];
		D_ERR[0] = ERR[0] - P_ERR[0];

		P_ERR[1] = ERR[1];
		P_ERR[0] = ERR[0];

		if(Enable_Flag)
		{
			Store_Ticks[1][j]=Ticks[1];
			Store_Ticks[0][j]=Ticks[0];
			j++;
			Update_Flag=1;
			count++;
		}

		Ticks[0] = 0;
		Ticks[1] = 0;
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM2_IRQn);

}

