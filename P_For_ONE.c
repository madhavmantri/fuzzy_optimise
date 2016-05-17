#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_flash.h>

volatile int Ticks=0,Uart_Receive_Flag=0,Target=0,Err=0,P_Err=0,D_Err=0,Store_Ticks[100]={0},Loop_Count=0,New=0;
volatile int8_t Data,Kp=0,Kd=0,Uart_Flag=0,Update_Flag=0;



void Initialise_Clock()
{
	/*!------------------------------------------- GPIO -------------------------------------------!*/

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);

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
	GPIO_LED.GPIO_Pin =GPIO_Pin_14|GPIO_Pin_15;
	GPIO_LED.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_LED.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_LED.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOB,&GPIO_LED);

	/*!------------------------------------------- For LEDs -------------------------------------------!*/

	/*!------------------------------------------- For PWM -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_PWM;

	GPIO_PWM.GPIO_Mode = GPIO_Mode_AF;
	GPIO_PWM.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_PWM.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_PWM.GPIO_OType = GPIO_OType_PP;
	GPIO_PWM.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9, GPIO_AF_TIM4);

	GPIO_Init(GPIOB,&GPIO_PWM);

	/*!------------------------------------------- For PWM -------------------------------------------!*/

	/*!------------------------------------------- For UART -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_UART;

	GPIO_UART.GPIO_Mode = GPIO_Mode_AF;
	GPIO_UART.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_UART.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_UART.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_UART.GPIO_OType = GPIO_OType_PP;

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);

	GPIO_Init(GPIOA,&GPIO_UART);

	/*!------------------------------------------- For UART -------------------------------------------!*/

	/*!------------------------------------ For External Interrupt ---------------------------------------!*/

	GPIO_InitTypeDef GPIO_C,GPIO_D;

	GPIO_C.GPIO_Mode = GPIO_Mode_IN;
	GPIO_C.GPIO_Pin = GPIO_Pin_4;
	GPIO_C.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_C.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_C.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOB,&GPIO_C);



	/*!------------------------------------ For External Interrupt ---------------------------------------!*/

	/*!------------------------------------ For Encoders ---------------------------------------!*/

	GPIO_InitTypeDef GPIO_E,GPIO_F;



	GPIO_F.GPIO_Mode = GPIO_Mode_IN;
	GPIO_F.GPIO_Pin = GPIO_Pin_5;
	GPIO_F.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_F.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_F.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOB,&GPIO_F);

	/*!------------------------------------ For Encoders ---------------------------------------!*/
}
void Initialise_TimerInterrupt()
{
	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	TIM_TimeBaseInitTypeDef TIM2_INIT;

	TIM2_INIT.TIM_Prescaler = 8400-1;
	TIM2_INIT.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM2_INIT.TIM_CounterMode = TIM_CounterMode_Up;
	TIM2_INIT.TIM_Period = 32;

	TIM_TimeBaseInit(TIM2,&TIM2_INIT);

	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	/*!----------------------------- Interrupt Controller ---------------------------------!*/

	NVIC_InitTypeDef NVIC_TIMER;

	NVIC_TIMER.NVIC_IRQChannel =  TIM2_IRQn;
	NVIC_TIMER.NVIC_IRQChannelCmd = ENABLE;
	NVIC_TIMER.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_TIMER.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_TIMER);

	/*!----------------------------- Interrupt Controller ---------------------------------!*/

	/*!----------------------------- Enable Interrupt ---------------------------------!*/


//	NVIC_EnableIRQ(TIM2_IRQn);

	/*!----------------------------- Enable Interrupt ---------------------------------!*/

	/*!----------------------------- Enable Timer ---------------------------------!*/

	TIM_Cmd(TIM2,ENABLE);

	/*!----------------------------- Enable Timer ---------------------------------!*/
}
void Initialise_ExternalInterrupt()
{
	/*!------------------------------------ Initialise Structures ---------------------------------------!*/

	EXTI_InitTypeDef EXTI_PD5,EXTI_PC10;

	EXTI_PC10.EXTI_Line = EXTI_Line4;
	EXTI_PC10.EXTI_LineCmd = ENABLE;
	EXTI_PC10.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_PC10.EXTI_Trigger = EXTI_Trigger_Falling;

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource4);

	EXTI_Init(&EXTI_PC10);


	/*!------------------------------------ Initialise Structures ---------------------------------------!*/

	/*!------------------------------------ Configure Lines ---------------------------------------!*/




	/*!------------------------------------ Configure Lines ---------------------------------------!*/

	/*!---------------------------------- Interrupt Controller ---------------------------------------!*/

	NVIC_InitTypeDef NVIC_PC10,NVIC_PD5;

	NVIC_PC10.NVIC_IRQChannel =  EXTI4_IRQn;
	NVIC_PC10.NVIC_IRQChannelCmd = ENABLE;
	NVIC_PC10.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_PC10.NVIC_IRQChannelSubPriority = 2;

//	NVIC_Init(&NVIC_PD5);
	NVIC_Init(&NVIC_PC10);
	NVIC_EnableIRQ(EXTI4_IRQn);
//	NVIC_EnableIRQ(EXTI9_5_IRQn);

	/*!---------------------------------- Interrupt Controller ---------------------------------------!*/
}
void Initialise_TimerPWM()
{
	/*!----------------------------- Initialise Base Structures -----------------------------!*/

	TIM_TimeBaseInitTypeDef TIM4_INIT;
	TIM_OCInitTypeDef TIM4_OC;

	TIM4_INIT.TIM_Prescaler = 25-1;
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


	/*!----------------------------- OC Mode -----------------------------!*/

	/*!----------------------------- Enable Timer -----------------------------!*/

	TIM_Cmd(TIM4, ENABLE);
	TIM_SetCompare1(TIM4, 0);

	TIM_SetCompare2(TIM4, 0);


	/*!----------------------------- Enable Timer -----------------------------!*/
}

void Give_Speed(int Left1 ,int Right1 )
{
	TIM4->CCR1=Left1;
	TIM4->CCR2=Right1;
}

void Update()
{
	//int New=0;
	New=((TIM4->CCR1)-(TIM4->CCR2))+Kp*Err+Kd*D_Err;
	if(New>254)
	{
		//return 254;
		Give_Speed(254,0);
	}
	else if(New<-254)
	{
		Give_Speed(0,254);
		//return -254;
	}
	else if(New<0&&New>-254)
	{
		Give_Speed(0,-New);
	}
	else
	{
		Give_Speed(New,0);
	}

}

void EXTI4_IRQHandler()
{
	if((EXTI->IMR & EXTI_IMR_MR4) && (EXTI->PR & EXTI_PR_PR4))
	{
		//Ticks++;

		if(GPIOB->IDR&GPIO_Pin_5)
		{
			Ticks++;

		}
		else
		{
			Ticks--;
		}
		EXTI->PR |= EXTI_PR_PR4 ;
		NVIC_ClearPendingIRQ(EXTI4_IRQn);

	}

}
void TIM2_IRQHandler()
{
	if( TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
	{
	//	GPIOE->ODR=0xff00;

		//if(Uart_Receive_Flag)

			Err=Target-Ticks;

			D_Err=Err-P_Err;

			P_Err=Err;

			Store_Ticks[Loop_Count]=Ticks;

			Loop_Count++;

			Ticks=0;

			Update_Flag=1;

		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		NVIC_ClearPendingIRQ(TIM2_IRQn);
	}
}

void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		//GPIOE->ODR=0xff00;
		Data = USART_ReceiveData(USART2);
		switch (Uart_Flag)
		{
			case 0:
				Kp=Data;
				Uart_Flag++;
				break;
			case 1:
				Kd=Data;
				Uart_Flag++;
				break;
			case 2:
				Target=Data;
				Uart_Flag=0;
				Uart_Receive_Flag=1;
				break;
		}
		//Target=Data;





		Ticks=0;




		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}

}

int main()
{
	long int i=0;
	SystemInit();
	//Frequency_Enhancer();
	Initialise_Clock();
	Initialise_GPIO();
	Initialise_UART();
	Initialise_TimerInterrupt();
	Initialise_ExternalInterrupt();
	Initialise_TimerPWM();
	Ticks=0;
	Give_Speed(100,0);
	i=1000000;
	while(i--);
	Give_Speed(0,0);

	while(1)
	{
		if(Uart_Receive_Flag)
		{
			TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
			Uart_Receive_Flag=0;
		}
		if(Update_Flag)
		{
			Update();
			Update_Flag=0;
		}
		if(Loop_Count>=100)
		{
			 TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
			 Err=0;
			 D_Err=0;
			 P_Err=0;
			 Target=0;
			 Ticks=0;
			 TIM4->CCR1=0;
			 TIM4->CCR2=0;
			 Uart_Receive_Flag=0;
			 for(i=0;i<Loop_Count;i++)
			 {
				 while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
				 USART_SendData( USART2,Store_Ticks[i]);

			 }

			 while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
			 //USART_SendData( USART2,Loop_Count);
			 Loop_Count=0;


		 }

	 }


}
