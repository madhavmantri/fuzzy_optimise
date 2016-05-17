#include <stm32f30x.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <stm32f30x_usart.h>
#include <stm32f30x_tim.h>
#include <stm32f30x_exti.h>
#include <stm32f30x_syscfg.h>
#include <stm32f30x_flash.h>

#include "Peripherals.h"
void Frequency_Enhancer()
{
	RCC_ClocksTypeDef rccinit;  							//structure that contains aur peripherals and system  clock frequency as feild
	FLASH_PrefetchBufferCmd(ENABLE);
	RCC_HSEConfig(RCC_HSE_ON);                            	// swutch on the external clock
	while((RCC_WaitForHSEStartUp())!=SUCCESS);            	//wait untill HSE will activate
															//Disable the HSI clock
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_9);    	//INPUT the HSE clock to PLL which is 8 Mhz and providing multiplying factor 9 making it 72 Mhz
	RCC_PLLCmd(ENABLE);                                   	//ENABLING PLL
	RCC_HSEConfig(RCC_HSE_Bypass);                        	//Bye passing external clockk
	FLASH_SetLatency( FLASH_Latency_2);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);            	//Provide SYSTEM THE PLL clock

	RCC_GetClocksFreq(&rccinit);                          	//must be after each configuration in any of the clock
	RCC_HSICmd(DISABLE);
	RCC_HCLKConfig(RCC_SYSCLK_Div1);                      	//HCLK =36 MHZ  System clock by 2
	RCC_PCLK1Config(RCC_HCLK_Div1);                       	//APB1 clock HCLK =36
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_GetClocksFreq(&rccinit);
}

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
	GPIO_C.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_C.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_C.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOC,&GPIO_C);

	GPIO_D.GPIO_Mode = GPIO_Mode_IN;
	GPIO_D.GPIO_Pin = GPIO_Pin_5;
	GPIO_D.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_D.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_D.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOD,&GPIO_D);

	/*!------------------------------------ For External Interrupt ---------------------------------------!*/

	/*!------------------------------------ For Encoders ---------------------------------------!*/

	GPIO_InitTypeDef GPIO_E,GPIO_F;

	GPIO_E.GPIO_Mode = GPIO_Mode_IN;
	GPIO_E.GPIO_Pin = GPIO_Pin_6;
	GPIO_E.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_E.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_E.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOE,&GPIO_E);

	GPIO_F.GPIO_Mode = GPIO_Mode_IN;
	GPIO_F.GPIO_Pin = GPIO_Pin_9;
	GPIO_F.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_F.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_F.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOF,&GPIO_F);

	/*!------------------------------------ For Encoders ---------------------------------------!*/
}

void Initialise_TimerInterrupt()
{
	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	TIM_TimeBaseInitTypeDef TIM2_INIT;

	TIM2_INIT.TIM_Prescaler = 900-1;
	TIM2_INIT.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM2_INIT.TIM_CounterMode = TIM_CounterMode_Up;
	TIM2_INIT.TIM_Period = 128;

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

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);

	/*!----------------------------- Enable Interrupt ---------------------------------!*/

	/*!----------------------------- Enable Timer ---------------------------------!*/

	TIM_Cmd(TIM2,ENABLE);

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
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	/*!---------------------------------- Interrupt Controller ---------------------------------------!*/
}

void Initialise_TimerPWM()
{
	/*!----------------------------- Initialise Base Structures -----------------------------!*/

	TIM_TimeBaseInitTypeDef TIM4_INIT;
	TIM_OCInitTypeDef TIM4_OC;

	TIM4_INIT.TIM_Prescaler = 125-1;
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
	TIM_SetCompare1(TIM4, 0);
	TIM_SetCompare2(TIM4, 0);
	TIM_SetCompare3(TIM4, 0);
	TIM_SetCompare4(TIM4, 0);

	/*!----------------------------- Enable Timer -----------------------------!*/
}


void delay(int i)
{
	long time;
	time = 100000*i;
	for( ; time > 0; time -- )
	{

	}
}
