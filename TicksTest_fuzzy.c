#include <stm32f30x.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <stm32f30x_usart.h>
#include <stm32f30x_tim.h>
#include <stm32f30x_exti.h>
#include <stm32f30x_syscfg.h>

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

#define Kp_Small_Value 1
#define Kp_Medium_Value 2
#define Kp_Large_Value 3
#define Kd_Small_Value 4
#define Kd_Medium_Value 6
#define Kd_Large_Value 11

#define BotMaxVelocity 120
#define BotMinVelocity -120

volatile int8_t Data;

volatile int Previous_Error[2] = {0}, Ticks[2] = {0}, D_Error[2] = {0}, Error[2] = {0}, Target[2] = {0};

volatile int Cycle_Complete_Flag = 0, Flag = 0, Dir[2] = {0};

int Motor_Velocity[2] = {0},
	Negative_Error[2] = {0}, Zero_Error[2] = {0}, Positive_Error[2] = {0},
	Negative_D_Error[2] = {0}, Zero_D_Error[2] = {0}, Positive_D_Error[2] = {0},
	Kp_Small[2] = {0}, Kp_Medium[2] = {0}, Kp_Large[2] = {0},
	Kd_Small[2] = {0}, Kd_Medium[2] = {0}, Kd_Large[2] = {0},
	Fuzzy_Matrix[2][3][3] = {{{0}}};

int Kp_Multiplier[2], Kd_Multiplier[2],
	Kp_Divider[2], Kd_Divider[2],
	Fault[2] = {0};


void Error_Fuzzification(void)
{
	int i = 0;
	for(i = 0; i < 2; i++)
	{
		Positive_Error[i] = Negative_Error[i] = Zero_Error[i] = 0;
		if(Error[i]>10)
		{
			Positive_Error[i] = 10;
		}
		else if(Error[i]<-10)
		{
			Negative_Error[i] = 10;
		}
		else
		{
			if(Error[i] > 0)
			{
				Positive_Error[i] = (Error[i]);
				Zero_Error[i] = 10 - ((Error[i]));
			}
			else
			{
				Negative_Error[i] =  -((Error[i]));
				Zero_Error[i] = 10 + (Error[i]);
			}
		}
		Positive_D_Error[i] = Negative_D_Error[i] = Zero_D_Error[i] = 0;
		if(D_Error[i]>10)
		{
			Positive_D_Error[i] = 10;
		}
		else if(D_Error[i]<-10)
		{
			Negative_D_Error[i] = 10;
		}
		else
		{
			if(D_Error[i] > 0)
			{
				Positive_D_Error[i] = (D_Error[i]);
				Zero_D_Error[i] = 10 - ((D_Error[i]));
			}
			else
			{
				Negative_D_Error[i] =  -(D_Error[i]);
				Zero_D_Error[i] = 10 + (D_Error[i]);
			}
		}


	}
}

void Create_Fuzzy_Matrix(void)
{
	int i = 0;

	for(i = 0; i < 2; i++)
	{
		Fuzzy_Matrix[i][0][0] = Negative_D_Error[i] < Negative_Error[i] ? Negative_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][0][1] = Zero_D_Error[i] < Negative_Error[i] ? Zero_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][0][2] = Positive_D_Error[i] < Negative_Error[i] ? Positive_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][1][0] = Negative_D_Error[i] < Zero_Error[i] ? Negative_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][1][1] = Zero_D_Error[i] < Zero_Error[i] ? Zero_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][1][2] = Positive_D_Error[i] < Zero_Error[i] ? Positive_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][2][0] = Negative_D_Error[i] < Positive_Error[i] ? Negative_D_Error[i] : Positive_Error[i];
		Fuzzy_Matrix[i][2][1] = Zero_D_Error[i] < Positive_Error[i] ? Zero_D_Error[i] : Positive_Error[i];
		Fuzzy_Matrix[i][2][2] = Positive_D_Error[i] < Positive_Error[i] ? Positive_D_Error[i] : Positive_Error[i];
	}

}
void Determine_Weights(void)
{
	int i ;
	Kp_Multiplier[1] = Kp_Multiplier[0] = Kd_Multiplier[1] = Kd_Multiplier[0] = 0;

	for(i = 0; i < 2; i++)
	{

		Kp_Small[i] = Fuzzy_Matrix[i][1][1];

		Kp_Medium[i] = Fuzzy_Matrix[i][1][0] > Fuzzy_Matrix[i][0][1] ? Fuzzy_Matrix[i][1][0] : Fuzzy_Matrix[i][0][1];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][2][1] ? Kp_Medium[i] : Fuzzy_Matrix[i][2][1];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][1][2] ? Kp_Medium[i] : Fuzzy_Matrix[i][1][2];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][0][0] ? Kp_Medium[i] : Fuzzy_Matrix[i][0][0];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][2][2] ? Kp_Medium[i] : Fuzzy_Matrix[i][2][2];


		Kp_Large[i] =  Fuzzy_Matrix[i][0][2] > Fuzzy_Matrix[i][2][0] ? Fuzzy_Matrix[i][0][2] : Fuzzy_Matrix[i][2][0];


		Kd_Small[i] = Fuzzy_Matrix[i][0][2] > Fuzzy_Matrix[i][2][0] ? Fuzzy_Matrix[i][0][2] : Fuzzy_Matrix[i][2][0];
		Kd_Small[i] = Kd_Small[i] > Fuzzy_Matrix[i][1][1] ? Kd_Small[i] : Fuzzy_Matrix[i][1][1];


		Kd_Medium[i] = Fuzzy_Matrix[i][0][0] > Fuzzy_Matrix[i][0][1] ? Fuzzy_Matrix[i][0][0] : Fuzzy_Matrix[i][0][1];
		Kd_Medium[i] = Kd_Medium[i] > Fuzzy_Matrix[i][2][1] ? Kd_Medium[i] : Fuzzy_Matrix[i][2][1];
		Kd_Medium[i] = Kd_Medium[i] > Fuzzy_Matrix[i][2][2] ? Kd_Medium[i] : Fuzzy_Matrix[i][2][2];


		Kd_Large[i] = Fuzzy_Matrix[i][1][0] > Fuzzy_Matrix[i][1][2] ? Fuzzy_Matrix[i][1][0] : Fuzzy_Matrix[i][1][2];


		Kp_Multiplier[i] = (Kp_Small[i] * Kp_Small_Value) + (Kp_Medium[i] * Kp_Medium_Value) + (Kp_Large[i] * Kp_Large_Value);
		Kd_Multiplier[i] = (Kd_Small[i] * Kd_Small_Value) + (Kd_Medium[i] * Kd_Medium_Value) + (Kd_Large[i] * Kd_Large_Value);

		Kp_Divider[i] = (Kp_Small[i] + Kp_Medium[i] + Kp_Large[i]);
		Kd_Divider[i] = (Kd_Small[i] + Kd_Medium[i] + Kd_Large[i]);
	}

}
void Give_Motor_Velocity(void)
{
	int16_t i;

	for(i = 0; i < 2; i++)
	{

		if(Kp_Divider[i] != 0 && Kd_Divider[i] != 0)
		{
			Fault[i] = (int32_t) ((Error[i] * Kp_Multiplier[i]) / Kp_Divider[i]);
			Fault[i] += (int32_t) ((D_Error[i] * Kd_Multiplier[i]) / Kd_Divider[i]);
		}
		Motor_Velocity[i] += Fault[i];
	}

	if(Motor_Velocity[0] < 0)
	{
		if(Motor_Velocity[0] < -255)
		{

			TIM_SetCompare2(TIM4,255);

			Motor_Velocity[0] = -255;
		}
		else
		TIM_SetCompare2(TIM4,-Motor_Velocity[0]);

		TIM_SetCompare4(TIM4,0);
	}
	else
	{
		if(Motor_Velocity[0] > 255)
		{
			TIM_SetCompare4(TIM4,255);
			Motor_Velocity[0] = 255;
		}
		else
		TIM_SetCompare4(TIM4,Motor_Velocity[0]);

		TIM_SetCompare2(TIM4,0);
	}

	if(Motor_Velocity[1] < 0)
	{
		if(Motor_Velocity[1] <-255)
		{
			TIM_SetCompare1(TIM4,255);
			Motor_Velocity[1] = -255;
		}
		else
		TIM_SetCompare1(TIM4,-Motor_Velocity[1]);

		TIM_SetCompare3(TIM4,0);
	}
	else
	{
		if(Motor_Velocity[1] >255)
		{
			TIM_SetCompare3(TIM4,255);
			Motor_Velocity[1] = 255;
		}
		else
		TIM_SetCompare3(TIM4,Motor_Velocity[1]);

		TIM_SetCompare1(TIM4,0);

	}
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
	GPIO_C.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_C.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_C.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOC,&GPIO_C);

	GPIO_D.GPIO_Mode = GPIO_Mode_IN;
	GPIO_D.GPIO_Pin = GPIO_Pin_5;
	GPIO_D.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_D.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_D.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOD,&GPIO_D);

	/*!------------------------------------ For External Interrupt ---------------------------------------!*/

	/*!------------------------------------ For Encoders ---------------------------------------!*/

	GPIO_InitTypeDef GPIO_E,GPIO_F;

	GPIO_E.GPIO_Mode = GPIO_Mode_IN;
	GPIO_E.GPIO_Pin = GPIO_Pin_6;
	GPIO_E.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_E.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_E.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOE,&GPIO_E);

	GPIO_F.GPIO_Mode = GPIO_Mode_IN;
	GPIO_F.GPIO_Pin = GPIO_Pin_9;
	GPIO_F.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_F.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_F.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOF,&GPIO_F);

	/*!------------------------------------ For Encoders ---------------------------------------!*/
}

void Initialise_TimerInterrupt()
{
	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	TIM_TimeBaseInitTypeDef TIM2_INIT;

	TIM2_INIT.TIM_Prescaler = 800;
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
	EXTI_PC10.EXTI_Trigger = EXTI_Trigger_Rising;

	EXTI_PD5.EXTI_Line = EXTI_Line5;
	EXTI_PD5.EXTI_LineCmd = ENABLE;
	EXTI_PD5.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_PD5.EXTI_Trigger = EXTI_Trigger_Rising;

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

	TIM4_INIT.TIM_Prescaler = 25;
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

int main()
{
	Initialise_Clock();
	Initialise_GPIO();
	Initialise_TimerPWM();
	Initialise_TimerInterrupt();
	Initialise_UART();
	Initialise_ExternalInterrupt();

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);


		Ticks[0] = Ticks[1] = 0;
	    while(1)
	    {

			if(Cycle_Complete_Flag == 1)
			{

				Error_Fuzzification();
				Create_Fuzzy_Matrix();
				Determine_Weights();
				Give_Motor_Velocity();
				Cycle_Complete_Flag = 0;

			}
		}



}


void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{

		Data = USART_ReceiveData(USART2);
		if(Data > BotMaxVelocity)
			Target[0] = BotMaxVelocity;
		else if(Data < BotMinVelocity)
			Target[0] = BotMinVelocity;
		else
			Target[0] = Data;

		Data = USART_ReceiveData(USART2);
		if(Data > BotMaxVelocity)
			Target[1] = BotMaxVelocity;
		else if(Data < BotMinVelocity)
			Target[1] = BotMinVelocity;
		else
			Target[1] = Data;

		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}

}

void EXTI15_10_IRQHandler()
{
	if(EXTI_GetFlagStatus(EXTI_Line10) == !RESET)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_8); // Pointless
				if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_9)==SET)
				{
					Ticks[1]++;
				}
				else
					Ticks[1]--;

		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}

void EXTI9_5_IRQHandler()
{
	if(EXTI_GetFlagStatus(EXTI_Line5) == !RESET)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_9); // Pointless
						if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)==SET)
						{
							Ticks[0]++;
						}
						else
							Ticks[0]--;
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

void TIM2_IRQHandler()
{
	Cycle_Complete_Flag = 1;
	Error[0] = Target[0] - Ticks[0] ;
	Error[1] = Target[1] - Ticks[1] ;

	D_Error[1] = Error[1] - Previous_Error[1];
	D_Error[0] = Error[0] - Previous_Error[0];

	Previous_Error[1] = Error[1];
	Previous_Error[0] = Error[0];

	Ticks[0] = 0;
	Ticks[1] = 0;

	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM2_IRQn);

}
