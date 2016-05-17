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


volatile int Enable_Flag=0,Count=0,Store_Ticks[2][100]={0},Latency_Flag=0,Data_Count=0,Count_Latency=0,Timer_Latency_Counter=0;;

volatile int8_t Data,TeamID=127;

volatile int Previous_Error[2] = {0}, Ticks[2] = {0}, D_Error[2] = {0}, Error[2] = {0}, Target[2] = {0},Target_latency[8]={0};

volatile int Cycle_Complete_Flag = 0, Flag = 0;

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
			Fault[i] = ((Error[i] * Kp_Multiplier[i]) / Kp_Divider[i]);
			Fault[i] += ((D_Error[i] * Kd_Multiplier[i]) / Kd_Divider[i]);
		}
		Motor_Velocity[i] += Fault[i];
	}

	if(Motor_Velocity[0] < 0)
	{
		if(Motor_Velocity[0] < -254)
		{

			TIM_SetCompare4(TIM4,254);

			Motor_Velocity[0] = -254;
		}
		else
		TIM_SetCompare4(TIM4,-Motor_Velocity[0]);

		TIM_SetCompare2(TIM4,0);
	}
	else
	{
		if(Motor_Velocity[0] > 254)
		{
			TIM_SetCompare2(TIM4,254);
			Motor_Velocity[0] = 254;
		}
		else
		TIM_SetCompare2(TIM4,Motor_Velocity[0]);

		TIM_SetCompare4(TIM4,0);
	}

	if(Motor_Velocity[1] < 0)
	{
		if(Motor_Velocity[1] <-254)
		{
			TIM_SetCompare1(TIM4,254);
			Motor_Velocity[1] = -254;
		}
		else
		TIM_SetCompare1(TIM4,-Motor_Velocity[1]);

		TIM_SetCompare3(TIM4,0);
	}
	else
	{
		if(Motor_Velocity[1] >254)
		{
			TIM_SetCompare3(TIM4,254);
			Motor_Velocity[1] = 254;
		}
		else
		TIM_SetCompare3(TIM4,Motor_Velocity[1]);

		TIM_SetCompare1(TIM4,0);

	}
}




int main()
{
	Frequency_Enhancer();
	Initialise_Clock();
	Initialise_GPIO();
	Initialise_TimerPWM();
	Initialise_UART();
	Initialise_ExternalInterrupt();
	Initialise_TimerInterrupt();
	Initialise_TimerInterrupt3();
	int i,j;


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
			if(Data_Count>100)
			{
				Target[0]=0;
				Target[1]=0;
				Count=0;
				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET)
				{

				}
				USART_SendData( USART2,Data_Count);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET)
				{

				}
				USART_SendData( USART2,Data_Count>>8);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET)
				{

				}
				USART_SendData( USART2,Timer_Latency_Counter);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET)
				{

				}
				USART_SendData( USART2,Count_Latency);
				Data_Count=0;
			}





		}



}


void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{

		Data = USART_ReceiveData(USART2);
		if(!Data_Count)
		{
			TIM_SetCounter(TIM3,0);
		}
		switch (Flag)
		{
			case 0:
				if(Data==TeamID)
				{
					Flag++;
					Latency_Flag=1;
				}
				break;
			case 1:
				if(Data > BotMaxVelocity)
					Target[0] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target[0] = BotMinVelocity;
				else
					Target[0] = Data;
				Flag++;

				break;
			case 2:
				if(Data > BotMaxVelocity)
					Target[1] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target[1] = BotMinVelocity;
				else
					Target[1] = Data;
				Flag++;
				break;
			case 3:
				if(Data > BotMaxVelocity)
					Target_latency[0] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target_latency[0] = BotMinVelocity;
				else
					Target_latency[0] = Data;
				Flag++;
				break;
			case 4:
				if(Data > BotMaxVelocity)
					Target_latency[1] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target_latency[1] = BotMinVelocity;
				else
					Target_latency[1] = Data;
				Flag++;
				break;
			case 5:
				if(Data > BotMaxVelocity)
					Target_latency[2] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target_latency[2] = BotMinVelocity;
				else
					Target_latency[2] = Data;
				Flag++;
				break;


			case 6:
				if(Data > BotMaxVelocity)
					Target_latency[3] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target_latency[3] = BotMinVelocity;
				else
					Target_latency[3] = Data;
				Flag++;
				break;
			case 7:
				if(Data > BotMaxVelocity)
					Target_latency[4] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target_latency[4] = BotMinVelocity;
				else
					Target_latency[4] = Data;
				Flag++;
				break;
			case 8:
				if(Data > BotMaxVelocity)
					Target_latency[5] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target_latency[5] = BotMinVelocity;
				else
					Target_latency[5] = Data;
				Flag++;
				break;
			case 9:
				if(Data > BotMaxVelocity)
					Target_latency[6] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target_latency[6] = BotMinVelocity;
				else
					Target_latency[6] = Data;
				Flag++;
				break;
			case 10:
				if(Data > BotMaxVelocity)
					Target_latency[7] = BotMaxVelocity;
				else if(Data < BotMinVelocity)
					Target_latency[7] = BotMinVelocity;
				else
					Target_latency[7] = Data;
				Flag=0;

				if(Latency_Flag==1&&Data==(uint8_t)'j')
				{
					Latency_Flag=0;
					Data_Count++;


				}
				if(Data_Count==100)
				{
					Timer_Latency_Counter=TIM_GetCounter(TIM3);

				}
				Enable_Flag++;
			}

		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}

}

void EXTI15_10_IRQHandler()
{
	if((EXTI->IMR & EXTI_IMR_MR10) && (EXTI->PR & EXTI_PR_PR10))
		{
			if(GPIOF->IDR&GPIO_Pin_9)
			{
				Ticks[1]++;

			}
			else
			{
				Ticks[1]--;

			}
			EXTI->PR |= EXTI_PR_PR10 ;
			NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

		}
}

void EXTI9_5_IRQHandler()
{

	if((EXTI->IMR & EXTI_IMR_MR5) && (EXTI->PR & EXTI_PR_PR5))
	{

		if(GPIOE->IDR&GPIO_Pin_6)
		{
			Ticks[0]++;

		}
		else
		{
			Ticks[0]--;

		}
		EXTI->PR |= EXTI_PR_PR5 ;
		NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	}
}

void TIM2_IRQHandler()
{
	Count++;
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
void TIM3_IRQHandler()
{
	if(Enable_Flag)
	{
		Count_Latency++;
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM3_IRQn);

}
void Initialise_TimerInterrupt3()
{
	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	TIM_TimeBaseInitTypeDef TIM3_INIT;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM3_INIT.TIM_Prescaler = 36-1;
	TIM3_INIT.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM3_INIT.TIM_CounterMode = TIM_CounterMode_Up;
	TIM3_INIT.TIM_Period = 30000;

	TIM_TimeBaseInit(TIM3,&TIM3_INIT);

	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	/*!----------------------------- Interrupt Controller ---------------------------------!*/

	NVIC_InitTypeDef NVIC_TIMER;

	NVIC_TIMER.NVIC_IRQChannel =  TIM3_IRQn;
	NVIC_TIMER.NVIC_IRQChannelCmd = ENABLE;
	NVIC_TIMER.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_TIMER.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_TIMER);

	/*!----------------------------- Interrupt Controller ---------------------------------!*/

	/*!----------------------------- Enable Interrupt ---------------------------------!*/

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn);

	/*!----------------------------- Enable Interrupt ---------------------------------!*/

	/*!----------------------------- Enable Timer ---------------------------------!*/

	TIM_Cmd(TIM3,ENABLE);

	/*!----------------------------- Enable Timer ---------------------------------!*/
}
