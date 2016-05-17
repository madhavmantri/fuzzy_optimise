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
volatile int Store_Ticks[2][100]={0},Err[2]={0,0},D_Err[2]={0,0},Prev_Err[2]={0,0};
volatile int8_t Data=0;
volatile int Kp=5,Kd=2;
volatile int Loop_Count=0,Uart_Receive_Flag=0;
volatile int Ticks[2]={0},Target[2]={0},T1=0,T2=0;
int Update_velocity(int OCR,int error,int d_error)
{
	if (( OCR+Kp*error+Kd*d_error)>254)
	{
		return 254;
	}
	else if((OCR+Kp*error+Kd*d_error)<-254)
	{
		return -254;
	}
	else
	{
		return (OCR+(Kp*error)+(Kd*d_error));
	}
	//return (((OCR+(Kp*error)+(Kd*d_error))<255)?(OCR+(Kp*error)+(Kd*d_error)):255);
}
void Disable_All()
{
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	TIM_Cmd(TIM2,DISABLE);
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	NVIC_DisableIRQ(EXTI15_10_IRQn);
	Give_Velocity(0,0,0,0);
	Target[0]=0;
	Target[1]=0;
}
void Give_Velocity(int Left1,int Right1,int Left2,int Right2)
{
	TIM4->CCR1= Left1;
	TIM4->CCR3= Right1;

	TIM4->CCR2= Left2;
	TIM4->CCR4= Right2;


/*	TIM_SetCompare1(TIM4, Left1);
	TIM_SetCompare3(TIM4, Right1);

	TIM_SetCompare2(TIM4, Left2);
	TIM_SetCompare4(TIM4, Right2);*/
}
int main()
{
//	SystemInit();

	int i=0,j=0;
	Frequency_Enhancer();
	Initialise_Clock();
	Initialise_GPIO();
	Initialise_UART();
	Initialise_TimerInterrupt();
	Initialise_ExternalInterrupt();
	Initialise_TimerPWM();

	//Give_Velocity(100,0,100,0);

	//delay(100);
	Give_Velocity(0,0,0,0);

	//TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	Ticks[0]=0;
	Ticks[1]=0;

	while(1)
	{
	//	GPIOE->ODR=0XFF00;
		//Give_Velocity(Target[1],0,Target[0],0);
		T1=Update_velocity((TIM4->CCR3>TIM4->CCR1?TIM4->CCR3:-(TIM4->CCR1)),Err[1],D_Err[1]);
		T2=Update_velocity((TIM4->CCR2>TIM4->CCR4?TIM4->CCR2:-(TIM4->CCR4)),Err[0],D_Err[0]);
		if(T1>=0&&T2>=0)
		{
			Give_Velocity(0,T1,T2,0);
		}
		else if (T1<0&&T2>=0)
		{
			Give_Velocity(-T1,0,T2,0);
		}
		else if (T1>=0&&T2<0)
		{
			Give_Velocity(0,T1,0,-T2);
		}
		else if(T1<0&&T2<0)
		{
			Give_Velocity(-T1,0,0,-T2);
		}
		//Give_Velocity(Update_velocity(TIM_GetCapture1(TIM4),Err[1],D_Err[1]),0,(TIM_GetCapture2(TIM4),Err[0],D_Err[0]),0);

		if(Loop_Count>=100)
		{

			TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
			Uart_Receive_Flag=0;
		//	Disable_All();
			Give_Velocity(0,0,0,0);
			Target[0]=0;
			Target[1]=0;
			Err[0]=0;
			Err[1]=0;
			D_Err[0]=0;
			D_Err[1]=0;

			Ticks[0]=0;
			Ticks[1]=0;

			//GPIOE->ODR=0XFF00;
			//Disable_All();
			for(i=0;i<2;i++)
			{
				for(j=0;j<Loop_Count;j++)
				{
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
					USART_SendData( USART2,Store_Ticks[i][j]);

				}
			}
			Loop_Count=0;
		}

	}

}

void EXTI9_5_IRQHandler()
{
	if((EXTI->IMR & EXTI_IMR_MR5) && (EXTI->PR & EXTI_PR_PR5))
	{
		//Ticks[0]++;
		if(GPIOE->IDR&GPIO_Pin_6)
		{
			Ticks[0]++;

		}
		else
		{
			Ticks[0]--;

		}
		EXTI->PR |= EXTI_PR_PR5 ;
	//	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	}
}
void EXTI15_10_IRQHandler()
{
	if((EXTI->IMR & EXTI_IMR_MR10) && (EXTI->PR & EXTI_PR_PR10))
	{
		//Ticks[1]++;

		if(GPIOF->IDR&GPIO_Pin_9)
		{
			Ticks[1]++;

		}
		else
		{
			Ticks[1]--;
		}
		EXTI->PR |= EXTI_PR_PR10 ;
		//NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

	}

}

void TIM2_IRQHandler()
{
	if( TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
	{
		//GPIOE->ODR^=0xff00;

		if(Uart_Receive_Flag)
		{
			Err[0]=Target[0]-Ticks[0];
			Err[1]=Target[1]-Ticks[1];

			D_Err[0]=Err[0]-Prev_Err[0];
			D_Err[1]=Err[1]-Prev_Err[1];

			Prev_Err[0]=Err[0];
			Prev_Err[1]=Err[1];


			Store_Ticks[0][Loop_Count]=Ticks[0];
			Store_Ticks[1][Loop_Count]=Ticks[1];

			Ticks[0]=0;
			Ticks[1]=0;
			Loop_Count++;
		}
		//TIM2->SR = (uint16_t)~TIM_IT_Update;
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		NVIC_ClearPendingIRQ(TIM2_IRQn);
	}


}
void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		//Data=USART2->RDR;
		Data = USART_ReceiveData(USART2);
		Target[0]=Data;
		Target[1]=Target[0];
		//TIM2->DIER|=TIM_IT_Update;
		//TIM2->CNT=0;
		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
		Uart_Receive_Flag=1;
		//Give_Velocity(Target[1],0,Target[0],0);

		Ticks[0]=0;
		Ticks[1]=0;

	/*	if(Flag == 0)
		{

			if(Data > BotMaxVelocity)
				Target[0] = BotMaxVelocity;
			else if(Data < BotMinVelocity)
				Target[0] = BotMinVelocity;
			else
				Target[0] = Data;
			Flag++;
		}

		else if(Flag == 1)
		{

			if(Data > BotMaxVelocity)
				Target[1] = BotMaxVelocity;
			else if(Data < BotMinVelocity)
				Target[1] = BotMinVelocity;
			else
				Target[1] = Data;
			Flag = 0;
		}  */

		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}

}
