/*
 * USART.c
 *
 *  Created on: 08-Jan-2014
 *      Author: AMAN
 */

						// Device header
#include <stm32f30x.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <stm32f30x_usart.h>
#include <stm32f30x_tim.h>
#include <stm32f30x_exti.h>
#include <stm32f30x_syscfg.h>
#include "Fuzzy_API.h"
volatile long int count=0,flag=0;
volatile int32_t err[2], d_err[2], prev_err[2];
volatile char T2_INT_FLAG = 0, Debug_Flag = 0;
volatile int Ticks[2]={0,0}, Target[2]={0,0};
volatile char Usart_data, Data_Flag = 0,Kp=10,Kd=0;
volatile int32_t Store_Ticks[2][100];
volatile int32_t j=0;//VARIABLES FOE STRORING TICKS
volatile int pwm[4] = {0,0,0,0};
uint8_t ocr_main[4],temp=0;

volatile int counter = 0;

void EXTI9_5_IRQHandler()//ISR for PE6 correspond to motor 2
{

	if(EXTI_GetITStatus(EXTI_Line5))
	{
		counter=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6);
		if(counter)
		{
			//GPIOE->ODR=0x1000;
			//wait(100);
			Ticks[0]++;
		}
		else
		{
			//GPIOE->ODR=0x2000;
			//wait(100);
			Ticks[1]++;
		}
	//	Ticks[0]++;
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

void EXTI15_10_IRQHandler()//ISR for PC14 correspond to motor 1
{
	if(EXTI_GetITStatus(EXTI_Line14))
	{
		//Ticks[1]++;
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
}

void Enable_all()
{
		NVIC_EnableIRQ(EXTI9_5_IRQn);
		NVIC_EnableIRQ(EXTI15_10_IRQn);
		NVIC_EnableIRQ(USART2_IRQn);
		NVIC_EnableIRQ(TIM2_IRQn);

		//
		//GPIOE->ODR=0XF000;
		//USART_Cmd(USART2,ENABLE);

		TIM_Cmd(TIM4,ENABLE);
		TIM_Cmd(TIM2,ENABLE);
		//TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
}

void USART2_IRQHandler(void	)
{
    /* RXNE handler */
    if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
    	switch(Data_Flag)
    	{
    		case 0:
    			Target[0]=Target[1]=USART_ReceiveData( USART2);
    			Data_Flag++;
    			break;
    		case 1:
    			Kp=USART_ReceiveData( USART2);
    			Data_Flag++;
    			break;
    		case 2:
    			Kd=USART_ReceiveData( USART2);
    			Enable_all();
    			TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    			Data_Flag=0;
    			j=0;
    			break;

    	}


		/*switch(Data_Flag)
		{
			case 0:
				/*if(Usart_data == (volatile char)('a'))
					Data_Flag ++;
				USART1->TDR= Usart_data;
			break;

			case 1:			// 0b(redundant)(redundant)(Right Wheel dir)(Left Wheel dir)(bot id)(bot id)(bot id)(team colour)

				Target[0] = Usart_data <= 100 ? Usart_data : 100;
				Data_Flag++;
			break;

			case 2:                                          // pwm for left wheel b/w 0 to 100 (according to FIRA_GUI packet's value will be b/w 0 to 100
				Target[1] = Usart_data <= 100 ? Usart_data : 100;

				Data_Flag++;
			break;

			case 3:                                         // pwm for right wheel b/w 0 to 100 (according to FIRA_GUI packet's value will be b/w 0 to 100

				Data_Flag = 0;

			break;
		}   */                                              // Important Note: Target is altered only by the coming packets, the output of the PID with FUZZY

    	//USART_SendData(USART2,USART_ReceiveData(USART2));
    	//while(!USART_GetFlagStatus(USART2,USART_FLAG_TXE));
    	USART_RequestCmd(USART2,USART_Request_RXFRQ,ENABLE);
    	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    	NVIC_ClearPendingIRQ(USART2_IRQn);
    }



}

void Left_Pid(uint8_t ocr)
{
	int new_ocr = ocr +  Kp* err[1] + Kd* d_err[1];
	if(new_ocr >=0)
	{
		if(new_ocr >= 255)
		{
			pwm[0] = 255;
		    pwm[1] = 0;
		}
		else
		{
			pwm[0] = new_ocr;
		    pwm[1] = 0;
		}

	}

	else
	{
		if(new_ocr <-255)
		{
			pwm[0] = 0;
		    pwm[1] = 255;
		}
		else
		{
			pwm[0] = 0;
		    pwm[1] = -new_ocr;
		}

	}
}
uint8_t Right_Pid(volatile int32_t error,volatile int32_t d_error, uint8_t ocr)
{
	int new_ocr = ocr+Kp* error+Kd* d_error;
		if(new_ocr >= 100)
			new_ocr = 100;
		return (new_ocr);
}
void TIM2_IRQHandler(void)
{
	counter ++;

	if((TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)  && (j<100))
	{
		int32_t i;

				T2_INT_FLAG = 1;

				for(i=0; i<2; i++)
				{
					//GPIOE->ODR=0xff00;
					Store_Ticks[i][j]=Ticks[i];
					err[i] = Target[i] - Ticks[i];    //max values of Ticks=(+-)251,target=(+-)100,err=(+-)351
					d_err[i] = err[i] - prev_err[i];  //max value of d_error=(+-)702
					prev_err[i] = err[i];
				//	USART_SendData(USART2,Ticks[i]);
					//while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);

					Ticks[i] = 0;

				}
				j++;
			/*	USART_SendData(USART2,Ticks[1]);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
				USART_SendData(USART2,d_err[1]);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
				Ticks[0] = 0;
*/


		/*if(flag==0)
		{

		}
		else
		{
			GPIOE->ODR++;
			USART_SendData(USART2,count>>8);
			while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
			USART_SendData(USART2,count&255);
			while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
		}

		flag++;*/


	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
			NVIC_ClearPendingIRQ(TIM2_IRQn);
}
void wait(int i)
{
	while(i--)
	{
	}
}
void PIN_PWM()
{
	GPIO_InitTypeDef GPIO;

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_2);
	// GPIO_PinAFConfig(GPIOA,GPIO_Pin_2,GPIO_AF_7);
	GPIO.GPIO_Mode=GPIO_Mode_AF;
	GPIO.GPIO_OType=GPIO_OType_PP;
	GPIO.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO);
}
void Pwm_Initialisation()
{
	TIM_TimeBaseInitTypeDef Init_Timer;
	TIM_OCInitTypeDef Init_PWM;
	NVIC_InitTypeDef NVIC_InitStructure1;

	Init_Timer.TIM_ClockDivision=TIM_CKD_DIV1;
	Init_Timer.TIM_CounterMode=TIM_CounterMode_Up;
	Init_Timer.TIM_Period=255;
	Init_Timer.TIM_Prescaler=2000;
	TIM_TimeBaseInit(TIM4,&Init_Timer);

	Init_PWM.TIM_OCMode=TIM_OCMode_PWM1;
	Init_PWM.TIM_OCPolarity=TIM_OCPolarity_High;
	Init_PWM.TIM_OutputState= TIM_OutputState_Enable;
	//Init_PWM.TIM_Pulse=1000;
	TIM_OC1Init(TIM4,&Init_PWM);
	TIM_OC2Init(TIM4,&Init_PWM);
	TIM_OC3Init(TIM4,&Init_PWM);
	TIM_OC4Init(TIM4,&Init_PWM);

}
void CLOCK_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

}
void GPIO_Initialize()

{
	GPIO_InitTypeDef GPIO;

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_7);
	 // GPIO_PinAFConfig(GPIOA,GPIO_Pin_2,GPIO_AF_7);
	GPIO.GPIO_Mode=GPIO_Mode_AF;
	GPIO.GPIO_OType=GPIO_OType_PP;
	GPIO.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	GPIO.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO);

 //GPIOE config
	GPIO.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO.GPIO_Mode = GPIO_Mode_OUT;
	GPIO.GPIO_OType = GPIO_OType_PP;
	GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO);

}
void Initialize_GPIO_External_Interrupt()
{
	GPIO_InitTypeDef GPIO;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;



	GPIO.GPIO_Pin=GPIO_Pin_14;
	GPIO.GPIO_Mode=GPIO_Mode_IN;
	GPIO.GPIO_OType=GPIO_OType_PP;
	GPIO.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO);

	GPIO.GPIO_Pin=GPIO_Pin_5;
	GPIO.GPIO_Mode=GPIO_Mode_IN;
	GPIO.GPIO_OType=GPIO_OType_PP;
	//GPIO.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO);

	GPIO.GPIO_Pin=GPIO_Pin_6;
	GPIO.GPIO_Mode=GPIO_Mode_IN;
	GPIO.GPIO_OType=GPIO_OType_PP;
	GPIO.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO);

	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource10);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

  /*  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);*/

   /* SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource5);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);*/

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource5);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    //CONFIGURE EXTI NVIC TO LEAST PRIORITY
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);



}
void Set_Usart()
{
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_InitStruct.USART_BaudRate = 38400;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No ;
	USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(USART2,&USART_InitStruct);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 //  configure the USART2 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                 // this sets the priority group of the USART2 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		     // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		    	 // the USART2 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);
	USART_Cmd(USART2,ENABLE);

}
void Init_timer()
{
	TIM_TimeBaseInitTypeDef Init_Timer;

	NVIC_InitTypeDef NVIC_InitStructure1;

	Init_Timer.TIM_ClockDivision=TIM_CKD_DIV1;
	Init_Timer.TIM_CounterMode=TIM_CounterMode_Up;
	Init_Timer.TIM_Period=32;
	Init_Timer.TIM_Prescaler=800;

	TIM_TimeBaseInit(TIM2,&Init_Timer);


	//TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

	NVIC_InitStructure1.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure1.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure1.NVIC_IRQChannelPreemptionPriority = 2;                 // this sets the priority group of the USART2 interrupts
	NVIC_InitStructure1.NVIC_IRQChannelSubPriority = 0;                                                         // the USART2 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure1);





}
void Disable_all()
{
	// GPIOA->ODR = 0;
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	NVIC_DisableIRQ(EXTI15_10_IRQn);
//	NVIC_DisableIRQ(USART2_IRQn);
	NVIC_DisableIRQ(TIM2_IRQn);

//	GPIOE->ODR=0XF000;
	//USART_Cmd(USART2,DISABLE);
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);
	//TIM_Cmd(TIM4,DISABLE);
	TIM_Cmd(TIM2,DISABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);

}


int main(void)
{
	CLOCK_Init();
	GPIO_Initialize();
	PIN_PWM();
	Pwm_Initialisation();
	Set_Usart();
	Initialize_GPIO_External_Interrupt();
	Init_timer();
	Enable_all();
	//GPIOE->ODR=0xaa00;
	//TIM_Cmd(TIM4,ENABLE);

	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);
	TIM_Cmd(TIM2,ENABLE);
	//TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

	int i=0,k=0;
//	TIM_SetCompare1(TIM4,100);
//	TIM_SetCompare2(TIM4,100);
//	TIM_SetCompare3(TIM4,50);
//	TIM_SetCompare4(TIM4,50);
	while(1)
	{
		TIM_SetCompare1(TIM4,100);
		TIM_SetCompare3(TIM4,0);
		/*if(GPIO_ReadInputDataBit( GPIOD,GPIO_Pin_6))
		{
			GPIOE->ODR|=0x0f00;
		}
		else
		{
			GPIOE->ODR|=0xf000;
		}*/
		//GPIOE->ODR=0x0000;
		USART_SendData( USART2,5);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
		if(T2_INT_FLAG==1)
		{

		/*	TIM_SetCompare1(TIM4,Right_Pid(err[1],d_err[1],ocr_main[2]));
			TIM_SetCompare2(TIM4,Left_Pid(err[0],d_err[0],ocr_main[0]));//motor 0
			TIM_SetCompare3(TIM4,0);
			TIM_SetCompare4(TIM4,0);//motor 0*/


			//Fuzzy_API(err,d_err,ocr_main);
			//TIM4->CCR1=ocr_main[0];	//ocr to motor1
		/*	TIM_SetCompare1(TIM4,ocr_main[2]);
			TIM_SetCompare2(TIM4,ocr_main[0]);//motor 0
			TIM_SetCompare3(TIM4,ocr_main[3]);
			TIM_SetCompare4(TIM4,ocr_main[1]);//motor 0
			//TIM4->CCR3=ocr_main[2];	*/		//ocr to motor2//
			/*TIM_SetCompare1(TIM4,0);
			TIM_SetCompare2(TIM4,0);
			TIM_SetCompare3(TIM4,0);
			TIM_SetCompare4(TIM4,0);*/
			temp=TIM_GetCapture1(TIM4);
			Left_Pid(temp);
			TIM_SetCompare1(TIM4,100);
			TIM_SetCompare3(TIM4,0);
			T2_INT_FLAG = 0;
			//j=0;
		}

		if(j >= 100)
		{

			Disable_all();
			j=0;
			T2_INT_FLAG = 0;

			for(i=0;i<2;i++)
			{

				for(k=0;k<100;k++)
				{
					USART_SendData(USART2,Store_Ticks[i][k]);
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
				}

			}


			//GPIOE->ODR = 0x8800;

		}
	}

}

