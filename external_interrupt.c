						// Device header
#include <stm32f30x.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <stm32f30x_usart.h>
#include <stm32f30x_tim.h>
#include <stm32f30x_exti.h>
#include <stm32f30x_syscfg.h>
volatile int Ticks=0;

void wait(int i)
{
	while(i--)
	{
	}
}

void CLOCK_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //APB clock for External interrup
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
}

void Initialize_GPIO()
{
	GPIO_InitTypeDef IO0,INITGPIO_E;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	IO0.GPIO_Pin=GPIO_Pin_5;
	IO0.GPIO_Mode=GPIO_Mode_IN;
	IO0.GPIO_OType=GPIO_OType_PP;
	IO0.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &IO0);

	IO0.GPIO_Pin=GPIO_Pin_10;
		IO0.GPIO_Mode=GPIO_Mode_IN;
		IO0.GPIO_OType=GPIO_OType_PP;
		IO0.GPIO_PuPd=GPIO_PuPd_DOWN;
		GPIO_Init(GPIOC, &IO0);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource5);
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource10);
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    //CONFIGURE EXTI NVIC TO LEAST PRIORITY
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);

     NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    //general output  for LEDS
    INITGPIO_E.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    INITGPIO_E.GPIO_Mode=GPIO_Mode_OUT;
    INITGPIO_E.GPIO_Speed=GPIO_Speed_50MHz;
    INITGPIO_E.GPIO_OType=GPIO_OType_PP;
    INITGPIO_E.GPIO_PuPd=GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE,&INITGPIO_E);


}

void EXTI9_5_IRQHandler()//ISR for PA
{


//GPIOE->ODR = 0;

//GPIOE->ODR = 0;

GPIOE->ODR=0xf000;
EXTI_ClearITPendingBit(EXTI_Line5);

}
void EXTI15_10_IRQHandler()//ISR for PA
{


//GPIOE->ODR = 0;

//GPIOE->ODR = 0;

GPIOE->ODR=0x0f00;
EXTI_ClearITPendingBit(EXTI_Line10);

}

int main(void)
{
	CLOCK_Init();
	Initialize_GPIO();
	int i=0;
	GPIOE->ODR = 0;
//	GPIOE->ODR = 0xffff;
//	wait(400000);
	GPIOE->ODR = 0;
	/*for(i=0;i<5;i++)
			{
				GPIOA->ODR=0x0001;
				wait(1000000);
				GPIOA->ODR=0x0000;
				wait(1000000);
						//GPIOE->ODR = 0;
			}*/
	while(1)
	{

	}

}
