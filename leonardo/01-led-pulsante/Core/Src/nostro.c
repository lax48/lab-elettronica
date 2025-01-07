#include "stm32h7xx.h"

// Funzione chiamata in main() dopo inizializzazione CubeMX
void our_main(void)
{
	// Da CubeMX: CK_PSC = 64 MHz

	TIM6->PSC = 64 * 1000;
	TIM6->ARR = 1000;
	TIM6->CNT = 0;

	TIM6->DIER |= TIM_DIER_UIE;

	TIM6->CR1 |= TIM_CR1_CEN;
}

int ledState = 0;
void tim6_interrupt(void)
{
	switch(ledState)
	{
	case 0:
		GPIOB->BSRR = GPIO_BSRR_BS0;
		GPIOE->BSRR = GPIO_BSRR_BR1;

		ledState++;
		break;
	case 1:
		GPIOB->BSRR = GPIO_BSRR_BR0;
		GPIOE->BSRR = GPIO_BSRR_BS1;

		ledState = 0;
		break;
	}

	TIM6->SR &= ~TIM_SR_UIF;
}

int speedState = 0;
void exti13_interrupt(void)
{
	switch(speedState)
	{
	case 0:
		TIM6->ARR = 100;
		speedState++;
		break;
	case 1:
		TIM6->ARR = 250;
		speedState++;
		break;
	case 2:
		TIM6->ARR = 500;
		speedState++;
		break;
	case 3:
		TIM6->ARR = 750;
		speedState++;
		break;
	case 4:
		TIM6->ARR = 1000;
		speedState = 0;
		break;
	}
	TIM6->EGR = TIM_EGR_UG;

	EXTI->PR1 = EXTI_PR1_PR13;
}
