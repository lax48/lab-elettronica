#include "stm32h7xx.h"

// Funzione chiamata in main() dopo inizializzazione CubeMX
void my_main(void)
{
	// Da CubeMX: CK_PSC = 64 MHz

	TIM6->PSC = 64 * 1000;
	TIM6->ARR = 1000;
	TIM6->CNT = 0;

	TIM6->DIER |= TIM_DIER_UIE;

	TIM6->CR1 |= TIM_CR1_CEN;
}

void tim6_interrupt(void)
{
	static int ledState = 0;
	switch(ledState)
	{
	case 0:
		GPIOB->BSRR = GPIO_BSRR_BS0;
		GPIOE->BSRR = GPIO_BSRR_BR1;
		GPIOB->BSRR = GPIO_BSRR_BR14;
		break;
	case 1:
		GPIOB->BSRR = GPIO_BSRR_BR0;
		GPIOE->BSRR = GPIO_BSRR_BS1;
		GPIOB->BSRR = GPIO_BSRR_BR14;
		break;
	case 2:
		GPIOB->BSRR = GPIO_BSRR_BR0;
		GPIOE->BSRR = GPIO_BSRR_BR1;
		GPIOB->BSRR = GPIO_BSRR_BS14;
		break;
	}

	ledState++;
	if (ledState >= 3)
		ledState = 0;

	TIM6->SR &= ~TIM_SR_UIF;
}

void exti13_interrupt(void)
{
	static int speedState = 0;
	switch(speedState)
	{
	case 0:
		TIM6->ARR = 100;
		break;
	case 1:
		TIM6->ARR = 250;
		break;
	case 2:
		TIM6->ARR = 500;
		break;
	case 3:
		TIM6->ARR = 750;
		break;
		case 4:
		TIM6->ARR = 1000;
		speedState = 0;
		break;
	}

	TIM6->EGR = TIM_EGR_UG;

	speedState++;
	if (speedState >= 4)
		speedState = 0;
	
	EXTI->PR1 = EXTI_PR1_PR13;
}
