
#include "stm32h7xx_hal.h"
#include "LED.h"

//varibile per indicare quale LED è in funzione:
//led = 0: giallo
//led = 1: verde
//led = 2: rosso
//led = 4: giallo spento
//led = 5: giallo acceso
volatile int led = 1;

//set iniziale timer
void LED_TIM6_iniz(void){
	TIM6 -> PSC = 5120;
	TIM6 -> ARR = 50000;
	TIM6 -> CR1 |= TIM_CR1_CEN;
	TIM6 -> DIER |= TIM_DIER_UIE;
}

//funzione interrupt TIM6
void LED_TIM6_interrupt(void){//implementazione semaforo
	//lampeggiamento giallo
	if(led == 4){
		GPIOE->BSRR |= GPIO_BSRR_BS1;//accendo giallo
		led = 5;//stato giallo: acceso
	}
	else if(led == 5) {
		GPIOE->BSRR |= GPIO_BSRR_BR1;//spengo giallo
		led = 4;//stato giallo: spento
	}

	//semaforo in funzione

	//il rosso e il verde sono attivi per più tempo rispetto al giallo
	else if(led == 1){ //stato verde
		GPIOE->BSRR |= GPIO_BSRR_BR1; //spengo verde
		GPIOB->BSRR |= GPIO_BSRR_BS14;//accendo rosso
		TIM6 -> PSC = 5120; //frequenza
		led = 2; //led rosso
	}
	else if(led == 0){ //stato giallo
		GPIOB->BSRR |= GPIO_BSRR_BR0; //spengo giallo
		GPIOE->BSRR |= GPIO_BSRR_BS1;//accendo verde
		led = 1; //verde
		TIM6 -> PSC = 5120; //frequenza
	}
	else if(led == 2){ //stato rosso
		GPIOB->BSRR |= GPIO_BSRR_BR14; //spengo rosso
		GPIOB->BSRR |= GPIO_BSRR_BS0; //accendo giallo
		led = 0; //giallo
		TIM6 -> PSC = 2560; //frequenza
	}
}

void LED_button_interrupt(void){
	//giallo lampeggiante al pressione del push-button
	if(led == 4 || led == 5) { //reset del semaforo
		//spengo led giallo
		GPIOE->BSRR |= GPIO_BSRR_BR1;
		//diminuisco frequenza
		TIM6 -> PSC = 5120;
		//accendo rosso
		led = 1;
	}
	else { //inizio lampeggiamento
		//spengo tutti i led
		GPIOE->BSRR |= GPIO_BSRR_BR1;
		GPIOB->BSRR |= GPIO_BSRR_BR0;
		GPIOB->BSRR |= GPIO_BSRR_BR14;
		//aumento la frequenza
		TIM6 -> PSC = 1280;
		//stato giallo: spento
		led = 4;
	}
}

