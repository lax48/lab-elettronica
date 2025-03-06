

#include "stm32h7xx_hal.h"
#include "stm32h743xx.h"
#include "exponential.h"


uint16_t indice=500; //inidice trasmissione

uint32_t esponenziale_1[100], esponenziale_2[100]; //vettori di salavtaggio
uint16_t stringa[100]; //sottrazione exp

uint32_t costante_1 = ((0.935506985)*(1<<10)); //costanti tau
uint32_t costante_2 = ((0.904837418)*(1<<10)); //costanti tau
uint8_t*stringa_p = (uint8_t *)stringa; //puntatore per invio


void exp_USART_iniz(void){

	//Accendo la trasmissione
	//USART3->CR1 |= USART_CR1_TE;
	//Accenderemo la ricezione
	USART3->CR1 |= USART_CR1_RE;
	//Abilito interruput ricezione
	USART3->CR1 |=USART_CR1_RXNEIE ;
	//Abilito interrupt trasmissione
	USART3->CR1 |=USART_CR1_TCIE ;
	//Accendo USART
	USART3->CR1 |= USART_CR1_UE;

}

//funzione calcolo esponenziali
void exponential(void) {
	esponenziale_1[0]= (uint32_t)(65000); //inizio vettore
	esponenziale_2[0] = (uint32_t)(65000); //inizio vettore
	for(indice = 1 ;indice < 100; indice++) {
		esponenziale_1[indice]= (esponenziale_1[indice - 1]*costante_1 ) >> 10;
		esponenziale_2[indice]= (esponenziale_2[indice - 1]*costante_2 )>> 10;
		stringa[indice] = esponenziale_1[indice] - esponenziale_2[indice];
	}
}


void exp_USART_interrupt(void){
	//codice per la gestione della trasmissione
	if( USART3->ISR & USART_ISR_TC){
		if (indice < 200){ //trasmissione a 8 bit. Vettore a 16 bit.
			USART3 -> TDR = stringa_p[indice];
			indice++;
		}else{
			USART3->CR1 &= ~USART_CR1_TCIE; //spengo interrupt trasm
			USART3->CR1 |= USART_CR1_RE; //accendo ricezione
			indice = 1;
		}
	}
	//codice per la gestione della ricezione
	if( USART3->ISR & USART_ISR_RXNE_RXFNE){
		if (USART3->RDR == 'w'){//'w' è il carattere scelto per MATLAB
			USART3->CR1 &= ~USART_CR1_RE; //spengo ricezione
			exponential(); //Calcolo
			USART3->CR1 |= USART_CR1_TCIE; //accendo interrupt trasmissione
			USART3 -> TDR = stringa_p[0]; //incvio il primo
			indice = 1;
		}
	}

	USART3->ICR |= USART_ICR_TCCF ;  //Azzeramento flag interrupt trasmissione
	USART3->RQR |= USART_RQR_RXFRQ;  //Azzeramento flag interrupt ricezione
	USART3->ICR |= USART_ICR_ORECF; //Cancella l'overrun

}
