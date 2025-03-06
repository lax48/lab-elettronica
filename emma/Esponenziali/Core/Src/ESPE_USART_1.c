/*
 * ESPE_seriale.c
 *
 *  Created on: Oct 20, 2023
 *      Author: utente
 */
//Zona include

/* *************************************************************************************************************
Ricordiamo di includere il file .h in ogni file .c dove useremo le funzioni definiti qui, ovvero nel main.c e stm32h7xx_it.c.
Nel main.c le funzioni da usare sono, in sequenza:
inizializzazione_vettore_da_spedire();
inizializzo_USART();

Mentre nella funzione USART3_IRQHandler() di stm32h7xx_it.c va usata:
ESPE_USART_interrupt();
************************************************************************************************************* */


#include "ESPE_USART_1.h"
#include "stm32h743xx.h"

#define costante_1   (uint16_t)((0.93551)*(1<<10) )
#define costante_2   (uint16_t)((0.90484)*(1<<10) )

unsigned char stringa[]={"Buongiorno!\r"};
unsigned char indice=0;
uint16_t  vet1[100];
uint16_t vet2[100];
uint16_t vet[100];
uint8_t* vet_punt = (uint8_t*)vet; 

void inizializzazione_vettore_da_spedire(void){
	indice=0;
}


void inizializzo_USART(void){
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |=USART_CR1_RXNEIE ;
	USART3->CR1 |=USART_CR1_TCIE ;
	USART3->CR1 |= USART_CR1_UE;

}


void costruisci_vettore(void){
	vet1[0] = 65000;
	vet2[0] = 65000;
	vet[0] = vet2[0]-vet1[0];

	for (int ii = 1; ii < 100; ii++){
		vet1[ii] = (uint16_t)(((uint32_t)vet1[ii-1]*(uint32_t)costante_1) >>10);
		vet2[ii] = (uint16_t)(((uint32_t)vet2[ii-1]*(uint32_t)costante_2) >>10);
		vet[ii] = vet2[ii]-vet1[ii];
	}
}

void ESPE_USART_interrupt(void){
	indice = 0;
	if( USART3->ISR & USART_ISR_TC){
		if (indice < lunghezza_stringa){
			USART3 -> TDR = stringa[indice];
			indice ++;
		}else{
			USART3->CR1 |= USART_CR1_RE;
			USART3->CR1 &= ~USART_CR1_TCIE ;
		}
	}

	if( USART3->ISR & USART_ISR_RXNE_RXFNE){
		if (USART3->RDR == 'w'){
			USART3->CR1 &= ~USART_CR1_RE; 
			costruisci_vettore();
			USART3->CR1 |=USART_CR1_TCIE ;
			USART3->TDR = vet_punt[0];
		}else{
			USART3->TDR = USART3->RDR;
		}

	}
	USART3->ICR |= USART_ICR_TCCF ;  
	USART3->RQR |= USART_RQR_RXFRQ;  
	USART3->ICR |= USART_ICR_ORECF; 
}
