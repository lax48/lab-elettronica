/*
 * ESPE_seriale..h
 *
 *  Created on: Oct 20, 2023
 *      Author: utente
 */

/* *************************************************************************************************************
Ricordiamo di includere il file .h in ogni file .c dove useremo le funzioni definiti qui, ovvero nel main.c e stm32h7xx_it.c.
Nel main.c le funzioni da usare sono, in sequenza:
inizializzazione_vettore_da_spedire();
inizializzo_USART();

Mentre nella funzione USART3_IRQHandler() di stm32h7xx_it.c va usata:
ESPE_USART_interrupt();
************************************************************************************************************* */

// #ifndef INC_ESPE_SERIALE__H_
// #define INC_ESPE_SERIALE__H_

extern unsigned char stringa[];

#define lunghezza_stringa sizeof(stringa) /sizeof(stringa[0]) - 1
//il -1 ci sta perchè viene aggiunto uno 0 alla fine del vettore automaticamente

extern unsigned char indice;

// #endif /* INC_ESPE_SERIALE__H_ */

//Ricordiamoci che tutto quanto definito nel .c qui va ripreso con extern

void ESPE_USART_interrupt(void);
void inizializzo_USART(void);
void inizializzazione_vettore_da_spedire(void);
void costruisci_vettore(void);

