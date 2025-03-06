/*
 * nostro.h
 *
 *  Created on: Nov 6, 2023
 *      Author: utente
 */

#ifndef INC_ESP_ADC_H_
#define INC_ESP_ADC_H_

#define V_SOGLIA     (uint16_t) (65535/3) // imposto il valore di soglia per il superamento per il trigger come metà del valore di riferimento
#define V_SOGLIA_Low (uint16_t) (V_SOGLIA*0.8)
#define Na 10 //definisco il numero di acquisizioni che voglio fare per ooi spedirle a matlab.

extern int trigger; //definizione di una variabile globale (che posso usare in tutti i file che includono nostro.h)
extern int volte;

void inizializza_USART(void);
void inizializza_ADC(void);
void inizializza_TIM6 (void);
void DMA_USART (void);
void DMA_ADC (void);
void MATLAB (uint8_t c);
void ACQUISIZIONE_MULTIPLA(void);
void TRIGGER(void);

#endif /* INC_ESP_ADC_H_ */

//#include "adc.h"
