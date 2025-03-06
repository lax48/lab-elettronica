/*
 * SiPM.h
 *
 *  Created on: Mar 4, 2025
 *      Author: giaco
 */

#ifndef INC_SIPM_H_
#define INC_SIPM_H_



void SiPM_ADC_iniz(void);
void SiPM_DMA_interrupt(void);
void SiPM_ADC_interrupt(void);
void SiPM_USART_interrupt(void);
void SiPM_comp_iniz(void);
void SiPM_USART_iniz(void);
void SiPM_DMA_iniz(void);
void SiPM_timer_iniz(void);


#endif /* INC_SIPM_H_ */
