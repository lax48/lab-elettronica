#include <stm32h743xx.h>

// Vado a prendere dalla memoria i valori di calibrazinoe del sensore di temperatura
// e della tensione interna di riferimento impostati di fabbrica
#define TCAL_30C	*(uint16_t*)(0x1FF1E820)
#define TCAL_110C	*(uint16_t*)(0x1FF1E840)
#define VCAL_REF	*(uint16_t*)(0x1FF1E860)

#define N 10 //numero di misurazioni che voglio compiere

uint16_t calibrazione[3];
uint16_t misure_temperatura[N];
uint16_t misure_tensione[N];


#define TX_STATE_IDLE 0
#define TX_STATE_CALIBRAZIONE 1
#define TX_STATE_TEMP 2
#define TX_STATE_TENS 3

uint8_t* tx_ptr;
uint16_t tx_size, tx_pos;
uint8_t tx_state = TX_STATE_IDLE;

void inizializza_calibrazione(void)
{
	calibrazione[0] = TCAL_30C;
	calibrazione[1] = TCAL_110C;
	calibrazione[2] = VCAL_REF;
}

void inizializza_seriale(void)
{
	USART3->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
	USART3->CR1 |= USART_CR1_TE | USART_CR1_TCIE;
	USART3->CR1 |= USART_CR1_UE;
}


void interrupt_seriale(void)
{
	if (USART3->ISR & USART_ISR_RXNE_RXFNE)
	{
		char c = USART3->RDR; 

		if (c == 'c')	
		{
			tx_state = TX_STATE_CALIBRAZIONE;
			tx_ptr = (uint8_t*)calibrazione;
			tx_size = sizeof(calibrazione); 

			tx_pos = 1;
			USART3->TDR = tx_ptr[0];
		}
	}

	if (USART3->ISR & USART_ISR_TC) 
	{
		if (tx_state == TX_STATE_IDLE)
		{
			USART3->ICR = USART_ICR_TCCF;
		}
		else if (tx_pos < tx_size)
		{
			tx_pos++;
			USART3->TDR = tx_ptr[tx_pos - 1]; 
		}
		else
		{
			if (tx_state == TX_STATE_TENS)
			{
				tx_state = TX_STATE_TEMP;

				tx_ptr = (uint8_t*)misure_temperatura;
				tx_size = sizeof(misure_temperatura);

				tx_pos = 1;
				USART3->TDR = tx_ptr[0];
			}
			else
			{
				tx_state = TX_STATE_IDLE;
				USART3->ICR = USART_ICR_TCCF; 
			}
		}
	}
}

void impostazioni_iniziali_ADC(void)
{
	ADC3->CR &= ~ADC_CR_ADCALDIF; 
	ADC3->CR |= ADC_CR_ADCALLIN; 
	ADC3->CR &= ~ADC_CR_ADEN; 
	ADC3->CR |= ADC_CR_ADCAL; 

	while (ADC3->CR & ADC_CR_ADCAL) 
	{}

	ADC3->ISR = ADC_ISR_ADRDY; 
	ADC3->CR |= ADC_CR_ADEN; 

	while (~ADC3->ISR & ADC_ISR_ADRDY) 
	{}

	ADC3->ISR = ADC_ISR_ADRDY;
	ADC3->IER |= ADC_IER_EOCIE; 
	ADC3 -> CFGR |= ADC_CFGR_CONT;

	ADC3->PCSEL |= ADC_PCSEL_PCSEL_19 | ADC_PCSEL_PCSEL_18;
	ADC3->SQR1 = (19 << ADC_SQR1_SQ1_Pos) // 19 = VREF
			| (18 << ADC_SQR1_SQ2_Pos)    // 18 = VSENSE (termometro)
			| (1 << ADC_SQR1_L_Pos); 


	ADC3->SMPR2 |= 7 << ADC_SMPR2_SMP19_Pos; 
	ADC3->SMPR2 |= 7 << ADC_SMPR2_SMP18_Pos; 

	ADC3_COMMON->CCR |= ADC_CCR_TSEN | ADC_CCR_VREFEN; 
}

int N_mis = 0;
int slot = 1; // la prima misura e' di tensione

void interrupt_adc(void)
{
	if (ADC3->ISR & ADC_ISR_EOC) 
	{
		uint16_t meas = ADC3->DR; 

		if (slot == 1) 
		{
			misure_tensione[N_mis] = meas;
			slot = 2;
		}
		else if (slot == 2) 
		{
			misure_temperatura[N_mis] = meas;
			slot = 1; 
			N_mis++;
		}

		if (N_mis == N){
			ADC3 -> CR |= ADC_CR_ADSTP;
			TIM6 -> CR1 &= ~TIM_CR1_CEN;
			N_mis = 0;
			//invio il primo carattere (di tensione tramite TDR)
			tx_state = TX_STATE_TENS;

			tx_ptr = (uint8_t*)misure_tensione;
			tx_size = sizeof(misure_tensione);

			tx_pos = 1;
			USART3->TDR = tx_ptr[0];

		}
	}
}

void inizializza_TIM6 (void) {
	  TIM6 -> PSC = 48; 
	  TIM6 -> ARR = 100; 
	  TIM6 -> CNT = 0; 
	  TIM6 -> DIER |= TIM_DIER_UIE; 
}
