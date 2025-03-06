#include <stm32h743xx.h>
#include <nostro.h>

#define N 100
#define P 20 //numero di dati di pretrigger


volatile uint16_t DMA_ADC_vettore[Na*(N+1)]; 

uint8_t* tx_ptr;
uint16_t tx_size, tx_pos;
uint32_t buffer_posizione;

void inizializza_USART(void)
{
	USART3->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE; 
	USART3->CR1 |= USART_CR1_TE; 
	USART3->CR1 |= USART_CR1_UE;

}

void inizializza_TIM6 (void) {
	  TIM6 -> PSC = 48; 
	  TIM6 -> ARR = 240; 
	  TIM6 -> CNT = 0; 
}

void inizializza_ADC(void)
{
	ADC3->CR &= ~ADC_CR_ADCALDIF; 
	ADC3->CR |= ADC_CR_ADCALLIN;
	ADC3->CR &= ~ADC_CR_ADEN; 
	ADC3->CR |= ADC_CR_ADCAL; 

	while (ADC3->CR & ADC_CR_ADCAL) // attendo fine calibrazione
	{}

	// abilito ADC
	ADC3->ISR = ADC_ISR_ADRDY;
	ADC3->CR |= ADC_CR_ADEN; // accendo ADC

	while (~ADC3->ISR & ADC_ISR_ADRDY) // aspetto che si accenda
	{}

	ADC3->ISR = ADC_ISR_ADRDY; 

	// Abilito gli interrupt
	ADC3->IER |= ADC_IER_EOCIE; 
	ADC3->PCSEL |= ADC_PCSEL_PCSEL_0; 
	ADC3->SQR1 = (0 << ADC_SQR1_SQ1_Pos) // 0 = canale di acquisizione dell'oscilloscopio
			| (0 << ADC_SQR1_L_Pos); 

	ADC3 -> CFGR |= (3<<ADC_CFGR_DMNGT_Pos);
	ADC3 -> CR |= ADC_CR_ADSTART;
	
}

void DMA_ADC (void){
	DMA2_Stream0 -> CR &= ~DMA_SxCR_EN;
	DMA2_Stream0 -> CR |= DMA_SxCR_CIRC;
	DMA2_Stream0 -> M0AR = (uint32_t) &DMA_ADC_vettore[volte*N];
	DMA2_Stream0 -> PAR = (uint32_t)(&ADC3->DR);
	DMA2_Stream0 -> NDTR = N;
	DMA2->LIFCR = 0xffffffff;
	DMA2->HIFCR = 0xffffffff;
	DMA2_Stream0 -> CR |= DMA_SxCR_EN;
}

int trigger = 0;
int volte = 0; 

void DMA_USART (void) {
	DMA1_Stream3 -> M0AR = (uint32_t) &DMA_ADC_vettore;
	DMA1_Stream3 -> PAR = (uint32_t)(&USART3->TDR);
	DMA1_Stream3 -> NDTR = Na*(N+1)*2;
	USART3 -> CR3 |= USART_CR3_DMAT; 
	DMA1_Stream3 -> CR |= DMA_SxCR_TCIE; 
	DMA1_Stream3 -> CR |= DMA_SxCR_EN; 
}

int count = 0;
int stato = 0; //flag che mi dice se sono sopra o sotto la soglia

void ACQUISIZIONE_MULTIPLA(void){
	if (volte < Na-1){
		DMA_ADC();
		TIM6 -> CR1 |= (TIM_CR1_CEN);
		count = 1;
		trigger = 1;
		stato = -1;
	}
	else if(volte == 9) {
		DMA_USART();
	}
}

void MATLAB (uint8_t c){
	volte = 0;
	ACQUISIZIONE_MULTIPLA();
}
void TRIGGER (void) {
	if (trigger) {
		uint16_t V = ADC3 -> DR; 
		if (V < V_SOGLIA_Low && !((DMA1_Stream3 -> CR & DMA_SxCR_EN) == DMA_SxCR_EN_Msk) && stato != 0){
			stato = 0;
		}
		if (V > V_SOGLIA  && !((DMA1_Stream3 -> CR & DMA_SxCR_EN) == DMA_SxCR_EN_Msk) && stato == 0) {
			stato = 1; 
			int punto_trigger = N - DMA2_Stream0 -> NDTR;
			DMA_ADC_vettore[N*Na+volte] = punto_trigger;
			
			volte++;
			trigger = 0;
		}
	}
	else if (stato == 1 && count < 79) {
		count++; 
	}
	else if (count == 79) {
		TIM6 -> CR1 &= ~(TIM_CR1_CEN); 
		ACQUISIZIONE_MULTIPLA();
	}
}

