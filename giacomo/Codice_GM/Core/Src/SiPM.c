
#include "SiPM.h"
#include "stm32h7xx_hal.h"


//Commento generale:
//il codice nasce per poter acquisire un vettore di set di misure ("DMA_ADC_vettore").
//Ogni set di misure è composto da "numero_misure_ADC" ed esso è accompagnato da un valore
//("NDTR") utile per ricostruire la traccia in MATLAB.
//Per l'esperienza "SiPM", MATLAB richiede 10000 forme d'onda chiedendole al microcontrollore.
//Si sarebbe potuto modificare direttamente il codice e chiedere tramite MATLAB un solo vettore
//contenente tutte le misure

volatile int NN = 0; //numero di elementi per post trigger
volatile int TT = 0; //controllo di trigger
volatile int numero_misure_ADC = 100;//dimensione per set di misura
volatile int dimensione_misure = 1; //numero set di misure
volatile uint16_t DMA_ADC_vettore[(100 + 1)*1]; //vettore di set di misure
volatile int DD = 0; //contatore set di misura

void SiPM_ADC_iniz(void) {//Inizializzazione ADC
//impostare gli ingressi per la
//gli ingressi ed il numero di ingressi mediante il registro SQR1
	ADC3 -> SQR1 = 0;
	ADC3 -> SQR1 |= (0 << ADC_SQR1_SQ1_Pos);
	ADC3 -> SQR1 |= (0 << ADC_SQR1_L_Pos);

//impostare PCSEL: indicare quali canali leggiamo
//impostare la velocità massima degli ingressi
	ADC3 -> PCSEL |= ADC_PCSEL_PCSEL_0;
	//calibrazione
	ADC3 -> CR &= ~ADC_CR_DEEPPWD_Pos; //deep power down a 0
	ADC3 -> CR |= (1 << ADC_CR_ADVREGEN_Pos);//voltage reference a 1

	ADC3 -> CR  &= ~ADC_CR_ADCALDIF_Pos;//misure no differenziale
	ADC3 -> CR |= (1 << ADC_CR_ADCALLIN_Pos); //calibrazione lineare
	ADC3 -> CR &= ~ADC_CR_ADEN_Pos;//disabilitare adc
	ADC3 -> CR |= (1 << ADC_CR_ADCAL_Pos);//parte calibrazione
	while(ADC3 -> CR & ADC_CR_ADCAL){//attendere fine
	}

	ADC3 -> ISR  &= ~ADC_ISR_ADRDY_Pos;//spengo bit
	ADC3 -> CR |= (1 << ADC_CR_ADEN_Pos);//accendo adc
	while(ADC3 -> ISR & ADC_ISR_ADRDY){//aspetto accensione
	}
	ADC3 -> ISR  &= ~ ADC_ISR_ADRDY_Pos;//spengo bit
	ADC3 -> IER |= ADC_IER_EOCIE;//interrupt end of conversion

	//DMA non circolare

	//ADC3->CFGR |= (1<< ADC_CFGR_DMNGT_Pos);

	//DMA circolare
	ADC3->CFGR |= (3<< ADC_CFGR_DMNGT_Pos);
}


void SiPM_DMA_interrupt(void){
	if (!(DMA1-> LISR & DMA_LIFCR_CTCIF3)) //controllo
		return;
	//rest registri
	DMA1-> LIFCR = 0xffffffff;
	DMA1-> HIFCR = 0xffffffff;

	//misure discrete
	DMA1_Stream3->CR &= ~DMA_SxCR_EN; //disabilito DMA
	USART3->CR1 |= USART_CR1_RE; //accendo ricezione

	//misure continue

	//DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	//DMA2_Stream0->CR |= DMA_SxCR_EN;
	//ADC3 -> CR |= ADC_CR_ADSTART;
	//TIM6 -> CR1 |= TIM_CR1_CEN;
}

//DMA non circolare

//void DMA2_interrupt(void){
//	TIM6 -> CR1 &= ~TIM_CR1_CEN;
//	DMA2-> LIFCR = 0xffffffff;
//	DMA2-> HIFCR = 0xffffffff;
//
//	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
//	DMA1_Stream3->CR |= DMA_SxCR_EN;
//
//}

void SiPM_ADC_interrupt(void){

	//trigger digitale

//	data_pre = data_post;
//	data_post = ADC3 -> DR;
//	if(data_post > 30000 && data_pre > data_post){
//		N = 1;
//	}
//	if(N != 0){
//		N++;
//	}
//	if(N == 250){
//		data_post = 0;
//		data_pre = 0;
//		N = 0;
//		DMA_ADC_vettore[numero_misure_ADC] = DMA2_Stream0 -> NDTR;
//		TIM6 -> CR1 &= ~TIM_CR1_CEN;
//		DMA2-> LIFCR = 0xffffffff;
//		DMA2-> HIFCR = 0xffffffff;
//		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
//		DMA1_Stream3->CR |= DMA_SxCR_EN;
//	}


	//trigger analogico

	if(TT ==0){//trigger
		if(COMP12->SR&COMP_SR_C1VAL){
			TT = 1;
			NN = 0;
		}
	}
	else{
		NN++;
	}
	if(NN == 90){//post trigger
		TT = 0;
		NN = 0;
		//salvataggio indice DMA circolare
		DMA_ADC_vettore[(numero_misure_ADC*dimensione_misure) + DD] = DMA2_Stream0 -> NDTR;
		DD++;
		if(DD == 1){
			DD = 0;
			TIM6 -> CR1 &= ~TIM_CR1_CEN; //spengo timer
			//reset registri
			DMA2-> LIFCR = 0xffffffff;
			DMA2-> HIFCR = 0xffffffff;
			DMA2_Stream0->CR &= ~DMA_SxCR_EN; //spengo DMA

			//reset registri
			DMA1-> LIFCR = 0xffffffff;
			DMA1-> HIFCR = 0xffffffff;
			//assegno trasmissione DMA per salvataggio per il nuovo set di misure
			//mando tutto a USART
			DMA1_Stream3->M0AR = (uint32_t)&DMA_ADC_vettore;
			DMA1_Stream3 -> PAR = (uint32_t)(&USART3->TDR);
			DMA1_Stream3-> NDTR = (numero_misure_ADC*dimensione_misure + 1)*2;
			DMA1_Stream3->CR |= DMA_SxCR_EN;
		}
		else{
			DMA2_Stream0->CR &= ~DMA_SxCR_EN;
			//cambio riferiemnto per ADC
			DMA2_Stream0->M0AR = (uint32_t)&DMA_ADC_vettore[numero_misure_ADC*DD];
			DMA2_Stream0->CR |= DMA_SxCR_EN;
		}
	}
}

void SiPM_USART_interrupt(void){
	//codice per la gestione della sola ricezione
	if( USART3->ISR & USART_ISR_RXNE_RXFNE){
		if (USART3->RDR == 'w'){//'w' è il carattere scelto per MATLAB
			USART3->CR1 &= ~USART_CR1_RE;
			DMA2_Stream0->CR |= DMA_SxCR_EN;
			ADC3 -> CR |= ADC_CR_ADSTART;
			TIM6 -> CR1 |= TIM_CR1_CEN;
		}
	}

	USART3->ICR |= USART_ICR_TCCF ;  //Azzeramento flag interrupt trasmissione
	USART3->RQR |= USART_RQR_RXFRQ;  //Azzeramento flag interrupt ricezione
	USART3->ICR |= USART_ICR_ORECF; //Cancella l'overrun.

}

void SiPM_comp_iniz(void){
	COMP1->CFGR |= COMP_CFGRx_EN; //attivo comparatore

	DAC1->CR|=DAC_CR_EN1; //attivo DAC
	DAC1->DHR12R1 = 1240; //in 12 bit con fondoscala 3.3
	DAC1->SWTRIGR |= DAC_SWTRIGR_SWTRIG1; //trigger
}



void SiPM_USART_iniz(void){
	USART3->CR1 |= USART_CR1_UE;

	//Accendere la trasmissione
	//USART3->CR1 |= USART_CR1_TE;
	//Accendere la ricezione
	USART3->CR1 |= USART_CR1_RE;
	//Abilitare interruput ricezione
	USART3->CR1 |=USART_CR1_RXNEIE ;
	//DMA
	USART3->CR3 |= USART_CR3_DMAT;

}

void SiPM_DMA_iniz(void){
	//assegno a DMA i vettori che devono mandare
	DMA2_Stream0->M0AR = (uint32_t)&DMA_ADC_vettore;
	DMA1_Stream3->M0AR = (uint32_t)&DMA_ADC_vettore;
	//assegno le periferiche
	DMA2_Stream0 -> PAR = (uint32_t)(&ADC3->DR);
	DMA1_Stream3 -> PAR = (uint32_t)(&USART3->TDR);
	//fisso il contatore NDTR per ADC
	DMA2_Stream0 -> NDTR = numero_misure_ADC;
	//DMA non circolare
	//DMA1_Stream3-> NDTR = numero_misure_ADC * 2;
	//DMA2_Stream0->CR |= DMA_SxCR_TCIE;

	//DMA circolare
	//fisso NDTR per USART
	DMA1_Stream3-> NDTR = (numero_misure_ADC*dimensione_misure + 1)*2;
	DMA1_Stream3->CR |= DMA_SxCR_TCIE;

}

void SiPM_timer_iniz(void){
	//max numero possibile è 65535 (16 bit)
	//CLOCK/PSC = ARR e CLOCK = 240Mhz
	//con PSC = 64 e ARR = 62500, ho un timer di 1 secondo
	// moltiplicando PSC divido il timer per quel fattore
	/*
	TIM6 -> CNT = 0;
	TIM6 -> PSC = 64*100; // timer ogni 10 ms
	TIM6 -> ARR = 62500;
	*/

	//1MHz:0 0 240
	TIM6 -> CNT = 0;
	TIM6 -> PSC = 0;
	TIM6 -> ARR = 240;

}






