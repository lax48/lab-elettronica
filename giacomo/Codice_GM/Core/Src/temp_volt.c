#include "temp_volt.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_it.h"

//costanti conversione
#define TCAL_30C *(uint16_t *)(0x1FF1E820)
#define TCAL_110C *(uint16_t *)(0x1FF1E840)

uint8_t n = 1; //contatore misura temp perchè più lunga
uint8_t k = 0; //contatore ADC
uint8_t t = 0; //variabile temporanea

//vettori misure
uint16_t vect_volt[100]; //16 bit
int32_t vect_temp[100]; //32 bit

//puntatori vettori
int8_t*temp_p = (int8_t *)vect_temp;
uint8_t*volt_p = (uint8_t *)vect_volt;

uint16_t trasm = 1000; //contatore trasmissione

void temp_volt_ADC_iniz(void) {//Inizializzazione ADC
	//impostare gli ingressi per la
	//gli ingressi ed il numero di ingressi mediante il registro SQR1
	ADC3 -> SQR1 = 0;
	ADC3 -> SQR1 |= (19 << ADC_SQR1_SQ1_Pos) | (18 << ADC_SQR1_SQ1_Pos);
	ADC3 -> SQR1 |= (1 << ADC_SQR1_L_Pos);

	//impostare PCSEL: indicare quali canali leggiamo
	//impostare la velocità massima degli ingressi
	ADC3 -> PCSEL |= ADC_PCSEL_PCSEL_19 | ADC_PCSEL_PCSEL_18;

	//calibrazione
	ADC3 -> CR &= ~ADC_CR_DEEPPWD_Pos; //deep power down a 0
	ADC3 -> CR |= (1 << ADC_CR_ADVREGEN_Pos); //voltage reference a 1

	ADC3 -> CR  &= ~ADC_CR_ADCALDIF_Pos; //misure single ended (no differenziali)
	ADC3 -> CR |= (1 << ADC_CR_ADCALLIN_Pos); //calibrazione lineare
	ADC3 -> CR &= ~ADC_CR_ADEN_Pos; //spengo ADC per calibrazione
	ADC3 -> CR |= (1 << ADC_CR_ADCAL_Pos); //inizio calibrazione
	while(ADC3 -> CR & ADC_CR_ADCAL){//attesa
	}

	//accensione
	ADC3 -> ISR  &= ~ADC_ISR_ADRDY_Pos; //spengo bit
	ADC3 -> CR |= (1 << ADC_CR_ADEN_Pos); //accensione ADC
	while(ADC3 -> ISR & ADC_ISR_ADRDY){ //aspetto accensione
	}
	ADC3 -> ISR  &= ~ ADC_ISR_ADRDY_Pos; //spengo bit
	ADC3 -> IER |= ADC_IER_EOCIE; //interrupt end of conversion

	//ritardo tra collegamento e misura: più preciso -> serve per temperatura
	ADC3_COMMON -> CCR |= 0xb << ADC_CCR_PRESC_Pos;
	ADC3 -> SMPR2 |= (7 << ADC_SMPR2_SMP18_Pos);


	//Per potere leggere la temperatura e la tensione occorre abilitare i flag 	TSEN e VREFEN nel CCR
	ADC3_COMMON -> CCR |= ADC_CCR_TSEN;
	ADC3_COMMON -> CCR |= ADC_CCR_VREFEN;

	//Nel registro CRGR indicare che misura si vuole considerare, CONT:
	//single o continua
	//con trigger per esempio con timer (TIM6)
	//oppure si può usare il trigger sw con ADSTART nel CR, consigliato all'inizio.
	ADC3 -> CFGR |= (1 << ADC_CFGR_CONT_Pos); //misure continue
	ADC3 -> CFGR |= ADC_CFGR_AUTDLY; //non inizia una misura se ce ne già una nel buffer
}//Fine inizializzazione ADC

//misure ADC
void temp_volt_ADC_interrupt(void){
	if(k < 100 && n == 1){
		vect_volt[k] = ADC3 -> DR; //verrà riscalato in MATLAB
		n = 0;
		k++;
	}
	else if(k < 101 && n == 0){
		t = ADC3 -> DR;
		//conversione interi temperatura
		vect_temp[k - 1] = 30000 + (((t - TCAL_30C) * 80000)/(TCAL_110C - TCAL_30C)); //mC
		n = 1;
	}
	else{ //fine misure
		ADC3 -> IER &= ~ADC_IER_EOCIE;
		USART3->CR1 |= USART_CR1_TCIE;
		USART3->CR1 |= USART_CR1_UE; //accendo USART
		trasm = 1; //set indice
		USART3 -> TDR = volt_p[0]; //inizio trasmissione
	}
}


void temp_volt_USART_interrupt(void){
	if( USART3->ISR & USART_ISR_TC){
		//codice per la gestione della sola trasmissione
		if (trasm < 200 && n == 1){
			USART3 -> TDR = volt_p[trasm];
			trasm++;
		}else if(trasm == 200 && n == 1){
			USART3 -> TDR = temp_p[0];
			trasm++;
			n = 0;
		//finisco di trasmettere la temperatura
		}else if(trasm > 200 && trasm < 600 && n == 0){
			USART3 -> TDR = temp_p[trasm - 200];
			trasm++;
		}else if(trasm == 600 && n == 0){
			USART3->CR1 &= ~USART_CR1_TCIE; //spengo l'interrupt trasmissione
			USART3->CR1 |= USART_CR1_RE; //inizio rice<ione
			trasm = 1000; //evito che USART entri nell'interrupt anche se non richiesto
			n = 1; //set per future misure
		}
	}

	if(USART3->ISR & USART_ISR_RXNE_RXFNE){
		//codice per la gestione della sola ricezione
		if (USART3->RDR == 'w'){//'w' è il carattere scelto epr rimandare il msg di benvenuto
			USART3->CR1 &= ~USART_CR1_RE;//Spego la ricezione fino a che trasmettiamo
			ADC3 -> CR |= ADC_CR_ADSTART;//inizio misure
			}
	}

	USART3->ICR |= USART_ICR_TCCF ;  //Azzeramento flag interrupt trasmissione
	USART3->RQR |= USART_RQR_RXFRQ;  //Azzeramento flag interrupt ricezione
	USART3->ICR |= USART_ICR_ORECF; //Cancella l'overrun

}



void temp_volt_USART_iniz(void){

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









