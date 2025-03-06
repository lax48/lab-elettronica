
#include "stm32h7xx_hal.h"
#include "USART_RT.h"

//**************************************//
//nel file sono oresenti due implememntazioni:
//quella selezionata è il semplice eco
//quella commentata è la risposta a MATLAB di un impotetico messaggio
//di arrivo.
//**************************************//





unsigned char ricezione[] = "abcde" ; //seganle da ricevere
unsigned char trasmissione[] = "CiaoMat\r"; //seganle da mandare
unsigned char index_trasm = 0; //indice di trasmissione
unsigned char index = 0; //indice di ricezione

void RT_USART_iniz(void){
	//Accendo la trasmissione e ricezione
	 USART3 -> CR1 |= USART_CR1_RE;
	 USART3 -> CR1 |= USART_CR1_TE;
	 //Abilito interruput ricezione
	 USART3 -> CR1 |= USART_CR1_RXNEIE;
	 //Accendo USART
	 USART3 -> CR1 |= USART_CR1_UE;

	 //reset USART
	 USART3->ICR |= USART_ICR_ORECF;//Azzeramento flag interrupt trasmissione
	 USART3->ICR |= USART_ICR_TCCF;//Azzeramento flag interrupt ricezione
	 USART3->RQR |= USART_RQR_RXFRQ;//Cancella l'overrun.
}

void RT_USART_interrupt(void){

	if(USART3->ISR & USART_ISR_RXNE_RXFNE){ //flag ricezione
		ricezione[index] =  USART3 -> RDR; //salvataggio info
		if (ricezione[index] != '\r'){ //segnale di stop
			index++;
		}
		else { //reset trasmizzione
			USART3 -> CR1 |= USART_CR1_TCIE; //accendo interrupt trasmissione
			//inizio trasmissione
			//eco
			index = 0;
			USART3 -> TDR = ricezione[index];
			index++;
			//risposta
//			USART3 -> TDR = trasmissione[index_trasm];
//			index_trasm++;
		}
	}
	//eco
	if(USART3->ISR & USART_ISR_TC){
		if (ricezione[index] != '\r'){
			USART3 -> TDR = ricezione[index];
			index++;
		}else{
		//spengo l'interrupt
			USART3 -> TDR = ricezione[index]; //mando segnale di fine
			index = 0; //reset
			}
		}
	//risposta
//	if(USART3->ISR & USART_ISR_TC){
//			if (trasmissione[index_trasm] != '\r'){
//				USART3 -> TDR = trasmissione[index_trasm];
//				index_trasm++;
//			}else{
//			//spengo l'interrupt
//				USART3 -> TDR = trasmissione[index_trasm]; //mando segnale di fine
//				index_trasm = 0; //reset
//				}
//			}


	USART3->ICR |= USART_ICR_ORECF; //Cancella l'overrun. Capita quando si entra in debug
	USART3->ICR |= USART_ICR_TCCF;  //Azzeramento flag interrupt trasmissione
	USART3->RQR |= USART_RQR_RXFRQ;  //Azzeramento flag interrupt ricezione
}
