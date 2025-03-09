#include <stm32h7xx.h>
#include "lz_adc.h"

void lz_adc_calibration(ADC_TypeDef* adc)
{
	adc->ISR = 0xFFFFFFFF;
	adc->CR &= ~ADC_CR_ADCALDIF; // calibrazione per misura non differenziale (single end)
	adc->CR |= ADC_CR_ADCALLIN; // calibrazione lineare
	adc->CR &= ~ADC_CR_ADEN; // disabilita ADC
	adc->CR |= ADC_CR_ADCAL; // parti calibrazione

	while (adc->CR & ADC_CR_ADCAL) // attendi fine calibrazione
	{
	}
}

void lz_adc_enable(ADC_TypeDef* adc)
{
	adc->ISR |= ADC_ISR_ADRDY; // azzero flag scrivendogli 1
	adc->CR |= ADC_CR_ADEN; // accendo ADC

	while (~adc->ISR & ADC_ISR_ADRDY) // aspetto che si accenda
	{
	}

	adc->ISR |= ADC_ISR_ADRDY; // (opzionale)

	//adc->IER |= ADC_IER_EOCIE | ADC_IER_EOSIE | ADC_IER_OVRIE; // abilito interrupt di conversione finita, fine sequenza e overrun
}

void lz_adc_prepare_channel(ADC_TypeDef* adc, uint8_t pcsel, enum lz_sample_time sampleTime)
{
	adc->PCSEL |= (2 << pcsel) << ADC_PCSEL_PCSEL_Pos;

	if (pcsel < 10)
	{
		adc->SMPR1 |= sampleTime << 3 * pcsel;
	}
	else
	{
		adc->SMPR2 |= sampleTime << 3 * (pcsel - 10);
	}
}

void lz_adc_sequence(ADC_TypeDef* adc, uint8_t* seq, uint8_t size)
{
	adc->SQR1 = 0;
	adc->SQR2 = 0;
	adc->SQR3 = 0;
	adc->SQR4 = 0;

	adc->SQR1 |= (size - 1) << ADC_SQR1_L_Pos;

	switch (size)
	{
	case 16:
		adc->SQR4 |= seq[15] << ADC_SQR4_SQ16_Pos;
	case 15:
		adc->SQR4 |= seq[14] << ADC_SQR4_SQ15_Pos;
	case 14:
		adc->SQR3 |= seq[13] << ADC_SQR3_SQ14_Pos;
	case 13:
		adc->SQR3 |= seq[12] << ADC_SQR3_SQ13_Pos;
	case 12:
		adc->SQR3 |= seq[11] << ADC_SQR3_SQ12_Pos;
	case 11:
		adc->SQR3 |= seq[10] << ADC_SQR3_SQ11_Pos;
	case 10:
		adc->SQR3 |= seq[9] << ADC_SQR3_SQ10_Pos;
	case 9:
		adc->SQR2 |= seq[8] << ADC_SQR2_SQ9_Pos;
	case 8:
		adc->SQR2 |= seq[7] << ADC_SQR2_SQ8_Pos;
	case 7:
		adc->SQR2 |= seq[6] << ADC_SQR2_SQ7_Pos;
	case 6:
		adc->SQR2 |= seq[5] << ADC_SQR2_SQ6_Pos;
	case 5:
		adc->SQR2 |= seq[4] << ADC_SQR2_SQ5_Pos;
	case 4:
		adc->SQR1 |= seq[3] << ADC_SQR1_SQ4_Pos;
	case 3:
		adc->SQR1 |= seq[2] << ADC_SQR1_SQ3_Pos;
	case 2:
		adc->SQR1 |= seq[1] << ADC_SQR1_SQ2_Pos;
	case 1:
		adc->SQR1 |= seq[0] << ADC_SQR1_SQ1_Pos;
		break;
	}
}
