#include <stm32h7xx.h>
#include "lz_adc.h"

void lz_adc_calibration(void)
{
	ADC3->ISR = 0xFFFFFFFF;
	ADC3->CR &= ~ADC_CR_ADCALDIF; // calibrazione per misura non differenziale (single end)
	ADC3->CR |= ADC_CR_ADCALLIN; // calibrazione lineare
	ADC3->CR &= ~ADC_CR_ADEN; // disabilita ADC
	ADC3->CR |= ADC_CR_ADCAL; // parti calibrazione

	while (ADC3->CR & ADC_CR_ADCAL) // attendi fine calibrazione
	{
	}
}

void lz_adc_enable(void)
{
	ADC3->ISR |= ADC_ISR_ADRDY; // azzero flag scrivendogli 1
	ADC3->CR |= ADC_CR_ADEN; // accendo ADC

	while (~ADC3->ISR & ADC_ISR_ADRDY) // aspetto che si accenda
	{
	}

	ADC3->ISR |= ADC_ISR_ADRDY; // (opzionale)

	//ADC3->IER |= ADC_IER_EOCIE | ADC_IER_EOSIE | ADC_IER_OVRIE; // abilito interrupt di conversione finita, fine sequenza e overrun
}

void lz_adc_prepare_channel(uint8_t pcsel, enum lz_sample_time sampleTime)
{
	ADC3->PCSEL |= (2 << pcsel) << ADC_PCSEL_PCSEL_Pos;

	if (pcsel < 10)
	{
		ADC3->SMPR1 |= sampleTime << 3 * pcsel;
	}
	else
	{
		ADC3->SMPR2 |= sampleTime << 3 * (pcsel - 10);
	}
}

void lz_adc_sequence(uint8_t* seq, uint8_t size)
{
	ADC3->SQR1 = 0;
	ADC3->SQR2 = 0;
	ADC3->SQR3 = 0;
	ADC3->SQR4 = 0;

	ADC3->SQR1 |= (size - 1) << ADC_SQR1_L_Pos;

	switch (size)
	{
	case 16:
		ADC3->SQR4 |= seq[15] << ADC_SQR4_SQ16_Pos;
	case 15:
		ADC3->SQR4 |= seq[14] << ADC_SQR4_SQ15_Pos;
	case 14:
		ADC3->SQR3 |= seq[13] << ADC_SQR3_SQ14_Pos;
	case 13:
		ADC3->SQR3 |= seq[12] << ADC_SQR3_SQ13_Pos;
	case 12:
		ADC3->SQR3 |= seq[11] << ADC_SQR3_SQ12_Pos;
	case 11:
		ADC3->SQR3 |= seq[10] << ADC_SQR3_SQ11_Pos;
	case 10:
		ADC3->SQR3 |= seq[9] << ADC_SQR3_SQ10_Pos;
	case 9:
		ADC3->SQR2 |= seq[8] << ADC_SQR2_SQ9_Pos;
	case 8:
		ADC3->SQR2 |= seq[7] << ADC_SQR2_SQ8_Pos;
	case 7:
		ADC3->SQR2 |= seq[6] << ADC_SQR2_SQ7_Pos;
	case 6:
		ADC3->SQR2 |= seq[5] << ADC_SQR2_SQ6_Pos;
	case 5:
		ADC3->SQR2 |= seq[4] << ADC_SQR2_SQ5_Pos;
	case 4:
		ADC3->SQR1 |= seq[3] << ADC_SQR1_SQ4_Pos;
	case 3:
		ADC3->SQR1 |= seq[2] << ADC_SQR1_SQ3_Pos;
	case 2:
		ADC3->SQR1 |= seq[1] << ADC_SQR1_SQ2_Pos;
	case 1:
		ADC3->SQR1 |= seq[0] << ADC_SQR1_SQ1_Pos;
		break;
	}
}

int lz_adc_interrupt(void)
{
	int handled = 0;
	static uint8_t state = 0;

	if (ADC3->ISR & ADC_ISR_EOC) {
		uint16_t dataRead = ADC3->DR; // disables interrupt
		__lz_adc_new_data(state, dataRead);

		state++;
		handled = 1;
	}

	if (ADC3->ISR & ADC_ISR_EOS)
	{
		state = 0;
		handled = 1;

		ADC3->ISR = ADC_ISR_EOS;
	}

	if (ADC3->ISR & ADC_ISR_OVR) {
		ADC3->ISR |= ADC_ISR_OVR;
		ADC3->CR |= ADC_CR_ADSTP;

		__lz_adc_error_overrun();

		handled = 1;
	}

	return handled;
}
