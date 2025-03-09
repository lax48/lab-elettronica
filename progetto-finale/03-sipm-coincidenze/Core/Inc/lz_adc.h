#pragma once

void lz_adc_calibration(ADC_TypeDef* adc);
void lz_adc_enable(ADC_TypeDef* adc);

enum lz_sample_time
{
	LZ_ST_1_5_CLOCK = 0,
	LZ_ST_2_5_CLOCK = 1,
	LZ_ST_8_5_CLOCK = 2,
	LZ_ST_16_5_CLOCK = 3,
	LZ_ST_32_5_CLOCK = 4,
	LZ_ST_64_5_CLOCK = 5,
	LZ_ST_387_5_CLOCK = 6,
	LZ_ST_810_5_CLOCK = 7

};

void lz_adc_prepare_channel(ADC_TypeDef* adc, uint8_t pcsel, enum lz_sample_time sampleTime);
void lz_adc_sequence(ADC_TypeDef* adc, uint8_t* seq, uint8_t size);

#define LZ_ADC_SEQUENCE(adc, ...) 																				\
	do {																									\
		uint8_t __lz_adc_sequence_buff##__LINE__[] = { __VA_ARGS__ };										\
		lz_adc_sequence(adc, __lz_adc_sequence_buff##__LINE__,													\
				sizeof(__lz_adc_sequence_buff##__LINE__) / sizeof(__lz_adc_sequence_buff##__LINE__[0]));	\
	} while(0)

// Events (implemented in main.c)
void __lz_adc_error_overrun(void);
void __lz_adc_new_data(uint8_t state, uint16_t dataRead);
