#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

#define PWM_RESOLUTION 1000

int main(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER5_Pos);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);

	GPIOA->MODER |= (0b11 << GPIO_MODER_MODER4_Pos);

	ADC1->SQR3 = (4 << ADC_SQR3_SQ1_Pos);
	ADC1->CR2 |= ADC_CR2_ADON;

	uint32_t brightness = 0;

	while (1) {
		ADC1->CR2 |= ADC_CR2_SWSTART;
		while ((ADC1->SR & ADC_SR_EOC) == 0)
			;
		uint16_t adc_value = ADC1->DR;

		brightness = (adc_value * PWM_RESOLUTION) / 4095;

		for (int i = 0; i < PWM_RESOLUTION; i++) {
			if (i > brightness) {
				GPIOA->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BS6 | GPIO_BSRR_BS7;
				GPIOB->BSRR = GPIO_BSRR_BS6;
			} else {
				GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
				GPIOB->BSRR = GPIO_BSRR_BR6;
			}
		}
	}
}
