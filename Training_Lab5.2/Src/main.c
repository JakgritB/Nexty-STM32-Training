#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

#define THRESHOLD 133333

void ADC_IRQHandler(void) {
	if ((ADC1->SR & ADC_SR_EOC) != 0) {
		if (ADC1->DR <= 2048) {
			GPIOA->ODR |= (GPIO_ODR_OD5);
		} else {
			GPIOA->ODR &= ~(GPIO_ODR_OD5);
		}
	}
}

int main(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// PA5 (LED)
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER5_Pos);

	// ADC 4
	GPIOA->MODER &= ~(GPIO_MODER_MODER4);
	GPIOA->MODER |= (0b11 << GPIO_MODER_MODER4_Pos);
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC1->SMPR2 |= ADC_SMPR2_SMP4;
	ADC1->SQR1 &= ~(ADC_SQR1_L);
	ADC1->SQR1 |= (1 << ADC_SQR1_L_Pos);
	ADC1->SQR3 &= ~(ADC_SQR3_SQ1);
	ADC1->SQR3 |= (0x04 << ADC_SQR3_SQ1_Pos);
	ADC1->CR1 |= ADC_CR1_EOCIE;

	//interrupt
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn, 0);

	//ADC1->CR1 |= (0b10 << ADC_CR1_RES_Pos);

	while (1) {
		ADC1->CR2 |= ADC_CR2_SWSTART;

		for (uint32_t iter = 0; iter < THRESHOLD; iter++) {

		}
	}
}
