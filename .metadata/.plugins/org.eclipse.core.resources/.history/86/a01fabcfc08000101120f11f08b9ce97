#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

const uint32_t THRESHOLD = 133333;

void display_bcd(int number) {
	// Bit 0 (LSB) PC7
	if (number & 0b0001) {
		GPIOC->BSRR = GPIO_BSRR_BS7;
	} else {
		GPIOC->BSRR = GPIO_BSRR_BR7;
	}

	// Bit 1 PA8
	if (number & 0b0010) {
		GPIOA->BSRR = GPIO_BSRR_BS8;
	} else {
		GPIOA->BSRR = GPIO_BSRR_BR8;
	}

	//Bit 2 PB10
	if (number & 0b0100) {
		GPIOB->BSRR = GPIO_BSRR_BS10;
	} else {
		GPIOB->BSRR = GPIO_BSRR_BR10;
	}

	// Bit 3 (MSB) PA9
	if (number & 0b1000) {
		GPIOA->BSRR = GPIO_BSRR_BS9;
	} else {
		GPIOA->BSRR = GPIO_BSRR_BR9;
	}
}

int main(void) {
	// Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	// GPIO Pins
	// PC7 , PA8, PB10, PA9
	GPIOC->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER8_Pos);
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODER10_Pos);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER9_Pos);

	// PA10
	GPIOA->MODER &= ~(0b11 << GPIO_MODER_MODER10_Pos);
	GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD10_Pos);

	// PB3
	GPIOB->MODER &= ~(0b11 << GPIO_MODER_MODER3_Pos);
	GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD3_Pos);

	// PB5
	GPIOB->MODER &= ~(0b11 << GPIO_MODER_MODER5_Pos);
	GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD5_Pos);

	int count = 0;
	display_bcd(count);

	while (1) {

		if ((GPIOA->IDR & GPIO_IDR_ID10) == 0) {
			count++;
			if (count > 9) {
				count = 0;
			}
			display_bcd(count);
			while ((GPIOA->IDR & GPIO_IDR_ID10) == 0);
		}

		else if ((GPIOB->IDR & GPIO_IDR_ID3) == 0) {
			count--;
			if (count < 0) {
				count = 9;
			}
			display_bcd(count);
			while ((GPIOB->IDR & GPIO_IDR_ID3) == 0);

		} else if ((GPIOB->IDR & GPIO_IDR_ID5) == 0) {
			count = 0;
			display_bcd(count);
			while ((GPIOB->IDR & GPIO_IDR_ID5) == 0);
		}
		for (uint32_t iter = 0; iter < THRESHOLD; iter++) {

		}
	}
	return 0;
}
