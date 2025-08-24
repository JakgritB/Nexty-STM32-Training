#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

// ค่าดีเลย์ระหว่างการเริ่มแปลง ADC แต่ละครั้ง
#define THRESHOLD 133333

// --- Interrupt Service Routine (ISR) สำหรับ ADC ---
// ตรรกะควบคุม LED ทั้งหมดจะอยู่ในนี้
void ADC_IRQHandler(void) {
	// ตรวจสอบว่า Interrupt เกิดจากธง End of Conversion (EOC)
	if ((ADC1->SR & ADC_SR_EOC) != 0) {
		// อ่านค่า ADC (การอ่านค่านี้จะล้างธง EOC โดยอัตโนมัติ)
		uint16_t adc_value = ADC1->DR;

		// --- ใช้ BSRR ควบคุม LED ตามค่า ADC ---
		if (adc_value > 3280) { // ระดับ 0: ปิดทุกดวง 3280
			GPIOB->BSRR = GPIO_BSRR_BR6;
			GPIOA->BSRR = GPIO_BSRR_BR7;
			GPIOA->BSRR = GPIO_BSRR_BR6;
			GPIOA->BSRR = GPIO_BSRR_BR5;
		} else if (adc_value > 2460) { // ระดับ 1: เปิด PB6 2460
			GPIOB->BSRR = GPIO_BSRR_BS6; // On
			GPIOA->BSRR = GPIO_BSRR_BR7;
			GPIOA->BSRR = GPIO_BSRR_BR6;
			GPIOA->BSRR = GPIO_BSRR_BR5;
		} else if (adc_value > 1640) { // ระดับ 2: เปิด PB6, PA7 1640
			GPIOB->BSRR = GPIO_BSRR_BS6;
			GPIOA->BSRR = GPIO_BSRR_BS7; // On
			GPIOA->BSRR = GPIO_BSRR_BR6;
			GPIOA->BSRR = GPIO_BSRR_BR5;
		} else if (adc_value > 820) { // ระดับ 3: เปิด PB6, PA7, PA6 820
			GPIOB->BSRR = GPIO_BSRR_BS6;
			GPIOA->BSRR = GPIO_BSRR_BS7;
			GPIOA->BSRR = GPIO_BSRR_BS6; // On
			GPIOA->BSRR = GPIO_BSRR_BR5;
		} else { // ระดับ 4: เปิดทุกดวง
			GPIOB->BSRR = GPIO_BSRR_BS6;
			GPIOA->BSRR = GPIO_BSRR_BS7;
			GPIOA->BSRR = GPIO_BSRR_BS6;
			GPIOA->BSRR = GPIO_BSRR_BS5; // On
		}
	}
}

int main(void) {
	// --- 1. เปิด Clock ---
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// --- 2. ตั้งค่า GPIO ---
	// Outputs for LEDs: PA5, PA6, PA7, PB6
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER5_Pos);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);
	// Analog Input for Potentiometer: PA4
	GPIOA->MODER |= (0b11 << GPIO_MODER_MODER4_Pos);

	// --- 3. ตั้งค่า ADC ---
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC1->SQR1 &= ~(ADC_SQR1_L); // L=0b0000 หมายถึง 1 conversion
	ADC1->SQR3 = (4 << ADC_SQR3_SQ1_Pos); // เลือก Channel 4 (PA4)
	ADC1->CR1 |= ADC_CR1_EOCIE; // เปิดใช้งาน EOC Interrupt

	// --- 4. ตั้งค่า NVIC ---
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn, 0);

	// --- 5. Main Loop ---
	while (1) {
		// สั่งให้ ADC เริ่มการแปลงค่า
		ADC1->CR2 |= ADC_CR2_SWSTART;

		// หน่วงเวลา เพื่อสร้างจังหวะในการอ่านค่าครั้งต่อไป
		for (volatile uint32_t iter = 0; iter < THRESHOLD; iter++) {
			// Do nothing, just wait
		}
	}
}
