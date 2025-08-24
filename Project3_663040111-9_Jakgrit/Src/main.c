#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define STM32F411xE
#include "stm32f4xx.h"

#define VREF 3.3f
#define VCC 3.3f
#define ADC_MAXRES 4095.0f

#define RX_NTC 10000.0f
#define R0_NTC 10000.0f
#define T0_NTC 298.15f
#define BETA_NTC 3950.0f

#define RX_LDR 10000.0f
#define SLOPE_LDR -0.6875f
#define OFFSET_LDR 5.1276f

volatile uint16_t g_adc_results[2];
volatile char g_received_char = 0;
volatile uint8_t g_new_data_flag = 0;
#define TX_BUFFER_SIZE 100
char g_tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t g_tx_read_idx = 0;
volatile uint16_t g_tx_write_idx = 0;

void uart_send_string(const char *str) {
	while (g_tx_read_idx != g_tx_write_idx)
		;
	strncpy(g_tx_buffer, str, TX_BUFFER_SIZE);
	g_tx_read_idx = 0;
	g_tx_write_idx = strlen(str);
	USART2->CR1 |= USART_CR1_TXEIE;
}

void USART2_IRQHandler(void) {
	if (USART2->SR & USART_SR_RXNE) {
		g_received_char = USART2->DR;
		g_new_data_flag = 1;
	}
	if (USART2->SR & USART_SR_TXE) {
		if (g_tx_read_idx < g_tx_write_idx) {
			USART2->DR = g_tx_buffer[g_tx_read_idx++];
		} else {
			USART2->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}

int main(void) {

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));

	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER5_Pos)
			| (0b01 << GPIO_MODER_MODER6_Pos) | (0b01 << GPIO_MODER_MODER7_Pos);
	GPIOA->MODER |= (0b11 << GPIO_MODER_MODER0_Pos)
			| (0b11 << GPIO_MODER_MODER1_Pos);
	GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos)
			| (0b10 << GPIO_MODER_MODER3_Pos);
	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);

	USART2->BRR = 139;
	USART2->CR1 |=
			USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream0->CR & DMA_SxCR_EN)
		;
	DMA2->LIFCR = 0x3F;
	DMA2_Stream0->PAR = (uint32_t) &(ADC1->DR);
	DMA2_Stream0->M0AR = (uint32_t) g_adc_results;
	DMA2_Stream0->NDTR = 2;

	DMA2_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC
			| (1 << DMA_SxCR_PSIZE_Pos) | (1 << DMA_SxCR_MSIZE_Pos) |
			DMA_SxCR_CIRC;
	DMA2_Stream0->CR |= DMA_SxCR_EN;

	ADC1->SQR1 = (1 << ADC_SQR1_L_Pos);
	ADC1->SQR3 = (0 << ADC_SQR3_SQ1_Pos) | (1 << ADC_SQR3_SQ2_Pos);
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
	ADC1->CR2 |= ADC_CR2_ADON;
	for (volatile int i = 0; i < 1000; i++)
		;
	ADC1->CR2 |= ADC_CR2_SWSTART;

	NVIC_EnableIRQ(USART2_IRQn);

	char buffer[60];
	while (1) {
		if (g_new_data_flag) {
			g_new_data_flag = 0;
			switch (g_received_char) {
			case '0':
				GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
				uart_send_string("OK\n");
				break;
			case '1':
				GPIOA->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BS6 | GPIO_BSRR_BS7;
				uart_send_string("OK\n");
				break;
			case 't': {
				float adc_voltage = (g_adc_results[0] * VREF) / ADC_MAXRES;
				float r_ntc = RX_NTC * adc_voltage / (VCC - adc_voltage);
				float temperature = ((BETA_NTC * T0_NTC)
						/ (T0_NTC * log(r_ntc / R0_NTC) + BETA_NTC)) - 273.15f;
				sprintf(buffer, "Temperature = %d millidegree Celcius\n",
						(int) (temperature * 1000));
				uart_send_string(buffer);
				break;
			}
			case 'l': {
				float adc_voltage = (g_adc_results[1] * VREF) / ADC_MAXRES;
				float r_ldr = RX_LDR * adc_voltage / (VCC - adc_voltage);
				float light_intensity = pow(10,
						(log10(r_ldr) - OFFSET_LDR) / SLOPE_LDR);
				sprintf(buffer, "Light intensity = %d Lux\n",
						(int) light_intensity);
				uart_send_string(buffer);
				break;
			}
			default:
				uart_send_string("Invalid Command\n");
				break;
			}
		}
	}
}
