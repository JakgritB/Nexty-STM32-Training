#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define STM32F411xE
#include "stm32f4xx.h"

// --- Configuration Constants ---
// Pin Mapping
#define POT_GEAR_PIN      4  // PA4
#define LIGHT_SENSOR_PIN  1  // PA1
#define TEMP_SENSOR_PIN   0  // PA0
#define CLUTCH_PIN        10 // PA10
#define BRAKE_PIN         3  // PB3
#define GAS_PIN           5  // PB5
#define WIPER_BTN_PIN     4  // PB4
#define LED_WIPER_PIN     5  // PA5
#define LED_HEADLIGHT_PIN 6  // PA6
#define LED_AC_PIN        7  // PA7
#define LED_ENGINE_PIN    6  // PB6
#define BCD_MSB_PIN       9  // PA9
#define BCD_B2_PIN        10 // PB10
#define BCD_B1_PIN        8  // PA8
#define BCD_LSB_PIN       7  // PC7

// ADC and Sensor Constants
#define VREF              3.3f
#define ADC_MAX_VAL       4095.0f
#define NTC_RX            10000.0f
#define NTC_R0            10000.0f
#define NTC_T0            298.15f
#define NTC_BETA          3950.0f
#define LDR_RX            10000.0f
#define LDR_SLOPE         -0.6875f
#define LDR_OFFSET        5.1276f
#define TEMP_THRESHOLD_C  25.0f // Temperature to turn on AC
#define LIGHT_THRESHOLD_LUX 50.0f // Light level to turn on Headlights

// Gear ADC Thresholds (4096 / 6 states ~ 682 per state)
#define GEAR_N_MAX        682
#define GEAR_1_MAX        1365
#define GEAR_2_MAX        2048
#define GEAR_3_MAX        2730
#define GEAR_4_MAX        3413
// GEAR_5 is above 3413

// Simulation Constants
#define SYSTICK_FREQ_HZ   100   // Run SysTick Handler 100 times per second (every 10ms)
#define SPEED_UPDATE_RATE 10    // Update speed every 10 ticks (100ms)
#define UART_UPDATE_RATE  50    // Send speed over UART every 50 ticks (500ms)
#define WIPER_DELAY_TICKS 100   // 100 ticks = 1 second for wiper delay

// --- Type Definitions ---
typedef enum { GEAR_N=0, GEAR_1, GEAR_2, GEAR_3, GEAR_4, GEAR_5 } Gear_t;
typedef enum { MODE_ECO, MODE_NORMAL, MODE_SPORT } DriveMode_t;
typedef enum { WIPER_OFF, WIPER_DELAY, WIPER_NORMAL, WIPER_FAST } WiperMode_t;

// --- Global State Variables (volatile because they are modified in ISRs) ---
volatile uint8_t g_engine_on = 0;
volatile float g_current_speed = 0.0f; // in km/h
volatile Gear_t g_current_gear = GEAR_N;
volatile Gear_t g_pot_gear = GEAR_N; // Gear selected by potentiometer
volatile DriveMode_t g_drive_mode = MODE_NORMAL;
volatile WiperMode_t g_wiper_mode = WIPER_OFF;
volatile uint16_t g_adc_results[3]; // 0:Temp, 1:Light, 2:Gear

// Button press flags
volatile uint8_t g_clutch_pressed = 0;
volatile uint8_t g_brake_pressed = 0;
volatile uint8_t g_gas_pressed = 0;

// --- Function Prototypes ---
void init_all();
void init_gpio();
void init_uart();
void init_adc_with_dma();
void init_systick();
void init_exti();
void init_fpu();
void uart_send_string(const char* str);
void display_gear_bcd(Gear_t gear);
void update_leds();
Gear_t get_gear_from_adc(uint16_t adc_value);
void check_stall_conditions();

// --- Main Function ---
int main(void) {
    init_all();
    while (1) {
        // Wait for interrupts to do all the work
        __WFI();
    }
}

// --- Initialization ---
void init_all() {
    init_fpu();
    init_gpio();
    init_uart();
    init_adc_with_dma();
    init_exti();
    init_systick(); // Start timer last
}

void init_fpu() {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
}

void init_gpio() {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // --- Outputs ---
    // LEDs: PA5, PA6, PA7, PB6
    GPIOA->MODER |= (1 << GPIO_MODER_MODER5_Pos) | (1 << GPIO_MODER_MODER6_Pos) | (1 << GPIO_MODER_MODER7_Pos);
    GPIOB->MODER |= (1 << GPIO_MODER_MODER6_Pos);

    // 7-Segment BCD: PA9, PB10, PA8, PC7
    GPIOA->MODER |= (1 << GPIO_MODER_MODER9_Pos) | (1 << GPIO_MODER_MODER8_Pos);
    GPIOB->MODER |= (1 << GPIO_MODER_MODER10_Pos);
    GPIOC->MODER |= (1 << GPIO_MODER_MODER7_Pos);

    // --- Inputs ---
    // Buttons: PA10, PB3, PB5, PB4 as Input with Pull-up
    GPIOA->PUPDR |= (1 << GPIO_PUPDR_PUPD10_Pos);
    GPIOB->PUPDR |= (1 << GPIO_PUPDR_PUPD3_Pos) | (1 << GPIO_PUPDR_PUPD5_Pos) | (1 << GPIO_PUPDR_PUPD4_Pos);

    // --- Analog ---
    // PA0 (Temp), PA1 (Light), PA4 (Gear)
    GPIOA->MODER |= (3 << GPIO_MODER_MODER0_Pos) | (3 << GPIO_MODER_MODER1_Pos) | (3 << GPIO_MODER_MODER4_Pos);

    // --- Alternate Function ---
    // UART TX/RX: PA2, PA3
    GPIOA->MODER |= (2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos);
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);
}

void init_uart() {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // Baud rate 115200 for 16MHz APB1 clock: 16,000,000 / (16 * 115200) = 8.68
    // BRR = 8.68 -> Mantissa = 8, Fraction = 0.68 * 16 = 11. -> 0x8B
    USART2->BRR = 0x8B;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void init_adc_with_dma() {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // ADC Config: Scan mode, Continuous, DMA
    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    ADC1->SQR1 = (2 << ADC_SQR1_L_Pos); // 3 conversions
    ADC1->SQR3 = (TEMP_SENSOR_PIN << ADC_SQR3_SQ1_Pos)  | // SQ1 = IN0 (Temp)
                 (LIGHT_SENSOR_PIN << ADC_SQR3_SQ2_Pos) | // SQ2 = IN1 (Light)
                 (POT_GEAR_PIN << ADC_SQR3_SQ3_Pos);      // SQ3 = IN4 (Gear)

    // DMA2 Stream 0 Channel 0 Config
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream0->CR & DMA_SxCR_EN);
    DMA2_Stream0->PAR = (uint32_t)&(ADC1->DR);
    DMA2_Stream0->M0AR = (uint32_t)g_adc_results;
    DMA2_Stream0->NDTR = 3;
    DMA2_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
                       (1 << DMA_SxCR_PSIZE_Pos) | (1 << DMA_SxCR_MSIZE_Pos) |
                       DMA_SxCR_CIRC;
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    // Start ADC
    ADC1->CR2 |= ADC_CR2_ADON;
    for(volatile int i=0; i<1000; i++); // Wait for ADC to stabilize
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void init_exti() {
    // Enable SYSCFG clock for EXTI
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Connect GPIO pins to EXTI lines
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB; // PB3
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB | SYSCFG_EXTICR2_EXTI5_PB; // PB4, PB5
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PA; // PA10

    // Configure EXTI lines
    EXTI->IMR |= (1 << CLUTCH_PIN) | (1 << BRAKE_PIN) | (1 << GAS_PIN) | (1 << WIPER_BTN_PIN);
    EXTI->RTSR &= ~((1 << CLUTCH_PIN) | (1 << BRAKE_PIN) | (1 << GAS_PIN) | (1 << WIPER_BTN_PIN));
    EXTI->FTSR |= (1 << CLUTCH_PIN) | (1 << BRAKE_PIN) | (1 << GAS_PIN) | (1 << WIPER_BTN_PIN); // Falling edge

    // Enable IRQs in NVIC
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void init_systick() {
    SysTick_Config(SystemCoreClock / SYSTICK_FREQ_HZ); // e.g., 16MHz / 100 = 160000 ticks
}

// --- Interrupt Service Routines (ISRs) ---
void SysTick_Handler(void) {
    static uint32_t tick_counter = 0;
    static uint32_t wiper_counter = 0;

    // --- Read button states from GPIO registers ---
    // (GPIOx->IDR & (1 << PIN)) == 0 means pressed for pull-up
    g_clutch_pressed = (GPIOA->IDR & (1 << CLUTCH_PIN)) == 0;
    g_brake_pressed = (GPIOB->IDR & (1 << BRAKE_PIN)) == 0;
    g_gas_pressed = (GPIOB->IDR & (1 << GAS_PIN)) == 0;

    if (g_engine_on) {
        // --- Speed Update Logic (every SPEED_UPDATE_RATE ticks) ---
        if (tick_counter % SPEED_UPDATE_RATE == 0) {
            float acceleration = 0.0f;
            // Priority: Brake > Clutch > Gas
            if (g_brake_pressed) {
                g_current_speed -= 2.0f; // Braking force
            } else if (g_clutch_pressed) {
                // Engine disengaged, slight deceleration due to friction
                g_current_speed -= 0.1f;
            } else if (g_gas_pressed) {
                if (g_current_gear != GEAR_N) {
                    acceleration = 1.0f; // Base acceleration
                    if (g_drive_mode == MODE_ECO) acceleration *= 0.7f;
                    if (g_drive_mode == MODE_SPORT) acceleration *= 1.5f;
                }
            } else {
                g_current_speed -= 0.2f; // Engine braking/friction
            }
            g_current_speed += acceleration;

            // Clamp speed to 0
            if (g_current_speed < 0) g_current_speed = 0;

            // Speed limits per gear
            const int max_speeds[] = {0, 30, 50, 70, 90, 130};
            if (g_current_speed > max_speeds[g_current_gear]) {
                g_current_speed = max_speeds[g_current_gear];
            }
        }

        // --- Sensor Processing ---
        // ADC values are updated automatically by DMA in the background
        float temp_c, light_lux;

        // Temperature
        float adc_v_temp = (g_adc_results[0] * VREF) / ADC_MAX_VAL;
        float r_ntc = NTC_RX * adc_v_temp / (VREF - adc_v_temp);
        temp_c = ((NTC_BETA * NTC_T0) / (NTC_T0 * log(r_ntc / NTC_R0) + NTC_BETA)) - 273.15f;

        // Light
        float adc_v_light = (g_adc_results[1] * VREF) / ADC_MAX_VAL;
        float r_ldr = LDR_RX * adc_v_light / (VREF - adc_v_light);
        light_lux = pow(10, (log10(r_ldr) - LDR_OFFSET) / LDR_SLOPE);

        // Control AC and Headlights
        (temp_c > TEMP_THRESHOLD_C) ? (GPIOA->BSRR = GPIO_BSRR_BS7) : (GPIOA->BSRR = GPIO_BSRR_BR7);
        (light_lux < LIGHT_THRESHOLD_LUX) ? (GPIOA->BSRR = GPIO_BSRR_BS6) : (GPIOA->BSRR = GPIO_BSRR_BR6);

        // --- Gear Logic ---
        g_pot_gear = get_gear_from_adc(g_adc_results[2]);
        if (!g_clutch_pressed && g_current_gear != g_pot_gear) {
            g_engine_on = 0; // Stall! Changed gear without clutch
        }
        if(g_clutch_pressed){
            g_current_gear = g_pot_gear;
        }

        // --- Wiper Logic ---
        if (g_wiper_mode == WIPER_DELAY) {
            if (wiper_counter >= WIPER_DELAY_TICKS) {
                GPIOA->BSRR = GPIO_BSRR_BS5; // Turn on
                for(volatile int i=0; i<100000; i++); // Fake wipe duration
                GPIOA->BSRR = GPIO_BSRR_BR5; // Turn off
                wiper_counter = 0;
            } else {
                wiper_counter++;
            }
        } else if (g_wiper_mode == WIPER_NORMAL) {
             GPIOA->ODR ^= (1 << LED_WIPER_PIN); // Blink fast
        } else if (g_wiper_mode == WIPER_FAST) {
             GPIOA->BSRR = GPIO_BSRR_BS5; // Always on
        } else { // WIPER_OFF
             GPIOA->BSRR = GPIO_BSRR_BR5;
             wiper_counter = 0;
        }

        check_stall_conditions();
    }

    // --- UART Update (every UART_UPDATE_RATE ticks) ---
    if (tick_counter % UART_UPDATE_RATE == 0) {
        char buffer[30];
        sprintf(buffer, "Speed: %d km/h\n", (int)g_current_speed);
        if(g_engine_on) uart_send_string(buffer);
    }

    // --- Update all visual outputs ---
    update_leds();
    display_gear_bcd(g_current_gear);

    tick_counter++;
}

void EXTI3_IRQHandler(void) { // Brake PB3
    if (EXTI->PR & (1 << BRAKE_PIN)) {
        EXTI->PR = (1 << BRAKE_PIN); // Clear pending bit
    }
}

void EXTI4_IRQHandler(void) { // Wiper Button PB4
    if (EXTI->PR & (1 << WIPER_BTN_PIN)) {
        if(g_engine_on){
            g_wiper_mode = (g_wiper_mode + 1) % 4; // Cycle through 0,1,2,3
        }
        EXTI->PR = (1 << WIPER_BTN_PIN);
    }
}

void EXTI9_5_IRQHandler(void) { // Gas PB5
    if (EXTI->PR & (1 << GAS_PIN)) {
        EXTI->PR = (1 << GAS_PIN);
    }
}

void EXTI15_10_IRQHandler(void) { // Clutch PA10
    if (EXTI->PR & (1 << CLUTCH_PIN)) {
        EXTI->PR = (1 << CLUTCH_PIN);
    }
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char received_char = USART2->DR;
        switch (received_char) {
            case 'O': case 'o':
                if (!g_engine_on && g_current_gear == GEAR_N) {
                    g_engine_on = 1;
                    g_current_speed = 0;
                    uart_send_string("Engine ON\n");
                }
                break;
            case 'F': case 'f':
                if (g_engine_on && g_current_speed == 0) {
                    g_engine_on = 0;
                    uart_send_string("Engine OFF\n");
                }
                break;
            case 'E': case 'e': g_drive_mode = MODE_ECO; uart_send_string("Eco Mode\n"); break;
            case 'N': case 'n': g_drive_mode = MODE_NORMAL; uart_send_string("Normal Mode\n"); break;
            case 'S': case 's': g_drive_mode = MODE_SPORT; uart_send_string("Sport Mode\n"); break;
        }
    }
}

// --- Helper Functions ---
void uart_send_string(const char* str) {
    for (int i = 0; str[i] != '\0'; i++) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = str[i];
    }
}

void display_gear_bcd(Gear_t gear) {
    if(!g_engine_on){
        // Turn off all segments if engine is off
        GPIOA->BSRR = GPIO_BSRR_BR9 | GPIO_BSRR_BR8;
        GPIOB->BSRR = GPIO_BSRR_BR10;
        GPIOC->BSRR = GPIO_BSRR_BR7;
        return;
    }

    int number = gear; // N=0, 1=1, ... 5=5
    (number & 0b1000) ? (GPIOA->BSRR = GPIO_BSRR_BS9)  : (GPIOA->BSRR = GPIO_BSRR_BR9);  // MSB
    (number & 0b0100) ? (GPIOB->BSRR = GPIO_BSRR_BS10) : (GPIOB->BSRR = GPIO_BSRR_BR10);
    (number & 0b0010) ? (GPIOA->BSRR = GPIO_BSRR_BS8)  : (GPIOA->BSRR = GPIO_BSRR_BR8);
    (number & 0b0001) ? (GPIOC->BSRR = GPIO_BSRR_BS7)  : (GPIOC->BSRR = GPIO_BSRR_BR7);  // LSB
}

void update_leds() {
    (g_engine_on) ? (GPIOB->BSRR = GPIO_BSRR_BS6) : (GPIOB->BSRR = GPIO_BSRR_BR6);
    // Other LEDs (Wiper, Headlight, AC) are controlled directly in SysTick
}

Gear_t get_gear_from_adc(uint16_t adc_value) {
    if (adc_value <= GEAR_N_MAX) return GEAR_N;
    if (adc_value <= GEAR_1_MAX) return GEAR_1;
    if (adc_value <= GEAR_2_MAX) return GEAR_2;
    if (adc_value <= GEAR_3_MAX) return GEAR_3;
    if (adc_value <= GEAR_4_MAX) return GEAR_4;
    return GEAR_5;
}

void check_stall_conditions() {
    const int min_speeds[] = {0, 0, 20, 40, 60, 80};
    if (g_current_gear != GEAR_N && g_current_speed < min_speeds[g_current_gear]) {
        g_engine_on = 0; // Stall! Speed too low for the current gear.
    }
}
