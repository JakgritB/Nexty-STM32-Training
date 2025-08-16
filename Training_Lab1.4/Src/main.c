#include <stdint.h>

#define RCC_Base    0x40023800
#define RCC_AHB1ENR (*(volatile uint32_t*) (RCC_Base + 0x30))

#define GPIOA_Base     0x40020000
#define GPIOA_MODER    (*(volatile uint32_t*) (GPIOA_Base + 0x00))
#define GPIOA_OTYPER   (*(volatile uint32_t*) (GPIOA_Base + 0x04))
#define GPIOA_OSPEEDR  (*(volatile uint32_t*) (GPIOA_Base + 0x08))
#define GPIOA_ODR      (*(volatile uint32_t*) (GPIOA_Base + 0x14))

#define GPIOB_Base     0x40020400
#define GPIOB_MODER    (*(volatile uint32_t*) (GPIOB_Base + 0x00))
#define GPIOB_OTYPER   (*(volatile uint32_t*) (GPIOB_Base + 0x04))
#define GPIOB_OSPEEDR  (*(volatile uint32_t*) (GPIOB_Base + 0x08))
#define GPIOB_PUPDR    (*(volatile uint32_t*) (GPIOB_Base + 0x0C))
#define GPIOB_IDR      (*(volatile uint32_t*) (GPIOB_Base + 0x10))
#define GPIOB_ODR      (*(volatile uint32_t*) (GPIOB_Base + 0x14))

uint32_t ulc_btnSts_cur;
uint32_t ulc_btnSts_del;

int main(void) {
    RCC_AHB1ENR |= (0b11);

    GPIOA_MODER &= ~(0b11 << 10);
    GPIOA_MODER |= (0b01 << 10);
    GPIOA_OTYPER &= ~(0b1 << 5);
    GPIOA_OSPEEDR &= ~(0b11 << 10);

    GPIOB_MODER &= ~(0b11 << 6);
    GPIOB_PUPDR &= ~(0b11 << 6);
    GPIOB_PUPDR |= (0b01 << 6);

    ulc_btnSts_cur = ((GPIOB_IDR >> 3) & 0x01);
    ulc_btnSts_del = ulc_btnSts_cur;

    while (1) {
        ulc_btnSts_cur = ((GPIOB_IDR >> 3) & 0x01);
        if((ulc_btnSts_cur == 0) && (ulc_btnSts_del == 1)) {
            GPIOA_ODR ^= (1 << 5);
        }
        else {

        }

        ulc_btnSts_del = ulc_btnSts_cur;
    }

    return 0;
}
