#include <stdint.h>

#define GPIOA_Base   0x40020000
#define GPIOA_MODER  (*(volatile uint32_t*) (GPIOA_Base + 0x00))
#define GPIOA_OTYPER (*(volatile uint32_t*) (GPIOA_Base + 0x04))
#define GPIOA_OSPEEDR (*(volatile uint32_t*) (GPIOA_Base + 0x08))
#define GPIOA_ODR    (*(volatile uint32_t*) (GPIOA_Base + 0x14))

int main(void) {
    asm volatile("LDR r0, =0x40023830\n\tLDR r1, [r0]\n\tORR r1, r1, #1\n\tSTR r1, [r0]");

    GPIOA_MODER &= ~(0b11 << 10);
    GPIOA_MODER |= (0b01 << 10);

    GPIOA_OTYPER &= ~(0b1 << 5);

    GPIOA_OSPEEDR &= ~(0b11 << 10);

    GPIOA_ODR |= (0b01 << 5);

    while (1) {
    }
    return 0;
}
