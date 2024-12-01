#include "Initialization.h"
#define I2C_ADDRESS 0x57     // USS module's I2C address
#define TIMEOUT 100000

void I2C_Init(void)
{
	// Enable GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Enable USART2 clock

    // Step 2: Configure PB6 (SCL) and PB7 (SDA) as alternate function
    GPIOB->MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2))); // Clear mode bits for PB6 and PB7
    GPIOB->MODER |= ((1 << (13)));  // Set PB6 and PB7 to alternate function
    GPIOB->MODER |= (1 << (15));
    GPIOB->AFR[0] |= (1U << (6 * 4)) | (1U << (7 * 4));   // Set AF1 (I2C1) for PB6 and PB7

    GPIOB->OTYPER |= (1U << 6) | (1U << 7); // Open-drain output type for I2C

    GPIOB->PUPDR &= ~((3U << (6 * 2)) | (3U << (7 * 2))); // Clear pull-up/down bits
    GPIOB->PUPDR |= (1U << (6 * 2)) | (1U << (7 * 2));    // Enable pull-up resistors for PB6 and PB7

    // Enable I2C1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure I2C timing for ~40 kHz
    I2C1->CR1 &= ~I2C_CR1_PE;  // Disable I2C
    // Step 3: Configure and initialize I2C1
    I2C1->TIMINGR = 0x12423030;  // Configure timing for 100kHz I2C (example)
    I2C1->CR1 |= I2C_CR1_PE;     // Enable I2C1
}


void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (uint32_t j = 0; j < 4000; j++) {
            __NOP(); // No operation, keeps the loop busy
        }
    }
}

void GPIO_Init(void) {
    // Enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Configure PA5 as output
    GPIOA->MODER &= ~(3U << (5 * 2)); // Clear mode bits for PA5
    GPIOA->MODER |= (1U << (5 * 2));  // Set PA5 to output mode
}

float ReadUSS(void) {
    uint8_t command = 0x01;       // Command to request measurement
    uint8_t data[3] = {0};        // Buffer to hold received data
    uint32_t raw_distance = 0;    // Raw distance data
    uint32_t timeout;

    // Send START condition and command (write transaction)
    I2C1->CR2 = (I2C_ADDRESS << 1) | (1 << 16) | I2C_CR2_START; // Write address + 1 byte
    timeout = TIMEOUT;
    while (!(I2C1->ISR & I2C_ISR_TXIS) && --timeout); // Wait for TXIS
    if (timeout == 0) {
        return -1.0f; // Timeout error
    }
    I2C1->TXDR = command; // Send the command byte

    // Wait for transfer complete
    timeout = TIMEOUT;
    while (!(I2C1->ISR & I2C_ISR_TC) && --timeout);
    if (timeout == 0) {
        return -1.0f; // Timeout error
    }

    // Send STOP condition manually
    I2C1->CR2 |= I2C_CR2_STOP;
    while (!(I2C1->ISR & I2C_ISR_STOPF)); // Wait for STOP flag
    I2C1->ICR |= I2C_ICR_STOPCF; // Clear STOP flag

    // Delay to allow the USS module to process the request
    delay_ms(120);

    // Configure for repeated START and read transaction
    I2C1->CR2 = (I2C_ADDRESS << 1) | (3 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START;

    // Read the data bytes
    for (int i = 0; i < 3; i++) {
        timeout = TIMEOUT;
        while (!(I2C1->ISR & I2C_ISR_RXNE) && --timeout);
        if (timeout == 0) {
            return -1.0f; // Timeout error
        }
        data[i] = I2C1->RXDR; // Read received byte
        printf("data: %d", data[i]);
    }

    I2C1->ICR |= I2C_ICR_STOPCF;

    // Combine the 3 bytes into a 24-bit value
    raw_distance = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2]);
    printf("dist: %d", raw_distance);

    // Convert raw distance to meters and return
    return raw_distance / 1000.0f;
}
