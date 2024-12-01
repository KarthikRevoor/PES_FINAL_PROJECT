#include "stm32f091xc.h"
#include "stm32f0xx.h"
#include <stdint.h>  // for uint8_t
#include <stdlib.h>  // for size_t
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

void I2C_Init(void);
float ReadUSS(void);
void delay_ms(uint32_t delay);
void GPIO_Init(void);
