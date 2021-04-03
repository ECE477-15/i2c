/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32l0xx.h"
#include "stm32l0538_discovery.h"

void I2C_Tx(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
void I2C_Init();

int main(void)
{
  I2C_Init();

  uint8_t data[2] = {0x3D, 0xFF};
  I2C_Tx(0x32<<1, data, 2);

  data[0] = 0x00;
  data[1] = 0x40;
  I2C_Tx(0x32<<1, data, 2);

  data[0] = 0x36;
  data[1] = 0x53;
  I2C_Tx(0x32<<1, data, 2);

  data[0] = 0x16;
  data[1] = 0xFF;
  I2C_Tx(0x32<<1, data, 2);

  int i = 0;
	while(i < 100000) {
		i++;
	}

  data[0] = 0x3D;
  data[1] = 0xFF;
  I2C_Tx(0x32<<1, data, 2);

  while (1);
}

void I2C_Tx(uint16_t DevAddress, uint8_t *pData, uint16_t Size) {
	while(I2C2->ISR & I2C_ISR_BUSY);

	I2C2->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
	I2C2->CR2 |= (DevAddress & I2C_CR2_SADD) | (Size << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;

	while(!(I2C2->ISR & I2C_ISR_TXIS));
	I2C2->TXDR = pData[0];

	while(!(I2C2->ISR & I2C_ISR_TXIS));
	I2C2->TXDR = pData[1];

	while(!(I2C2->ISR & I2C_ISR_STOPF));
	I2C2->ICR |= I2C_ICR_STOPCF;
}

void I2C_Init() {
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;	// GPIOB Clock enable

    GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDER_OSPEED10_Pos) | (0x3 << GPIO_OSPEEDER_OSPEED11_Pos); // very high speed
    GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11;	// Open drain
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD10_Msk | GPIO_PUPDR_PUPD11_Msk);	// clear pullup/pulldown
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0);		// Set pullup
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk);					// Clear PB10, PB11 Alternate Function Reg
	GPIOB->AFR[1] |= (0x6 << GPIO_AFRH_AFSEL10_Pos) | (0x6 << GPIO_AFRH_AFSEL11_Pos);	// Set PB10, PB11 Alternate Function Reg to 0x6 (I2C2)
	GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);							// Clear PB10, PB11 mode
	GPIOB->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;							// Set PB10, PB11 mode to alternate 0b10

    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;	// I2C2 clock enable

    I2C2->CR1 &= ~I2C_CR1_PE;		// Disable I2C2
    I2C2->TIMINGR = 0x00707CBB & 0xF0FFFFFFU; // 0xF0FFFFFFU = TIMING_CLEAR_MASK

    I2C2->OAR1 &= ~I2C_OAR1_OA1EN_Msk; 	// Disable Own Address
    I2C2->OAR1 = (I2C_OAR1_OA1EN | 0x0); // set own address and re-enable

    I2C2->CR2 |= (I2C_CR2_AUTOEND | I2C_CR2_NACK);

    I2C2->OAR2 &= ~I2C_OAR2_OA2EN_Msk;	// Disable own address 2

    I2C2->CR1 = (I2C_GENERALCALL_DISABLE | I2C_NOSTRETCH_DISABLE);

    I2C2->CR1 |= I2C_CR1_PE;		// Enable I2C2
}


