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

int main(void)
{
	// data to send out to i2c
	uint8_t data[2] = {0x0, 0b01000000};

	// Clock setup
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// GPIO Setup
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE10)) | (GPIO_MODER_MODE10_1); // PB10 AF
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE11)) | (GPIO_MODER_MODE11_1); // PB11 AF

	GPIOB->AFR[1] = (GPIOB->AFR[1] & (0xF << 8)) | (0x6 << 8);		// PB10 AF=6
	GPIOB->AFR[1] = (GPIOB->AFR[1] & (0xF << 12)) | (0x6 << 12);	// PB11 AF=6

	// DMA setup
	DMA1_Channel4->CCR = (DMA1_Channel4->CCR & ~(DMA_CCR_EN | DMA_CCR_MEM2MEM_Msk | DMA_CCR_MSIZE_Msk | DMA_CCR_PSIZE_Msk | DMA_CCR_PINC_Msk | DMA_CCR_CIRC_Msk)) | DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel4->CPAR = (uint32_t) (&(I2C2->TXDR));
	DMA1_Channel4->CMAR = (uint32_t) (&data);
	DMA1_Channel4->CNDTR = 2;
	DMA1_Channel4->CCR |= DMA_CCR_EN;

	// I2C Setup
	I2C2->TIMINGR = (uint32_t)0x00300619;
	I2C2->CR1 |= I2C_CR1_PE;
	I2C2->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES);
	I2C2->CR2 |= (2<<I2C_CR2_NBYTES_Pos) | (0x32<<1); // address = 0x32
	I2C2->CR1 |= I2C_CR1_TXDMAEN;

	// I2C start
	I2C2->CR2 |= I2C_CR2_START;

	 // Wait for DMA to finish.
	while ( !( DMA1->ISR & DMA_ISR_TCIF1 ) ) {};
	DMA1->IFCR |= DMA_IFCR_CTCIF1;

	// Stop the I2C transmission.
	while ( !( I2C2->ISR & I2C_ISR_TC ) ) {};
	I2C2->CR2  |=  ( I2C_CR2_STOP );
	while ( I2C2->ISR & I2C_ISR_BUSY ) {};

	// Data for second transmission
	data[0] = 0x16;
	data[1] = 0xFF;

	// I2C start
	I2C2->CR2 |= I2C_CR2_START; /* Go */

	for(;;);
}



