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
#include "babysitter.h"
#include "hdc2010.h"

void I2C_Init();
void I2C_Mem_Tx(uint16_t device_addr, uint16_t reg_addr, uint16_t reg_addr_size, uint8_t *data, uint16_t data_size);
void I2C_Mem_Rx(uint16_t device_addr, uint16_t reg_addr, uint16_t reg_addr_size, uint8_t *data, uint16_t data_size);
void hdc2010_enable();
uint16_t BB_getSOC(void);
uint16_t BB_getVolt(void);
uint16_t BB_getSOH(void);
uint16_t BB_getRemCap(void);
uint16_t getHDCTemp(void);
uint16_t getHDCHumidity(void);

uint16_t Volt;
uint16_t Soh;
uint16_t sohPercent;
uint16_t Rcap;
uint16_t Soc;
uint16_t temperature;
uint16_t humidity;

int main(void)
{
  I2C_Init();
  hdc2010_enable();

//  uint8_t data[1] = {0xF8};
//  I2C_Mem_Tx(HTAddr, MEASUREMENT_CONFIG, 1, data, 1);
//  data = data | INTERRUPT_DRDY;


//
//  uint8_t data[1] = {0xFF};
//  I2C_Mem_Tx(0x32, 0x3D, 1, data, 1); // reset chip
////
//  data[0] = 0x40;
//  I2C_Mem_Tx(0x32, 0x00, 1, data, 1); // chip enable
//
//  data[0] = 0x53;
//  I2C_Mem_Tx(0x32, 0x36, 1, data, 1); // chip clock enable
//
//  data[0] = 0xFF;
//  I2C_Mem_Tx(0x32, 0x16, 1, data, 1); // turn on LED
//
//  data[0] = 0b00000100;
//  I2C_Mem_Tx(0x32, 0x3E, 1, data, 1); // take temperature sample



  int i = 0;
  while(i < 10000) {
    i++;
  }

  // Get the battery state of charge from the Battery Babysitter (in %)
//  Soc = BB_getSOC();   // take battery babysitter Reads and returns the battery state-of-charge (in %)
//
//  Volt = BB_getVolt(); // take battery babysitter Reads and returns the battery voltage (in mV)
//
//  Soh = BB_getSOH();  // take battery babysitter Reads and returns the battery state-of-health (in %)
//
//  Rcap = BB_getRemCap(); // take battery babysitter Reads and returns the remaining capacity (in mAh)

  // Start conversion
//  uint8_t convert = 0;
//  I2C_Mem_Tx(HTAddr, CONFIG, 1, &convert, 1); // Perform a 2 byte I2C Write Transaction
//  I2C_Mem_Tx(HTAddr, TEMP_HIGH, 1, &convert, 1); // Perform a 2 byte I2C Write Transaction

//  HAL_Delay(20);  // Instruct your MCU to NO-OP for a number of CPU cycles equivalent to 2 ms

  temperature = getHDCTemp(); // take HDC2010 sensor reads and returns the room temperature (in degree Celsius)

  humidity = getHDCHumidity();  // take HDC2010 sensor reads and returns the humidity ( in %)



//  I2C_Mem_Rx(0x32, 0x3F, 1, data, 1); // read temperature

  i = 0;
	while(i < 100000) {
		i++;
	}

//  data[0] = 0xFF;
//  I2C_Mem_Tx(0x32, 0x3D, 1, data, 1); // reset chip

  while (1);
}

void I2C_Mem_Tx(uint16_t device_addr, uint16_t reg_addr, uint16_t reg_addr_size, uint8_t *data, uint16_t data_size) {
	while(I2C2->ISR & I2C_ISR_BUSY);

	device_addr <<= 1;
	uint16_t size = reg_addr_size + data_size;

	I2C2->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
	I2C2->CR2 |= (device_addr & I2C_CR2_SADD) | (size << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;

	if(reg_addr_size == 2) {	// send reg_addr MSB
		while(!(I2C2->ISR & I2C_ISR_TXIS));
		I2C2->TXDR = ((reg_addr >> 8) & 0xFF);
	}

	while(!(I2C2->ISR & I2C_ISR_TXIS));	// send reg_addr LSB
	I2C2->TXDR = (reg_addr & 0xFF);

	uint8_t * data_pointer = data;
	for(uint16_t tx_remaining = data_size; tx_remaining > 0; tx_remaining--) {
		while(!(I2C2->ISR & I2C_ISR_TXIS));
		I2C2->TXDR = *data_pointer;

		data_pointer++;
	}

	while(!(I2C2->ISR & I2C_ISR_STOPF));
	I2C2->ICR |= I2C_ICR_STOPCF;
}

void I2C_Mem_Rx(uint16_t device_addr, uint16_t reg_addr, uint16_t reg_addr_size, uint8_t *data, uint16_t data_size) {
	while(I2C2->ISR & I2C_ISR_BUSY);

	device_addr <<= 1;

	I2C2->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
	I2C2->CR2 |= (device_addr & I2C_CR2_SADD) | (reg_addr_size << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;

	if(reg_addr_size == 2) {	// send reg_addr MSB
		while(!(I2C2->ISR & I2C_ISR_TXIS));
		I2C2->TXDR = ((reg_addr >> 8) & 0xFF);
	}

	while(!(I2C2->ISR & I2C_ISR_TXIS));	// send reg_addr LSB
	I2C2->TXDR = (reg_addr & 0xFF);

	while(!(I2C2->ISR & I2C_ISR_TC));	// wait for tx complete

	I2C2->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
	I2C2->CR2 |= (device_addr & I2C_CR2_SADD) | (data_size << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START | I2C_CR2_RD_WRN;


	uint8_t * data_pointer = data;
	for(uint16_t tx_remaining = data_size; tx_remaining > 0; tx_remaining--) {
		while(!(I2C2->ISR & I2C_ISR_RXNE));	// wait for read data
		*data_pointer = I2C2->RXDR;

		data_pointer++;
	}

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

    I2C2->CR1 |= I2C_CR1_PE;		// Enable I2C2
}

uint16_t BB_getSOC(void) {
	uint8_t data[2];
	I2C_Mem_Rx(BQ72441_I2C_ADDRESS, BQ27441_COMMAND_SOC, 1, data, 2);
	uint16_t Soc = (data[1]<<8) | data[0];
	return Soc;
}

uint16_t BB_getVolt(void){
	uint8_t data[2];
	I2C_Mem_Rx(BQ72441_I2C_ADDRESS, BQ27441_COMMAND_VOLTAGE, 1, data, 2);
	uint16_t Volt = (data[1]<<8) | data[0];
	return Volt;

}

uint16_t BB_getSOH(void){
	uint8_t data[2];
	I2C_Mem_Rx(BQ72441_I2C_ADDRESS, BQ27441_COMMAND_SOH, 1, data, 2);
	Soh = (data[1]<<8) | data[0];
	sohPercent = Soh & 0x00FF;
	return sohPercent;
}

uint16_t BB_getRemCap(void){
	uint8_t data[2];
	I2C_Mem_Rx(BQ72441_I2C_ADDRESS, BQ27441_COMMAND_REM_CAPACITY, 1, data, 2);
	uint16_t Rcap = (data[1]<<8) | data[0];
	return Rcap;
}

uint16_t getHDCTemp(void){
	uint8_t data[2];
	uint8_t command[2] =  {0x01,0x00};
	I2C_Mem_Tx(HTAddr, MEASUREMENT_CONFIG, 1, command, 1);
//	I2C_Mem_Tx(HTAddr, TEMP_LOW, 1, command[0], 2);
	I2C_Mem_Rx(HTAddr, TEMP_LOW, 1, data, 2);
	uint16_t temperature = (data[1]<<8) | data[0];
    temperature = (((float)(temperature) * 165) / 65536) - 40;
	return temperature;
}

uint16_t getHDCHumidity(void){
	uint8_t data[2];
	uint8_t command[3] = {0x01, 0x02 ,0x03};
	I2C_Mem_Tx(HTAddr, MEASUREMENT_CONFIG, 1, command, 1);
//	I2C_Mem_Tx(HTAddr, HUMID_LOW, 1, command, 2);
	I2C_Mem_Rx(HTAddr, HUMID_LOW, 1, data, 2);
	uint16_t humidity = (data[1]<<8) | data[0];
	humidity = (float)(humidity)/( 65536 )* 100;
	return humidity;

}

void hdc2010_enable()
{
	// the startup sequence for single acquisition
    uint8_t value = 0;
    I2C_Mem_Tx(HTAddr, CONFIG, 1, &value, 1);
    I2C_Mem_Tx(HTAddr, MEASUREMENT_CONFIG, 1, &value, 1);


}
