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
#include "stm32l0538_discovery_epd.h"
#include <stdio.h>
#include <math.h>

void initGPIO();
void initDisplay();

#define I2C_TIMING 0x00B21847 //400k

#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03
#define INTERRUPT_DRDY 0x04
#define TEMP_MAX 0x05
#define HUMID_MAX 0x06
#define INTERRUPT_CONFIG 0x07
#define TEMP_OFFSET_ADJUST 0x08
#define HUM_OFFSET_ADJUST 0x09
#define TEMP_THR_L 0x0A
#define TEMP_THR_H 0x0B
#define HUMID_THR_L 0x0C
#define HUMID_THR_H 0x0D
#define CONFIG 0x0E
#define MEASUREMENT_CONFIG 0x0F
#define MID_L 0xFC
#define MID_H 0xFD
#define DEVICE_ID_L 0xFE
#define DEVICE_ID_H 0xFF
#define HTAddr 0x40
#define altHTAddr 0x41


/////
/////  ______________________________________________________________
/////            init GPIO

void initGPIO()
{
	// high level overview
	// enable pins pB8 and pb9 for i2c1
	// enable i2c1
	// enable high speed clock
	// set i2c clock speed
	__GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin       = GPIO_PIN_9|GPIO_PIN_8;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
//	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH  ;
	GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/////
/////  ______________________________________________________________
/////			 init Display

void initDisplay()
{
	BSP_EPD_Init();
	//BSP_EPD_Clear(EPD_COLOR_WHITE);
	BSP_EPD_DisplayChar(0, 12, 'T');
	BSP_EPD_DisplayChar(8, 12, 'e');
	BSP_EPD_DisplayChar(16, 12, 'm');
	BSP_EPD_DisplayChar(24, 12, 'p');
	BSP_EPD_DisplayChar(32, 12, ':');
	BSP_EPD_DisplayChar(0, 8, 'H');
	BSP_EPD_DisplayChar(8, 8, 'u');
	BSP_EPD_DisplayChar(16, 8, 'm');
	BSP_EPD_DisplayChar(24, 8, 'i');
	BSP_EPD_DisplayChar(32, 8, 'd');
	BSP_EPD_DisplayChar(40, 8, 'i');
	BSP_EPD_DisplayChar(48, 8, 't');
	BSP_EPD_DisplayChar(56, 8, 'y');
	BSP_EPD_DisplayChar(64, 8, ':');
	BSP_EPD_RefreshDisplay();
}

/////
/////  ______________________________________________________________
/////			 clearDisp
// must refresh after
void clearDisp()
{
	BSP_EPD_Clear(EPD_COLOR_LIGHTGRAY);
	BSP_EPD_DisplayChar(0, 12, 'T');
	BSP_EPD_DisplayChar(8, 12, 'e');
	BSP_EPD_DisplayChar(16, 12, 'm');
	BSP_EPD_DisplayChar(24, 12, 'p');
	BSP_EPD_DisplayChar(32, 12, ':');
	BSP_EPD_DisplayChar(0, 8, 'H');
	BSP_EPD_DisplayChar(8, 8, 'u');
	BSP_EPD_DisplayChar(16, 8, 'm');
	BSP_EPD_DisplayChar(24, 8, 'i');
	BSP_EPD_DisplayChar(32, 8, 'd');
	BSP_EPD_DisplayChar(40, 8, 'i');
	BSP_EPD_DisplayChar(48, 8, 't');
	BSP_EPD_DisplayChar(56, 8, 'y');
	BSP_EPD_DisplayChar(64, 8, ':');
}

/////
///////HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
/////  ______________________________________________________________
/////			 read Temp

float readTemp(I2C_HandleTypeDef *hi2c) {
	uint8_t byte[2];
	uint16_t temp;
	if (HAL_OK != HAL_I2C_Mem_Read(hi2c, HTAddr, 0x00, I2C_MEMADD_SIZE_8BIT, &byte[0], 0x01, HAL_MAX_DELAY))
	{
		BSP_EPD_DisplayChar(80, 12, 'N');
		BSP_EPD_DisplayChar(88, 12, 'M');
		BSP_EPD_RefreshDisplay();
	}
	if (HAL_OK != HAL_I2C_Mem_Read(hi2c, HTAddr, 0x01, I2C_MEMADD_SIZE_8BIT, &byte[1], 0x01, HAL_MAX_DELAY))
	{
		BSP_EPD_DisplayChar(80, 12, 'N');
		BSP_EPD_DisplayChar(88, 12, 'M');
		BSP_EPD_RefreshDisplay();
	}

	temp = (unsigned int)byte[1] << 8 | byte[0];

	return (float)(temp) * 165 / 65536 - 40;
}

/////
/////  ______________________________________________________________
/////			 read Humidity

float readHumi(I2C_HandleTypeDef *hi2c) {
	uint8_t byte[2];
	uint16_t humidity;
	if (HAL_OK != HAL_I2C_Mem_Read(hi2c, HTAddr, 0x02, I2C_MEMADD_SIZE_8BIT, &byte[0], 0x01, HAL_MAX_DELAY))
	{
		BSP_EPD_DisplayChar(80, 12, 'N');
		BSP_EPD_DisplayChar(88, 12, 'M');
		BSP_EPD_RefreshDisplay();
	}
	if (HAL_OK != HAL_I2C_Mem_Read(hi2c, HTAddr, 0x03, I2C_MEMADD_SIZE_8BIT, &byte[1], 0x01, HAL_MAX_DELAY))
	{
		BSP_EPD_DisplayChar(80, 12, 'N');
		BSP_EPD_DisplayChar(88, 12, 'M');
		BSP_EPD_RefreshDisplay();
	}

	humidity = (unsigned int)byte[1] << 8 | byte[0];

	return (float)(humidity)/( 65536 )* 100;

}


/////
/////  ______________________________________________________________
/////

// LD3 = PB4
// LD4 = PA5

int main(void)
{

	//_________INITIALIZATIONS______________________________________________
	I2C_HandleTypeDef hi2c;

	HAL_Init();
	initGPIO();
	initDisplay();

	//______________________________________________________________________
	// Enable/ Initialize I2C
	__I2C1_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	// Initialize i2c struct
	hi2c.Instance = I2C1;
	hi2c.Init.Timing = I2C_TIMING; // 400 khz
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	if(HAL_I2C_Init(&hi2c) != HAL_OK)
	{
		// error handler
		BSP_EPD_DisplayChar(80, 12, 'N');
		BSP_EPD_DisplayChar(88, 12, 'I');
		BSP_EPD_RefreshDisplay();
	}


	//______________________________________________________________________
	// Check I2C Device

	if (HAL_OK != HAL_I2C_IsDeviceReady(&hi2c, altHTAddr, 1, HAL_MAX_DELAY)) // test if device is ready
	{

		BSP_EPD_DisplayChar(80, 12, 'N');
		BSP_EPD_DisplayChar(88, 12, 'R');
		BSP_EPD_RefreshDisplay();
	}

	HAL_Delay(500);

	//______________________________________________________________________
	// Read device registers I2C

	for (int n = 0; n < 10; n++){

		int data = 0xF8;
		//HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if (HAL_OK != HAL_I2C_Mem_Write(&hi2c, HTAddr, 0x0E, I2C_MEMADD_SIZE_8BIT, (data), 0x01, HAL_MAX_DELAY))
		{
			BSP_EPD_DisplayChar(80, 12, 'N');
			BSP_EPD_DisplayChar(88, 12, 'M');
			BSP_EPD_RefreshDisplay();
		}




		float temp = readTemp(&hi2c);
		float humi = readHumi(&hi2c);

		char snum[3];
		char dnum[3];
		for (int i = 0; i < 3; i++)
		{
			snum[i] = ' ';
			dnum[i] = ' ';
		}


		snum[2] = ((int) humi % 10) + '0';
		humi = humi / 10;
		snum[1] = ((int) humi % 10) + '0';

		dnum[2] = ((int) temp % 10) + '0';
		temp = temp / 10;
		dnum[1] = ((int) temp % 10) + '0';


		clearDisp();
		BSP_EPD_DisplayChar(72, 8, snum[0]);
		BSP_EPD_DisplayChar(80, 8, snum[1]);
		BSP_EPD_DisplayChar(88, 8, snum[2]);
		BSP_EPD_DisplayChar(96, 8, '%');
		BSP_EPD_DisplayChar(40, 12, dnum[0]);
		BSP_EPD_DisplayChar(48, 12, dnum[1]);
		BSP_EPD_DisplayChar(56, 12, dnum[2]);
		BSP_EPD_DisplayChar(64, 12, '*');
		BSP_EPD_DisplayChar(72, 12, 'C');
		BSP_EPD_RefreshDisplay();
		HAL_Delay(500);

	}

}



