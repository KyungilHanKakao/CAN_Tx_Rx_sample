#include "w25q.h"
#include "spi.h"
#include <stdio.h>

void W25QXX_Reset(SPI_HandleTypeDef *hspi)
{
	uint8_t tData[2];
	tData[0] = 0x66;  // enable Reset
	tData[1] = 0x99;  // Reset

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, tData, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t W25Q_ReadID(SPI_HandleTypeDef *hspi) {
    uint8_t cmd = 0x9F; // JEDEC ID command
    uint8_t id[3]; // Array to store ID bytes

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS low
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, id, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS high

    printf("0 : %x - %x - %x \n",id[0], id[1],id[2] );
    return ((id[0] << 16) | (id[1] << 8) | id[2]); // Combine ID bytes
}

void W25Q_WriteData(uint32_t address, uint8_t* data, uint16_t size) {
    // Implement write enable command and data write sequence
}

void W25Q_ReadData(uint32_t address, uint8_t* buffer, uint16_t size) {
    // Implement read command sequence
}
