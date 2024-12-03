
#ifndef INC_W25Q_H_
#define INC_W25Q_H_

#include "main.h"

#ifdef __cplusplus
#define EXPORT extern "C"
#else
#define EXPORT
#endif

void W25QXX_Reset(SPI_HandleTypeDef *hspi);
uint8_t W25Q_ReadID(SPI_HandleTypeDef *hspi);
void W25Q_WriteData(uint32_t address, uint8_t* data, uint16_t size);
void W25Q_ReadData(uint32_t address, uint8_t* buffer, uint16_t size);

#endif /* INC_W25Q_H_ */
