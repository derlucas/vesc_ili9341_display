#ifndef SPI_H
#define SPI_H 200

#include "stm32f10x_conf.h"


#define SPI_IS_BUSY(SPIx)                   (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
#define SPI_WAIT(SPIx)                      while (SPI_IS_BUSY(SPIx))
#define SPI_CHECK_ENABLED(SPIx)             if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return;}
#define SPI_CHECK_ENABLED_RESP(SPIx, val)   if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return (val);}


void display_spi_init(SPI_TypeDef* SPIx);

void display_spi_setdatasize(SPI_TypeDef* SPIx, uint16_t DataSize);


static __INLINE uint8_t display_spi_send(SPI_TypeDef* SPIx, uint8_t data) {
	/* Check if SPI is enabled */
	SPI_CHECK_ENABLED_RESP(SPIx, 0);
	
	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	SPI_WAIT(SPIx);
	
	/* Fill output buffer with data */
	SPIx->DR = data;
	
	/* Wait for transmission to complete */
	SPI_WAIT(SPIx);
	
	/* Return data from buffer */
	return SPIx->DR;
}


uint8_t display_spi_dma_working();
uint8_t display_spi_dma_sendhalfword(uint16_t value, uint16_t count);

#endif

