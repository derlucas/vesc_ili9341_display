#include "display_spi.h"


DMA_InitTypeDef DMA_InitStructure;
uint16_t dummy = 0x0000;


void display_spi_init(SPI_TypeDef* SPIx) {

    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;      // SPI_Mode 0
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;    // SPI_Mode 0
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx, &SPI_InitStructure);


    // DMA Channel 3 - SPI TX
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPIx->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&dummy;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_CalculateCRC(SPI1, DISABLE);
    SPI_Cmd(SPI1, ENABLE);


    // Enable DMA1 Channel Transfer Complete interrupt
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    //Enable DMA1 channel IRQ Channel
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void DMA1_Channel3_IRQHandler(void) {
    //Test on DMA1 Channel1 Transfer Complete interrupt
    if(DMA_GetITStatus(DMA1_IT_TC3)) {
        //Clear DMA1 Channel1 Half Transfer, Transfer Complete and Global interrupt pending bits
        DMA_ClearITPendingBit(DMA1_IT_GL3);
    }
}

void display_spi_setdatasize(SPI_TypeDef* SPIx, uint16_t DataSize) {

    /* Disable SPI first */
    SPIx->CR1 &= ~SPI_CR1_SPE;

    /* Set proper value */
    if (DataSize == SPI_DataSize_16b) {
        /* Set bit for frame */
        SPIx->CR1 |= SPI_CR1_DFF;
    } else {
        /* Clear bit for frame */
        SPIx->CR1 &= ~SPI_CR1_DFF;
    }

    /* Enable SPI back */
    SPIx->CR1 |= SPI_CR1_SPE;
}

uint8_t display_spi_dma_working(SPI_TypeDef* SPIx) {

    return (
        DMA_GetFlagStatus(DMA1_FLAG_TC3) || SPI_IS_BUSY(SPIx)
    );
}

uint8_t display_spi_dma_sendhalfword(uint16_t value, uint16_t count) {

    dummy = value;

    DMA_Cmd(DMA1_Channel3, DISABLE);

    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_BufferSize = count;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel3, ENABLE);


    /* Return OK */
    return 1;
}
