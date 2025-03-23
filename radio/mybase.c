//
// Created by lsc on 25-3-23.
//

#include "mybase.h"

void SpiInit(void)
{
    RADIO_NSS_H();
    // RadioSpiHandle = hspi2;
}

void GpioWrite( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t value )
{
    HAL_GPIO_WritePin( GPIOx, GPIO_Pin , ( GPIO_PinState ) value );
}

uint32_t GpioRead( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin )
{
    return HAL_GPIO_ReadPin( GPIOx, GPIO_Pin );
}
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    // GpioLaunchIrqHandler( GPIO_Pin );
}

void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size )
{

#ifdef STM32L4XX_FAMILY
    HAL_SPIEx_FlushRxFifo( &hspi2 ); // Comment For STM32L0XX and STM32L1XX Int間ration, uncomment for STM32L4XX Int間ration
#endif
#ifdef USE_DMA
    blockingDmaFlag = true;
        HAL_SPI_TransmitReceive_DMA( &SpiHandle, txBuffer, rxBuffer, size );
        WAIT_FOR_BLOCKING_FLAG
#else
    RADIO_NSS_L();
    HAL_SPI_TransmitReceive( &hspi3, txBuffer, rxBuffer, size, HAL_MAX_DELAY );
    RADIO_NSS_H();
#endif
}

void SpiIn( uint8_t *txBuffer, uint16_t size )
{
#ifdef USE_DMA
    blockingDmaFlag = true;
        HAL_SPI_Transmit_DMA( &SpiHandle, txBuffer, size );
        WAIT_FOR_BLOCKING_FLAG
#else
    RADIO_NSS_L();
    HAL_SPI_Transmit( &hspi3, txBuffer, size, HAL_MAX_DELAY );
    RADIO_NSS_H();
#endif
}