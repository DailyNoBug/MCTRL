/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
*/
#include <string.h>  
#include "sx1280-hal.h"
#include "radio.h"
#include "stm32g4xx_hal.h"
#include "mybase.h"
/*!
 * \brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE   255    //0xFFF

#define IRQ_HIGH_PRIORITY     0

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1280Init,
    SX1280HalReset,
    SX1280GetStatus,
    SX1280HalWriteCommand,
    SX1280HalReadCommand,
    SX1280HalWriteRegisters,
    SX1280HalWriteRegister,
    SX1280HalReadRegisters,
    SX1280HalReadRegister,
    SX1280HalWriteBuffer,
    SX1280HalReadBuffer,
    SX1280HalGetDioStatus,
    SX1280GetFirmwareVersion,
    SX1280SetRegulatorMode,
    SX1280SetStandby,
    SX1280SetPacketType,
    SX1280SetModulationParams,
    SX1280SetPacketParams,
    SX1280SetRfFrequency,
    SX1280SetBufferBaseAddresses,
    SX1280SetTxParams,
    SX1280SetDioIrqParams,
    SX1280SetSyncWord,
    SX1280SetRx,
    SX1280GetPayload,
    SX1280SendPayload,
    SX1280SetRangingRole,
    SX1280SetPollingMode,
    SX1280SetInterruptMode,
    SX1280SetRegistersDefault,
    SX1280GetOpMode,
    SX1280SetSleep,
    SX1280SetFs,
    SX1280SetTx,
    SX1280SetRxDutyCycle,
    SX1280SetCad,
    SX1280SetTxContinuousWave,
    SX1280SetTxContinuousPreamble,
    SX1280GetPacketType,
    SX1280SetCadParams,
    SX1280GetRxBufferStatus,
    SX1280GetPacketStatus,
    SX1280GetRssiInst,
    SX1280GetIrqStatus,
    SX1280ClearIrqStatus,
    SX1280Calibrate,
    SX1280SetSaveContext,
    SX1280SetAutoTx,
    SX1280SetAutoFS,
    SX1280SetLongPreamble,
    SX1280SetPayload,
    SX1280SetSyncWordErrorTolerance,
    SX1280SetCrcSeed,
    SX1280SetBleAccessAddress,
    SX1280SetBleAdvertizerAccessAddress,
    SX1280SetCrcPolynomial,
    SX1280SetWhiteningSeed,
    SX1280SetRangingIdLength,
    SX1280SetDeviceRangingAddress,
    SX1280SetRangingRequestAddress,
    SX1280GetRangingResult,
    SX1280SetRangingCalibration,
    SX1280RangingClearFilterResult,
    SX1280RangingSetFilterNumSamples,
    SX1280GetFrequencyError,
};

static uint8_t halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
void rf_delay(uint32_t time_ms)
{
    HAL_Delay(time_ms);
}
/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
void SX1280HalWaitOnBusy( void )
{
    while( HAL_GPIO_ReadPin( RADIO_BUSY_PORT, RADIO_BUSY_PIN ) == 1 );
}

void SX1280HalInit( DioIrqHandler **irqHandlers )
{
    SX1280HalReset( );
    SX1280HalIoIrqInit( irqHandlers );
}

void SX1280HalIoIrqInit( DioIrqHandler **irqHandlers )
{
    //GpioSetIrq( RADIO_DIOx_PORT, RADIO_DIOx_PIN, IRQ_HIGH_PRIORITY, irqHandlers[0] );
}

void SX1280HalReset( void )
{
    rf_delay( 20 );
    HAL_GPIO_WritePin( RADIO_nRESET_PORT, RADIO_nRESET_PIN, GPIO_PIN_RESET );
    rf_delay( 50 );
    HAL_GPIO_WritePin( RADIO_nRESET_PORT, RADIO_nRESET_PIN, GPIO_PIN_SET );
    rf_delay( 20 );
}

void SX1280HalClearInstructionRam( void )
{
	uint16_t index;
    // Clearing the instruction RAM is writing 0x00s on every bytes of the
    // instruction RAM
    uint16_t halSize = 3 + IRAM_SIZE;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( IRAM_START_ADDRESS >> 8 ) & 0x00FF;
    halTxBuffer[2] = IRAM_START_ADDRESS & 0x00FF;
    for( index = 0; index < IRAM_SIZE; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
//    RF_SPI_NSS_L();

    SpiIn( halTxBuffer, halSize );
//    SpiWriteData( halTxBuffer, halSize );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
//    RF_SPI_NSS_H();

    SX1280HalWaitOnBusy( );
}

void SX120HalWakeup( void )
{
	uint16_t halSize = 2;
    __disable_irq( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
//    RF_SPI_NSS_L();

    
    halTxBuffer[0] = RADIO_GET_STATUS;
    halTxBuffer[1] = 0x00;
    SpiIn( halTxBuffer, halSize );
//    SpiWriteData( halTxBuffer, halSize );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
//    RF_SPI_NSS_H();

    // Wait for chip to be ready.
    SX1280HalWaitOnBusy( );

    __enable_irq( );
}

void SX1280HalWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize  = size + 1;
    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
//    RF_SPI_NSS_L();

    halTxBuffer[0] = command;
    memcpy( halTxBuffer + 1, ( uint8_t * )buffer, size * sizeof( uint8_t ) );

    SpiIn( halTxBuffer, halSize );
//    SpiWriteData( halTxBuffer, halSize );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
//    RF_SPI_NSS_H();

    if( command != RADIO_SET_SLEEP )
    {
        SX1280HalWaitOnBusy( );
    }
}

void SX1280HalReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	uint16_t index;
    uint16_t halSize = 2 + size;
    halTxBuffer[0] = command;
    halTxBuffer[1] = 0x00;
    for(  index = 0; index < size; index++ )
    {
        halTxBuffer[2+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
//    RF_SPI_NSS_L();

    SpiInOut( halTxBuffer, halRxBuffer, halSize );
//    SpiWriteReadData( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 2, size );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
//    RF_SPI_NSS_H();

    SX1280HalWaitOnBusy( );
}

void SX1280HalWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    memcpy( halTxBuffer + 3, buffer, size );

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
//    RF_SPI_NSS_L();

    SpiIn( halTxBuffer, halSize );
//    SpiWriteData( halTxBuffer, halSize );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
//    RF_SPI_NSS_H();

    SX1280HalWaitOnBusy( );
}

void SX1280HalWriteRegister( uint16_t address, uint8_t value )
{
    SX1280HalWriteRegisters( address, &value, 1 );
}

void SX1280HalReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
	uint16_t index;
    uint16_t halSize = 4 + size;
    halTxBuffer[0] = RADIO_READ_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    halTxBuffer[3] = 0x00;
    for( index = 0; index < size; index++ )
    {
        halTxBuffer[4+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
//    RF_SPI_NSS_L();

    SpiInOut(halTxBuffer, halRxBuffer, halSize);
//    SpiWriteReadData( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 4, size );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
//    RF_SPI_NSS_H();

    SX1280HalWaitOnBusy( );
}

uint8_t SX1280HalReadRegister( uint16_t address )
{
    uint8_t data;

    SX1280HalReadRegisters( address, &data, 1 );

    return data;
}

void SX1280HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint16_t halSize = size + 2;
    halTxBuffer[0] = RADIO_WRITE_BUFFER;
    halTxBuffer[1] = ( offset ) >> 8;
    memcpy( halTxBuffer + 2, buffer, size );

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
//    RF_SPI_NSS_L();

    SpiIn( halTxBuffer, halSize );
//    SpiWriteData( halTxBuffer, halSize );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
//    RF_SPI_NSS_H();

    SX1280HalWaitOnBusy( );
}

void SX1280HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
	uint16_t index;
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_READ_BUFFER;
    halTxBuffer[1] = offset;
    halTxBuffer[2] = 0x00;
    for( index = 0; index < size; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
//    RF_SPI_NSS_L();

    SpiInOut( halTxBuffer, halRxBuffer, halSize );
//    SpiWriteReadData( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 3, size );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
//    RF_SPI_NSS_H();

    SX1280HalWaitOnBusy( );
}

uint8_t SX1280HalGetDioStatus( void )
{
    return ((HAL_GPIO_ReadPin( RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin ) << 1) | (HAL_GPIO_ReadPin( RADIO_BUSY_PORT, RADIO_BUSY_PIN ) << 0));
//    return ( READ_RF_SX128x_IO1() << 1 ) | ( READ_RF_SX128x_BUSY() << 0 );
}
