//
// Created by lsc on 25-3-23.
//

#ifndef MCTRL_MYBASE_H
#define MCTRL_MYBASE_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>


#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "sx1280.h"
#include "hw.h"
/*****
 * 说明：这个函数实现sx1280官方库的底层硬件函数
 *
*/

extern SPI_HandleTypeDef hspi3;



#define RADIO_nRESET_PORT    SX1280_RST_GPIO_Port
#define RADIO_nRESET_PIN     SX1280_RST_Pin
#define RADIO_NSS_PORT       SPI3_NSS_GPIO_Port
#define RADIO_NSS_PIN        SPI3_NSS_Pin
#define RADIO_DIO1_GPIO_Port SX1280_DIO1_GPIO_Port
#define RADIO_DIO1_Pin       SX1280_DIO1_Pin
#define RADIO_BUSY_PORT      SX1280_BUSY_GPIO_Port
#define RADIO_BUSY_PIN       SX1280_BUSY_Pin
#define RADIO_NSS_L()        HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_RESET)
#define RADIO_NSS_H()        HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_SET)


void GpioWrite( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,  uint32_t value );
uint32_t GpioRead( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );


void SpiInit( void );
void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size );
void SpiIn( uint8_t *txBuffer, uint16_t size );

#endif //MCTRL_MYBASE_H
