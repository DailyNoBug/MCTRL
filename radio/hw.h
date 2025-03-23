//
// Created by lsc on 25-3-23.
//

#ifndef MCTRL_HW_H
#define MCTRL_HW_H

#include "stm32g4xx_hal.h"

#define SX1280_RST_Pin GPIO_PIN_0
#define SX1280_RST_GPIO_Port GPIOB
#define SX1280_RXEN_Pin GPIO_PIN_12
#define SX1280_RXEN_GPIO_Port GPIOA
#define SX1280_TXEN_Pin GPIO_PIN_11
#define SX1280_TXEN_GPIO_Port GPIOA
#define SX1280_BUSY_Pin GPIO_PIN_7
#define SX1280_BUSY_GPIO_Port GPIOA
#define SX1280_DIO1_Pin GPIO_PIN_6
#define SX1280_DIO1_GPIO_Port GPIOA
#define SPI3_NSS_Pin GPIO_PIN_10
#define SPI3_NSS_GPIO_Port GPIOA


#endif //MCTRL_HW_H
