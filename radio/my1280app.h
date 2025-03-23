//
// Created by lsc on 25-3-23.
//

#ifndef MCTRL_MY1280APP_H
#define MCTRL_MY1280APP_H
#include "sx1280-hal.h"
#include "sx1280.h"
#include "radio.h"
#include "stm32g4xx_hal.h"
#include "mybase.h"
/*!
 * Select mode of operation for the Ping Ping application
 */
//#define MODE_BLE
#define MODE_LORA
//#define MODE_GFSK
//#define MODE_FLRC


#define RF_BL_ADV_CHANNEL_38             			2426000000 // Hz
#define RF_BL_ADV_CHANNEL_0                     	2404000000 // Hz

/*!
 * \brief Defines the nominal frequency
 */
#define RF_FREQUENCY                                RF_BL_ADV_CHANNEL_0 // Hz

/*!
 * \brief Defines the output power in dBm
 *
 * \remark The range of the output power is [-18..+13] dBm
 */
#define TX_OUTPUT_POWER                             -18

/*!
 * \brief Defines the buffer size, i.e. the payload size
 */
#define BUFFER_SIZE                                 6

/*!
 * \brief Number of tick size steps for tx timeout
 */
#define TX_TIMEOUT_VALUE                            10000 // ms

/*!
 * \brief Number of tick size steps for rx timeout
 */
#define RX_TIMEOUT_VALUE                            1000 // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US

/*!
 * \brief Defines the size of the token defining message type in the payload
 */
#define PINGPONGSIZE                                4


/*!
 * \brief Defines the states of the application
 */
typedef enum
{
    APP_LOWPOWER,
    APP_RX,
    APP_RX_TIMEOUT,
    APP_RX_ERROR,
    APP_TX,
    APP_TX_TIMEOUT,
}AppStates_t;


/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(int error);
void OnCadDone( bool channelActivityDetected );

void LoRa_init(void);
void LoRa_process(void);
#endif //MCTRL_MY1280APP_H
