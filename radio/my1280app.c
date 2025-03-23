//
// Created by lsc on 25-3-23.
//

#include "my1280app.h"
#include "stdio.h"

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

RadioCallbacks_t Callbacks =
        {
                &OnTxDone,        // txDone
                &OnRxDone,        // rxDone
                NULL,             // syncWordDone
                NULL,             // headerDone
                &OnTxTimeout,     // txTimeout
                &OnRxTimeout,     // rxTimeout
                NULL,       // rxError
                NULL,             // rangingDone
                &OnCadDone,       // cadDone
        };

uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;
uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
AppStates_t AppState = APP_LOWPOWER;

PacketParams_t packetParams;
PacketStatus_t packetStatus;
ModulationParams_t modulationParams;

bool isMaster = true;
// bool isMaster = false;

void LoRa_init()
{
    SpiInit();
    if (isMaster==true)
    {
        HAL_GPIO_WritePin(SX1280_TXEN_GPIO_Port,SX1280_TXEN_Pin,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(SX1280_RXEN_GPIO_Port,SX1280_RXEN_Pin,GPIO_PIN_SET);
    }
    HAL_Delay(100);
    HAL_GPIO_WritePin(RADIO_nRESET_PORT, RADIO_nRESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(RADIO_nRESET_PORT, RADIO_nRESET_PIN, GPIO_PIN_SET);

    printf( "Radio firmware version 0x%x\r\n", SX1280GetFirmwareVersion() );
    if(isMaster==true)
    {
        printf( "Master LORA mode\n\r" );
    }
    else
    {
        printf( "Slaver LORA mode\n\r" );
    }

    Radio.Init( &Callbacks );
    Radio.SetRegulatorMode( USE_LDO );

    modulationParams.PacketType = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SF12;
    modulationParams.Params.LoRa.Bandwidth = LORA_BW_1600;
    modulationParams.Params.LoRa.CodingRate = LORA_CR_LI_4_7;

    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = 12;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
    packetParams.Params.LoRa.PayloadLength = BUFFER_SIZE;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

    Radio.SetStandby( STDBY_RC );
    switch(modulationParams.Params.LoRa.SpreadingFactor){
        case LORA_SF5:
        case LORA_SF6:
            Radio.WriteRegister(0x0925,0x1E);
            break;
        case LORA_SF7:
        case LORA_SF8:
            Radio.WriteRegister(0x0925,0x37);
            break;
        case LORA_SF9:
        case LORA_SF10:
        case LORA_SF11:
        case LORA_SF12:
            Radio.WriteRegister(0x0925,0x32);
            break;
    }
    Radio.SetPacketType( modulationParams.PacketType );
    Radio.SetModulationParams( &modulationParams );
    Radio.SetPacketParams( &packetParams );
    Radio.SetRfFrequency( RF_FREQUENCY );
    Radio.SetBufferBaseAddresses( 0x00, 0x00 );
    Radio.SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_06_US );

    SX1280SetPollingMode();
//    SX1281SetInterruptMode();

    if(isMaster==true)
    {
        //发送
//        HAL_GPIO_WritePin(SX1280_TXEN_GPIO_Port,SX1280_TXEN_Pin,GPIO_PIN_SET);
        Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
        Radio.SendPayload((uint8_t*)"123456",6, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE });
    }
    else
    {
//        HAL_GPIO_WritePin(SX1280_RXEN_GPIO_Port,SX1280_RXEN_Pin,GPIO_PIN_SET);
        Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
        //Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
        Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, 0xFFFF } );//RX_TIMEOUT_VALUE
    }
    AppState = APP_LOWPOWER;
}

void LoRa_process()
{
    SX1280ProcessIrqs( );
    HAL_Delay(1000);
}

void OnTxDone( void )
{
    AppState = APP_TX;
    printf( "<>>>>>>>>OnTxDone\n\r" );
    Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SendPayload((uint8_t*)"123456",6, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE });
}

void OnRxDone( void )
{
    AppState = APP_RX;
    printf( "<>>>>>>>>OnRxDone\n\r" );
    BufferSize = 0;
    Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
    Buffer[BufferSize+1] = 0;
    printf("size = %d ,%s",BufferSize,Buffer);
    // printf("OnRxDone\r\n",Buffer,BufferSize);
    //Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
}

void OnTxTimeout( void )
{
    AppState = APP_TX_TIMEOUT;
    printf( "<>>>>>>>>TXE\n\r" );
    Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SendPayload((uint8_t*)"12345",5, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE });
}

void OnRxTimeout( void )
{
    AppState = APP_RX_TIMEOUT;
    printf( "<>>>>>>>>OnRxTimeout\n\r" );
    //Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
    Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, 0xFFFF } );
}

void OnRxError(int error)
{
    AppState = APP_RX_ERROR;
    printf( "RXE<>>>>>>>>error code=%d\n\r",error);
    Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
}


void OnCadDone( bool channelActivityDetected )
{
    printf( "<>>OnCadDone  CAD code=%d\n\r",channelActivityDetected);
}