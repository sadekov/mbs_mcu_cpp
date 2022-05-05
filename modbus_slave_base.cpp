#include "modbus_slave_base.h"
#include "modbus_crc_table.h"
#include "lockfree.h"

/* -------------------------------------------------------------------- */
ModbusSlaveBase::ModbusSlaveBase()
{
    mThread = new Thread(this, &ModbusSlaveBase::taskThread, configMINIMAL_STACK_SIZE, 3);
    _modbusState = modbusStateNone;
}


/* -------------------------------------------------------------------- */
__attribute((noreturn)) void ModbusSlaveBase::taskThread()
{
    initInterface();
    waitForParcing = xSemaphoreCreateBinary();

    while (true) {
        xSemaphoreTake(waitForParcing, portMAX_DELAY);
        if (_modbusState == modbusStateUsartIdle) {
            _modbusState = modbusState_t::modbusStatePacketParcing;
            parceRecievedResponce();
            _modbusState = modbusStateNone;
        }
    }
}

/* -------------------------------------------------------------------- */
void ModbusSlaveBase::processRecievedByte(uint8_t _data)
{
    if (_modbusState != modbusStatePacketParcing) {
        mExchangeBuffer.rxData[mExchangeBuffer.count++] = _data;

        if (mExchangeBuffer.count > maxModbusBufferSize - 1) {
            setModbusState(modbusStateNone);
            mExchangeBuffer.count = 0;
        }
    }

}

/* -------------------------------------------------------------------- */
uint16_t ModbusSlaveBase::calculateCrc(uint8_t *_data, uint16_t _length)
{
    uint16_t  uchCRC = 0xFFFF;

    while (_length--) {
        uchCRC = (uchCRC >> 8) ^ crcTable [ (uchCRC ^ *_data++) & 0xff ];
    }

    return (uchCRC << 8) | (uchCRC >> 8);
}


/* -------------------------------------------------------------------- */

bool ModbusSlaveBase::parceRecievedResponce()
{
    /* ~~~ Структура пакета modbus ~~~
     * uint8_t   - адрес клиента
     * uint8_t   - функциональный код
     * uint16_t  - адрес первого регистра (с которого регистра начинать запись)
     * uint16_t  - количество регистров
     * uint8_t   - количество байт
     * uint16_t  - значение регистра 1
     * ...
     * uint16_t  - значение регистра n
     * uint16_t  - значение регистра 1
     * uint16_t  - контрольная сумма
     * uint8_t   - адрес отправителя (???)
     */

    uint8_t dataCount = mExchangeBuffer.count;

    modbusRxMessage = {};
    memset(&modbusRxMessage.data, 0, sizeof(modbusRxMessage.data));

    modbusRxMessage.address = mExchangeBuffer.rxData[0];

    if (modbusRxMessage.address != deviceAddress) {
        mExchangeBuffer.count = 0;
        memset(&mExchangeBuffer.rxData, 0 ,sizeof(mExchangeBuffer.rxData));
        return false;
    }

    modbusRxMessage.functionalCode =  (modbusFunctionalCodes)mExchangeBuffer.rxData[1];
    if (dataCount > sizeof(modbusRxMessage.data) - 2) {
        mExchangeBuffer.count = 0;
        return false;
    } else {
        if (dataCount > 5) {
            memcpy(&modbusRxMessage.data[0], &mExchangeBuffer.rxData[2],
                dataCount - 4);
            mExchangeBuffer.count = 0;

        } else {
            mExchangeBuffer.count = 0;
            return false;
        }

    }

    // -------------------

    uint16_t crc = calculateCrc(mExchangeBuffer.rxData, dataCount - 2);
    uint16_t recievedCrc = mExchangeBuffer.rxData[dataCount - 2] << 8 |
                           mExchangeBuffer.rxData[dataCount - 1];

    mExchangeBuffer.count = 0;
    memset(&mExchangeBuffer.rxData, 0 ,sizeof(mExchangeBuffer.rxData));

    if (crc != recievedCrc) {
        return false;
    }
    // -------------------

    switch (modbusRxMessage.functionalCode) {
    case ModbusSlaveBase::modbusFunctionalCodes::notDefined:
        break;
    case ModbusSlaveBase::modbusFunctionalCodes::readCoil:
        handleReceivedPacket();
        break;
    case ModbusSlaveBase::modbusFunctionalCodes::readDiscrete:
        handleReceivedPacket();
        break;
    case ModbusSlaveBase::modbusFunctionalCodes::readHoldingRegisters:
        handleReceivedPacket();
        break;
    case ModbusSlaveBase::modbusFunctionalCodes::readInputRegisters:
        handleReceivedPacket();
        break;
    case ModbusSlaveBase::modbusFunctionalCodes::writeCoil:
        prepareAndSendRequestPacket();
        break;
    case ModbusSlaveBase::modbusFunctionalCodes::writeHoldingRegister:
        handleReceivedPacket();
        prepareAndSendRequestPacket();
        break;
    case ModbusSlaveBase::modbusFunctionalCodes::writeSeveralCoils:
        prepareAndSendRequestPacket();
        break;
    case ModbusSlaveBase::modbusFunctionalCodes::writeSeveralHoldingRegisters:
        handleReceivedPacket();
        prepareAndSendRequestPacket();
        break;

    }

    return true;
}

/* -------------------------------------------------------------------- */
void ModbusSlaveBase::prepareAndSendRequestPacket()
{
    /* ~~~ Структура пакета modbus ~~~
     * uint8_t   - адрес отправителя
     * uint8_t   - функциональный код
     * uint16_t  - адрес первого регистра (с которого регистра начинать запись)
     * uint16_t  - количество регистров
     * uint16_t  - контрольная сумма
     */

    memset(mExchangeBuffer.txData, 0, sizeof (mExchangeBuffer.txData));
    uint8_t bytesToSend = 6;

    mExchangeBuffer.txData[0] = modbusRxMessage.address;
    mExchangeBuffer.txData[1] = static_cast<uint8_t>(
        modbusFunctionalCodes::writeSeveralHoldingRegisters);

    memcpy(&mExchangeBuffer.txData[2], &modbusRxMessage.data[0], 4);

    uint16_t crc = calculateCrc(mExchangeBuffer.txData, bytesToSend);
    mExchangeBuffer.txData[6] = (crc >> 8) & 0xFF;
    mExchangeBuffer.txData[7] = (crc >> 0) & 0xFF;
    bytesToSend += 2;

    mExchangeBuffer.count = 0;
    // acquireBus();
    transmissionRoutine(mExchangeBuffer.txData, bytesToSend);// + 2
    // releaseBus();


}

/* --------------------------------------------------------------- */
void ModbusSlaveBase::transmissionRoutine(uint8_t *_data, uint16_t length)
{
    acquireBus();
    transmit(_data, length);// + 2
    releaseBus();
}


/**
 * @brief Занимает линию на передачу
 */
void ModbusSlaveBase::acquireBus()
{
    busState = busStateAcquired;
    portENTER_CRITICAL();
    GPIO_SetBits(rs485.port, rs485.driverEnablePin);
    GPIO_SetBits(rs485.port, rs485.receiverEnablePin);
    portEXIT_CRITICAL();
}

/**
 * @brief Занимает линию на приём
 */
void ModbusSlaveBase::releaseBus()
{
//    EvEnergyMeter::instance()->indicatorShortBlink();
    portENTER_CRITICAL();
    GPIO_ResetBits(rs485.port, rs485.driverEnablePin);
    GPIO_ResetBits(rs485.port, rs485.receiverEnablePin);
    portEXIT_CRITICAL();
    busState = busStateReleased;
}

void ModbusSlaveBase::transmit(uint8_t *_data, uint16_t _length)
{
    if (busState!=busStateAcquired){
        return;
    }

    portENTER_CRITICAL();
    while (_length--) {
        // Ожидание флага transmit data register empty
        while(USART_GetFlagStatus(rs485.usart, USART_FLAG_TXE)== RESET){};
        USART_SendData(USART1, *_data++);
        // Ожидание флага transmission complete
        while(USART_GetFlagStatus(rs485.usart, USART_FLAG_TC) == RESET){};
    }
    portEXIT_CRITICAL();
}

void ModbusSlaveBase::transmit(uint8_t *_data)
{
    if (busState!=busStateAcquired){
       return;
    }

    while (*_data) {
        // Ожидание флага transmit data register empty
       while(USART_GetFlagStatus(rs485.usart, USART_FLAG_TXE)== RESET){};
        USART_SendData(USART1, *_data++);
        // Ожидание флага transmission complete
        while(USART_GetFlagStatus(rs485.usart, USART_FLAG_TC) == RESET){};
    }
}


/* ---------------------------------------------------------- */

/**
 * @brief Инициализация переферии
 */
void ModbusSlaveBase::initInterface()
{
    portENTER_CRITICAL();

    /* Порты ввода вывода */
    /* Активация тактирования GPIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Настраиваем UART TX pin
    GPIO_InitTypeDef init = {0};
    init.GPIO_Pin   = rs485.txPin;
    init.GPIO_Mode  = GPIO_Mode_AF_PP;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(rs485.port, &init);

    // Настраиваем UART RX pin
    init.GPIO_Pin   = rs485.rxPin;
    init.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(rs485.port, &init);

    /* Reset, TX enable */
    init.GPIO_Pin   = rs485.driverEnablePin | rs485.receiverEnablePin;
    init.GPIO_Mode  = GPIO_Mode_Out_PP;
    init.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(rs485.port, &init);

    /* Активация тактирования  USART1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);

    /* Настройка NVIC */
    memset(&usartNvic, 0, sizeof(usartNvic));

    usartNvic.NVIC_IRQChannel = USART1_IRQn;
    usartNvic.NVIC_IRQChannelPreemptionPriority = 0x07;
    usartNvic.NVIC_IRQChannelSubPriority = 0;
    usartNvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&usartNvic);

    /* Настройка USART */
    memset(&usartStruct, 0, sizeof(usartStruct));

    usartStruct.USART_BaudRate = usartBaudrate;
    usartStruct.USART_WordLength = USART_WordLength_8b;
    usartStruct.USART_Parity = USART_Parity_No;
    usartStruct.USART_StopBits = USART_StopBits_1;
    usartStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(rs485.usart, &usartStruct);
    /* Активация USART1 */
    USART_Cmd(rs485.usart, ENABLE);

    /* Разрешение флагов прерывания USART */
    USART_ITConfig(rs485.usart, USART_IT_RXNE, ENABLE);
    USART_ITConfig(rs485.usart, USART_IT_IDLE, ENABLE);

    portEXIT_CRITICAL();

}


void ModbusSlaveBase::usartReinit(uint32_t baudrate)
{
    portENTER_CRITICAL();

    /* Настройка NVIC */

    usartNvic.NVIC_IRQChannel = USART1_IRQn;
    usartNvic.NVIC_IRQChannelPreemptionPriority = 0x06;
    usartNvic.NVIC_IRQChannelSubPriority = 1;
    usartNvic.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&usartNvic);

    /* Настройка USART */
    USART_DeInit(rs485.usart);
    USART_Cmd(rs485.usart, DISABLE);

    usartStruct.USART_BaudRate = baudrate;
    usartStruct.USART_WordLength = USART_WordLength_8b;
    usartStruct.USART_Parity = USART_Parity_No;
    usartStruct.USART_StopBits = USART_StopBits_1;
    usartStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(rs485.usart, &usartStruct);
    /* Активация USART1 */
    USART_Cmd(rs485.usart, ENABLE);

    /* Разрешение флагов прерывания USART */
    USART_ITConfig(rs485.usart, USART_IT_RXNE, ENABLE);
    USART_ITConfig(rs485.usart, USART_IT_IDLE, ENABLE);

    usartNvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&usartNvic);

    portEXIT_CRITICAL();
}


void ModbusSlaveBase::setModbusState(modbusState_t state) {
    _modbusState = state;
    switch (state) {
    case modbusStateUsartIdle:
        xSemaphoreGiveFromISR(waitForParcing, 0);
        break;

    default:
        break;
    }

}

