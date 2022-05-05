#include "modbus_slave1.h"
#include "ev_energy_meter.h"
#include "lockfree.h"

EvEnergyMeter *powerMeter = EvEnergyMeter::instance();

uint32_t htonl(uint32_t const net) {
    uint8_t data[4] = {};
    memcpy(&data, &net, sizeof(data));

    return ((uint32_t) data[0] << 0)
         | ((uint32_t) data[1] << 8)
         | ((uint32_t) data[2] << 16)
         | ((uint32_t) data[3] << 24);
}

STATIC_CLASS_INSTANCE(ModbusSlave1);

/**
 *@brief Прерывание по приёму от UART1
 */
__irq void USART1_IRQHandler();
extern "C" __irq void USART1_IRQHandler()
{
    static bool idleFlag = false;
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET) {
        idleFlag = false;
        ModbusSlave1::instance()->processRecievedByte(USART_ReceiveData(USART1));
    }

    if ((USART1->SR & USART_FLAG_IDLE) != (u16)RESET) {
        if (!idleFlag) {
            ModbusSlave1::instance()->setModbusState(ModbusSlave1::modbusStateUsartIdle);
            USART1->CR1 |= USART_CR1_IDLEIE;
            idleFlag = true;
        }
        uint32_t dummy = USART1->DR; (void) dummy;

    }

    USART_ClearITPendingBit(USART1, USART_IT_TC);

}


ModbusSlave1::ModbusSlave1() : ModbusSlaveBase()
{
    rs485.usart = USART1;
    rs485.port = GPIOA;
    rs485.rxPin = GPIO_Pin_10;
    rs485.txPin = GPIO_Pin_9;
    rs485.driverEnablePin = GPIO_Pin_11;
    rs485.receiverEnablePin = GPIO_Pin_12;

}

void ModbusSlave1::init() {
    deviceAddress = defaultDeviceAddress;
    ms1 = ModbusSlave1::instance();
}


void ModbusSlave1::debugMessageSend(uint8_t *_data, uint16_t _length)
{
    if (_length == 0) {
        transmit(_data);
    } else {
        transmit(_data, _length);
    }
}


/**
 * @brief Обработчик modbus запросов от сервера
 * @return состояние выполнения запроса
 */
bool ModbusSlave1::handleReceivedPacket()
{
    uint8_t txDataLength = 0;

    // Шапка сообщения
    modbusTxMessage.address = modbusRxMessage.address;
    modbusTxMessage.functionalCode = modbusRxMessage.functionalCode;

    // Откуда-куда
    // txDataLength += 4;
    // memcpy(&modbusTxMessage.data[0], &modbusRxMessage.data[0], txDataLength);

    addressCode.value = modbusRxMessage.data[0] << 8 | modbusRxMessage.data[1];
    uint16_t additionalData[2];
    additionalData[0] = modbusRxMessage.data[7] << 8 |
                        modbusRxMessage.data[8];
    additionalData[1] = modbusRxMessage.data[5] << 8 |
                        modbusRxMessage.data[6];

    union{
        uint32_t u32;
        uint16_t u16[2];
        uint8_t u8[4];
    }value;

    bool codeIsSupported = true;

    switch (ms1->addressCode.value) {
    case (addressCodes::instantPowerValue):
        value.u32 = powerMeter->getInstantCommonPowerMetered();
        break;

    case (addressCodes::totalPowerMetered):
        value.u32 = powerMeter->getTotalCommonPowerMetered();
        break;

    case (addressCodes::voltageL1N):
        value.u32 = powerMeter->getInstantVoltage(1);
        break;

    case (addressCodes::voltageL2N):
        value.u32 = powerMeter->getInstantVoltage(2);
        break;

    case (addressCodes::voltageL3N):
        value.u32 = powerMeter->getInstantVoltage(3);
        break;

    case (addressCodes::currentL1):
        value.u32 = powerMeter->getInstantCurrent(1);
        break;

    case (addressCodes::currentL2):
        value.u32 = powerMeter->getInstantCurrent(2);
        break;

    case (addressCodes::currentL3):
        value.u32 = powerMeter->getInstantCurrent(3);
        break;

    case (addressCodes::powerL1):
        value.u32 = powerMeter->getInstantPower(1);
        break;

    case (addressCodes::powerL2):
        value.u32 = powerMeter->getInstantPower(2);
        break;

    case (addressCodes::powerL3):
        value.u32 = powerMeter->getInstantPower(3);
        break;

    case (addressCodes::totalPowerMeteredL1):
        value.u32 = powerMeter->getTotalPowerMetered(1);
        break;

    case (addressCodes::totalPowerMeteredL2):
        value.u32 = powerMeter->getTotalPowerMetered(2);
        break;

    case (addressCodes::totalPowerMeteredL3):
        value.u32 = powerMeter->getTotalPowerMetered(3);
        break;

    case (addressCodes::calibrateLowerZone):
        value.u32 = powerMeter->doLowerZoneCalibration();
        break;

    case (addressCodes::calibrateUpperZone):
        value.u32 = powerMeter->doUpperZoneCalibration();
        break;

    case (addressCodes::calibrateValuesClear):
        value.u32 = powerMeter->doCleanupOfCalibration();
        break;

    // Показания частот определённых каналов
    case (addressCodes::frequencyChannel1):
        value.u32 = powerMeter->getFrequeencyOnChannel(1);
        break;
    case (addressCodes::frequencyChannel2):
        value.u32 = powerMeter->getFrequeencyOnChannel(2);
        break;
    case (addressCodes::frequencyChannel3):
        value.u32 = powerMeter->getFrequeencyOnChannel(3);
        break;
    case (addressCodes::frequencyChannel4):
        value.u32 = powerMeter->getFrequeencyOnChannel(4);
        break;
    case (addressCodes::frequencyChannel5):
        value.u32 = powerMeter->getFrequeencyOnChannel(5);
        break;
    case (addressCodes::frequencyChannel6):
        value.u32 = powerMeter->getFrequeencyOnChannel(6);
        break;

    // Сырые RMS значения каналов
    case (addressCodes::rmsRawCurrent1):
        value.u32 = powerMeter->getRawCurrent(1);
        break;
    case (addressCodes::rmsRawCurrent2):
        value.u32 = powerMeter->getRawCurrent(2);
        break;
    case (addressCodes::rmsRawCurrent3):
        value.u32 = powerMeter->getRawCurrent(3);
        break;
    case (addressCodes::rmsRawVoltage1):
        value.u32 = powerMeter->getRawVoltage(1);
        break;
    case (addressCodes::rmsRawVoltage2):
        value.u32 = powerMeter->getRawVoltage(2);
        break;
    case (addressCodes::rmsRawVoltage3):
        value.u32 = powerMeter->getRawVoltage(3);
        break;

    // Сырые значения каналов
    case (addressCodes::rawChannel1):
        value.u32 = powerMeter->getRawDataOfChannel(1);
        break;
    case (addressCodes::rawChannel2):
        value.u32 = powerMeter->getRawDataOfChannel(2);
        break;
    case (addressCodes::rawChannel3):
        value.u32 = powerMeter->getRawDataOfChannel(3);
        break;
    case (addressCodes::rawChannel4):
        value.u32 = powerMeter->getRawDataOfChannel(4);
        break;
    case (addressCodes::rawChannel5):
        value.u32 = powerMeter->getRawDataOfChannel(5);
        break;
    case (addressCodes::rawChannel6):
        value.u32 = powerMeter->getRawDataOfChannel(6);
        break;

    case (addressCodes::rmsBufferPacketCh1): {
        modbusTransmitArray(Adc1::instance()->getRmsBufferPointer(1),
                            Adc1::instance()->getRmsBufferLength(),
                            Adc1::instance()->getRmsBufferWidth());
        codeIsSupported = false;
    } break;
    case (addressCodes::rmsBufferPacketCh2): {
        modbusTransmitArray(Adc1::instance()->getRmsBufferPointer(2),
                            Adc1::instance()->getRmsBufferLength(),
                            Adc1::instance()->getRmsBufferWidth());
        codeIsSupported = false;
    } break;
    case (addressCodes::rmsBufferPacketCh3): {
        modbusTransmitArray(Adc1::instance()->getRmsBufferPointer(3),
                            Adc1::instance()->getRmsBufferLength(),
                            Adc1::instance()->getRmsBufferWidth());
        codeIsSupported = false;
    } break;
    case (addressCodes::rmsBufferPacketCh4): {
        modbusTransmitArray(Adc1::instance()->getRmsBufferPointer(4),
                            Adc1::instance()->getRmsBufferLength(),
                            Adc1::instance()->getRmsBufferWidth());
        codeIsSupported = false;
    } break;
    case (addressCodes::rmsBufferPacketCh5): {
        modbusTransmitArray(Adc1::instance()->getRmsBufferPointer(5),
                            Adc1::instance()->getRmsBufferLength(),
                            Adc1::instance()->getRmsBufferWidth());
        codeIsSupported = false;
    } break;
    case (addressCodes::rmsBufferPacketCh6): {
        modbusTransmitArray(Adc1::instance()->getRmsBufferPointer(6),
                            Adc1::instance()->getRmsBufferLength(),
                            Adc1::instance()->getRmsBufferWidth());
        codeIsSupported = false;
    } break;

    case (addressCodes::rmsBufferPacketPhaseA): {
        modbusTransmitPhase(Adc1::instance()->getRmsBufferPointer(1),
                            Adc1::instance()->getRmsBufferLength(),
                            Adc1::instance()->getRmsBufferWidth());
        codeIsSupported = false;
    } break;

    case (addressCodes::rmsBufferPacketPhaseB): {
        modbusTransmitPhase(Adc1::instance()->getRmsBufferPointer(3),
                            Adc1::instance()->getRmsBufferLength(),
                            Adc1::instance()->getRmsBufferWidth());
        codeIsSupported = false;
    } break;

    case (addressCodes::rmsBufferPacketPhaseC): {
        modbusTransmitPhase(Adc1::instance()->getRmsBufferPointer(5),
                            Adc1::instance()->getRmsBufferLength(),
                            Adc1::instance()->getRmsBufferWidth());
        codeIsSupported = false;
    } break;

    case (addressCodes::phaseDifferenceLane1):
        value.u32 = powerMeter->getPhaseDifference(1);
        break;
    case (addressCodes::phaseDifferenceLane2):
        value.u32 = powerMeter->getPhaseDifference(2);
        break;
    case (addressCodes::phaseDifferenceLane3):
        value.u32 = powerMeter->getPhaseDifference(3);
        break;

    case (addressCodes::usartFastMode):
        value.u32 = powerMeter->setUsartBaudrate(usartFastBaudrate);
        break;
    case (addressCodes::usartNormalMode):
        value.u32 = powerMeter->setUsartBaudrate(usartSlowBaudrate);
        break;

    case (addressCodes::equalizeMeters): {
        uint32_t val = additionalData[0]*UINT16_MAX;
        val += additionalData[1];
        powerMeter->setTotalPowerMetered(val);
    }
        break;
    case (addressCodes::illegalDataAddress):
        codeIsSupported = false;
        break;

    default:
        codeIsSupported = false;
        break;
    }

    if (codeIsSupported) {
        memset(&mExchangeBuffer.txData, 0, sizeof(mExchangeBuffer.txData));
        txDataLength = 0;
        modbusTxMessage.data[0] = 4; // DATALENGTH
        txDataLength++;

        modbusTxMessage.data[txDataLength + 0] = (value.u16[0] >> 8) & 0xFF;
        modbusTxMessage.data[txDataLength + 1] = (value.u16[0] >> 0) & 0xFF;
        modbusTxMessage.data[txDataLength + 2] = (value.u16[1] >> 8) & 0xFF;
        modbusTxMessage.data[txDataLength + 3] = (value.u16[1] >> 0) & 0xFF;
        txDataLength += 4;

        mExchangeBuffer.txData[0] = modbusTxMessage.address;
        mExchangeBuffer.txData[1] = static_cast<uint8_t>(modbusTxMessage.functionalCode);
        txDataLength += 2;

        memcpy(&mExchangeBuffer.txData[2], &modbusTxMessage.data[0], txDataLength - 2);

        uint16_t crc = calculateCrc(mExchangeBuffer.txData, txDataLength);
        mExchangeBuffer.txData[txDataLength]     = (crc >> 8) & 0xFF;
        mExchangeBuffer.txData[txDataLength + 1] = (crc >> 0) & 0xFF;

        transmissionRoutine(mExchangeBuffer.txData, txDataLength + 3);

        return true;

    }

    return false;

}

/**
 * @brief Метод отправки значений канала двумерного массива
 *        array[size][width]
 *
 * @param array - указатель на массив
 * @param size - его размер (количество выборок)
 * @param width - ширина массива (число каналов)
 * @param chnl - номер канала
 */
bool ModbusSlave1::modbusTransmitArray(uint16_t *array, uint16_t size, uint8_t width, uint8_t chnl)
{
    if (array == nullptr) {
        return false;
    }
    // taskENTER_CRITICAL();
    static uint8_t txDataLength = 0;
    uint16_t arraySize = size*width;

    memset(&mExchangeBuffer.txData, 0, sizeof(mExchangeBuffer.txData));
    txDataLength = 0;

    // Так как буфер 16ти битный
    uint16_t bytesToSend = size;

    uint16_t posIncrement = 1;

    // Размер буфера для даты за вычетом контрольной суммы и шапки
    uint16_t exchSize = sizeof(mExchangeBuffer.txData)-10;

    // Если 16битный буфер вмещается в буфер на передачу
    if (bytesToSend < (exchSize/2)){
        posIncrement = 1;
    } else {
        bytesToSend = (exchSize/2) - 1;

        posIncrement = (size) / (exchSize/2);
        posIncrement++;

    }

    bytesToSend--;
    txDataLength++;

    // Точное количество байт на отправку
    uint16_t byteCounter = 1;
    // Проходимся по всему массиву с заданным шагом
    uint16_t pos = 1;

    while (pos < size - posIncrement) {
        uint16_t posWidth = pos*width;
        static uint16_t val = 1000;
        val++;
        if (posWidth < arraySize) {
            val = array[posWidth];
        }
        modbusTxMessage.data[byteCounter + 0] = (val >> 8) & 0xFF;
        modbusTxMessage.data[byteCounter + 1] = (val >> 0) & 0xFF;

        byteCounter+=2;
        pos += posIncrement;
    }

    byteCounter--;
    modbusTxMessage.data[0] = byteCounter;
    txDataLength += byteCounter;

    mExchangeBuffer.txData[0] = modbusTxMessage.address;
    mExchangeBuffer.txData[1] = static_cast<uint8_t>(modbusTxMessage.functionalCode);
    txDataLength += 2;

    memcpy(&mExchangeBuffer.txData[2], &modbusTxMessage.data[0], txDataLength - 2);

    uint16_t crc = calculateCrc(mExchangeBuffer.txData, txDataLength);
    mExchangeBuffer.txData[txDataLength]     = (crc >> 8) & 0xFF;
    mExchangeBuffer.txData[txDataLength + 1] = (crc >> 0) & 0xFF;

    transmissionRoutine(mExchangeBuffer.txData, txDataLength + 3);

    // taskEXIT_CRITICAL();
    return true;

}



bool ModbusSlave1::modbusTransmitPhase(uint16_t *array, uint16_t size, uint8_t phase)
{
    if (array == nullptr) {
        return false;
    }
    phase--;
    // taskENTER_CRITICAL();
    static uint8_t txDataLength = 0;

    memset(&mExchangeBuffer.txData, 0, sizeof(mExchangeBuffer.txData));
    txDataLength = 0;

    // Так как буфер 16ти битный
    uint16_t bytesToSend = size;

    uint16_t posIncrement = 1;

    // Размер буфера для даты за вычетом контрольной суммы и шапки
    uint16_t exchSize = sizeof(mExchangeBuffer.txData)-10;

    // Если 16битный буфер вмещается в буфер на передачу
    if (bytesToSend*2 < (exchSize/2)){
        posIncrement = 1;
    } else {
        bytesToSend = (exchSize/2) - 1;

        posIncrement = (size) / (exchSize/2);
        posIncrement++;
        posIncrement *= 2;
    }

    bytesToSend--;
    txDataLength++;

    // Точное количество байт на отправку
    uint16_t byteCounter = 1;
    // Проходимся по всему массиву с заданным шагом
    uint16_t pos = 1;
    uint16_t txBufferSize = size - posIncrement;
    while (pos < txBufferSize) {
        uint16_t val;
        val = array[pos*(phase)];
        modbusTxMessage.data[byteCounter + 0] = (val >> 8) & 0xFF;
        modbusTxMessage.data[byteCounter + 1] = (val >> 0) & 0xFF;

        byteCounter+=2;
        pos += posIncrement;
    }

    pos = 1;
    while (pos < txBufferSize) {
        uint16_t val;
        val = array[pos*(phase)+1];
        modbusTxMessage.data[byteCounter + 0] = (val >> 8) & 0xFF;
        modbusTxMessage.data[byteCounter + 1] = (val >> 0) & 0xFF;

        byteCounter+=2;
        pos += posIncrement;
    }

    byteCounter--;
    modbusTxMessage.data[0] = byteCounter;
    txDataLength += byteCounter;

    mExchangeBuffer.txData[0] = modbusTxMessage.address;
    mExchangeBuffer.txData[1] = static_cast<uint8_t>(modbusTxMessage.functionalCode);
    txDataLength += 2;

    memcpy(&mExchangeBuffer.txData[2], &modbusTxMessage.data[0], txDataLength - 2);

    uint16_t crc = calculateCrc(mExchangeBuffer.txData, txDataLength);
    mExchangeBuffer.txData[txDataLength]     = (crc >> 8) & 0xFF;
    mExchangeBuffer.txData[txDataLength + 1] = (crc >> 0) & 0xFF;

    transmissionRoutine(mExchangeBuffer.txData, txDataLength + 3);

    // taskEXIT_CRITICAL();
    return true;
}


/**
 * @brief обработчик запроса на получение значения
 *        суммарного мгновенного показания мощности
 * @return состояние выполнения запроса
 */
bool ModbusSlave1::handleInstantPowerValueRequest()
{
    // uint16_t value = powerMeter->getInstantCommonPowerMetered();

    return true;
}


/**
 * @brief обработчик запроса на получение значения
 *        насчитанной энергии за всё время работы счётчика
 * @return состояние выполнения запроса
 */
bool ModbusSlave1::handleTotalPowerMeteredRequest()
{
    // uint16_t value = powerMeter->getTotalCommonPowerMetered();

    return true;
}
