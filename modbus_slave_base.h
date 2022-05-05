#ifndef _MODBUS_SLAVE_H_
#define _MODBUS_SLAVE_H_

#include "../unilib/thread.h"
#include "../unilib/software_timer.h"

#include "freertos.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include <string.h>
#include <stdbool.h>

#include "spl.h"

class ModbusSlaveBase
{
public:
    ModbusSlaveBase();
    virtual ~ModbusSlaveBase() = default;
    virtual void init() = 0;

protected:
    uint32_t usartBaudrate = 9600;
    NVIC_InitTypeDef usartNvic;
    USART_InitTypeDef usartStruct;
    uint16_t deviceAddress;

    /* Структура конфигурации пинов RS-485 */
    struct rs485Conf_s{
        USART_TypeDef * usart;
        GPIO_TypeDef * port;
        uint16_t rxPin;
        uint16_t txPin;
        uint16_t driverEnablePin;
        uint16_t receiverEnablePin;
    }rs485;

    static const struct {
        uint16_t discreteOutput = 1;
        uint16_t coils = 1;
        uint16_t discreteInput = 10001;
        uint16_t contacts = 10001;
        uint16_t analogInput = 30001;
        uint16_t analogOutput = 40001;
        uint16_t holding = 40001;
    }registerOffset;

    enum class modbusFunctionalCodes : int{
        notDefined = 0,

        readCoil = 1,
        readDiscrete = 2,
        readHoldingRegisters = 3,
        readInputRegisters = 4,

        writeCoil = 5,
        writeHoldingRegister = 6,
        writeSeveralCoils = 15,
        writeSeveralHoldingRegisters = 16

    };

    struct defaultAddressCodes {
        enum codes{
            empty = 0xff0,
            // сервисные коды modbus
            illegalFunction,
            illegalDataAddress,
            illegalDataValue
        };

        uint16_t value = {empty};

    };

    struct modbusMessage_s {
        uint8_t address;
        modbusFunctionalCodes functionalCode = modbusFunctionalCodes::notDefined;
        uint8_t data[150];
        uint16_t receivedCrc;
        uint16_t calculatedCrc;
    } modbusRxMessage;


public:
    /* Состояния шины RS-485 */
    typedef enum {
        busStateReady,
        busStateAcquired,
        busStateReleased,
        busStateBusy,
        busStateBroken

    }busState_t;

    typedef enum{
        modbusStateNone,
        modbusStateUsartIdle,
        modbusStateByteReceived,
        modbusStateByteSaved,
        modbusStatePacketReceived,
        modbusStatePacketParcing
    }modbusState_t;
protected:
    busState_t busState;
    SemaphoreHandle_t waitForParcing;

protected:
    modbusState_t _modbusState;
    void setModbusState(modbusState_t state);

private:
    /**
     * @brief Указатель на поток
     */
    Thread *mThread;

protected:
    /**
     * @brief Обработка принятого байта
     * @param _data принятый байт
     */
    void processRecievedByte(uint8_t _data);

    /**
     * @brief Инициализация переферии
     */
    void initInterface();

    void transmissionRoutine(uint8_t *_data, uint16_t length);

    /**
     * @brief Занимает линию для передачи данных
     */
    void acquireBus();

    /**
     * @brief Освобождает линию для приёма данных
     */
    void releaseBus();

    /**
     * @brief Передача данных через интерфейс
     * @param _data указатель на данные для передачи
     * @param _length длина передаваемых данных
     */
    void transmit(uint8_t *_data, uint16_t _length);
    void transmit(uint8_t *_data);

private:
    /**
     * @brief Задача слейва
     */
    void taskThread();

protected:
    /**
     * @brief Буфер для приёма/отправки данных
     */
    static constexpr const uint8_t maxModbusBufferSize = 150;
    struct exchangeBuffer {
        uint8_t rxData[maxModbusBufferSize];      /*!< Массив обмена данными */
        uint8_t txData[maxModbusBufferSize];
        uint8_t count;          /*!< Колличество данных для отправки / принятых данных */
    } mExchangeBuffer;


protected:
    /**
     * @brief Подсчитывает CRC
     * @param _data указатель на данные
     * @param _length длина подсчитываемых данныъ
     * @return расчитанное CRC
     */
    uint16_t calculateCrc(uint8_t *_data, uint16_t _length);

    /**
     * @brief Подготавливает пакет с запросом и отправляем его
     */
    void prepareAndSendRequestPacket();

    /**
     * @brief Обрабатывает ответ от устройства
     * @return успешность операции
     */
    virtual bool parceRecievedResponce();

    virtual bool handleReceivedPacket() = 0;

    /**
     * @brief Обработчик события таймаута ожидания ответа на запрос
     */
    void onResponceTimeout();

public:
    void usartReinit (uint32_t baudrate);

};


#endif //!_MODBUS_MASTER_H_
