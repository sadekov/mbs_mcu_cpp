#pragma once

#include "modbus_slave_base.h"
#include "../unilib/staticclass.h"

#include "spl.h"


extern "C" void USART1_IRQHandler();

class ModbusSlave1 : public ModbusSlaveBase
{
    STATIC_CLASS(ModbusSlave1);
public:
    ModbusSlave1 *ms1;
    void init();
    void debugMessageSend(uint8_t *_data, uint16_t _length = 0);
    USART_TypeDef* usart = USART1;

private:
    /**
     * @brief Обработчик прерывания
     */
    friend void USART1_IRQHandler();

private:
    bool handleReceivedPacket();

    static const uint32_t usartFastBaudrate = 230400;
    static const uint32_t usartSlowBaudrate = 9600;

    uint32_t usartBaudrate = usartSlowBaudrate;

    static const uint16_t defaultDeviceAddress = 0x2;

    struct addressCodes : defaultAddressCodes{
        // Коды счётчика
        static const uint16_t serviceCommandsOffset = 0xff00;
        enum codes{
            // V * 10
            voltageL1N = 0x0,
            voltageL2N = 0x2,
            voltageL3N = 0x4,

            // A * 1000
            currentL1 = 0xc,
            currentL2 = 0xe,
            currentL3 = 0x10,

            // kwh * 10
            powerL1 = 0x12,
            powerL2 = 0x14,
            powerL3 = 0x16,

            // W * 10
            instantPowerValue       = 0x28,

            // kW * 10
            totalPowerMetered       = 0x3e,

            totalPowerMeteredL1     = 0x70,
            totalPowerMeteredL2     = 0x72,
            totalPowerMeteredL3     = 0x74,


            seviceOffset            = serviceCommandsOffset,

            frequencyChannel1,
            frequencyChannel2,
            frequencyChannel3,
            frequencyChannel4,
            frequencyChannel5,
            frequencyChannel6,

            rmsRawVoltage1,
            rmsRawVoltage2,
            rmsRawVoltage3,
            rmsRawCurrent1,
            rmsRawCurrent2,
            rmsRawCurrent3,

            rawChannel1,
            rawChannel2,
            rawChannel3,
            rawChannel4,
            rawChannel5,
            rawChannel6,

            rmsBufferPacketCh1,
            rmsBufferPacketCh2,
            rmsBufferPacketCh3,
            rmsBufferPacketCh4,
            rmsBufferPacketCh5,
            rmsBufferPacketCh6,

            rmsBufferPacketPhaseA,
            rmsBufferPacketPhaseB,
            rmsBufferPacketPhaseC,

            phaseDifferenceLane1,
            phaseDifferenceLane2,
            phaseDifferenceLane3,

            usartFastMode,
            usartNormalMode,

            calibrateLowerZone      , //= serviceCommandsOffset + 1,
            calibrateUpperZone      , //= serviceCommandsOffset + 2,
            calibrateValuesClear    , //= serviceCommandsOffset + 3

            equalizeMeters,

            last

        };

    };
    addressCodes addressCode;

    bool handleTotalPowerMeteredRequest();
    bool handleInstantPowerValueRequest();

    bool modbusTransmitArray(uint16_t *array, uint16_t size, uint8_t width, uint8_t chnl = 0);
    bool modbusTransmitPhase(uint16_t *array, uint16_t size, uint8_t phase);

protected:
    modbusMessage_s modbusTxMessage;

};
