/*
 * RS485 implementation for ESP32
 *
 * Author: Andrey tepliakov
 */
#include "RS485.h"

#ifndef SOC_RTS0
#if CONFIG_IDF_TARGET_ESP32
#define SOC_RTS0 22
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define SOC_RTS0 15
#endif
#endif

#if SOC_UART_NUM > 1
#ifndef RTS1
#if CONFIG_IDF_TARGET_ESP32
#define RTS1 11
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define RTS1 19
#endif
#endif
#endif /* SOC_UART_NUM > 1 */

#if SOC_UART_NUM > 2
#ifndef RTS2
#if CONFIG_IDF_TARGET_ESP32
#define RTS2 7
#endif
#endif
#endif /* SOC_UART_NUM > 2 */

void RS485::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, int8_t dirPin, bool invert, unsigned long timeout_ms, uint8_t rxfifo_full_thrhd)
{
    HardwareSerial::begin(baud, config, rxPin, txPin, invert, timeout_ms, rxfifo_full_thrhd);
    uint8_t __rtsPin;
    if (dirPin >= 0)
        __rtsPin = dirPin;
    else
        switch (_uart_nr)
        {
        case UART_NUM_0:
            __rtsPin = SOC_RTS0;
            break;
#if SOC_UART_NUM > 1
        case UART_NUM_1:
            __rtsPin = RTS1;
            break;
#endif
#if SOC_UART_NUM > 2
        case UART_NUM_2:
            __rtsPin = RTS2;
            break;
#endif
        }
    HardwareSerial::setPins(rxPin, txPin, -1, __rtsPin);
    setHwFlowCtrlMode(HW_FLOWCTRL_DISABLE);
    uart_set_mode(_uart_nr, UART_MODE_RS485_HALF_DUPLEX);
}
