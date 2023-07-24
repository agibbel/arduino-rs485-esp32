/*
 * RS485 implementation for ESP32
 * 
 * Author: Andrey tepliakov
 */
#ifndef __RS485_HPP__
#define __RS485_HPP__

#include <HardwareSerial.h>
#include <driver/uart.h>

class RS485 : protected HardwareSerial
{
public:
    RS485(int uart_nr) : HardwareSerial(uart_nr) {}

    // setRxTimeout sets the timeout after which onReceive callback will be called (after receiving data, it waits for this time of UART rx inactivity to call the callback fnc)
    // param symbols_timeout defines a timeout threshold in uart symbol periods. Setting 0 symbol timeout disables the callback call by timeout.
    //                       Maximum timeout setting is calculacted automatically by IDF. If set above the maximum, it is ignored and an error is printed on Serial0 (check console).
    //                       Examples: Maximum for 11 bits symbol is 92 (SERIAL_8N2, SERIAL_8E1, SERIAL_8O1, etc), Maximum for 10 bits symbol is 101 (SERIAL_8N1).
    //                       For example symbols_timeout=1 defines a timeout equal to transmission time of one symbol (~11 bit) on current baudrate. 
    //                       For a baudrate of 9600, SERIAL_8N1 (10 bit symbol) and symbols_timeout = 3, the timeout would be 3 / (9600 / 10) = 3.125 ms
    inline void setRxTimeout(uint8_t symbols_timeout)
    {
        HardwareSerial::setRxTimeout(symbols_timeout);
    }

    // setRxFIFOFull(uint8_t fifoBytes) will set the number of bytes that will trigger UART_INTR_RXFIFO_FULL interrupt and fill up RxRingBuffer
    // This affects some functions such as Serial::available() and Serial.read() because, in a UART flow of receiving data, Serial internal 
    // RxRingBuffer will be filled only after these number of bytes arrive or a RX Timeout happens.
    // This parameter can be set to 1 in order to receive byte by byte, but it will also consume more CPU time as the ISR will be activates often.
    inline void setRxFIFOFull(uint8_t fifoBytes)
    {
        HardwareSerial::setRxFIFOFull(fifoBytes);
    }

    // onReceive will setup a callback that will be called whenever an UART interruption occurs (UART_INTR_RXFIFO_FULL or UART_INTR_RXFIFO_TOUT)
    // UART_INTR_RXFIFO_FULL interrupt triggers at UART_FULL_THRESH_DEFAULT bytes received (defined as 120 bytes by default in IDF)
    // UART_INTR_RXFIFO_TOUT interrupt triggers at UART_TOUT_THRESH_DEFAULT symbols passed without any reception (defined as 10 symbos by default in IDF)
    // onlyOnTimeout parameter will define how onReceive will behave:
    // Default: true -- The callback will only be called when RX Timeout happens. 
    //                  Whole stream of bytes will be ready for being read on the callback function at once.
    //                  This option may lead to Rx Overflow depending on the Rx Buffer Size and number of bytes received in the streaming
    //         false -- The callback will be called when FIFO reaches 120 bytes and also on RX Timeout.
    //                  The stream of incommig bytes will be "split" into blocks of 120 bytes on each callback.
    //                  This option avoid any sort of Rx Overflow, but leaves the UART packet reassembling work to the Application.
    inline void onReceive(OnReceiveCb function, bool onlyOnTimeout = false)
    {
        HardwareSerial::onReceive(function, onlyOnTimeout);
    }

    // onReceive will be called on error events (see hardwareSerial_error_t)
    inline void onReceiveError(OnReceiveErrorCb function)
    {
        HardwareSerial::onReceiveError(function);
    }

    // eventQueueReset clears all events in the queue (the events that trigger onReceive and onReceiveError) - maybe usefull in some use cases
    inline void eventQueueReset()
    {
        HardwareSerial::eventQueueReset();
    }

    void begin(unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, int8_t dirPin = -1, bool invert = false, unsigned long timeout_ms = 20000UL, uint8_t rxfifo_full_thrhd = 112);

    inline void end(bool fullyTerminate = true)
    {
        HardwareSerial::end(fullyTerminate);
    }

    inline void updateBaudRate(unsigned long baud)
    {
        HardwareSerial::updateBaudRate(baud);
    }

    inline int available(void)
    {
        return HardwareSerial::available();
    }

    inline int availableForWrite(void)
    {
        return HardwareSerial::availableForWrite();
    }

    inline int peek(void)
    {
        return HardwareSerial::peek();
    }

    inline int read(void)
    {
        return HardwareSerial::read();
    }

    inline size_t read(uint8_t* buffer, size_t size)
    {
        return HardwareSerial::read(buffer, size);
    }

    inline size_t read(char* buffer, size_t size)
    {
        return HardwareSerial::read((uint8_t*)buffer, size);
    }

    // Overrides Stream::readBytes() to be faster using IDF
    inline size_t readBytes(uint8_t* buffer, size_t length)
    {
        return HardwareSerial::readBytes(buffer, length);
    }

    inline size_t readBytes(char* buffer, size_t length)
    {
        return HardwareSerial::readBytes((uint8_t*)buffer, length);
    }

    inline void flush(void)
    {
        HardwareSerial::flush();
    }

    inline void flush(bool txOnly)
    {
        HardwareSerial::flush(txOnly);
    }

    inline size_t write(uint8_t c)
    {
        return HardwareSerial::write(c);
    }

    inline size_t write(const uint8_t* buffer, size_t size)
    {
        return HardwareSerial::write(buffer, size);
    }

    inline size_t write(const char* buffer, size_t size)
    {
        return HardwareSerial::write((uint8_t*)buffer, size);
    }

    inline size_t write(const char* s)
    {
        return HardwareSerial::write((uint8_t*)s, strlen(s));
    }

    inline size_t write(unsigned long n)
    {
        return HardwareSerial::write((uint8_t)n);
    }

    inline size_t write(long n)
    {
        return HardwareSerial::write((uint8_t)n);
    }

    inline size_t write(unsigned int n)
    {
        return HardwareSerial::write((uint8_t)n);
    }

    inline size_t write(int n)
    {
        return HardwareSerial::write((uint8_t)n);
    }

    inline uint32_t baudRate(void)
    {
        return HardwareSerial::baudRate();
    }

    inline void setDebugOutput(bool en)
    {
        HardwareSerial::setDebugOutput(en);
    }

    inline void setRxInvert(bool invert)
    {
        HardwareSerial::setRxInvert(invert);
    }

    // Negative Pin Number will keep it unmodified, thus this function can set individual pins
    // SetPins shall be called after Serial begin()
    inline void setPins(int8_t rxPin, int8_t txPin, int8_t rtsPin)
    {
        HardwareSerial::setPins(rxPin, txPin, -1, rtsPin);
    }

    inline size_t setRxBufferSize(size_t new_size)
    {
        return HardwareSerial::setRxBufferSize(new_size);
    }

    inline size_t setTxBufferSize(size_t new_size)
    {
        return HardwareSerial::setTxBufferSize(new_size);
    }
};

#endif // __RS485_HPP__
