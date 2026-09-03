#pragma once

#include "iuart_driver.hpp"
#include "main.h"

#include <cstddef>
#include <cstdint>

class UartDriver : public IUartDriver
{
public:
    static constexpr std::size_t RX_BUFFER_SIZE = 100U;

    explicit UartDriver(UART_HandleTypeDef* huart);
    ~UartDriver() override = default;

    UartStatus init() override;
    UartStatus deinit() override;

    UartStatus send(const uint8_t* data, std::size_t length) override;

    UartStatus sendDMA(const uint8_t* data, std::size_t length) override;

    UartStatus receive( uint8_t* data, std::size_t length, uint32_t timeoutMs) override;


    // =========================================================
    // RX RING BUFFER
    //
    // Burada UART'tan gelen HAM byte'lar tutulur.
    //
    // Ornek:
    // A5 5A 02 00 01 01
    //
    // Protocol burada COZULMEZ.
    // =========================================================

    bool readByte(uint8_t& byte);

    bool readData( uint8_t* destination,std::size_t length);

    std::size_t getAvailableDataCount() const;
    std::size_t getFreeSpace() const;

    void clear();
    void flushRxBuffer();
    bool isOverflow() const;
    void clearOverflow();


    bool isTxBusy() const;

    void onRxByteReceived();
    void onTxComplete();
    void onError();


private:

    void startReceiveIT();
    bool pushRxByte(uint8_t byte);

    UART_HandleTypeDef* huart_{nullptr};

    bool isInitialized_{false};
    volatile bool txBusy_{false};
    volatile bool isOverflow_{false};

    uint8_t rxBuffer_[RX_BUFFER_SIZE]{};

    volatile std::size_t head_{0U};
    volatile std::size_t tail_{0U};
    volatile std::size_t rxCount_{0U};

    uint8_t rxRawByte_{0U};
};
