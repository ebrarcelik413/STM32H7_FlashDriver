#pragma once

#include "iuart_driver.hpp"
#include "main.h"

class UartDriver : public IUartDriver
{
public:
    explicit UartDriver(UART_HandleTypeDef* huart);
    ~UartDriver() override = default;

    UartStatus init() override;
    UartStatus deinit() override;
    UartStatus write(const uint8_t* data, std::size_t length) override;
    UartStatus writeString(const char* str) override;
    UartStatus read(uint8_t* data, std::size_t length, uint32_t timeoutMs) override;

private:
    UART_HandleTypeDef* huart_{nullptr};
    bool isInitialized_{false};
};
