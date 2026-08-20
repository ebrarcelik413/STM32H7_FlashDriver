#include "uart_driver.hpp"
#include <cstring>

UartDriver::UartDriver(UART_HandleTypeDef* huart)
    : huart_(huart)
{
}

UartStatus UartDriver::init()
{
    if (huart_ == nullptr)
    {
        return UartStatus::InvalidParam;
    }

    isInitialized_ = true;
    return UartStatus::Ok;
}

UartStatus UartDriver::deinit()
{
    isInitialized_ = false;
    return UartStatus::Ok;
}

UartStatus UartDriver::write(const uint8_t* data, std::size_t length)
{
    if (!isInitialized_ || (huart_ == nullptr))
    {
        return UartStatus::NotInitialized;
    }

    if ((data == nullptr) || (length == 0U))
    {
        return UartStatus::InvalidParam;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart_, const_cast<uint8_t*>(data), static_cast<uint16_t>(length), HAL_MAX_DELAY);
    return (status == HAL_OK) ? UartStatus::Ok : UartStatus::Error;
}

UartStatus UartDriver::writeString(const char* str)
{
    if (str == nullptr)
    {
        return UartStatus::InvalidParam;
    }

    return write(reinterpret_cast<const uint8_t*>(str), std::strlen(str));
}

UartStatus UartDriver::read(uint8_t* data, std::size_t length, uint32_t timeoutMs)
{
    if (!isInitialized_ || (huart_ == nullptr))
    {
        return UartStatus::NotInitialized;
    }

    if ((data == nullptr) || (length == 0U))
    {
        return UartStatus::InvalidParam;
    }

    HAL_StatusTypeDef status = HAL_UART_Receive(huart_, data, static_cast<uint16_t>(length), timeoutMs);
    return (status == HAL_OK) ? UartStatus::Ok : UartStatus::Timeout;
}
