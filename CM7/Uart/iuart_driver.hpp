#pragma once

#include <cstddef>
#include <cstdint>

enum class UartStatus : uint8_t
{
    Ok = 0,
    Error,
    Timeout,
    Busy,
    InvalidParam,
    NotInitialized
};

class IUartDriver
{
public:
    virtual ~IUartDriver() = default;

    virtual UartStatus init() = 0;

    virtual UartStatus deinit() = 0;

    virtual UartStatus send(
        const uint8_t* data,
        std::size_t length) = 0;

    virtual UartStatus sendDMA(
        const uint8_t* data,
        std::size_t length) = 0;

    virtual UartStatus receive(
        uint8_t* data,
        std::size_t length,
        uint32_t timeoutMs) = 0;
};
