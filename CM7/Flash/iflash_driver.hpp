#pragma once

#include "flash_types.hpp"
#include <cstdint>
#include <cstddef>

class IFlashDriver
{
public:
    virtual ~IFlashDriver() = default;

    virtual FlashStatus init() = 0;
    virtual FlashStatus deinit() = 0;
    virtual FlashStatus erase() = 0;
    virtual FlashStatus write(uint32_t targetAddress, const uint8_t* data, std::size_t length) = 0;
    virtual FlashStatus read(uint32_t sourceAddress, uint8_t* destination, std::size_t length) const = 0;
};
