#pragma once

#include "flash_types.hpp"
#include <cstddef>
#include <cstdint>

class FlashDriver final
{
public:
    static constexpr std::size_t FlashWordSize = 32U;

    FlashDriver(uint32_t bank, uint32_t sector, uint32_t startAddress, std::size_t size);
    FlashDriver() = delete;
    ~FlashDriver() = default;

    FlashStatus initialize();
    FlashStatus erase();

    FlashStatus write(uint32_t targetAddress, const uint8_t* data, std::size_t length);
    FlashStatus read(uint32_t sourceAddress, uint8_t* destination, std::size_t length) const;

    bool isErased(uint32_t address, std::size_t length) const;

    uint32_t getStartAddress() const { return startAddress_; }
    uint32_t getEndAddress() const { return endAddress_; }
    std::size_t getSize() const { return size_; }

private:
    uint32_t bank_;
    uint32_t sector_;
    uint32_t startAddress_;
    std::size_t size_;
    uint32_t endAddress_;
    bool isInitialized_{false};

    FlashStatus programWord(uint32_t address, const uint8_t* data);
    FlashStatus verify(uint32_t address, const uint8_t* expected, std::size_t length) const;
    bool isFlashWordAligned(uint32_t address) const;
    bool isRangeValid(uint32_t address, std::size_t length) const;
    static FlashStatus mapHalFailure(uint32_t flashError, int halStatus, bool eraseOperation);
};
