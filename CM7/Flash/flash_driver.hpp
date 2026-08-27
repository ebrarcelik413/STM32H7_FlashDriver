#pragma once

#include "stm32h7xx_hal.h"
#include "flash_types.hpp"
#include "iflash_driver.hpp"

#include <cstdint>
#include <cstddef>

class FlashDriver : public IFlashDriver
{
public:
    static constexpr std::size_t FlashWordSize = 32U;

    FlashDriver(uint32_t bank,
                uint32_t sector,
                uint32_t startAddress,
                std::size_t size);

    ~FlashDriver() override = default;

    FlashStatus init() override;
    FlashStatus deinit() override;
    FlashStatus erase() override;

    FlashStatus write(uint32_t targetAddress,
                      const uint8_t* data,
                      std::size_t length) override;

    FlashStatus read(uint32_t sourceAddress,
                     uint8_t* destination,
                     std::size_t length) const override;

    FlashStatus writeIT(uint32_t targetAddress,
                        const uint8_t* data,
                        std::size_t length);

    bool isBusy() const
    {
        return isBusy_;
    }

    uint32_t getLastError() const
    {
        return lastError_;
    }

    uint32_t getStartAddress() const
    {
        return startAddress_;
    }

    uint32_t getEndAddress() const
    {
        return endAddress_;
    }

    std::size_t getSize() const
    {
        return size_;
    }

    void onOperationComplete();
    void process();
    void onError(uint32_t errorCode);

private:
    enum class Operation
    {
        None,
        Erase,
        Write
    };

    uint32_t bank_;
    uint32_t sector_;
    uint32_t startAddress_;
    std::size_t size_;
    uint32_t endAddress_;

    bool isInitialized_{false};
    volatile bool isBusy_{false};
    volatile uint32_t lastError_{0U};

    Operation currentOperation_{Operation::None};
    volatile bool flashWordCompleted_{false};

    const uint8_t* txData_{nullptr};
    std::size_t txRemaining_{0U};
    uint32_t currentWriteAddr_{0U};

    alignas(32) uint8_t alignBuffer_[FlashWordSize];


    FlashStatus programWord(uint32_t address,
                            const uint8_t* data);

    FlashStatus verify(uint32_t address,
                       const uint8_t* expected,
                       std::size_t length) const;

    bool isErased(uint32_t address,
                  std::size_t length) const;

    bool isFlashWordAligned(uint32_t address) const;

    bool isRangeValid(uint32_t address,
                      std::size_t length) const;
};

extern FlashDriver g_flashDriver;
