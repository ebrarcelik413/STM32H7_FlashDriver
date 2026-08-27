#include "flash_driver.hpp"
#include <cstring>

/*
 * FlashDriver class'ının constructor'ına constructor member parametreleirni
 * class'ın private member variable'larına atıyoruz.
 * bank_, sector_ ... driver obeject'inin kendi state'ini tutuyor.
 */
FlashDriver::FlashDriver(uint32_t bank,
                         uint32_t sector,
                         uint32_t startAddress,
                         std::size_t size)
    : bank_(bank),
      sector_(sector),
      startAddress_(startAddress),
      size_(size),
      endAddress_(startAddress + static_cast<uint32_t>(size) - 1U)
{
}

/*
 * driver başlatılırken storage başlangıç adresinin FlashWord sınırına hizalı...
 */

FlashStatus FlashDriver::init()
{
    if (!isFlashWordAligned(startAddress_) || (size_ == 0U))
    {
        return FlashStatus::AlignmentError;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return FlashStatus::UnlockError;
    }

    isInitialized_ = true;  //driver kullanılabilir durumda bilgisini kendi state'imde tutuyorum.
    return FlashStatus::Ok;
}

FlashStatus FlashDriver::deinit()
{
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        return FlashStatus::LockError;
    }

    isInitialized_ = false;
    return FlashStatus::Ok;
}

FlashStatus FlashDriver::erase()
{
    if (!isInitialized_)
    {
        return FlashStatus::NotInitialized;
    }

    if (isBusy_)
    {
        return FlashStatus::Busy;
    }
/*
 * Yeni erase operasyonundan önce Bank2’de
 * önceki operasyondan kalmış EOP ve error flaglerini temizliyorum.
 * Macro aslında Flash’ın CCR2 register’ındaki clear bitlerine yazıyor.
 */
    __HAL_FLASH_CLEAR_FLAG_BANK2(
        FLASH_FLAG_ALL_ERRORS_BANK2 |
        FLASH_FLAG_EOP_BANK2);
/*
 * HAL erase fonksiyonuna erase parametrelerini tek tek vermek yerine
 * HAL’in tanımladığı configuration struct’ını hazırlıyorum.
 */
    FLASH_EraseInitTypeDef eraseConfig{};

    eraseConfig.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseConfig.Banks        = bank_;
    eraseConfig.Sector       = sector_;
    eraseConfig.NbSectors    = 1U;
    eraseConfig.VoltageRange = FLASH_VOLTAGE_RANGE_4;

    currentOperation_ = Operation::Erase;
    flashWordCompleted_  = false;
    lastError_ = 0U;
    isBusy_ = true;

    if (HAL_FLASHEx_Erase_IT(&eraseConfig) != HAL_OK)
    {
        currentOperation_ = Operation::None;
        isBusy_ = false;
        lastError_ = HAL_FLASH_GetError();

        return FlashStatus::EraseError;
    }

    return FlashStatus::Ok;
}

FlashStatus FlashDriver::write(uint32_t targetAddress,
                               const uint8_t* data,
                               std::size_t length)
{
	/*
	 * write=writeIT üzerine kurulmuş senkron/blocking wrapper
	 */
    FlashStatus status = writeIT(targetAddress, data, length);

    if (status != FlashStatus::Ok)
    {
        return status;
    }

    while (isBusy_)
    {
        __WFI();
        process();
    }

    return (lastError_ == 0U)
        ? FlashStatus::Ok
        : FlashStatus::ProgramError;
}

FlashStatus FlashDriver::writeIT(uint32_t targetAddress,
                                 const uint8_t* data,
                                 std::size_t length)
{
    if (!isInitialized_)
    {
        return FlashStatus::NotInitialized;
    }

    if (isBusy_)
    {
        return FlashStatus::Busy;
    }

    if (data == nullptr)
    {
        return FlashStatus::InvalidArgument;
    }

    if (length == 0U)
    {
        return FlashStatus::InvalidLength;
    }

    if (!isFlashWordAligned(targetAddress))
    {
        return FlashStatus::AlignmentError;
    }

    if (!isRangeValid(targetAddress, length))
    {
        return FlashStatus::NoSpace;
    }

    /*
     * non-blocking işlem tek fonks çagrısında bitmeyecegi için
     * yazmanın state'ini class memberlarında saklıyorum.
     */
    txData_ = data;
    txRemaining_ = length;
    currentWriteAddr_ = targetAddress;

    currentOperation_ = Operation::Write;
    flashWordCompleted_  = false;
    lastError_ = 0U;
    isBusy_ = true;

    std::memset(alignBuffer_, 0xFF, FlashWordSize);

    const std::size_t chunk =
        (txRemaining_ >= FlashWordSize)
        ? FlashWordSize
        : txRemaining_;

    std::memcpy(alignBuffer_, txData_, chunk);

    FlashStatus status =
        programWord(currentWriteAddr_, alignBuffer_);

    if (status != FlashStatus::Ok)
    {
        currentOperation_ = Operation::None;
        isBusy_ = false;
        return status;
    }

    txData_ += chunk;
    txRemaining_ -= chunk;
    currentWriteAddr_ += FlashWordSize;

    return FlashStatus::Ok;
}

FlashStatus FlashDriver::programWord(uint32_t address,
                                     const uint8_t* data)
{
    if (!isErased(address, FlashWordSize))
    {
        return FlashStatus::NotErased;
    }

    __HAL_FLASH_CLEAR_FLAG_BANK2(
        FLASH_FLAG_ALL_ERRORS_BANK2 |
        FLASH_FLAG_EOP_BANK2);

    if (HAL_FLASH_Program_IT(
            FLASH_TYPEPROGRAM_FLASHWORD,
            address,
            reinterpret_cast<uint32_t>(data)) != HAL_OK)
    {
        lastError_ = HAL_FLASH_GetError();
        return FlashStatus::ProgramError;
    }

    return FlashStatus::Ok;
}

void FlashDriver::onOperationComplete()
{
    if (currentOperation_ == Operation::Erase)
    {
        currentOperation_ = Operation::None;
        isBusy_ = false;
        return;
    }

    if (currentOperation_ == Operation::Write)
    {
    	flashWordCompleted_  = true;
    }
}

void FlashDriver::process()
{
    if (!isBusy_ || !flashWordCompleted_)
    {
        return;
    }

    flashWordCompleted_ = false;

    if (currentOperation_ != Operation::Write)
    {
        return;
    }

    // Son FlashWord da tamamlandı.
    if (txRemaining_ == 0U)
    {
        currentOperation_ = Operation::None;
        isBusy_ = false;
        return;
    }

    std::memset(alignBuffer_, 0xFF, FlashWordSize);

    const std::size_t chunk =
        (txRemaining_ >= FlashWordSize)
        ? FlashWordSize
        : txRemaining_;

    std::memcpy(alignBuffer_, txData_, chunk);

    FlashStatus status =
        programWord(currentWriteAddr_, alignBuffer_);

    if (status != FlashStatus::Ok)
    {
        currentOperation_ = Operation::None;
        isBusy_ = false;
        return;
    }

    txData_ += chunk;
    txRemaining_ -= chunk;
    currentWriteAddr_ += FlashWordSize;
}

FlashStatus FlashDriver::read(uint32_t sourceAddress,
                              uint8_t* destination,
                              std::size_t length) const
{
    if (!isInitialized_)
    {
        return FlashStatus::NotInitialized;
    }

    if (destination == nullptr)
    {
        return FlashStatus::InvalidArgument;
    }

    if (length == 0U)
    {
        return FlashStatus::InvalidLength;
    }

    if (!isRangeValid(sourceAddress, length))
    {
        return FlashStatus::InvalidAddress;
    }

    std::memcpy(
        destination,
        reinterpret_cast<const void*>(sourceAddress),
        length);

    return FlashStatus::Ok;
}

FlashStatus FlashDriver::verify(uint32_t address,
                                const uint8_t* expected,
                                std::size_t length) const
{
    const auto* actual =
        reinterpret_cast<const uint8_t*>(address);

    for (std::size_t i = 0U; i < length; ++i)
    {
        if (actual[i] != expected[i])
        {
            return FlashStatus::VerifyError;
        }
    }

    return FlashStatus::Ok;
}

bool FlashDriver::isErased(uint32_t address,
                           std::size_t length) const
{
    if (!isRangeValid(address, length))
    {
        return false;
    }

    const auto* source =
        reinterpret_cast<const uint8_t*>(address);

    for (std::size_t i = 0U; i < length; ++i)
    {
        if (source[i] != 0xFFU)
        {
            return false;
        }
    }

    return true;
}

bool FlashDriver::isFlashWordAligned(uint32_t address) const
{
    return (address % FlashWordSize) == 0U;
}

bool FlashDriver::isRangeValid(uint32_t address,
                               std::size_t length) const
{
    if ((length == 0U) || (address < startAddress_))
    {
        return false;
    }

    const uint64_t lastAddress =
        static_cast<uint64_t>(address) +
        static_cast<uint64_t>(length) -
        1ULL;

    return lastAddress <= endAddress_;
}

//burada çagrılıyor HAL_FLASH_OperationErrorCallback(...)
void FlashDriver::onError(uint32_t errorCode)
{
    lastError_ = errorCode;

    currentOperation_ = Operation::None;
    flashWordCompleted_  = false;

    isBusy_ = false;
    txRemaining_ = 0U;
}

/*
 * STM32 HAL C ile yazildigi icin callback'leri C linkage ile tanimliyoruz.
 * Boylece C++ name mangling uygulanmaz ve HAL bekledigi callback symbol'unu bulabilir.
 */

extern "C" void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
    (void)ReturnValue;

    g_flashDriver.onOperationComplete();
}

extern "C" void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
    (void)ReturnValue;

    g_flashDriver.onError(HAL_FLASH_GetError());
}
