#include "flash_driver.hpp"
#include "stm32h7xx_hal.h"
#include <cstring>


namespace
{
	constexpr uint32_t MaxRetrtCount=10U;
	constexpr uint32_t RetryDelayMs_1U;
}

FlashDriver::FlashDriver(uint32_t bank, uint32_t sector, uint32_t startAddress, std::size_t size)
    : bank_(bank),
      sector_(sector),
      startAddress_(startAddress),
      size_(size),
      endAddress_(startAddress + static_cast<uint32_t>(size) - 1U),
      isInitialized_(false)
{
}

FlashStatus FlashDriver::initialize()
{
    if (!isFlashWordAligned(startAddress_) || (size_ == 0U))
    {
        return FlashStatus::InvalidArgument;
    }

    isInitialized_ = true;
    return FlashStatus::Ok;
}

FlashStatus FlashDriver::erase()
{
    if (!isInitialized_)
    {
        return FlashStatus::NotInitialized;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return FlashStatus::UnlockError;
    }

    // İlgili banka ait önceki hata bayraklarını temizle
    if (bank_ == FLASH_BANK_1)
    {
        __HAL_FLASH_CLEAR_FLAG_BANK1(
            FLASH_FLAG_ALL_ERRORS_BANK1 |
            FLASH_FLAG_EOP_BANK1 |
            FLASH_FLAG_WRPERR_BANK1 |
            FLASH_FLAG_PGSERR_BANK1 |
            FLASH_FLAG_STRBERR_BANK1 |
            FLASH_FLAG_INCERR_BANK1);
    }
    else
    {
        __HAL_FLASH_CLEAR_FLAG_BANK2(
            FLASH_FLAG_ALL_ERRORS_BANK2 |
            FLASH_FLAG_EOP_BANK2 |
            FLASH_FLAG_WRPERR_BANK2 |
            FLASH_FLAG_PGSERR_BANK2 |
            FLASH_FLAG_STRBERR_BANK2 |
            FLASH_FLAG_INCERR_BANK2);
    }

    FLASH_EraseInitTypeDef eraseConfig{};
    eraseConfig.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseConfig.Banks        = bank_;
    eraseConfig.Sector       = sector_;
    eraseConfig.NbSectors    = 1U;
    eraseConfig.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sectorError = 0xFFFFFFFFU;
    HAL_StatusTypeDef result = HAL_BUSY;

    for (uint32_t attempt=0U ; attemp=0; attempt<MaxRetryCount; ++attempt)
    {
    	result = HAL_FLASHEx_Erase(&eraseConfig, &sectorError);
    	if (result != HAL_BUSY)
    	{
    		break;
    	}
    	HAL_Delay(RetryDelayMs);
    }

    const uint32_t flashError = HAL_FLASH_GetError();
    const HAL_StatusTypeDef lockResult = HAL_FLASH_Lock();

    if (result != HAL_OK)
    {
        return mapHalFailure(flashError, static_cast<int>(result), true);
    }

    if (lockResult != HAL_OK)
    {
        return FlashStatus::LockError;
    }

    return FlashStatus::Ok;
}

FlashStatus FlashDriver::write(uint32_t targetAddress, const uint8_t* data, std::size_t length)
{
    if (!isInitialized_)
    {
        return FlashStatus::NotInitialized;
    }

    if (data == nullptr)
    {
        return FlashStatus::InvalidArgument;
    }

    if (length == 0U)
    {
        return FlashStatus::InvalidLength;
    }

    // Sektör sınırları dışına taşıyor mu?
    if (!isRangeValid(targetAddress, length))
    {
        return FlashStatus::NoSpace;
    }

    if (!isFlashWordAligned(targetAddress))
    {
        return FlashStatus::AlignmentError;
    }

    std::size_t offset = 0U;
    alignas(32) uint8_t wordBuffer[FlashWordSize];

    while (offset < length)
    {
        std::memset(wordBuffer, 0xFF, sizeof(wordBuffer));

        const std::size_t remaining = length - offset; //toplam boyuttan o ana kadar yazılan kısım cıkar
        const std::size_t chunk = (remaining < FlashWordSize) ? remaining : FlashWordSize; //chunk 10 ise 32 yapıyor

        std::memcpy(wordBuffer, &data[offset], chunk);   //chunk kadar bayt alınıp ramden wordbuffer iine kopylanır zaten geri kalan ff

        const FlashStatus status = programWord(targetAddress + static_cast<uint32_t>(offset), wordBuffer);
        if (status != FlashStatus::Ok)
        {
            return status;
        }

        offset += FlashWordSize;
    }

    return FlashStatus::Ok;
}

FlashStatus FlashDriver::read(uint32_t sourceAddress, uint8_t* destination, std::size_t length) const
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

    std::memcpy(destination, reinterpret_cast<const void*>(sourceAddress), length);
    return FlashStatus::Ok;
}

FlashStatus FlashDriver::programWord(uint32_t address, const uint8_t* data)
{
    // Alan daha önce silinmiş mi (0xFF mi)?
    if (!isErased(address, FlashWordSize))
    {
        return FlashStatus::ProgramError;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return FlashStatus::UnlockError;
    }

    HAL_StatusTypeDef result = HAL_BUSY;

    // Retry / Polling Döngüsü
    for (uint32_t attempt = 0U; attempt < MaxRetryCount; ++attempt)
    {
        result = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD,
            address,
            reinterpret_cast<uint32_t>(data));

        if (result != HAL_BUSY)
        {
            break;
        }
        HAL_Delay(RetryDelayMs);
    }

    const uint32_t flashError = HAL_FLASH_GetError();
    const HAL_StatusTypeDef lockResult = HAL_FLASH_Lock();

    if (result != HAL_OK)
    {
        return mapHalFailure(flashError, static_cast<int>(result), false);
    }

    if (lockResult != HAL_OK)
    {
        return FlashStatus::LockError;
    }

    return verify(address, data, FlashWordSize);
}

FlashStatus FlashDriver::verify(uint32_t address, const uint8_t* expected, std::size_t length) const
{
    const auto* actual = reinterpret_cast<const uint8_t*>(address);
    for (std::size_t i = 0U; i < length; ++i)
    {
        if (actual[i] != expected[i])
        {
            return FlashStatus::VerifyError;
        }
    }
    return FlashStatus::Ok;
}

bool FlashDriver::isErased(uint32_t address, std::size_t length) const
{
    if (!isRangeValid(address, length))
    {
        return false;
    }

    const auto* source = reinterpret_cast<const uint8_t*>(address);
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

bool FlashDriver::isRangeValid(uint32_t address, std::size_t length) const
{
    if (length == 0U || address < startAddress_)
    {
        return false;
    }

    const uint64_t lastAddress = static_cast<uint64_t>(address) + static_cast<uint64_t>(length) - 1ULL;
    return lastAddress <= endAddress_;     // lastaddress=sonbaytınadresi eger sınırı aşarsa false no space döner.
}

FlashStatus FlashDriver::mapHalFailure(uint32_t flashError, int halStatus, bool eraseOperation)
{
    if (halStatus == static_cast<int>(HAL_TIMEOUT))
    {
        return FlashStatus::Timeout;
    }

    if (halStatus == static_cast<int>(HAL_BUSY))
    {
        return FlashStatus::Busy;
    }

    return eraseOperation ? FlashStatus::EraseError : FlashStatus::ProgramError;
}
