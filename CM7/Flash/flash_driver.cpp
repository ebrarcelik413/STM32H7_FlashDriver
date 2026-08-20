#include "flash_driver.hpp"
#include "stm32h7xx_hal.h"
#include <cstring>


extern FlashDriver g_flashDriver;

/* soldaki bank_ sınıftaki private üye deeğişkeni
 * parantez içindeki bank kullanıcın fonk parametresi
 * dışarıdan gelen bank degerini sınıfın içindeki bank değişkenine atıyor.
 */

FlashDriver::FlashDriver(uint32_t bank, uint32_t sector, uint32_t startAddress, std::size_t size)
    : bank_(bank),
      sector_(sector),
      startAddress_(startAddress),
      size_(size),
      endAddress_(startAddress + static_cast<uint32_t>(size) - 1U),
      isInitialized_(false),
	  isBusy_(false),
	  lastError_(0U)
{
}

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

    // Bank 2 Interrupts Enable (EOP & OPERR)
    if (bank_ == FLASH_BANK_2)
    {
        __HAL_FLASH_ENABLE_IT_BANK2(FLASH_IT_EOP_BANK2 | FLASH_IT_OPERR_BANK2);
    }
    else
    {
        __HAL_FLASH_ENABLE_IT_BANK1(FLASH_IT_EOP_BANK1 | FLASH_IT_OPERR_BANK1);
    }

    // NVIC Flash Interrupt Aç (CM7 çekirdeğinde kesmeyi dinlemek için zorunlu)
    HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FLASH_IRQn);

    isInitialized_ = true;
    return FlashStatus::Ok;
}

FlashStatus FlashDriver::deinit()
{
    if (bank_ == FLASH_BANK_2)
    {
        __HAL_FLASH_DISABLE_IT_BANK2(FLASH_IT_EOP_BANK2 | FLASH_IT_OPERR_BANK2);
    }
    else
    {
        __HAL_FLASH_DISABLE_IT_BANK1(FLASH_IT_EOP_BANK1 | FLASH_IT_OPERR_BANK1);
    }

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

    // Yalnızca hedef Bank 2 hata ve durum bayraklarını temizle
    __HAL_FLASH_CLEAR_FLAG_BANK2(
        FLASH_FLAG_ALL_ERRORS_BANK2 |
        FLASH_FLAG_EOP_BANK2 |
        FLASH_FLAG_WRPERR_BANK2 |
        FLASH_FLAG_PGSERR_BANK2 |
        FLASH_FLAG_STRBERR_BANK2 |
        FLASH_FLAG_INCERR_BANK2);

    FLASH_EraseInitTypeDef eraseConfig{};
    eraseConfig.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseConfig.Banks        = bank_;
    eraseConfig.Sector       = sector_;
    eraseConfig.NbSectors    = 1U;
    eraseConfig.VoltageRange = FLASH_VOLTAGE_RANGE_4;

    isBusy_ = true;

    if (HAL_FLASHEx_Erase_IT(&eraseConfig) != HAL_OK)
    {
        isBusy_ = false;
        return FlashStatus::EraseError;
    }

    return FlashStatus::Ok;
}

FlashStatus FlashDriver::write(uint32_t targetAddress, const uint8_t* data, std::size_t length)
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

        const std::size_t remaining = length - offset;
        const std::size_t chunk = (remaining < FlashWordSize) ? remaining : FlashWordSize;

        std::memcpy(wordBuffer, &data[offset], chunk);

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
    if (!isErased(address, FlashWordSize))
    {
        return FlashStatus::NotErased;
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, reinterpret_cast<uint32_t>(data)) != HAL_OK)
    {
        return FlashStatus::ProgramError;
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

// Flash işlemi (silme vb.) donanım seviyesinde başarıyla bittiğinde çağrılır;
// meşguliyet bayrağını indirerek bekleyen döngülerin kilidini açar.
void FlashDriver::onOperationComplete()
{
    isBusy_ = false;
}

// Flash işlemi sırasında donanımsal bir hata oluştuğunda çağrılır;
// hata kodunu kaydeder ve sistemin sonsuz döngüde kilitli kalmasını önler.
void FlashDriver::onError(uint32_t errorCode)
{
    lastError_ = errorCode;
    isBusy_ = false;
}

// ST HAL kütüphanesi saf C ile yazıldığı için C++ Name Mangling'i (isim bozulmasını)
// engellemek adına extern "C" kullanılır. Donanım işlemi tamamlandığında HAL bu
// global C fonksiyonunu tetikler, fonksiyon da sinyali C++ nesnemize iletir.
extern "C" void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
    (void)ReturnValue; // Kullanılmayan parametre uyarısını engeller
    g_flashDriver.onOperationComplete();
}

// Flash donanımında yazma koruması, voltaj dalgalanması veya sıra hatası
// gibi bir kesme hatası oluştuğunda ST HAL tarafından tetiklenen köprü fonksiyondur.
extern "C" void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
    g_flashDriver.onError(ReturnValue);
}
