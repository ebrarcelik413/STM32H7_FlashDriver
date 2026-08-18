#include "app_main.h"
#include "main.h"
#include "flash_driver.hpp"
#include <cstring>

constexpr uint32_t STORAGE_BANK        = FLASH_BANK_2;
constexpr uint32_t STORAGE_SECTOR      = FLASH_SECTOR_6;
constexpr uint32_t STORAGE_START_ADDR  = 0x081C0000UL;
constexpr std::size_t STORAGE_SEC_SIZE = 128U * 1024U; // 128 KB

static FlashDriver g_flashDriver(
    STORAGE_BANK,
    STORAGE_SECTOR,
    STORAGE_START_ADDR,
    STORAGE_SEC_SIZE
);

// Test verisi olarak kullanabileceğimiz örnek bir yapı
struct SensorCalibration
{
    float gyroOffset[3];
    float accelOffset[3];
    uint32_t sampleCount;
};

// Debug takibi için durum değişkenleri
volatile FlashStatus g_testStatus = FlashStatus::InvalidArgument;
volatile bool g_testPassed = false;

void app_init(void)
{
    // A. Sürücüyü Başlat
    g_testStatus = g_flashDriver.initialize();
    if (g_testStatus != FlashStatus::Ok)
    {
        return;
    }

    // B. Sektörü Sil (Eski artık verilerden temizlemek için)
    g_testStatus = g_flashDriver.erase();
    if (g_testStatus != FlashStatus::Ok)
    {
        return;
    }

    // C. Örnek Veri Hazırla
    SensorCalibration writeCalib{};
    writeCalib.gyroOffset[0] = 0.12f;
    writeCalib.gyroOffset[1] = -0.05f;
    writeCalib.gyroOffset[2] = 0.98f;
    writeCalib.accelOffset[0] = 0.01f;
    writeCalib.accelOffset[1] = 0.02f;
    writeCalib.accelOffset[2] = 9.81f;
    writeCalib.sampleCount = 500U;

    // D. Veriyi Sektörün Başlangıcına Yaz
    uint32_t targetAddress = STORAGE_START_ADDR;
    g_testStatus = g_flashDriver.write(
        targetAddress,
        reinterpret_cast<const uint8_t*>(&writeCalib),
        sizeof(writeCalib)
    );

    if (g_testStatus != FlashStatus::Ok)
    {
        return;
    }

    // E. Yazılan Veriyi Geri Oku
    SensorCalibration readCalib{};
    g_testStatus = g_flashDriver.read(
        targetAddress,
        reinterpret_cast<uint8_t*>(&readCalib),
        sizeof(readCalib)
    );

    if (g_testStatus != FlashStatus::Ok)
    {
        return;
    }


    if (std::memcmp(&writeCalib, &readCalib, sizeof(SensorCalibration)) == 0)
    {
        g_testPassed = true;
    }
    else
    {
        g_testStatus = FlashStatus::VerifyError;
    }

    __NOP();
}

void app_loop(void)
{
}
