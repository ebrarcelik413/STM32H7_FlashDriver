#include "app_main.h"
#include "main.h"
#include "flash_driver.hpp"
#include <cstring>

constexpr uint32_t STORAGE_BANK        = FLASH_BANK_2;
constexpr uint32_t STORAGE_SECTOR      = FLASH_SECTOR_6;
constexpr uint32_t STORAGE_START_ADDR  = 0x081C0000U;
constexpr std::size_t STORAGE_SEC_SIZE = 128U * 1024U;

FlashDriver g_flashDriver(STORAGE_BANK, STORAGE_SECTOR, STORAGE_START_ADDR, STORAGE_SEC_SIZE);
FlashStatus g_testStatus = FlashStatus::Ok;

alignas(32) uint8_t writeBufA[32];
alignas(32) uint8_t writeBufB[32];
uint8_t readBuf[32];

void app_init(void)
{
    // 1. Sürücüyü başlat (Unlock + Kesmeleri aç)
    g_testStatus = g_flashDriver.init();
    if (g_testStatus != FlashStatus::Ok) return;

    // 2. Bank 2 Sektör 7'yi kesmeli olarak sil
    g_testStatus = g_flashDriver.erase();
    if (g_testStatus != FlashStatus::Ok) return;

    // 3. Kesme tamamlanana kadar bekle
    while (g_flashDriver.isBusy())
    {
    }

    // 4. Test verilerini hazırla
    std::memset(writeBufA, 0xAA, sizeof(writeBufA));
    std::memset(writeBufB, 0xBB, sizeof(writeBufB));

    // 5. 32-byte Flash Word yazımı
    g_testStatus = g_flashDriver.write(STORAGE_START_ADDR, writeBufA, sizeof(writeBufA));
    if (g_testStatus != FlashStatus::Ok) return;

    g_testStatus = g_flashDriver.write(STORAGE_START_ADDR + 0x60, writeBufB, sizeof(writeBufB));
    if (g_testStatus != FlashStatus::Ok) return;

    // 6. Okuma ve doğrulama
    g_testStatus = g_flashDriver.read(STORAGE_START_ADDR, readBuf, sizeof(readBuf));
    if (g_testStatus != FlashStatus::Ok) return;

    // 7. Donanımı kilitle
    g_flashDriver.deinit();

    __NOP();
}

void app_loop(void)
{
}
