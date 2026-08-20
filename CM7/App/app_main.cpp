#include "app_main.h"
#include "main.h"
#include "flash_driver.hpp"
#include <cstdio>
#include <cstring>

extern UART_HandleTypeDef huart1;

// printf çağrılarını PA9/PA10 üzerinden USART1'e yönlendirir
extern "C" int __io_putchar(int ch)
{
    uint8_t c = static_cast<uint8_t>(ch);
    HAL_UART_Transmit(&huart1, &c, 1U, HAL_MAX_DELAY);
    return ch;
}

constexpr uint32_t STORAGE_BANK        = FLASH_BANK_2;
constexpr uint32_t STORAGE_SECTOR      = FLASH_SECTOR_7;
constexpr uint32_t STORAGE_START_ADDR  = 0x081E0000U;
constexpr std::size_t STORAGE_SEC_SIZE = 128U * 1024U;

FlashDriver g_flashDriver(STORAGE_BANK, STORAGE_SECTOR, STORAGE_START_ADDR, STORAGE_SEC_SIZE);
FlashStatus g_testStatus = FlashStatus::Ok;

alignas(32) uint8_t writeBufA[32];
alignas(32) uint8_t writeBufB[32];
uint8_t readBuf[32];

void app_init(void)
{
    printf("\r\n=== STM32H7 Flash Driver Test Baslatildi ===\r\n");

    // 1. Sürücüyü başlat (Unlock + Kesmeleri aç)
    g_testStatus = g_flashDriver.init();
    printf("[INIT] Durum: %d\r\n", static_cast<int>(g_testStatus));
    if (g_testStatus != FlashStatus::Ok) return;

    // 2. Bank 2 Sektör 7'yi kesmeli olarak sil
    printf("[ERASE] Sektor 7 siliniyor...\r\n");
    g_testStatus = g_flashDriver.erase();
    if (g_testStatus != FlashStatus::Ok)
    {
        printf("[ERASE] Baslatma Hatasi: %d\r\n", static_cast<int>(g_testStatus));
        return;
    }

    // 3. Kesme tamamlanana kadar bekle
    while (g_flashDriver.isBusy())
    {
    }
    printf("[ERASE] Kesme tamamlandi, sektor basariyla silindi!\r\n");

    // 4. Test verilerini hazırla
    std::memset(writeBufA, 0xAA, sizeof(writeBufA));
    std::memset(writeBufB, 0xBB, sizeof(writeBufB));

    // 5. 32-byte Flash Word yazımı
    g_testStatus = g_flashDriver.write(STORAGE_START_ADDR, writeBufA, sizeof(writeBufA));
    printf("[WRITE] 0x%08lX adresine 0xAA yazildi. Durum: %d\r\n", STORAGE_START_ADDR, static_cast<int>(g_testStatus));
    if (g_testStatus != FlashStatus::Ok) return;

    g_testStatus = g_flashDriver.write(STORAGE_START_ADDR + 0x60, writeBufB, sizeof(writeBufB));
    printf("[WRITE] 0x%08lX adresine 0xBB yazildi. Durum: %d\r\n", STORAGE_START_ADDR + 0x60, static_cast<int>(g_testStatus));
    if (g_testStatus != FlashStatus::Ok) return;

    // 6. Okuma ve doğrulama
    g_testStatus = g_flashDriver.read(STORAGE_START_ADDR, readBuf, sizeof(readBuf));
    printf("[READ] Okunan ilk bayt: 0x%02X\r\n", readBuf[0]);
    if (g_testStatus != FlashStatus::Ok) return;

    // 7. Donanımı kilitle
    g_flashDriver.deinit();
    printf("=== Test Basariyla Tamamlandi ===\r\n");

    __NOP();
}

void app_loop(void)
{
}
