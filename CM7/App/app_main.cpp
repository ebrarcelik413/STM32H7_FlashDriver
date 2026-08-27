#include "app_main.h"

#include "main.h"

#include "flash_driver.hpp"

#include "../Uart/uart_driver.hpp"

#include <cstdio>
#include <cstring>
#include <algorithm>


extern UART_HandleTypeDef huart1;


constexpr uint32_t STORAGE_BANK        = FLASH_BANK_2;
constexpr uint32_t STORAGE_SECTOR      = FLASH_SECTOR_7;
constexpr uint32_t STORAGE_START_ADDR  = 0x081E0000U;
constexpr std::size_t STORAGE_SEC_SIZE = 128U * 1024U;

constexpr std::size_t IMU_DATA_SIZE = 70U;
constexpr std::size_t GPS_DATA_SIZE = 30U;

constexpr std::size_t DMA_TEST_SIZE = 10000U;


FlashDriver g_flashDriver(
    STORAGE_BANK,
    STORAGE_SECTOR,
    STORAGE_START_ADDR,
    STORAGE_SEC_SIZE
);

UartDriver g_uartDriver(&huart1);


// Siradaki bos adres
static uint32_t s_writeOffset = 0U;


// Non-blocking flash write devam ederken
// verinin gecerliligini korumasi icin kalici bufferlar
static uint8_t s_imuData[IMU_DATA_SIZE];
static uint8_t s_gpsData[GPS_DATA_SIZE];


// DMA TX non-blocking oldugu icin buffer kalici olmali.
static uint8_t s_dmaTestData[DMA_TEST_SIZE];


// DMA devam ederken kac TX isteginin reddedildigini tutar.
static uint32_t s_dmaBusyCount = 0U;

// Busy disinda DMA baslatma hatasi oldu mu?
static bool s_dmaErrorOccurred = false;


// DWT timestamp'lari
static uint32_t s_eraseStartCycle = 0U;
static uint32_t s_writeStartCycle = 0U;


// Flash okuma bufferi
static uint8_t readBuf[128];


enum class ActiveWrite
{
    None,
    Imu,
    Gps
};

static ActiveWrite s_activeWrite = ActiveWrite::None;
static uint32_t s_activeWriteSize = 0U;


// Async erase durumunu takip eder
static bool s_eraseInProgress = false;


/**
 * @brief DWT cycle counter baslatir.
 */
static void initDwtCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0U;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


static inline uint32_t alignTo32(uint32_t size)
{
    return (size + 31U) & ~31U;
}


/**
 * Driver'da string'e ozel API yok.
 *
 * Bu helper sadece app_main test/CLI katmaninda
 * okunabilir mesaj gonderebilmek icin var.
 */
static void sendText(const char* text)
{
    if (text == nullptr)
    {
        return;
    }

    g_uartDriver.send(
        reinterpret_cast<const uint8_t*>(text),
        std::strlen(text)
    );
}


static void printMenu()
{
    static const char menuText[] =
        "\r\n==============================================\r\n"
        " [e] -> Sektoru Sil ve Isaretciyi Sifirla\r\n"
        " [1] -> 70 Bayt IMU Sensor Verisi Yaz\r\n"
        " [2] -> 30 Bayt GPS Sensor Verisi Yaz\r\n"
        " [t] -> Flash Benchmark (32B Sure Olc)\r\n"
        " [r] -> Ilk 128 Bayti Oku\r\n"
        " [d] -> UART DMA TX Test\r\n"
        " [h] -> Menuyu Goster\r\n"
        "==============================================\r\n> ";

    g_uartDriver.send(
        reinterpret_cast<const uint8_t*>(menuText),
        sizeof(menuText) - 1U
    );
}


void app_init(void)
{
    initDwtCycleCounter();

    g_uartDriver.init();
    g_flashDriver.init();

    s_writeOffset = 0U;

    std::memset(
        s_imuData,
        0xAA,
        sizeof(s_imuData)
    );

    std::memset(
        s_gpsData,
        0xBB,
        sizeof(s_gpsData)
    );

    /*
     * 1000 byte DMA TX testi.
     *
     * 0x00 terminalde gorunmez.
     * Ama UART hattindan gercekten 1000 byte gonderilir.
     */
    std::memset(
        s_dmaTestData,
        0x00,
        sizeof(s_dmaTestData)
    );

    sendText(
        "\r\n*** STM32H7 Sensor Flash CLI Hazir ***\r\n"
    );

    printMenu();
}


void app_loop(void)
{
    /*
     * Flash write state machine'ini ilerletir.
     */
    g_flashDriver.process();


    /*
     * DMA TX tamamlandiysa ve DMA devam ederken
     * reddedilen TX istekleri varsa raporla.
     */
    if (!g_uartDriver.isTxBusy())
    {
        if (s_dmaBusyCount > 0U)
        {
            char report[100];

            std::snprintf(
                report,
                sizeof(report),
                "\r\n[BUSY] DMA TX devam ederken %lu TX istegi reddedildi.\r\n> ",
                static_cast<unsigned long>(s_dmaBusyCount)
            );

            s_dmaBusyCount = 0U;

            sendText(report);
        }

        if (s_dmaErrorOccurred)
        {
            s_dmaErrorOccurred = false;

            sendText(
                "\r\n[ERROR] DMA TX baslatilamadi.\r\n> "
            );
        }
    }


    /*
     * Async erase tamamlandi mi?
     */
    if (s_eraseInProgress &&
        !g_flashDriver.isBusy())
    {
        uint32_t eraseEndCycle =
            DWT->CYCCNT;

        uint32_t eraseCycles =
            eraseEndCycle -
            s_eraseStartCycle;

        uint32_t eraseUs =
            eraseCycles /
            (SystemCoreClock / 1000000U);

        s_eraseInProgress = false;

        if (g_flashDriver.getLastError() == 0U)
        {
            s_writeOffset = 0U;

            char report[100];

            std::snprintf(
                report,
                sizeof(report),
                "\r\n[TIME] Erase: %lu us (%lu cycle)\r\n",
                eraseUs,
                eraseCycles
            );

            sendText(report);

            sendText(
                "[SUCCESS] Sektor silme tamamlandi.\r\n> "
            );
        }
        else
        {
            sendText(
                "\r\n[ERROR] Sektor silme islemi basarisiz!\r\n> "
            );
        }
    }


    /*
     * Async Flash write tamamlandi mi?
     */
    if ((s_activeWrite != ActiveWrite::None) &&
        !g_flashDriver.isBusy())
    {
        uint32_t writeEndCycle =
            DWT->CYCCNT;

        uint32_t writeCycles =
            writeEndCycle -
            s_writeStartCycle;

        uint32_t writeUs =
            writeCycles /
            (SystemCoreClock / 1000000U);

        if (g_flashDriver.getLastError() == 0U)
        {
            s_writeOffset +=
                s_activeWriteSize;

            if (s_activeWrite ==
                ActiveWrite::Imu)
            {
                char report[100];

                std::snprintf(
                    report,
                    sizeof(report),
                    "\r\n[TIME] 70B Write: %lu us (%lu cycle)\r\n",
                    writeUs,
                    writeCycles
                );

                sendText(report);

                sendText(
                    "[SUCCESS] IMU verisi yazildi.\r\n> "
                );
            }

            else if (s_activeWrite ==
                     ActiveWrite::Gps)
            {
                char report[100];

                std::snprintf(
                    report,
                    sizeof(report),
                    "\r\n[TIME] 30B Write: %lu us (%lu cycle)\r\n",
                    writeUs,
                    writeCycles
                );

                sendText(report);

                sendText(
                    "[SUCCESS] GPS verisi yazildi.\r\n> "
                );
            }
        }
        else
        {
            sendText(
                "\r\n[ERROR] Flash yazma islemi basarisiz!\r\n> "
            );
        }

        s_activeWrite =
            ActiveWrite::None;

        s_activeWriteSize = 0U;
    }


    uint8_t cmd = 0U;

    /*
     * RX hala polling.
     *
     * 1 ms timeout ile loop'un uzun sure
     * bloklanmasini engelliyoruz.
     */
    if (g_uartDriver.receive(
            &cmd,
            1U,
            1U
        ) == UartStatus::Ok)
    {
        if ((cmd == '\r') ||
            (cmd == '\n'))
        {
            return;
        }


        /*
         * Gelen komutu terminale geri yaz.
         *
         * DMA TX devam ediyorsa send()
         * UartStatus::Busy donebilir.
         */
        g_uartDriver.send(
            &cmd,
            1U
        );

        sendText("\r\n");


        switch (cmd)
        {
            case 'e':
            case 'E':
            {
                if (g_flashDriver.isBusy())
                {
                    sendText(
                        "[BUSY] Flash su anda baska bir islem yapiyor.\r\n"
                    );

                    break;
                }

                sendText(
                    "[ERASE] Sektor 7 silme islemi baslatiliyor...\r\n"
                );

                s_eraseStartCycle =
                    DWT->CYCCNT;

                FlashStatus status =
                    g_flashDriver.erase();

                if (status ==
                    FlashStatus::Ok)
                {
                    s_eraseInProgress =
                        true;
                }
                else
                {
                    sendText(
                        "[ERROR] Sektor silme islemi baslatilamadi!\r\n"
                    );
                }

                break;
            }


            case 't':
            case 'T':
            {
                sendText(
                    "\r\n[BENCHMARK] Baslatiliyor...\r\n"
                );

                uint8_t testWord[32];

                std::memset(
                    testWord,
                    0xEE,
                    sizeof(testWord)
                );

                uint32_t t_start =
                    DWT->CYCCNT;

                (void)g_flashDriver.write(
                    STORAGE_START_ADDR,
                    testWord,
                    sizeof(testWord)
                );

                uint32_t t_end =
                    DWT->CYCCNT;

                uint32_t cycles =
                    t_end - t_start;

                uint32_t us =
                    cycles /
                    (SystemCoreClock / 1000000U);

                char report[100];

                std::snprintf(
                    report,
                    sizeof(report),
                    "[RESULT] 32B Yazma: %lu us (%lu cycle)\r\n",
                    us,
                    cycles
                );

                sendText(report);

                break;
            }


            case '1':
            {
                if (g_flashDriver.isBusy())
                {
                    sendText(
                        "[BUSY] Flash su anda baska bir islem yapiyor.\r\n"
                    );

                    break;
                }

                if (s_writeOffset +
                    alignTo32(IMU_DATA_SIZE) >
                    STORAGE_SEC_SIZE)
                {
                    sendText(
                        "[ERROR] Sektor doldu! Once [e] ile silmelisiniz.\r\n"
                    );

                    break;
                }

                uint32_t targetAddr =
                    STORAGE_START_ADDR +
                    s_writeOffset;

                char logMsg[90];

                std::snprintf(
                    logMsg,
                    sizeof(logMsg),
                    "[WRITE-1] 0x%08lX adresine %uB IMU verisi yaziliyor...\r\n",
                    targetAddr,
                    static_cast<unsigned int>(
                        IMU_DATA_SIZE
                    )
                );

                sendText(logMsg);

                s_writeStartCycle =
                    DWT->CYCCNT;

                FlashStatus status =
                    g_flashDriver.writeIT(
                        targetAddr,
                        s_imuData,
                        sizeof(s_imuData)
                    );

                if (status ==
                    FlashStatus::Ok)
                {
                    s_activeWrite =
                        ActiveWrite::Imu;

                    s_activeWriteSize =
                        alignTo32(
                            sizeof(s_imuData)
                        );
                }
                else
                {
                    sendText(
                        "[ERROR] Yazma islemi baslatilamadi!\r\n"
                    );
                }

                break;
            }


            case '2':
            {
                if (g_flashDriver.isBusy())
                {
                    sendText(
                        "[BUSY] Flash su anda baska bir islem yapiyor.\r\n"
                    );

                    break;
                }

                if (s_writeOffset +
                    alignTo32(GPS_DATA_SIZE) >
                    STORAGE_SEC_SIZE)
                {
                    sendText(
                        "[ERROR] Sektor doldu! Once [e] ile silmelisiniz.\r\n"
                    );

                    break;
                }

                uint32_t targetAddr =
                    STORAGE_START_ADDR +
                    s_writeOffset;

                char logMsg[90];

                std::snprintf(
                    logMsg,
                    sizeof(logMsg),
                    "[WRITE-2] 0x%08lX adresine %uB GPS verisi yaziliyor...\r\n",
                    targetAddr,
                    static_cast<unsigned int>(
                        GPS_DATA_SIZE
                    )
                );

                sendText(logMsg);

                s_writeStartCycle =
                    DWT->CYCCNT;

                FlashStatus status =
                    g_flashDriver.writeIT(
                        targetAddr,
                        s_gpsData,
                        sizeof(s_gpsData)
                    );

                if (status ==
                    FlashStatus::Ok)
                {
                    s_activeWrite =
                        ActiveWrite::Gps;

                    s_activeWriteSize =
                        alignTo32(
                            sizeof(s_gpsData)
                        );
                }
                else
                {
                    sendText(
                        "[ERROR] Yazma islemi baslatilamadi!\r\n"
                    );
                }

                break;
            }


            case 'r':
            case 'R':
            {
                sendText(
                    "[READ] Flash'in ilk 128 bayti:\r\n"
                );

                FlashStatus status =
                    g_flashDriver.read(
                        STORAGE_START_ADDR,
                        readBuf,
                        sizeof(readBuf)
                    );

                if (status ==
                    FlashStatus::Ok)
                {
                    char hexStr[8];

                    for (std::size_t i = 0U;
                         i < sizeof(readBuf);
                         ++i)
                    {
                        if ((i % 32U) == 0U)
                        {
                            char header[40];

                            std::snprintf(
                                header,
                                sizeof(header),
                                "\r\n  [0x%08lX]: ",
                                STORAGE_START_ADDR +
                                static_cast<uint32_t>(i)
                            );

                            sendText(header);
                        }

                        std::snprintf(
                            hexStr,
                            sizeof(hexStr),
                            "%02X ",
                            readBuf[i]
                        );

                        sendText(hexStr);
                    }

                    sendText("\r\n");
                }
                else
                {
                    sendText(
                        "[ERROR] Okuma basarisiz!\r\n"
                    );
                }

                break;
            }


            /*
             * UART TX DMA TEST
             */
            case 'd':
            case 'D':
            {
                UartStatus status =
                    g_uartDriver.sendDMA(
                        s_dmaTestData,
                        sizeof(s_dmaTestData)
                    );

                /*
                 * DMA devam ederken yeni TX istegi gelirse
                 * driver Busy dondurur.
                 *
                 * Kac kere Busy oldugunu sayiyoruz.
                 */
                if (status == UartStatus::Busy)
                {
                    ++s_dmaBusyCount;
                }

                /*
                 * Busy disinda gercek bir hata varsa
                 * DMA bittikten sonra raporla.
                 */
                else if (status != UartStatus::Ok)
                {
                    s_dmaErrorOccurred = true;
                }

                break;
            }


            case 'h':
            case 'H':
            {
                printMenu();

                break;
            }


            default:
            {
                sendText(
                    "[WARN] Gecersiz komut! [h] tusuna basin.\r\n"
                );

                break;
            }
        }


        /*
         * d komutu DMA TX baslattigi icin
         * hemen arkasindan polling TX ile prompt gonderme.
         */
        if ((cmd != 'd') &&
            (cmd != 'D'))
        {
            sendText("> ");
        }
    }
}
