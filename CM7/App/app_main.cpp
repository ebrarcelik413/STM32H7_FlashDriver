#include "app_main.h"
#include "main.h"

#include "flash_driver.hpp"
#include "uart_driver.hpp"
#include "protocol_parser.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>


extern UART_HandleTypeDef huart1;

constexpr uint32_t STORAGE_BANK        = FLASH_BANK_2;
constexpr uint32_t STORAGE_SECTOR      = FLASH_SECTOR_7;
constexpr uint32_t STORAGE_START_ADDR  = 0x081E0000U;
constexpr std::size_t STORAGE_SEC_SIZE = 128U * 1024U;


FlashDriver g_flashDriver(
    STORAGE_BANK,
    STORAGE_SECTOR,
    STORAGE_START_ADDR,
    STORAGE_SEC_SIZE);

UartDriver g_uartDriver(&huart1);

ProtocolParser g_protocolParser;


static uint8_t s_imuTestData[70];
static uint8_t s_gpsTestData[30];


static constexpr std::size_t DATA_BUFFER_SIZE = 100U;


static uint8_t s_dataBuffer[DATA_BUFFER_SIZE]{};


// Bir sonraki yazma konumu
static std::size_t s_dataHead = 0U;


// Bir sonraki okuma konumu
static std::size_t s_dataTail = 0U;


// Buffer'daki okunmamis byte sayisi
static std::size_t s_dataCount = 0U;



volatile uint32_t g_uartAvailableCount = 0U;
volatile uint32_t g_uartFreeSpace      = 100U;
volatile bool     g_uartOverflow       = false;


// ============================================================
// APPLICATION DATA DEBUG
// ============================================================

volatile uint32_t g_dataHead = 0U;
volatile uint32_t g_dataTail = 0U;
volatile uint32_t g_dataCount = 0U;
volatile uint32_t g_dataFreeSpace = 100U;
volatile bool g_dataOverflow = false;

// ============================================================
// PROTOCOL / TX DEBUG
// ============================================================

volatile uint8_t g_lastCommandId = 0U;
volatile bool g_lastAppendSuccess = false;
volatile bool g_lastReadSuccess = false;
volatile uint8_t g_lastRequestedRead = 0U;
volatile bool g_checksumError = false;
volatile UartStatus g_lastTxStatus = UartStatus::Ok;


// ============================================================
// READ OUTPUT BUFFER
//
// CMD04 ile okunan veri once buraya gelir.
//
// Sonra DMA TX ile terminale gonderilir.
// ============================================================

uint8_t
g_readOutBuffer[100] = {0U};


// ============================================================
// MANUAL FLASH TEST
// ============================================================

volatile bool g_triggerFlashErase = false;
volatile bool g_triggerFlashWrite = false;
volatile uint32_t g_dwtEraseCycles = 0U;
volatile uint32_t g_dwtWriteCycles = 0U;


alignas(32) uint8_t g_testWriteData[32] =
{
    0xAA,
    0xBB,
    0xCC,
    0xDD
};

static void clearApplicationData()
{
    /*
     * Fiziksel buffer'i da temizle.
     *
     * Debugger'da tum elemanlar 0 gorunsun.
     */
    std::memset(s_dataBuffer, 0, sizeof(s_dataBuffer));

    s_dataHead = 0U;
    s_dataTail = 0U;
    s_dataCount = 0U;


    g_dataOverflow = false;
}

static bool appendApplicationData(
    const uint8_t* data,
    std::size_t length)
{
    if (data == nullptr || length == 0U)
    {
        return false;
    }


    /*
     * Tum veri sigmiyorsa hicbirini yazma.
     * Okunmamis eski verinin ustune yazma.
     */
    if (length > (DATA_BUFFER_SIZE - s_dataCount))
    {
        g_dataOverflow = true;
        return false;
    }


    for (std::size_t i = 0U; i < length; ++i)
    {
        s_dataBuffer[s_dataHead] =
            data[i];

        ++s_dataHead;

        /*
         * Ring buffer sonuna geldiysek
         * basa sar.
         */
        if (s_dataHead >= DATA_BUFFER_SIZE)
        {
            s_dataHead = 0U;
        }
    }

    s_dataCount += length;

    return true;
}


// ============================================================
// APPLICATION DATA BUFFER READ
//
// SADECE CMD04 CAGIRIR.
//
// 1) Data destination'a kopyalanir.
// 2) Okunan fiziksel alan 0 yapilir.
// 3) Tail ilerler.
// 4) Count azalir.
// ============================================================

static bool readApplicationData(
    uint8_t* destination,
    std::size_t length)
{
    if (destination == nullptr || length == 0U)
    {
        return false;
    }


    /*
     * Yeterli okunmamis veri yoksa
     * hicbir seyi degistirme.
     */
    if (s_dataCount < length)
    {
        return false;
    }


    for (std::size_t i = 0U; i < length; ++i)
    {
        /*
         * Veriyi oku.
         */
        destination[i] =
            s_dataBuffer[s_dataTail];


        /*
         * Okunan fiziksel yeri temizle.
         * Debugger'da artik 0 goreceksin.
         */
        s_dataBuffer[s_dataTail] =
            0U;

        /*
         * Tail sonraki okunacak konuma ilerler.
         */
        ++s_dataTail;


        if (s_dataTail >= DATA_BUFFER_SIZE)
        {
            s_dataTail = 0U;
        }
    }

    /*
     * Okunan byte'lar artik buffer'da yok.
     */
    s_dataCount -= length;

    return true;
}


static void updateDebugState()
{

    g_uartAvailableCount =
        static_cast<uint32_t>( g_uartDriver .getAvailableDataCount());


    g_uartFreeSpace =
        static_cast<uint32_t>(  g_uartDriver .getFreeSpace());


    g_uartOverflow =
        g_uartDriver .isOverflow();


    g_dataHead =
        static_cast<uint32_t>(s_dataHead);


    g_dataTail =
        static_cast<uint32_t>(s_dataTail);


    g_dataCount =
        static_cast<uint32_t>(s_dataCount);


    g_dataFreeSpace =
        static_cast<uint32_t>( DATA_BUFFER_SIZE - s_dataCount);
}

static void processProtocolPacket(
    const BinaryPacket& packet)
{
    g_lastCommandId =
        packet.cmdId;


    switch (static_cast<CommandId>(packet.cmdId))
    {

    // ========================================================
    // CMD01
    //
    // CLEAR APPLICATION DATA BUFFER
    //
    // A5 5A 01 00 00 01
    // ========================================================

    case CommandId::ClearBuffer:
    {
        clearApplicationData();

        g_lastAppendSuccess = false;
        g_lastReadSuccess = false;
        g_lastRequestedRead = 0U;

        break;
    }


    // ========================================================
    // CMD02
    //
    // 70 BYTE IMU EKLE
    //
    // A5 5A 02 00 01 01
    //
    // 70 tane 0xAA application buffer'a eklenir.
    // ========================================================

    case CommandId::WriteImu:
    {
        g_lastAppendSuccess =
            appendApplicationData(
                s_imuTestData,
                sizeof(s_imuTestData));


        break;
    }

    // ========================================================
    // CMD03
    //
    // 30 BYTE GPS EKLE
    //
    // A5 5A 03 00 02 01
    //
    // 30 tane 0xBB application buffer'a eklenir.
    // ========================================================

    case CommandId::WriteGps:
    {
        g_lastAppendSuccess =
            appendApplicationData(
                s_gpsTestData,
                sizeof(s_gpsTestData));


        break;
    }


    // ========================================================
    // CMD04
    //
    // DATA OKU + DMA TX
    //
    // Ornek:
    //
    // A5 5A 04 01 3C 40 01
    //
    // CMD    = 04
    // LENGTH = 01
    // DATA   = 3C
    //
    // 0x3C = 60 decimal
    //
    // Yani:
    //
    // "60 byte oku ve terminale gonder."
    // ========================================================

    case CommandId::ReadData:
    {
        /*
         * CMD04 icin protocol payload uzunlugu
         * 1 byte olmali.
         */
        if (packet.length != 1U)
        {
            g_lastReadSuccess = false;

            break;
        }

        /*
         * Kac byte okunacak?
         */
        const uint8_t requested =
            packet.payload[0];


        g_lastRequestedRead =
            requested;

        /*
         * 0 byte okumaya izin verme.
         */
        if (requested == 0U)
        {
            g_lastReadSuccess = false;

            break;
        }

        /*
         * Output buffer'dan buyuk veri istenemez.
         */
        if (requested > sizeof(g_readOutBuffer))
        {
            g_lastReadSuccess = false;

            break;
        }

        /*
         * Onceki DMA TX bitmediyse
         * yeni TX baslatma.
         */
        if (g_uartDriver.isTxBusy())
        {
            g_lastReadSuccess = false;

            g_lastTxStatus =
                UartStatus::Busy;

            break;
        }

        /*
         * APPLICATION BUFFER'DAN OKU.
         *
         * Burada:
         *
         * tail ilerler
         * count azalir
         * okunan yerler 0 olur
         */
        g_lastReadSuccess =
            readApplicationData(
                g_readOutBuffer,
                requested);

        /*
         * Okuma basarisizsa TX yapma.
         */
        if (!g_lastReadSuccess)
        {
            break;
        }

        /*
         * Okunan veriyi DMA TX ile
         * terminale gonder.
         */
        g_lastTxStatus =
            g_uartDriver.sendDMA(g_readOutBuffer, requested);

        break;
    }

    default:
    {
        break;
    }
    }

    updateDebugState();
}


// ============================================================
// UART -> APPMAIN -> PROTOCOL PARSER
//
// BU KISIM SADECE UART RAW RX BUFFER'I TUKETIR.
//
// s_dataBuffer'A DOKUNMAZ.
// ============================================================

static void processUartProtocol()
{
    uint8_t incomingByte = 0U;


    /*
     * UART'tan fiziksel olarak gelen
     * protokol byte'larini oku.
     */
    while (
        g_uartDriver.readByte(
            incomingByte))
    {
        /*
         * AppMain ham byte'i Parser'a verir.
         */
        (void)g_protocolParser.pushByte(
            incomingByte);


        BinaryPacket packet{};


        /*
         * Paket tamamlandiysa
         * AppMain hemen isler.
         */
        if (
            g_protocolParser.getPacket(
                packet))
        {
            processProtocolPacket(
                packet);
        }
    }


    /*
     * Checksum error kontrolu.
     */
    if (
        g_protocolParser
            .hasChecksumError())
    {
        g_checksumError = true;


        g_protocolParser
            .clearChecksumError();
    }
}

extern "C"
void app_init(void)
{

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;


    DWT->CYCCNT = 0U;


    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


    for (std::size_t i = 0U; i < sizeof(s_imuTestData); ++i)
    {
        s_imuTestData[i] =
            0xAAU;
    }


    // --------------------------------------------------------
    // TEST GPS
    //
    // 30 tane 0xBB
    // --------------------------------------------------------

    for (std::size_t i = 0U;
         i < sizeof(s_gpsTestData);
         ++i)
    {
        s_gpsTestData[i] =
            0xBBU;
    }


    // --------------------------------------------------------
    // APPLICATION DATA BUFFER
    // --------------------------------------------------------

    clearApplicationData();


    // --------------------------------------------------------
    // READ OUTPUT BUFFER
    // --------------------------------------------------------

    std::memset(
        g_readOutBuffer,
        0,
        sizeof(g_readOutBuffer));


    // --------------------------------------------------------
    // PROTOCOL
    // --------------------------------------------------------

    g_protocolParser.reset();


    // --------------------------------------------------------
    // DRIVERS
    // --------------------------------------------------------

    (void)g_uartDriver.init();

    (void)g_flashDriver.init();

    updateDebugState();
}

extern "C"
void app_loop(void)
{
    g_flashDriver.process();

    processUartProtocol();


    updateDebugState();


    // ========================================================
    // MANUAL FLASH ERASE
    // ========================================================

    if (g_triggerFlashErase)
    {
        g_triggerFlashErase =
            false;


        if (!g_flashDriver.isBusy())
        {
            const uint32_t startCycles =
                DWT->CYCCNT;


            (void)g_flashDriver
                .eraseBlocking();


            g_dwtEraseCycles =
                DWT->CYCCNT -
                startCycles;
        }
    }


    // ========================================================
    // MANUAL FLASH WRITE
    // ========================================================

    if (g_triggerFlashWrite)
    {
        g_triggerFlashWrite =
            false;


        if (!g_flashDriver.isBusy())
        {
            const uint32_t startCycles =
                DWT->CYCCNT;


            (void)g_flashDriver
                .writeBlocking(
                    STORAGE_START_ADDR,
                    g_testWriteData,
                    sizeof(g_testWriteData));


            g_dwtWriteCycles =
                DWT->CYCCNT -
                startCycles;
        }
    }
}
