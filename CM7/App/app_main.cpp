#include "app_main.h"
#include "main.h"

#include "flash_driver.hpp"
#include "uart_driver.hpp"
#include "protocol_parser.hpp"

#include <cstddef>
#include <cstdint>

extern UART_HandleTypeDef huart1;

// ============================================================
// FLASH
// UART komutlari Flash'a DOKUNMAZ.
// Flash testleri simdilik manuel trigger ile yapilir.
// ============================================================

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


// ============================================================
// UART TEST DATA
//
// CMD 02 geldiginde terminalden 70 byte gondermek yerine
// bu hazir IMU test dizisi FIFO'ya eklenir.
//
// CMD 03 geldiginde bu hazir GPS test dizisi eklenir.
// ============================================================

static uint8_t s_imuTestData[70];
static uint8_t s_gpsTestData[30];


// ============================================================
// UART DEBUG / OBSERVATION
// Expressions ekranindan bakmak icin.
// ============================================================

volatile uint32_t g_uartAvailableCount = 0U;
volatile uint32_t g_uartFreeSpace      = 100U;
volatile bool     g_uartOverflow       = false;

volatile uint8_t  g_lastCommandId      = 0U;
volatile bool     g_lastAppendSuccess  = false;
volatile bool     g_lastReadSuccess    = false;
volatile uint8_t  g_lastRequestedRead  = 0U;
volatile bool     g_checksumError      = false;

// readData() sonucu once buraya gelir.
// Ardindan sendDMA() bu diziyi terminale yollar.
uint8_t g_readOutBuffer[100] = {0U};


// ============================================================
// MANUAL FLASH TEST
// ============================================================

volatile bool     g_triggerFlashErase = false;
volatile bool     g_triggerFlashWrite = false;

volatile uint32_t g_dwtEraseCycles = 0U;
volatile uint32_t g_dwtWriteCycles = 0U;

alignas(32) uint8_t g_testWriteData[32] =
{
    0xAA, 0xBB, 0xCC, 0xDD
};


// ============================================================
// HELPERS
// ============================================================

static void updateUartDebugState()
{
    g_uartAvailableCount =
        static_cast<uint32_t>(g_uartDriver.getAvailableDataCount());

    g_uartFreeSpace =
        static_cast<uint32_t>(g_uartDriver.getFreeSpace());

    g_uartOverflow =
        g_uartDriver.isOverflow();
}


static void processUartPacket(const BinaryPacket& packet)
{
    g_lastCommandId = packet.cmdId;

    switch (static_cast<CommandId>(packet.cmdId))
    {
    // --------------------------------------------------------
    // CMD 01
    // A5 5A 01 00 00 01
    // --------------------------------------------------------
    case CommandId::ClearBuffer:
    {
        if (packet.length == 0U)
        {
            g_uartDriver.clear();
        }

        break;
    }

    // --------------------------------------------------------
    // CMD 02
    // A5 5A 02 00 01 01
    //
    // Terminal 70 byte gondermez.
    // App_main hazir 70-byte IMU test verisini ekler.
    // --------------------------------------------------------
    case CommandId::WriteImu:
    {
        if (packet.length == 0U)
        {
            g_lastAppendSuccess =
                g_uartDriver.appendData(
                    s_imuTestData,
                    sizeof(s_imuTestData));
        }
        else
        {
            g_lastAppendSuccess = false;
        }

        break;
    }

    // --------------------------------------------------------
    // CMD 03
    // A5 5A 03 00 02 01
    //
    // Hazir 30-byte GPS test verisini ekler.
    // --------------------------------------------------------
    case CommandId::WriteGps:
    {
        if (packet.length == 0U)
        {
            g_lastAppendSuccess =
                g_uartDriver.appendData(
                    s_gpsTestData,
                    sizeof(s_gpsTestData));
        }
        else
        {
            g_lastAppendSuccess = false;
        }

        break;
    }

    // --------------------------------------------------------
    // CMD 04
    //
    // DATA[0] = kac byte okunacak?
    //
    // Ornek 60:
    // A5 5A 04 01 3C 40 01
    // --------------------------------------------------------
    case CommandId::ReadData:
    {
        if ((packet.length == 1U) &&
            !g_uartDriver.isTxBusy())
        {
            const uint8_t requested = packet.payload[0];

            g_lastRequestedRead = requested;

            g_lastReadSuccess =
                g_uartDriver.readData(
                    g_readOutBuffer,
                    requested);

            if (g_lastReadSuccess)
            {
                // readData TX degildir.
                // Terminale cikis burada DMA ile yapilir.
                g_uartDriver.sendDMA(
                    g_readOutBuffer,
                    requested);
            }
        }
        else
        {
            g_lastReadSuccess = false;
        }

        break;
    }

    default:
        break;
    }

    updateUartDebugState();
}


// ============================================================
// APP INIT
// ============================================================

extern "C" void app_init(void)
{
    // DWT cycle counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Test IMU = 70 x AA
    for (std::size_t i = 0U; i < sizeof(s_imuTestData); ++i)
    {
        s_imuTestData[i] = 0xAAU;
    }

    // Test GPS = 30 x BB
    for (std::size_t i = 0U; i < sizeof(s_gpsTestData); ++i)
    {
        s_gpsTestData[i] = 0xBBU;
    }

    g_uartDriver.init();
    g_flashDriver.init();

    updateUartDebugState();
}


// ============================================================
// APP LOOP
// ============================================================

extern "C" void app_loop(void)
{
    // Flash driver'in kendi background islemleri.
    g_flashDriver.process();


    // --------------------------------------------------------
    // UART PROTOCOL
    // --------------------------------------------------------

    BinaryPacket packet{};

    if (g_uartDriver.getReceivedPacket(packet))
    {
        processUartPacket(packet);
    }

    if (g_uartDriver.hasProtocolChecksumError())
    {
        g_checksumError = true;
        g_uartDriver.clearProtocolChecksumError();
    }

    updateUartDebugState();


    // --------------------------------------------------------
    // MANUAL FLASH ERASE
    // UART ile baglantisi YOK.
    // --------------------------------------------------------

    if (g_triggerFlashErase)
    {
        g_triggerFlashErase = false;

        if (!g_flashDriver.isBusy())
        {
            const uint32_t startCycles = DWT->CYCCNT;

            g_flashDriver.eraseBlocking();

            g_dwtEraseCycles =
                DWT->CYCCNT - startCycles;
        }
    }


    // --------------------------------------------------------
    // MANUAL FLASH WRITE
    // UART ile baglantisi YOK.
    // --------------------------------------------------------

    if (g_triggerFlashWrite)
    {
        g_triggerFlashWrite = false;

        if (!g_flashDriver.isBusy())
        {
            const uint32_t startCycles = DWT->CYCCNT;

            g_flashDriver.writeBlocking(
                STORAGE_START_ADDR,
                g_testWriteData,
                sizeof(g_testWriteData));

            g_dwtWriteCycles =
                DWT->CYCCNT - startCycles;
        }
    }
}
