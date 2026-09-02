#include "uart_driver.hpp"
#include <cstring>

extern UartDriver g_uartDriver;


// ============================================================
// CONSTRUCTOR
// ============================================================

UartDriver::UartDriver(UART_HandleTypeDef* huart)
    : huart_(huart)
{
}


// ============================================================
// INIT / DEINIT
// ============================================================

UartStatus UartDriver::init()
{
    if (huart_ == nullptr)
    {
        return UartStatus::InvalidParam;
    }

    clear();

    parser_.reset();

    txBusy_ = false;
    isInitialized_ = true;

    startReceiveIT();

    return UartStatus::Ok;
}


UartStatus UartDriver::deinit()
{
    if (huart_ != nullptr)
    {
        HAL_UART_AbortReceive(huart_);
    }

    isInitialized_ = false;

    return UartStatus::Ok;
}


// ============================================================
// RX INTERRUPT
// ============================================================

void UartDriver::startReceiveIT()
{
    if (isInitialized_ && huart_ != nullptr)
    {
        (void)HAL_UART_Receive_IT(
            huart_,
            &rxRawByte_,
            1U);
    }
}


void UartDriver::onRxByteReceived()
{
    /*
     * Gelen byte artik direkt data buffer'a yazilmaz.
     *
     * Once ProtocolParser'a gider:
     *
     * A5 5A CMD LENGTH DATA CHECKSUM
     *
     * Paket tamamlaninca app_main:
     *
     * CMD 01 -> clear()
     * CMD 02 -> 70 byte IMU appendData()
     * CMD 03 -> 30 byte GPS appendData()
     * CMD 04 -> readData()
     *
     * yapar.
     */

    parser_.pushByte(rxRawByte_);

    // Sonraki byte'i bekle.
    startReceiveIT();
}


// ============================================================
// DATA BUFFER - WRITE
// ============================================================

bool UartDriver::appendData(
    const uint8_t* data,
    std::size_t length)
{
    if (data == nullptr || length == 0U)
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /*
     * Tum veri sigmiyorsa hicbir sey yazma.
     *
     * Ornek:
     *
     * buffer = 40 byte
     * yeni veri = 70 byte
     *
     * 40 + 70 = 110
     *
     * -> false
     * -> eski 40 byte korunur
     */
    if (length > (RX_BUFFER_SIZE - rxCount_))
    {
        isOverflow_ = true;

        if (primask == 0U)
        {
            __enable_irq();
        }

        return false;
    }

    std::memcpy(
        &rxBuffer_[rxCount_],
        data,
        length);

    rxCount_ += length;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return true;
}


// ============================================================
// DATA BUFFER - READ
// ============================================================

bool UartDriver::readByte(uint8_t& byte)
{
    return readData(&byte, 1U);
}


bool UartDriver::readData(
    uint8_t* destination,
    std::size_t length)
{
    if (destination == nullptr || length == 0U)
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /*
     * Yeterli veri yoksa buffer'a dokunma.
     *
     * buffer = 70
     * readData(..., 80)
     *
     * -> false
     * -> buffer hala 70
     */
    if (rxCount_ < length)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }

        return false;
    }

    // Ilk N byte'i kullaniciya ver.
    std::memcpy(
        destination,
        rxBuffer_,
        length);

    /*
     * Okunan kisim buffer'dan dusurulur.
     *
     * Ornek:
     *
     * 70 byte var
     * 60 byte okundu
     *
     * remaining = 10
     */
    const std::size_t remaining =
        rxCount_ - length;

    if (remaining > 0U)
    {
        /*
         * Kalan verileri buffer'in basina kaydir.
         */
        std::memmove(
            rxBuffer_,
            rxBuffer_ + length,
            remaining);
    }

    /*
     * Debug ekraninda eski veriler gorunmesin.
     */
    std::memset(
        rxBuffer_ + remaining,
        0,
        length);

    rxCount_ = remaining;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return true;
}


// ============================================================
// BUFFER INFO
// ============================================================

std::size_t UartDriver::getAvailableDataCount() const
{
    return rxCount_;
}


std::size_t UartDriver::getFreeSpace() const
{
    return RX_BUFFER_SIZE - rxCount_;
}


void UartDriver::clear()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    std::memset(
        rxBuffer_,
        0,
        sizeof(rxBuffer_));

    rxCount_ = 0U;
    isOverflow_ = false;

    if (primask == 0U)
    {
        __enable_irq();
    }
}


void UartDriver::flushRxBuffer()
{
    clear();
}


bool UartDriver::isOverflow() const
{
    return isOverflow_;
}


// ============================================================
// PROTOCOL
// ============================================================

bool UartDriver::getReceivedPacket(
    BinaryPacket& packet)
{
    return parser_.getPacket(packet);
}


bool UartDriver::hasProtocolChecksumError() const
{
    return parser_.hasChecksumError();
}


void UartDriver::clearProtocolChecksumError()
{
    parser_.clearChecksumError();
}


// ============================================================
// TX
// ============================================================

bool UartDriver::isTxBusy() const
{
    return txBusy_;
}


UartStatus UartDriver::send(
    const uint8_t* data,
    std::size_t length)
{
    /*
     * IUartDriver interface'inde oldugu icin tutuluyor.
     *
     * Bizim asil TX yolumuz sendDMA().
     */

    if (!isInitialized_ ||
        data == nullptr ||
        length == 0U)
    {
        return UartStatus::InvalidParam;
    }

    return (
        HAL_UART_Transmit(
            huart_,
            const_cast<uint8_t*>(data),
            static_cast<uint16_t>(length),
            1000U) == HAL_OK)
            ? UartStatus::Ok
            : UartStatus::Error;
}


UartStatus UartDriver::sendDMA(
    const uint8_t* data,
    std::size_t length)
{
    if (!isInitialized_ ||
        data == nullptr ||
        length == 0U)
    {
        return UartStatus::InvalidParam;
    }

    if (txBusy_)
    {
        return UartStatus::Busy;
    }

    txBusy_ = true;

    if (HAL_UART_Transmit_DMA(
            huart_,
            const_cast<uint8_t*>(data),
            static_cast<uint16_t>(length))
        != HAL_OK)
    {
        txBusy_ = false;

        return UartStatus::Error;
    }

    return UartStatus::Ok;
}


// ============================================================
// OLD POLLING RECEIVE
// ============================================================

UartStatus UartDriver::receive(
    uint8_t* data,
    std::size_t length,
    uint32_t timeoutMs)
{
    /*
     * IUartDriver interface'inde hala mevcut.
     *
     * app_main bunu KULLANMIYOR.
     *
     * Gercek RX:
     *
     * HAL_UART_Receive_IT()
     *
     * ile yapiliyor.
     */

    if (!isInitialized_ ||
        data == nullptr ||
        length == 0U)
    {
        return UartStatus::InvalidParam;
    }

    return (
        HAL_UART_Receive(
            huart_,
            data,
            static_cast<uint16_t>(length),
            timeoutMs) == HAL_OK)
            ? UartStatus::Ok
            : UartStatus::Error;
}


// ============================================================
// CALLBACK HELPERS
// ============================================================

void UartDriver::onError()
{
    if (huart_ == nullptr)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_NEFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);

    startReceiveIT();
}


void UartDriver::onTxComplete()
{
    txBusy_ = false;
}


// ============================================================
// HAL CALLBACKS
// ============================================================

extern "C"
void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        g_uartDriver.onRxByteReceived();
    }
}


extern "C"
void HAL_UART_ErrorCallback(
    UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        g_uartDriver.onError();
    }
}


extern "C"
void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        g_uartDriver.onTxComplete();
    }
}
