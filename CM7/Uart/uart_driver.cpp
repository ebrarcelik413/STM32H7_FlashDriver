#include "uart_driver.hpp"

extern UartDriver g_uartDriver;


UartDriver::UartDriver(UART_HandleTypeDef* huart): huart_(huart)
{
}

UartStatus UartDriver::init()
{
    if (huart_ == nullptr)
    {
        return UartStatus::InvalidParam;
    }

    head_ = 0U;
    tail_ = 0U;
    rxCount_ = 0U;

    isOverflow_ = false;
    txBusy_ = false;
    isInitialized_ = true;

    /*
     * UART RX interrupt zincirini ilk kez baslat.
     *
     * Bundan sonra her tamamlanan RX interrupt'i
     * onRxByteReceived() icinden bir sonraki
     * Receive_IT'i tekrar baslatacak.
     */
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


void UartDriver::startReceiveIT()
{
    if (!isInitialized_ ||
        huart_ == nullptr)
    {
        return;
    }

    (void)HAL_UART_Receive_IT(
        huart_,
        &rxRawByte_,
        1U);
}


// ============================================================
// RING BUFFER PUSH
//
// Bu fonksiyon ISR tarafindan kullaniliyor.
// ============================================================

bool UartDriver::pushRxByte(
    uint8_t byte)
{
    /*
     * rxCount_ == RX_BUFFER_SIZE ise
     * okunmamis 100 byte var demektir.
     */
    if (rxCount_ >= RX_BUFFER_SIZE)
    {
        isOverflow_ = true;

        return false;
    }

    rxBuffer_[head_] = byte;

    ++head_;

    if (head_ >= RX_BUFFER_SIZE)
    {
        head_ = 0U;
    }

    ++rxCount_;

    return true;
}

void UartDriver::onRxByteReceived()
{
    /*
     * ISR sadece gelen RAW UART byte'ini
     * ring buffer'a koyar.
     */

    (void)pushRxByte(rxRawByte_);

    startReceiveIT();
}

bool UartDriver::readByte(
    uint8_t& byte)
{
    /*
     * readByte app_main tarafinda,
     * pushRxByte interrupt tarafinda calisiyor.
     *
     * tail/rxCount islemini atomik tutmak icin
     * kisa sure IRQ kapatiyoruz.
     */
    const uint32_t primask =
        __get_PRIMASK();

    __disable_irq();

    if (rxCount_ == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }

        return false;
    }

    byte = rxBuffer_[tail_];

    ++tail_;

    if (tail_ >= RX_BUFFER_SIZE)
    {
        tail_ = 0U;
    }

    --rxCount_;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return true;
}

bool UartDriver::readData(
    uint8_t* destination,
    std::size_t length)
{
    if (destination == nullptr || length == 0U)
    {
        return false;
    }

    const uint32_t primask =
        __get_PRIMASK();

    __disable_irq();

    /*
     * Yeterli byte yoksa hicbir sey okuma.
     */
    if (rxCount_ < length)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }

        return false;
    }

    for (std::size_t i = 0U; i < length; ++i)
    {
        destination[i] =
            rxBuffer_[tail_];

        ++tail_;

        if (tail_ >= RX_BUFFER_SIZE)
        {
            tail_ = 0U;
        }
    }

    rxCount_ -= length;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return true;
}


std::size_t
UartDriver::getAvailableDataCount() const
{
    return rxCount_;
}


std::size_t
UartDriver::getFreeSpace() const
{
    return RX_BUFFER_SIZE - rxCount_;
}


void UartDriver::clear()
{
    const uint32_t primask =
        __get_PRIMASK();

    __disable_irq();

    /*
     * RAM'in icini sifirlamamiza gerek yok.
     *
     * Ring buffer mantiginda head/tail/count
     * resetlemek buffer'i mantiksal olarak bosaltir.
     */
    head_ = 0U;
    tail_ = 0U;
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


void UartDriver::clearOverflow()
{
    isOverflow_ = false;
}

bool UartDriver::isTxBusy() const
{
    return txBusy_;
}


UartStatus UartDriver::send(
    const uint8_t* data,
    std::size_t length)
{
    if (!isInitialized_ || data == nullptr || length == 0U)
    {
        return UartStatus::InvalidParam;
    }

    const HAL_StatusTypeDef status =
        HAL_UART_Transmit(
            huart_,
            const_cast<uint8_t*>(data),
            static_cast<uint16_t>(length),
            1000U);

    return (status == HAL_OK)
               ? UartStatus::Ok
               : UartStatus::Error;
}


UartStatus UartDriver::sendDMA(
    const uint8_t* data,
    std::size_t length)
{
    if (!isInitialized_ || data == nullptr ||length == 0U)
    {
        return UartStatus::InvalidParam;
    }

    if (txBusy_)
    {
        return UartStatus::Busy;
    }

    txBusy_ = true;

    if (HAL_UART_Transmit_DMA( huart_, const_cast<uint8_t*>(data), static_cast<uint16_t>(length)) != HAL_OK)
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
     * IUartDriver interface'inde oldugu icin
     * simdilik tutuyoruz.
     *
     * Normal uygulama RX yolumuz bu degil.
     */
    if (!isInitialized_ ||
        data == nullptr ||
        length == 0U)
    {
        return UartStatus::InvalidParam;
    }

    const HAL_StatusTypeDef status =
        HAL_UART_Receive(
            huart_,
            data,
            static_cast<uint16_t>(length),
            timeoutMs);

    return (status == HAL_OK)
               ? UartStatus::Ok
               : UartStatus::Error;
}

void UartDriver::onError()
{
    if (huart_ == nullptr)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_NEFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);

    /*
     * Bir hata sonrasi RX zincirini yeniden baslat.
     */
    startReceiveIT();
}

void UartDriver::onTxComplete()
{
    txBusy_ = false;
}


extern "C"
void HAL_UART_RxCpltCallback( UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        g_uartDriver.onRxByteReceived();
    }
}


extern "C"
void HAL_UART_ErrorCallback( UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        g_uartDriver.onError();
    }
}


extern "C"
void HAL_UART_TxCpltCallback( UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        g_uartDriver.onTxComplete();
    }
}
