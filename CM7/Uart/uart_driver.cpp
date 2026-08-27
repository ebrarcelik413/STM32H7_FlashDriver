#include "uart_driver.hpp"


/**
 * Sinifin kurucu metodudur.
 * huart parametresi driver'in kullanacagi UART handle'ini tutar.
 */
UartDriver::UartDriver(UART_HandleTypeDef* huart)
    : huart_(huart)
{
}


UartStatus UartDriver::init()
{
    if (huart_ == nullptr)
    {
        return UartStatus::InvalidParam;
    }

    txBusy_ = false;

    rxBusy_ = false;
    rxDataReady_ = false;

    isInitialized_ = true;

    return UartStatus::Ok;
}


UartStatus UartDriver::deinit()
{
    /*
     * TX veya RX islemi devam ediyorsa
     * driver'i kapatmiyoruz.
     */
    if (txBusy_ || rxBusy_)
    {
        return UartStatus::Busy;
    }

    isInitialized_ = false;

    return UartStatus::Ok;
}


UartStatus UartDriver::send(
    const uint8_t* data,
    std::size_t length)
{
    if (!isInitialized_ || (huart_ == nullptr))
    {
        return UartStatus::NotInitialized;
    }

    if ((data == nullptr) || (length == 0U))
    {
        return UartStatus::InvalidParam;
    }

    /*
     * DMA TX devam ederken ayni UART uzerinden
     * polling TX baslatilmasina izin vermiyoruz.
     */
    if (txBusy_)
    {
        return UartStatus::Busy;
    }

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(
            huart_,
            const_cast<uint8_t*>(data),
            static_cast<uint16_t>(length),
            HAL_MAX_DELAY
        );

    if (status == HAL_BUSY)
    {
        return UartStatus::Busy;
    }

    return (status == HAL_OK)
        ? UartStatus::Ok
        : UartStatus::Error;
}


UartStatus UartDriver::sendDMA(
    const uint8_t* data,
    std::size_t length)
{
    if (!isInitialized_ || (huart_ == nullptr))
    {
        return UartStatus::NotInitialized;
    }

    if ((data == nullptr) || (length == 0U))
    {
        return UartStatus::InvalidParam;
    }

    /*
     * Onceki DMA TX hala devam ediyorsa
     * ikinci transferi baslatmiyoruz.
     */
    if (txBusy_)
    {
        return UartStatus::Busy;
    }

    txBusy_ = true;

    HAL_StatusTypeDef status =
        HAL_UART_Transmit_DMA(
            huart_,
            const_cast<uint8_t*>(data),
            static_cast<uint16_t>(length)
        );

    /*
     * DMA baslatilamadiysa aktif TX yoktur.
     */
    if (status != HAL_OK)
    {
        txBusy_ = false;

        return (status == HAL_BUSY)
            ? UartStatus::Busy
            : UartStatus::Error;
    }

    return UartStatus::Ok;
}


/*
 * Polling RX.
 *
 * Bunu kaldirmak zorunda degiliz.
 * Eski polling kullanimlari icin driver'da kalabilir.
 */
UartStatus UartDriver::receive(
    uint8_t* data,
    std::size_t length,
    uint32_t timeoutMs)
{
    if (!isInitialized_ || (huart_ == nullptr))
    {
        return UartStatus::NotInitialized;
    }

    if ((data == nullptr) || (length == 0U))
    {
        return UartStatus::InvalidParam;
    }

    /*
     * Interrupt RX devam ederken ayni UART icin
     * polling RX baslatmiyoruz.
     */
    if (rxBusy_)
    {
        return UartStatus::Busy;
    }

    HAL_StatusTypeDef status =
        HAL_UART_Receive(
            huart_,
            data,
            static_cast<uint16_t>(length),
            timeoutMs
        );

    if (status == HAL_BUSY)
    {
        return UartStatus::Busy;
    }

    if (status == HAL_TIMEOUT)
    {
        return UartStatus::Timeout;
    }

    return (status == HAL_OK)
        ? UartStatus::Ok
        : UartStatus::Error;
}


/*
 * Interrupt tabanli RX baslatir.
 *
 * Fonksiyon verinin gelmesini beklemez.
 * Sadece RX islemini baslatir ve geri doner.
 */
UartStatus UartDriver::receiveIT(
    uint8_t* data,
    std::size_t length)
{
    if (!isInitialized_ || (huart_ == nullptr))
    {
        return UartStatus::NotInitialized;
    }

    if ((data == nullptr) || (length == 0U))
    {
        return UartStatus::InvalidParam;
    }

    /*
     * Onceki RX hala tamamlanmadiysa
     * ayni buffer icin yeni RX baslatmiyoruz.
     */
    if (rxBusy_)
    {
        return UartStatus::Busy;
    }

    /*
     * Yeni RX basliyor.
     *
     * Buffer henuz application tarafindan
     * okunmaya hazir degil.
     */
    rxBusy_ = true;
    rxDataReady_ = false;

    HAL_StatusTypeDef status =
        HAL_UART_Receive_IT(
            huart_,
            data,
            static_cast<uint16_t>(length)
        );

    /*
     * HAL RX interrupt islemini baslatamadiysa
     * aktif bir RX yoktur.
     */
    if (status != HAL_OK)
    {
        rxBusy_ = false;

        return (status == HAL_BUSY)
            ? UartStatus::Busy
            : UartStatus::Error;
    }

    return UartStatus::Ok;
}


/*
 * TX DMA halen devam ediyor mu?
 */
bool UartDriver::isTxBusy() const
{
    return txBusy_;
}


/*
 * RX interrupt islemi halen devam ediyor mu?
 */
bool UartDriver::isRxBusy() const
{
    return rxBusy_;
}


/*
 * RX buffer tamamen doldu mu ve
 * application tarafindan okunabilir mi?
 */
bool UartDriver::isRxDataReady() const
{
    return rxDataReady_;
}


/*
 * Application RX verisini isledikten sonra
 * ready flag'ini temizler.
 */
void UartDriver::clearRxDataReady()
{
    rxDataReady_ = false;
}


/*
 * UART TX tamamen bittiginde
 * HAL_UART_TxCpltCallback tarafindan cagrilir.
 */
void UartDriver::onTxComplete()
{
    txBusy_ = false;
}


/*
 * Istenen RX verisinin tamami geldiginde
 * HAL_UART_RxCpltCallback tarafindan cagrilir.
 */
void UartDriver::onRxComplete()
{
    /*
     * Artik UART buffer'a veri yazmiyor.
     */
    rxBusy_ = false;

    /*
     * Buffer tamamen doldu.
     * Application artik okuyabilir.
     */
    rxDataReady_ = true;
}


/*
 * UART tarafinda hata olustugunda
 * HAL_UART_ErrorCallback tarafindan cagrilir.
 */
void UartDriver::onError()
{
    /*
     * Hata durumunda driver'in Busy state'lerinde
     * takili kalmasini engelliyoruz.
     */
    txBusy_ = false;
    rxBusy_ = false;
}


/*
 * extern sadece:
 * "Bu nesne baska dosyada var, burada kullanacagim."
 * demektir.
 */
extern UART_HandleTypeDef huart1;
extern UartDriver g_uartDriver;


/*
 * TX tamamen bittiginde HAL tarafindan cagrilir.
 */
extern "C" void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef* huart)
{
    if (huart == &huart1)
    {
        g_uartDriver.onTxComplete();
    }
}


/*
 * RX icin istenen tum byte'lar geldiginde
 * HAL tarafindan cagrilir.
 */
extern "C" void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef* huart)
{
    if (huart == &huart1)
    {
        g_uartDriver.onRxComplete();
    }
}


/*
 * UART hata olusturdugunda HAL tarafindan cagrilir.
 */
extern "C" void HAL_UART_ErrorCallback(
    UART_HandleTypeDef* huart)
{
    if (huart == &huart1)
    {
        g_uartDriver.onError();
    }
}
