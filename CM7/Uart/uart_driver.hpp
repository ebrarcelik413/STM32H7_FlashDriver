#pragma once

#include "iuart_driver.hpp"
#include "main.h"


class UartDriver : public IUartDriver
{
public:

    explicit UartDriver(UART_HandleTypeDef* huart);

    ~UartDriver() override = default;


    UartStatus init() override;

    UartStatus deinit() override;


    /*
     * Polling TX
     */
    UartStatus send(
        const uint8_t* data,
        std::size_t length
    ) override;


    /*
     * DMA TX
     */
    UartStatus sendDMA(
        const uint8_t* data,
        std::size_t length
    ) override;


    /*
     * Polling RX
     *
     * IUartDriver bunu pure virtual olarak istedigi icin
     * simdilik burada kalmasi gerekiyor.
     */
    UartStatus receive(
        uint8_t* data,
        std::size_t length,
        uint32_t timeoutMs
    ) override;


    /*
     * Interrupt RX
     */
    UartStatus receiveIT(
        uint8_t* data,
        std::size_t length
    );


    /*
     * TX durum bilgisi
     */
    bool isTxBusy() const;


    /*
     * RX durum bilgileri
     */
    bool isRxBusy() const;

    bool isRxDataReady() const;

    void clearRxDataReady();


    /*
     * HAL callback'lerinin cagiracagi
     * driver fonksiyonlari
     */
    void onTxComplete();

    void onRxComplete();

    void onError();


private:

    UART_HandleTypeDef* huart_{nullptr};

    bool isInitialized_{false};


    /*
     * TX DMA state
     */
    volatile bool txBusy_{false};


    /*
     * RX Interrupt state
     */
    volatile bool rxBusy_{false};

    volatile bool rxDataReady_{false};
};
