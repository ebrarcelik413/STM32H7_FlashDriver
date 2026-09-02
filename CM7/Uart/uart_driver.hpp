#pragma once

#include "iuart_driver.hpp"
#include "main.h"
#include "protocol_parser.hpp"

#include <cstddef>
#include <cstdint>

class UartDriver : public IUartDriver
{
public:
    static constexpr std::size_t RX_BUFFER_SIZE = 100U;

    explicit UartDriver(UART_HandleTypeDef* huart);

    ~UartDriver() override = default;

    // --------------------------------------------------------
    // IUartDriver interface
    // --------------------------------------------------------

    UartStatus init() override;

    UartStatus deinit() override;

    UartStatus send(
        const uint8_t* data,
        std::size_t length) override;

    UartStatus sendDMA(
        const uint8_t* data,
        std::size_t length) override;

    UartStatus receive(
        uint8_t* data,
        std::size_t length,
        uint32_t timeoutMs) override;


    // --------------------------------------------------------
    // 100-BYTE LINEAR FIFO
    // --------------------------------------------------------

    // Veriyi FIFO'nun sonuna ekler.
    // Tamami sigmiyorsa hicbir byte eklemez ve false doner.
    bool appendData(
        const uint8_t* data,
        std::size_t length);

    // FIFO'dan ilk N byte'i okur.
    // Yeterli veri yoksa false doner ve FIFO degismez.
    // Basariliysa kalan veriler basa kayar.
    bool readData(
        uint8_t* destination,
        std::size_t length);

    // Tek byte okumak icin yardimci fonksiyon.
    bool readByte(
        uint8_t& byte);

    // Buffer'da kac byte veri var?
    std::size_t getAvailableDataCount() const;

    // Buffer'da kac byte bos yer var?
    std::size_t getFreeSpace() const;

    // FIFO'yu tamamen temizler.
    void clear();

    // clear() ile ayni isi yapan compatibility helper.
    void flushRxBuffer();

    // Buffer'a sigmayan veri gelmis mi?
    bool isOverflow() const;


    // --------------------------------------------------------
    // PROTOCOL
    // --------------------------------------------------------

    // Parser tamamlanmis binary paket urettiyse true doner.
    bool getReceivedPacket(
        BinaryPacket& packet);

    bool hasProtocolChecksumError() const;

    void clearProtocolChecksumError();


    // --------------------------------------------------------
    // TX / CALLBACK STATE
    // --------------------------------------------------------

    bool isTxBusy() const;

    void onTxComplete();

    void onRxByteReceived();

    void onError();


private:
    // Bir sonraki UART byte'i icin RX interrupt baslatir.
    void startReceiveIT();

    UART_HandleTypeDef* huart_{nullptr};

    bool isInitialized_{false};

    volatile bool txBusy_{false};

    volatile bool isOverflow_{false};


    // --------------------------------------------------------
    // APPLICATION DATA FIFO
    //
    // Burada A5 5A CMD LENGTH CHECKSUM tutulmaz.
    // Sadece IMU/GPS gibi asil DATA tutulur.
    // --------------------------------------------------------

    uint8_t rxBuffer_[RX_BUFFER_SIZE]{};

    volatile std::size_t rxCount_{0U};


    // --------------------------------------------------------
    // RX INTERRUPT
    //
    // HAL_UART_Receive_IT() her seferinde 1 byte buraya alir.
    // Daha sonra bu byte ProtocolParser'a verilir.
    // --------------------------------------------------------

    uint8_t rxRawByte_{0U};


    // --------------------------------------------------------
    // BINARY PROTOCOL PARSER
    //
    // A5 5A CMD LENGTH DATA CHECKSUM
    // --------------------------------------------------------

    ProtocolParser parser_{};
};
