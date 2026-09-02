#pragma once

#include <cstddef>
#include <cstdint>

enum class CommandId : uint8_t
{
    ClearBuffer = 0x01,
    WriteImu    = 0x02,
    WriteGps    = 0x03,
    ReadData    = 0x04
};

struct BinaryPacket
{
    uint8_t cmdId{0U};
    uint8_t length{0U};
    uint8_t payload[100]{};
};

class ProtocolParser
{
public:
    static constexpr uint8_t Header1 = 0xA5U;
    static constexpr uint8_t Header2 = 0x5AU;
    static constexpr std::size_t MaxPayloadSize = 100U;

    // Paket:
    // A5 5A | CMD | LENGTH | DATA[LENGTH] | CHECKSUM_L | CHECKSUM_H
    //
    // CHECKSUM = Header1 + Header2 + CMD + LENGTH + tum DATA byte'lari
    bool pushByte(uint8_t byte)
    {
        if (packetReady_)
        {
            return false;
        }

        switch (state_)
        {
        case State::WaitHeader1:
            if (byte == Header1)
            {
                checksum_ = Header1;
                state_ = State::WaitHeader2;
            }
            break;

        case State::WaitHeader2:
            if (byte == Header2)
            {
                checksum_ += Header2;
                state_ = State::WaitCommand;
            }
            else
            {
                resetWorkingState();

                if (byte == Header1)
                {
                    checksum_ = Header1;
                    state_ = State::WaitHeader2;
                }
            }
            break;

        case State::WaitCommand:
            workingPacket_.cmdId = byte;
            checksum_ += byte;
            state_ = State::WaitLength;
            break;

        case State::WaitLength:
        {
            workingPacket_.length = byte;
            checksum_ += byte;
            payloadIndex_ = 0U;

            // Komuta gore beklenen LENGTH kontrolu
            bool validLength = false;

            switch (static_cast<CommandId>(workingPacket_.cmdId))
            {
            case CommandId::ClearBuffer:
            case CommandId::WriteImu:
            case CommandId::WriteGps:
                validLength = (workingPacket_.length == 0U);
                break;

            case CommandId::ReadData:
                validLength = (workingPacket_.length == 1U);
                break;

            default:
                validLength = false;
                break;
            }

            if (!validLength)
            {
                resetWorkingState();
                break;
            }

            state_ = (workingPacket_.length == 0U)
                         ? State::WaitChecksumLow
                         : State::WaitPayload;

            break;
        }


        case State::WaitPayload:
            workingPacket_.payload[payloadIndex_++] = byte;
            checksum_ += byte;

            if (payloadIndex_ >= workingPacket_.length)
            {
                state_ = State::WaitChecksumLow;
            }
            break;

        case State::WaitChecksumLow:
            receivedChecksum_ = byte;
            state_ = State::WaitChecksumHigh;
            break;

        case State::WaitChecksumHigh:
            receivedChecksum_ |= static_cast<uint16_t>(byte) << 8U;

            if (receivedChecksum_ == checksum_)
            {
                readyPacket_ = workingPacket_;
                packetReady_ = true;
            }
            else
            {
                checksumError_ = true;
            }

            resetWorkingState();
            break;
        }

        return packetReady_;
    }

    bool getPacket(BinaryPacket& packet)
    {
        if (!packetReady_)
        {
            return false;
        }

        packet = readyPacket_;
        packetReady_ = false;
        return true;
    }

    bool hasChecksumError() const
    {
        return checksumError_;
    }

    void clearChecksumError()
    {
        checksumError_ = false;
    }

    void reset()
    {
        packetReady_ = false;
        checksumError_ = false;
        resetWorkingState();
    }

private:
    enum class State : uint8_t
    {
        WaitHeader1,
        WaitHeader2,
        WaitCommand,
        WaitLength,
        WaitPayload,
        WaitChecksumLow,
        WaitChecksumHigh
    };

    void resetWorkingState()
    {
        state_ = State::WaitHeader1;
        workingPacket_ = BinaryPacket{};
        payloadIndex_ = 0U;
        checksum_ = 0U;
        receivedChecksum_ = 0U;
    }

    State state_{State::WaitHeader1};

    BinaryPacket workingPacket_{};
    BinaryPacket readyPacket_{};

    std::size_t payloadIndex_{0U};

    uint16_t checksum_{0U};
    uint16_t receivedChecksum_{0U};

    bool packetReady_{false};
    bool checksumError_{false};
};
