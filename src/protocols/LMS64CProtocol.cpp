/**
    @file LMS64CProtocol.cpp
    @author Lime Microsystems
    @brief Implementation of LMS64C protocol.
*/

#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRDescriptor.h"
#include "ISerialPort.h"
#include "LMS64CProtocol.h"
#include <chrono>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <ciso646> // alternative operators for visual c++: not, and, or...
#include "ADCUnits.h"
#include <cstring>
#include <iomanip>
#include <sstream>

using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

//! CMD_LMS7002_RST options
// const int LMS_RST_DEACTIVATE = 0;
// const int LMS_RST_ACTIVATE = 1;
const int LMS_RST_PULSE = 2;

using namespace std::literals::string_literals;
using namespace lime::LMS64CProtocol;

namespace lime {

LMS64CPacketMemoryWriteView::LMS64CPacketMemoryWriteView(LMS64CPacket* pkt)
    : packet(pkt)
{
}

void LMS64CPacketMemoryWriteView::SetMode(int mode)
{
    packet->payload[0] = mode;
}

void LMS64CPacketMemoryWriteView::SetChunkIndex(int index)
{
    packet->payload[1] = (index >> 24) & 0xFF;
    packet->payload[2] = (index >> 16) & 0xFF;
    packet->payload[3] = (index >> 8) & 0xFF;
    packet->payload[4] = index & 0xFF;
}

void LMS64CPacketMemoryWriteView::SetChunkSize(int size)
{
    packet->payload[5] = size;
}

void LMS64CPacketMemoryWriteView::SetAddress(int addr)
{
    packet->payload[6] = (addr >> 24) & 0xFF;
    packet->payload[7] = (addr >> 16) & 0xFF;
    packet->payload[8] = (addr >> 8) & 0xFF;
    packet->payload[9] = addr & 0xFF;
}

void LMS64CPacketMemoryWriteView::SetDevice(LMS64CProtocol::ProgramWriteTarget device)
{
    auto targetAsInteger = static_cast<int>(device);

    packet->payload[10] = (targetAsInteger >> 8) & 0xFF;
    packet->payload[11] = targetAsInteger & 0xFF;
}

void LMS64CPacketMemoryWriteView::SetData(const uint8_t* src, size_t len)
{
    assert(len <= 32);
    len = len > 32 ? 32 : len;
    memcpy(&packet->payload[24], src, len);
}

void LMS64CPacketMemoryWriteView::GetData(uint8_t* dest, size_t len) const
{
    assert(len <= 32);
    len = len > 32 ? 32 : len;
    memcpy(dest, &packet->payload[24], len);
}

constexpr size_t LMS64CPacketMemoryWriteView::GetMaxDataSize()
{
    return 32;
}

LMS64CPacketSerialCommandView::LMS64CPacketSerialCommandView(LMS64CPacket* pkt)
    : packet(pkt)
{
}

void LMS64CPacketSerialCommandView::SetStorageType(Storage type)
{
    packet->payload[0] = static_cast<uint8_t>(type);
}

void LMS64CPacketSerialCommandView::SetUnlockKey(uint8_t key)
{
    packet->payload[2] = static_cast<uint8_t>(key);
}

void LMS64CPacketSerialCommandView::SetSerial(const std::vector<uint8_t>& bytes)
{
    assert(bytes.size() <= 32);
    packet->payload[1] = bytes.size();
    memcpy(&packet->payload[24], bytes.data(), bytes.size());
}
void LMS64CPacketSerialCommandView::GetSerial(std::vector<uint8_t>& bytes) const
{
    const uint8_t bytesCount = packet->payload[1];
    bytes.resize(bytesCount);
    memcpy(bytes.data(), &packet->payload[24], bytesCount);
}
constexpr size_t LMS64CPacketSerialCommandView::GetMaxSerialLength()
{
    return 32;
}

namespace LMS64CProtocol {

static constexpr std::array<const std::string_view, static_cast<size_t>(CommandStatus::Count)> COMMAND_STATUS_TEXT = {
    "Undefined/Failure"sv,
    "Completed"sv,
    "Unknown command"sv,
    "Busy"sv,
    "Too many blocks"sv,
    "Error"sv,
    "Wrong order"sv,
    "Resource denied"sv,
};

static constexpr const std::string_view status2string(const CommandStatus status)
{
    if (status >= CommandStatus::Undefined && status < CommandStatus::Count)
        return COMMAND_STATUS_TEXT.at(static_cast<int>(status));
    return "Unknown status"sv;
}

static constexpr std::array<char, 16> ADC_UNITS_PREFIX = {
    ' ', 'k', 'M', 'G', 'T', 'P', 'E', 'Z', 'y', 'z', 'a', 'f', 'p', 'n', 'u', 'm'
};

static OpStatus SPI16(ISerialPort& port,
    uint8_t chipSelect,
    Command writeCmd,
    const uint32_t* MOSI,
    Command readCmd,
    uint32_t* MISO,
    size_t count,
    uint32_t subDevice)
{
    LMS64CPacket pkt;

    size_t srcIndex = 0;
    size_t destIndex = 0;
    constexpr int maxBlocks = LMS64CPacket::payloadSize / (sizeof(uint32_t) / sizeof(uint8_t)); // = 14
    while (srcIndex < count)
    {
        pkt.status = CommandStatus::Undefined;
        pkt.blockCount = 0;
        pkt.periphID = chipSelect;
        pkt.subDevice = subDevice;

        // fill packet with same direction operations
        const bool willDoWrite = MOSI[srcIndex] & (1 << 31);
        for (int i = 0; i < maxBlocks && srcIndex < count; ++i)
        {
            const bool isWrite = MOSI[srcIndex] & (1 << 31);
            if (isWrite != willDoWrite)
                break; // change between write/read, flush packet

            if (isWrite)
            {
                pkt.cmd = writeCmd;
                int payloadOffset = pkt.blockCount * 4;
                pkt.payload[payloadOffset + 0] = MOSI[srcIndex] >> 24;
                pkt.payload[payloadOffset + 1] = MOSI[srcIndex] >> 16;
                pkt.payload[payloadOffset + 2] = MOSI[srcIndex] >> 8;
                pkt.payload[payloadOffset + 3] = MOSI[srcIndex];
            }
            else
            {
                pkt.cmd = readCmd;
                int payloadOffset = pkt.blockCount * 2;
                pkt.payload[payloadOffset + 0] = MOSI[srcIndex] >> 8;
                pkt.payload[payloadOffset + 1] = MOSI[srcIndex];
            }
            ++pkt.blockCount;
            ++srcIndex;
        }

        int sent = 0;
        int recv = 0;

        sent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
        if (sent != sizeof(pkt))
            return OpStatus::IOFailure;

        recv = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 1000);
        if (recv != sizeof(pkt) || pkt.status != CommandStatus::Completed)
            return OpStatus::IOFailure;

        for (int i = 0; MISO && i < pkt.blockCount && destIndex < count; ++i)
        {
            //MISO[destIndex] = 0;
            //MISO[destIndex] = pkt.payload[0] << 24;
            //MISO[destIndex] |= pkt.payload[1] << 16;
            MISO[destIndex] = (pkt.payload[i * 4 + 2] << 8) | pkt.payload[i * 4 + 3];
            ++destIndex;
        }
    }

    return OpStatus::Success;
}

OpStatus GetFirmwareInfo(ISerialPort& port, FirmwareInfo& info, uint32_t subDevice)
{
    info.deviceId = LMS_DEV_UNKNOWN;
    info.expansionBoardId = EXP_BOARD_UNKNOWN;
    info.firmware = 0;
    info.hardware = 0;
    info.protocol = 0;
    info.boardSerialNumber = 0;

    LMS64CPacket pkt;
    pkt.cmd = Command::GET_INFO;
    pkt.subDevice = subDevice;
    int sent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (sent != sizeof(pkt))
    {
        return OpStatus::IOFailure;
    }

    int recv = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 1000);
    if (recv != sizeof(pkt) || pkt.status != CommandStatus::Completed)
    {
        return OpStatus::IOFailure;
    }

    info.firmware = pkt.payload[0];
    info.deviceId = pkt.payload[1];
    info.protocol = pkt.payload[2];
    info.hardware = pkt.payload[3];
    info.expansionBoardId = pkt.payload[4];
    info.boardSerialNumber = 0;

    for (int i = 10; i < 18; i++)
    {
        info.boardSerialNumber <<= 8;
        info.boardSerialNumber |= pkt.payload[i];
    }

    return OpStatus::Success;
}

void FirmwareToDescriptor(const FirmwareInfo& fw, SDRDescriptor& descriptor)
{
    if (fw.deviceId >= eLMS_DEV::LMS_DEV_COUNT)
    {
        char strTemp[64];
        std::snprintf(strTemp, sizeof(strTemp), "Unknown (0x%X)", fw.deviceId);
        descriptor.name = std::string(strTemp);
    }
    else
        descriptor.name = GetDeviceName(static_cast<eLMS_DEV>(fw.deviceId));
    if (fw.expansionBoardId >= eEXP_BOARD::EXP_BOARD_COUNT)
    {
        char strTemp[64];
        std::snprintf(strTemp, sizeof(strTemp), "Unknown (0x%X)", fw.expansionBoardId);
        descriptor.expansionName = std::string(strTemp);
    }
    else
        descriptor.expansionName = GetExpansionBoardName(static_cast<eEXP_BOARD>(fw.expansionBoardId));
    descriptor.firmwareVersion = std::to_string(fw.firmware);
    descriptor.hardwareVersion = std::to_string(fw.hardware);
    descriptor.protocolVersion = std::to_string(fw.protocol);
    descriptor.serialNumber = fw.boardSerialNumber;
}

OpStatus LMS7002M_SPI(ISerialPort& port, uint8_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, size_t count, uint32_t subDevice)
{
    return SPI16(port, chipSelect, Command::LMS7002_WR, MOSI, Command::LMS7002_RD, MISO, count, subDevice);
}

OpStatus FPGA_SPI(ISerialPort& port, const uint32_t* MOSI, uint32_t* MISO, size_t count, uint32_t subDevice)
{
    return SPI16(port, 0, Command::BRDSPI_WR, MOSI, Command::BRDSPI_RD, MISO, count, subDevice);
}

OpStatus ADF4002_SPI(ISerialPort& port, const uint32_t* MOSI, size_t count, uint32_t subDevice)
{
    // only writes are supported
    LMS64CPacket pkt;

    size_t srcIndex = 0;
    constexpr int maxBlocks = LMS64CPacket::payloadSize / (sizeof(uint32_t) / sizeof(uint8_t)); // = 14
    const int blockSize = 3;

    while (srcIndex < count)
    {
        pkt.cmd = Command::ADF4002_WR;
        pkt.status = CommandStatus::Undefined;
        pkt.blockCount = 0;
        pkt.periphID = 0;
        pkt.subDevice = subDevice;

        for (int i = 0; i < maxBlocks && srcIndex < count; ++i)
        {
            int payloadOffset = pkt.blockCount * blockSize;
            pkt.payload[payloadOffset + 0] = MOSI[srcIndex] >> 16;
            pkt.payload[payloadOffset + 1] = MOSI[srcIndex] >> 8;
            pkt.payload[payloadOffset + 2] = MOSI[srcIndex];
            ++pkt.blockCount;
            ++srcIndex;
        }

        int sent = 0;
        int recv = 0;

        sent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
        if (sent != sizeof(pkt))
            return OpStatus::IOFailure;

        recv = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 1000);
        if (recv != sizeof(pkt) || pkt.status != CommandStatus::Completed)
            return OpStatus::IOFailure;
    }
    return OpStatus::Success;
}

OpStatus I2C_Write(ISerialPort& port, uint32_t address, const uint8_t* mosi, size_t count)
{
    return OpStatus::NotImplemented;
}
OpStatus I2C_Read(ISerialPort& port, uint32_t address, uint8_t* data, size_t count)
{
    return OpStatus::NotImplemented;
}

OpStatus CustomParameterWrite(ISerialPort& port, const std::vector<CustomParameterIO>& parameters, uint32_t subDevice)
{
    LMS64CPacket pkt;
    std::size_t index = 0;

    while (index < parameters.size())
    {
        pkt.cmd = Command::ANALOG_VAL_WR;
        pkt.status = CommandStatus::Undefined;
        pkt.blockCount = 0;
        pkt.periphID = 0;
        pkt.subDevice = subDevice;
        int byteIndex = 0;
        constexpr int maxBlocks = LMS64CPacket::payloadSize / (sizeof(uint32_t) / sizeof(uint8_t)); // = 14

        while (pkt.blockCount < maxBlocks && index < parameters.size())
        {
            pkt.payload[byteIndex++] = parameters[index].id;
            int powerOf10 = 0;

            if (parameters[index].value > 65535.0 && (parameters[index].units != ""s))
                powerOf10 = log10(parameters[index].value / 65.536) / 3;

            if (parameters[index].value < 65.536 && (parameters[index].units != ""s))
                powerOf10 = log10(parameters[index].value / 65535.0) / 3;

            int unitsId = 0; // need to convert given units to their enum
            pkt.payload[byteIndex++] = unitsId << 4 | powerOf10;

            int value = parameters[index].value / pow(10, 3 * powerOf10);
            pkt.payload[byteIndex++] = (value >> 8);
            pkt.payload[byteIndex++] = (value & 0xFF);

            ++pkt.blockCount;
            ++index;
        }

        int sent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
        if (sent != sizeof(pkt))
            return OpStatus::IOFailure;

        int recv = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
        if (recv < pkt.headerSize || pkt.status != CommandStatus::Completed)
            return OpStatus::Busy;
    }

    return OpStatus::Success;
}

OpStatus CustomParameterRead(ISerialPort& port, std::vector<CustomParameterIO>& parameters, uint32_t subDevice)
{
    LMS64CPacket pkt;
    std::size_t index = 0;

    while (index < parameters.size())
    {
        pkt.cmd = Command::ANALOG_VAL_RD;
        pkt.status = CommandStatus::Undefined;
        pkt.blockCount = 0;
        pkt.periphID = 0;
        pkt.subDevice = subDevice;
        int byteIndex = 0;
        constexpr int maxBlocks = LMS64CPacket::payloadSize / (sizeof(uint32_t) / sizeof(uint8_t)); // = 14

        while (pkt.blockCount < maxBlocks && index < parameters.size())
        {
            pkt.payload[byteIndex++] = parameters[index].id;
            ++pkt.blockCount;
            ++index;
        }

        int sent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
        if (sent != sizeof(pkt))
            throw std::runtime_error("CustomParameterRead write failed"s);

        int recv = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
        if (recv < pkt.headerSize || pkt.status != CommandStatus::Completed)
            throw std::runtime_error("CustomParameterRead read failed"s);

        for (std::size_t i = 0; i < pkt.blockCount; ++i)
        {
            int unitsIndex = pkt.payload[i * 4 + 1];
            std::size_t parameterIndex = index - pkt.blockCount + i;

            if (unitsIndex & 0x0F)
                parameters[parameterIndex].units = ADC_UNITS_PREFIX[unitsIndex & 0x0F];

            parameters[parameterIndex].units += adcUnits2string((unitsIndex & 0xF0) >> 4);

            if ((unitsIndex & 0xF0) >> 4 == RAW)
                parameters[parameterIndex].value = static_cast<uint16_t>(pkt.payload[i * 4 + 2] << 8 | pkt.payload[i * 4 + 3]);
            else
            {
                parameters[parameterIndex].value = static_cast<int16_t>(pkt.payload[i * 4 + 2] << 8 | pkt.payload[i * 4 + 3]);

                if ((unitsIndex & 0xF0) >> 4 == TEMPERATURE)
                    parameters[parameterIndex].value /= 10;
            }
        }
    }

    return OpStatus::Success;
}

OpStatus ProgramWrite(ISerialPort& port,
    const char* data,
    size_t length,
    int prog_mode,
    ProgramWriteTarget device,
    ProgressCallback callback,
    uint32_t subDevice)
{
#ifndef NDEBUG
    auto t1 = std::chrono::high_resolution_clock::now();
#endif
    //erasing FLASH can take up to 3 seconds before reply is received
    const int progTimeout_ms = 5000;
    std::string progressMsg = "in progress..."s;
    bool abortProgramming = false;
    size_t bytesSent = 0;

    bool needsData = true;
    if (device == ProgramWriteTarget::FPGA && prog_mode == 2)
        needsData = false;
    if (device == ProgramWriteTarget::FX3 && (prog_mode == 0 || prog_mode == 1))
        needsData = false;

    Command cmd;
    if (device == ProgramWriteTarget::HPM || device == ProgramWriteTarget::FX3)
        cmd = Command::MEMORY_WR;
    else if (device == ProgramWriteTarget::FPGA)
        cmd = Command::ALTERA_FPGA_GW_WR;
    else
    {
        progressMsg = "Programming failed! Target device not supported"s;
        if (callback)
            callback(bytesSent, length, progressMsg);
        return ReportError(OpStatus::NotSupported, progressMsg);
    }

    LMS64CPacket packet;
    LMS64CPacket inPacket;
    packet.cmd = cmd;
    packet.blockCount = packet.payloadSize;
    packet.subDevice = subDevice;

    LMS64CPacketMemoryWriteView progView(&packet);

    const size_t chunkSize = 32;
    static_assert(chunkSize <= progView.GetMaxDataSize(), "chunk must fit into packet payload");
    const uint32_t chunkCount = length / chunkSize + (length % chunkSize > 0) + 1; // +1 programming end packet

    for (uint32_t chunkIndex = 0; chunkIndex < chunkCount && !abortProgramming; ++chunkIndex)
    {
        memset(packet.payload, 0, packet.payloadSize);
        progView.SetMode(prog_mode);
        progView.SetChunkIndex(chunkIndex);
        progView.SetChunkSize(std::min(length - bytesSent, chunkSize));

        if (cmd == Command::MEMORY_WR)
        {
            progView.SetAddress(0x0000);
            progView.SetDevice(device);
        }

        if (needsData)
        {
            memcpy(&packet.payload[24], data, chunkSize);
            data += chunkSize;
        }

        if (port.Write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet), progTimeout_ms) != sizeof(packet))
        {
            if (callback)
                callback(bytesSent, length, "Programming failed! Write operation failed");
            return OpStatus::IOFailure;
        }
        if (port.Read(reinterpret_cast<uint8_t*>(&inPacket), sizeof(inPacket), progTimeout_ms) != sizeof(inPacket))
        {
            if (callback)
                callback(bytesSent, length, "Programming failed! Read operation failed");
            return OpStatus::IOFailure;
        }

        if (inPacket.status != CommandStatus::Completed)
        {
            progressMsg = "Programming failed! "s + std::string{ status2string(inPacket.status) };
            if (callback)
                callback(bytesSent, length, progressMsg);
            return ReportError(OpStatus::Error, progressMsg);
        }
        bytesSent += packet.payload[5];
        if (needsData == false) //only one packet is needed to initiate bitstream from flash
        {
            bytesSent = length;
            break;
        }
        if (callback)
        {
            bool completed = chunkIndex == chunkCount - 1;
            if (completed)
                progressMsg = "Programming: completed"s;
            abortProgramming = callback(bytesSent, length, progressMsg);
            if (abortProgramming && !completed)
                return OpStatus::Aborted;
        }
    }
#ifndef NDEBUG
    auto t2 = std::chrono::high_resolution_clock::now();
    if ((device == ProgramWriteTarget::FPGA && prog_mode == 2) == false)
        lime::log(LogLevel::Info,
            "Programming finished, %li bytes sent! %li ms",
            length,
            std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
    else
        lime::log(LogLevel::Info, "FPGA configuring initiated"s);
#endif
    return OpStatus::Success;
}

OpStatus DeviceReset(ISerialPort& port, uint32_t socIndex, uint32_t subDevice)
{
    LMS64CPacket pkt;
    pkt.cmd = Command::LMS7002_RST;
    pkt.status = CommandStatus::Undefined;
    pkt.blockCount = 1;
    pkt.periphID = socIndex;
    pkt.subDevice = subDevice;

    pkt.payload[0] = LMS_RST_PULSE;

    int sent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (sent != sizeof(pkt))
        throw std::runtime_error("DeviceReset write failed"s);
    int recv = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (recv < pkt.headerSize || pkt.status != CommandStatus::Completed)
        throw std::runtime_error("DeviceReset read failed"s);
    return OpStatus::Success;
}

OpStatus GPIODirRead(ISerialPort& port, uint8_t* buffer, const size_t bufLength)
{
    if (bufLength > LMS64CPacket::payloadSize)
    {
        throw std::invalid_argument("Buffer is too big for one packet."s);
    }

    LMS64CPacket pkt;
    pkt.cmd = Command::GPIO_DIR_RD;
    pkt.blockCount = bufLength;

    int bytesSent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (bytesSent != sizeof(pkt))
    {
        throw std::runtime_error("GPIODirRead write failed"s);
    }

    int bytesReceived = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (bytesReceived < pkt.headerSize || pkt.status != CommandStatus::Completed)
    {
        throw std::runtime_error("GPIODirRead read failed"s);
    }

    for (size_t i = 0; i < bufLength; ++i)
    {
        buffer[i] = pkt.payload[i];
    }

    return OpStatus::Success;
}

OpStatus GPIORead(ISerialPort& port, uint8_t* buffer, const size_t bufLength)
{
    if (bufLength > LMS64CPacket::payloadSize)
    {
        throw std::invalid_argument("Buffer is too big for one packet."s);
    }

    LMS64CPacket pkt;
    pkt.cmd = Command::GPIO_RD;
    pkt.blockCount = bufLength;

    int bytesSent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (bytesSent != sizeof(pkt))
    {
        throw std::runtime_error("GPIORead write failed"s);
    }

    int bytesReceived = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (bytesReceived < pkt.headerSize || pkt.status != CommandStatus::Completed)
    {
        throw std::runtime_error("GPIORead read failed"s);
    }

    for (size_t i = 0; i < bufLength; ++i)
    {
        buffer[i] = pkt.payload[i];
    }

    return OpStatus::Success;
}

OpStatus GPIODirWrite(ISerialPort& port, const uint8_t* buffer, const size_t bufLength)
{
    if (bufLength > LMS64CPacket::payloadSize)
    {
        throw std::invalid_argument("Buffer is too big for one packet."s);
    }

    LMS64CPacket pkt;
    pkt.cmd = Command::GPIO_DIR_WR;
    pkt.blockCount = bufLength;

    for (size_t i = 0; i < bufLength; ++i)
    {
        pkt.payload[i] = buffer[i];
    }

    int bytesSent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (bytesSent != sizeof(pkt))
    {
        throw std::runtime_error("GPIODirWrite write failed"s);
    }

    int bytesReceived = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (bytesReceived < pkt.headerSize || pkt.status != CommandStatus::Completed)
    {
        throw std::runtime_error("GPIODirWrite read failed"s);
    }

    return OpStatus::Success;
}

OpStatus GPIOWrite(ISerialPort& port, const uint8_t* buffer, const size_t bufLength)
{
    if (bufLength > LMS64CPacket::payloadSize)
    {
        throw std::invalid_argument("Buffer is too big for one packet."s);
    }

    LMS64CPacket pkt;
    pkt.cmd = Command::GPIO_WR;
    pkt.blockCount = bufLength;

    for (size_t i = 0; i < bufLength; ++i)
    {
        pkt.payload[i] = buffer[i];
    }

    int bytesSent = port.Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (bytesSent != sizeof(pkt))
    {
        throw std::runtime_error("GPIOWrite write failed"s);
    }

    int bytesReceived = port.Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (bytesReceived < pkt.headerSize || pkt.status != CommandStatus::Completed)
    {
        throw std::runtime_error("GPIOWrite read failed"s);
    }

    return OpStatus::Success;
}

OpStatus MemoryWrite(ISerialPort& port, uint32_t address, const void* data, size_t dataLen, uint32_t subDevice)
{
    const int timeout_ms = 100;
    size_t bytesSent = 0;
    const uint8_t* src = static_cast<const uint8_t*>(data);

    LMS64CPacket packet;
    LMS64CPacket inPacket;
    packet.cmd = Command::MEMORY_WR;
    packet.blockCount = packet.payloadSize;
    packet.subDevice = subDevice;

    LMS64CPacketMemoryWriteView progView(&packet);

    const size_t chunkSize = 32;
    static_assert(chunkSize <= progView.GetMaxDataSize(), "chunk must fit into packet payload");
    const uint32_t chunkCount = dataLen / chunkSize + (dataLen % chunkSize > 0);

    for (uint32_t chunkIndex = 0; chunkIndex < chunkCount; ++chunkIndex)
    {
        memset(packet.payload, 0, packet.payloadSize);
        progView.SetMode(0);
        progView.SetChunkIndex(chunkIndex);
        progView.SetChunkSize(std::min(dataLen - bytesSent, chunkSize));

        progView.SetAddress(address + bytesSent);
        progView.SetDevice(ProgramWriteTarget::FPGA);

        progView.SetData(src, chunkSize);
        src += chunkSize;

        if (port.Write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet), timeout_ms) != sizeof(packet))
            return OpStatus::IOFailure;
        if (port.Read(reinterpret_cast<uint8_t*>(&inPacket), sizeof(inPacket), timeout_ms) != sizeof(inPacket))
            return OpStatus::IOFailure;

        if (inPacket.status != CommandStatus::Completed)
            return OpStatus::IOFailure;

        bytesSent += chunkSize;
    }
    return OpStatus::Success;
}

OpStatus MemoryRead(ISerialPort& port, uint32_t address, void* data, size_t dataLen, uint32_t subDevice)
{
    const int timeout_ms = 100;
    size_t bytesGot = 0;
    uint8_t* dest = static_cast<uint8_t*>(data);

    LMS64CPacket packet;
    LMS64CPacket inPacket;
    packet.cmd = Command::MEMORY_RD;
    packet.blockCount = 0;
    packet.subDevice = subDevice;
    memset(packet.payload, 0, packet.payloadSize);

    LMS64CPacketMemoryWriteView writeView(&packet);
    writeView.SetMode(0);
    writeView.SetDevice(ProgramWriteTarget::FPGA);

    const size_t chunkSize = 32;
    static_assert(chunkSize <= writeView.GetMaxDataSize(), "chunk must fit into packet payload");
    const uint32_t chunkCount = dataLen / chunkSize + (dataLen % chunkSize > 0);

    for (uint32_t chunkIndex = 0; chunkIndex < chunkCount; ++chunkIndex)
    {
        writeView.SetAddress(address + bytesGot);
        writeView.SetChunkSize(std::min(dataLen - bytesGot, chunkSize));

        if (port.Write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet), timeout_ms) != sizeof(packet))
            return OpStatus::IOFailure;
        if (port.Read(reinterpret_cast<uint8_t*>(&inPacket), sizeof(inPacket), timeout_ms) != sizeof(inPacket))
            return OpStatus::IOFailure;

        if (inPacket.status != CommandStatus::Completed)
            return OpStatus::IOFailure;
        LMS64CPacketMemoryWriteView readView(&inPacket);
        int bToGet = std::min(chunkSize, dataLen - bytesGot);
        readView.GetData(dest, bToGet);
        dest += chunkSize;
        bytesGot += chunkSize;
    }
    return OpStatus::Success;
}

OpStatus WriteSerialNumber(ISerialPort& port, const std::vector<uint8_t>& serialBytes)
{
    if (serialBytes.empty())
        return OpStatus::InvalidValue;
    std::vector<uint8_t> bytes = serialBytes;
    {
        std::vector<uint8_t> currentSerial;
        OpStatus readStatus = ReadSerialNumber(port, currentSerial);
        if (readStatus != OpStatus::Success)
            return readStatus;

        if (currentSerial.empty())
            return ReportError(OpStatus::NotSupported, "Serial number is not supported");
        for (uint8_t value : currentSerial)
            if (value != 0xFF)
                return ReportError(OpStatus::Error,
                    "One time programable serial number already set, additional attempts to write can corrupt the value");
        if (bytes.size() > currentSerial.size())
            return ReportError(OpStatus::OutOfRange, "Serial number to be written is too long");

        if (currentSerial.size() > bytes.size())
        {
            bytes.resize(currentSerial.size());
            std::stringstream ss;
            ss << "Padding serial number to:";
            for (size_t i = 0; i < bytes.size(); ++i)
                ss << std::setw(2) << std::setfill('0') << std::hex << bytes[i] << " ";
            lime::debug(ss.str());
        }
    }

    const int timeout_ms = 100;

    LMS64CPacket packet;
    memset(packet.payload, 0, packet.payloadSize);

    packet.cmd = Command::SERIAL_WR;
    packet.blockCount = 1;
    packet.subDevice = 0;

    if (bytes.size() > LMS64CPacketSerialCommandView::GetMaxSerialLength())
        return OpStatus::OutOfRange;

    LMS64CPacketSerialCommandView payloadView(&packet);
    const bool permanentWrite = true;
    if (permanentWrite)
    {
        payloadView.SetStorageType(LMS64CPacketSerialCommandView::Storage::OneTimeProgramable);
        payloadView.SetUnlockKey(0x5A);

        payloadView.SetSerial(bytes); // not necessary when sending just the key

        // OTP, has to be done in two packets:
        // 1. Send the correct key
        // 2. Send the serial number
        if (port.Write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet), timeout_ms) != sizeof(packet))
            return OpStatus::IOFailure;
        if (port.Read(reinterpret_cast<uint8_t*>(&packet), sizeof(packet), timeout_ms) != sizeof(packet))
            return OpStatus::IOFailure;
        if (packet.status != CommandStatus::Completed)
            return ReportError(OpStatus::Error, "Failed to send one time programming key");

        // key has been sent, continue to send the serial number bytes
        payloadView.SetStorageType(LMS64CPacketSerialCommandView::Storage::OneTimeProgramable);
        payloadView.SetUnlockKey(0x5A);
    }
    else
    {
        // payloadView.SetStorageType(LMS64CPacketSerialCommandView::Storage::NonVolatile);
        payloadView.SetStorageType(LMS64CPacketSerialCommandView::Storage::Volatile);
        payloadView.SetUnlockKey(0x00);
    }
    payloadView.SetSerial(bytes);

    if (port.Write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet), timeout_ms) != sizeof(packet))
        return OpStatus::IOFailure;
    if (port.Read(reinterpret_cast<uint8_t*>(&packet), sizeof(packet), timeout_ms) != sizeof(packet))
        return OpStatus::IOFailure;
    if (packet.status != CommandStatus::Completed)
        return OpStatus::Error;

    std::stringstream ss;
    ss << "Serial number written:";
    for (uint8_t b : bytes)
        ss << std::hex << b << " ";
    lime::info(ss.str());
    // wait for things to settle, otherwise reading serial immediately will return zeroes
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return OpStatus::Success;
}

OpStatus ReadSerialNumber(ISerialPort& port, std::vector<uint8_t>& data)
{
    const int timeout_ms = 100;

    LMS64CPacket packet;
    memset(packet.payload, 0, packet.payloadSize);

    packet.cmd = Command::SERIAL_RD;
    packet.blockCount = 1;
    packet.subDevice = 0;

    if (data.size() > LMS64CPacketSerialCommandView::GetMaxSerialLength())
        return OpStatus::OutOfRange;

    LMS64CPacketSerialCommandView payloadView(&packet);

    if (port.Write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet), timeout_ms) != sizeof(packet))
        return OpStatus::IOFailure;
    if (port.Read(reinterpret_cast<uint8_t*>(&packet), sizeof(packet), timeout_ms) != sizeof(packet))
        return OpStatus::IOFailure;

    if (packet.status != CommandStatus::Completed)
        return OpStatus::Error;
    data.clear();
    payloadView.GetSerial(data);
    return OpStatus::Success;
}

} // namespace LMS64CProtocol

} // namespace lime
