#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"

#include "limesuiteng/Logger.h"

using namespace lime;
using namespace std::literals::string_literals;

StreamConfig::Extras::Extras()
    : usePoll{ true }
    , negateQ{ false }
    , waitPPS{ false }
{
}

StreamConfig::Extras::PacketTransmission::PacketTransmission()
    : samplesInPacket{ 0 }
    , packetsInBatch{ 0 }
{
}

StreamConfig::StreamConfig()
    : format{ DataFormat::I16 }
    , linkFormat{ DataFormat::I16 }
    , bufferSize{ 0 }
    , hintSampleRate{ 0 }
    , alignPhase{ false }
    , statusCallback{ nullptr }
    , userData{ nullptr }
    , extraConfig{}
{
    channels[TRXDir::Rx] = {};
    channels[TRXDir::Tx] = {};
}

SDRDevice::~SDRDevice()
{
}

OpStatus SDRDevice::UploadTxWaveform(const StreamConfig& config, uint8_t moduleIndex, const void** samples, uint32_t count)
{
    return OpStatus::NotImplemented;
}

OpStatus SDRDevice::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return ReportError(OpStatus::NotImplemented, "TransactSPI not implemented"s);
}

OpStatus SDRDevice::I2CWrite(int address, const uint8_t* data, uint32_t length)
{
    return ReportError(OpStatus::NotImplemented, "WriteI2C not implemented"s);
}

OpStatus SDRDevice::I2CRead(int addr, uint8_t* dest, uint32_t length)
{
    return ReportError(OpStatus::NotImplemented, "ReadI2C not implemented"s);
}

OpStatus SDRDevice::GPIOWrite(const uint8_t* buffer, const size_t bufLength)
{
    return ReportError(OpStatus::NotImplemented, "GPIOWrite not implemented"s);
}

OpStatus SDRDevice::GPIORead(uint8_t* buffer, const size_t bufLength)
{
    return ReportError(OpStatus::NotImplemented, "GPIORead not implemented"s);
}

OpStatus SDRDevice::GPIODirWrite(const uint8_t* buffer, const size_t bufLength)
{
    return ReportError(OpStatus::NotImplemented, "GPIODirWrite not implemented"s);
}

OpStatus SDRDevice::GPIODirRead(uint8_t* buffer, const size_t bufLength)
{
    return ReportError(OpStatus::NotImplemented, "GPIODirRead not implemented"s);
}

OpStatus SDRDevice::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return ReportError(OpStatus::NotImplemented, "CustomParameterWrite not implemented"s);
}

OpStatus SDRDevice::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return ReportError(OpStatus::NotImplemented, "CustomParameterRead not implemented"s);
}

void SDRDevice::SetMessageLogCallback(LogCallbackType callback)
{
}

OpStatus SDRDevice::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    return OpStatus::NotImplemented;
}

OpStatus SDRDevice::MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data)
{
    return OpStatus::NotImplemented;
}

OpStatus SDRDevice::MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data)
{
    return OpStatus::NotImplemented;
}

void SDRDevice::StreamStart(const std::vector<uint8_t>& moduleIndexes)
{
    for (uint8_t i : moduleIndexes)
        StreamStart(i);
}

void SDRDevice::StreamStop(const std::vector<uint8_t>& moduleIndexes)
{
    for (uint8_t i : moduleIndexes)
        StreamStop(i);
}

OpStatus SDRDevice::OEMTest(OEMTestReporter* reporter)
{
    return OpStatus::NotImplemented;
}

OpStatus SDRDevice::WriteSerialNumber(uint64_t serialNumber)
{
    return OpStatus::NotImplemented;
}
