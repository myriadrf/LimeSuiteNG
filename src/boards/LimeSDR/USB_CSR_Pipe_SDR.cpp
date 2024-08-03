#include "USB_CSR_Pipe_SDR.h"

#include <cstdint>
#include <set>

#include "comms/USB/FX3/FX3.h"
#include "LMS64CProtocol.h"

using namespace lime;

static const std::set<LMS64CProtocol::Command> commandsToBulkTransfer = {
    LMS64CProtocol::Command::BRDSPI_WR,
    LMS64CProtocol::Command::BRDSPI_RD,
    LMS64CProtocol::Command::LMS7002_WR,
    LMS64CProtocol::Command::LMS7002_RD,
    LMS64CProtocol::Command::ANALOG_VAL_WR,
    LMS64CProtocol::Command::ANALOG_VAL_RD,
    LMS64CProtocol::Command::ADF4002_WR,
    LMS64CProtocol::Command::LMS7002_RST,
    LMS64CProtocol::Command::GPIO_DIR_WR,
    LMS64CProtocol::Command::GPIO_DIR_RD,
    LMS64CProtocol::Command::GPIO_WR,
    LMS64CProtocol::Command::GPIO_RD,
};

USB_CSR_Pipe_SDR::USB_CSR_Pipe_SDR(FX3& port)
    : USB_CSR_Pipe()
    , port(port)
{
}

int USB_CSR_Pipe_SDR::Write(const uint8_t* data, size_t length, int timeout_ms)
{
    const LMS64CPacket* pkt = reinterpret_cast<const LMS64CPacket*>(data);

    if (commandsToBulkTransfer.find(pkt->cmd) != commandsToBulkTransfer.end())
    {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        return port.BulkTransfer(FX3::CONTROL_BULK_OUT_ADDRESS, const_cast<uint8_t*>(data), length, timeout_ms);
    }

    constexpr int CTR_W_REQCODE = 0xC1;
    constexpr int CTR_W_VALUE = 0x0000;
    constexpr int CTR_W_INDEX = 0x0000;

    return port.ControlTransfer(FX3::CTR_WRITE_REQUEST_VALUE,
        CTR_W_REQCODE,
        CTR_W_VALUE,
        CTR_W_INDEX,
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<uint8_t*>(data),
        length,
        timeout_ms);
}

int USB_CSR_Pipe_SDR::Read(uint8_t* data, size_t length, int timeout_ms)
{
    const LMS64CPacket* pkt = reinterpret_cast<const LMS64CPacket*>(data);

    if (commandsToBulkTransfer.find(pkt->cmd) != commandsToBulkTransfer.end())
    {
        return port.BulkTransfer(FX3::CONTROL_BULK_IN_ADDRESS, data, length, timeout_ms);
    }

    constexpr int CTR_R_REQCODE = 0xC0;
    constexpr int CTR_R_VALUE = 0x0000;
    constexpr int CTR_R_INDEX = 0x0000;

    return port.ControlTransfer(FX3::CTR_READ_REQUEST_VALUE, CTR_R_REQCODE, CTR_R_VALUE, CTR_R_INDEX, data, length, timeout_ms);
}

OpStatus USB_CSR_Pipe_SDR::RunControlCommand(uint8_t* data, size_t length, int timeout_ms)
{
    return RunControlCommand(data, data, length, timeout_ms);
}

OpStatus USB_CSR_Pipe_SDR::RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms)
{
    size_t len = Write(request, length, timeout_ms);

    if (len != length)
        return OpStatus::IOFailure;

    len = Read(response, length, timeout_ms);

    if (len != length)
        return OpStatus::IOFailure;

    return OpStatus::Success;
}
