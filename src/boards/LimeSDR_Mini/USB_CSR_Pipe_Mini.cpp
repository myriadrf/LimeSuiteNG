#include "USB_CSR_Pipe_Mini.h"

#include "comms/USB/FT601/FT601.h"

using namespace lime;

static const int CONTROL_BULK_WRITE_ADDRESS = 0x02;
static const int CONTROL_BULK_READ_ADDRESS = 0x82;

USB_CSR_Pipe_Mini::USB_CSR_Pipe_Mini(FT601& port)
    : USB_CSR_Pipe()
    , port(port)
{
}

int USB_CSR_Pipe_Mini::Write(const uint8_t* data, size_t length, int timeout_ms)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    return port.BulkTransfer(CONTROL_BULK_WRITE_ADDRESS, const_cast<uint8_t*>(data), length, timeout_ms);
}

int USB_CSR_Pipe_Mini::Read(uint8_t* data, size_t length, int timeout_ms)
{
    return port.BulkTransfer(CONTROL_BULK_READ_ADDRESS, data, length, timeout_ms);
}

OpStatus USB_CSR_Pipe_Mini::RunControlCommand(uint8_t* data, size_t length, int timeout_ms)
{
    return RunControlCommand(data, data, length, timeout_ms);
}

OpStatus USB_CSR_Pipe_Mini::RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms)
{
    size_t len = Write(request, length, timeout_ms);

    if (len != length)
        return OpStatus::IOFailure;

    len = Read(response, length, timeout_ms);

    if (len != length)
        return OpStatus::IOFailure;

    return OpStatus::Success;
}
