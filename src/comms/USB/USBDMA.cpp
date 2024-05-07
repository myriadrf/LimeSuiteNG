#include "USBDMA.h"

#include "comms/IDMA.h"
#include "protocols/DataPacket.h"
#include "USBGeneric.h"

#include <cstdint>
#include <memory>
#include <string>

using namespace std::literals::string_literals;

namespace lime {

USBDMA::DirectionState::DirectionState(uint8_t endpoint, std::byte* buffer)
    : endpoint(endpoint)
    , buffer(buffer)
    , state()
{
    contextHandles.fill(-1);
}

USBDMA::DirectionState::~DirectionState()
{
    if (buffer != nullptr)
    {
        delete[] buffer;
        buffer = nullptr;
    }
}

USBDMA::USBDMA(std::shared_ptr<USBGeneric> port, uint8_t rxEndpoint, uint8_t txEndpoint)
    : port(port)
    , rx(rxEndpoint, new std::byte[GetBufferSize() * GetBufferCount()])
    , tx(txEndpoint, new std::byte[GetBufferSize() * GetBufferCount()])
{
}

USBDMA::DirectionState& USBDMA::GetDirectionState(TRXDir direction)
{
    switch (direction)
    {
    case TRXDir::Rx:
        return rx;
    case TRXDir::Tx:
        return tx;
    }
}

const USBDMA::DirectionState& USBDMA::GetDirectionState(TRXDir direction) const
{
    switch (direction)
    {
    case TRXDir::Rx:
        return rx;
    case TRXDir::Tx:
        return tx;
    }
}

void USBDMA::RxEnable(uint32_t bufferSize, uint8_t irqPeriod)
{
    rx.state.isEnabled = true;
}

void USBDMA::TxEnable()
{
    tx.state.isEnabled = true;
}

void USBDMA::Disable(TRXDir direction)
{
    GetDirectionState(direction).state.isEnabled = false;
}

bool USBDMA::IsOpen() const
{
    return port->IsConnected();
}

inline int USBDMA::GetBufferSize() const
{
    static_assert(sizeof(FPGA_TxDataPacket) == sizeof(FPGA_RxDataPacket));
    return sizeof(FPGA_TxDataPacket);
}

inline int USBDMA::GetBufferCount() const
{
    return USBGeneric::GetBufferCount();
}

std::byte* USBDMA::GetMemoryAddress(TRXDir direction) const
{
    return GetDirectionState(direction).buffer;
}

std::size_t USBDMA::GetTransferArrayIndexFromState(TRXDir direction)
{
    return GetDirectionState(direction).state.hardwareIndex % USBGeneric::GetBufferCount();
}

constexpr std::size_t GetTransferArrayIndex(TRXDir direction, uint16_t index)
{
    return index % USBGeneric::GetBufferCount();
}

int USBDMA::GetContextHandle(TRXDir direction)
{
    return GetDirectionState(direction).contextHandles.at(GetTransferArrayIndexFromState(direction));
}

void USBDMA::SetContextHandle(TRXDir direction, int handle)
{
    GetDirectionState(direction).contextHandles.at(GetTransferArrayIndexFromState(direction)) = handle;
}

uint8_t USBDMA::GetEndpointAddress(TRXDir direction)
{
    return GetDirectionState(direction).endpoint;
}

uint32_t USBDMA::GetStateSoftwareIndex(TRXDir direction)
{
    return GetDirectionState(direction).state.softwareIndex;
}

void USBDMA::IncreaseHardwareIndex(TRXDir direction)
{
    const auto handle{ GetContextHandle(direction) };

    if (handle == -1) // invalid
    {
        return;
    }

    while (port->WaitForXfer(handle, 0))
    {
        GetDirectionState(direction).state.hardwareIndex++;
    }
}

IDMA::DMAState USBDMA::GetState(TRXDir direction)
{
    IncreaseHardwareIndex(direction);

    return GetDirectionState(direction).state;
}

void USBDMA::HandleIncreasedSoftwareIndex(TRXDir direction, uint32_t oldIndex)
{
    if (TRXDir::Rx == direction)
    {
        return;
    }

    const auto index{ GetStateSoftwareIndex(direction) };
    while (index > oldIndex)
    {
        const auto handle{ port->BeginDataXfer(
            reinterpret_cast<uint8_t*>(GetIndexAddress(direction, GetTransferArrayIndexFromState(direction))),
            GetBufferSize(),
            GetEndpointAddress(direction)) };
        SetContextHandle(direction, handle);
        oldIndex++;
    }
}

int USBDMA::SetState(TRXDir direction, DMAState state)
{
    const auto old{ GetStateSoftwareIndex(direction) };

    GetDirectionState(direction).state = state;

    HandleIncreasedSoftwareIndex(direction, old);
    return 0;
}

bool USBDMA::Wait(TRXDir direction)
{
    return port->WaitForXfer(GetTransferArrayIndexFromState(direction), 0);
}

std::byte* USBDMA::GetIndexAddress(TRXDir direction, uint16_t index)
{
    return GetDirectionState(direction).buffer + GetBufferSize() * GetTransferArrayIndex(direction, index);
}

static constexpr bool DoFlush(const TRXDir samplesDirection, const DataTransferDirection dataDirection)
{
    if (samplesDirection == TRXDir::Tx && dataDirection == DataTransferDirection::HostToDevice)
    {
        return true;
    }

    if (samplesDirection == TRXDir::Rx && dataDirection == DataTransferDirection::DeviceToHost)
    {
        return true;
    }

    return false;
}

void USBDMA::CacheFlush(TRXDir samplesDirection, DataTransferDirection dataDirection, uint16_t index)
{
    if (!DoFlush(samplesDirection, dataDirection))
    {
        return;
    }

    const auto address{ GetIndexAddress(samplesDirection, index) };

    if (address == nullptr)
    {
        return;
    }

    const auto received{ port->FinishDataXfer(
        reinterpret_cast<uint8_t*>(address), GetBufferSize(), GetContextHandle(samplesDirection)) };

    if (received != GetBufferSize())
    {
        throw std::runtime_error("Did not transfer all bytes"s);
    }

    SetContextHandle(samplesDirection, -1);
}

} // namespace lime
