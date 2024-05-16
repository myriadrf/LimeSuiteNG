#include "USBDMA.h"

#include "comms/IDMA.h"
#include "protocols/DataPacket.h"
#include "USBGeneric.h"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

using namespace std::literals::string_literals;

namespace lime {

USBDMA::DirectionState::DirectionState(uint8_t endpoint, uint8_t* const buffer)
    : endpoint(endpoint)
    , buffer(buffer)
    , state()
{
}

USBDMA::DirectionState::~DirectionState()
{
    if (buffer != nullptr)
    {
        delete[] buffer;
    }
}

USBDMA::USBDMA(std::shared_ptr<USBGeneric> port, uint8_t rxEndpoint, uint8_t txEndpoint)
    : port(port)
    , rx(rxEndpoint, new uint8_t[GetBufferSize() * GetBufferCount()])
    , tx(txEndpoint, new uint8_t[GetBufferSize() * GetBufferCount()])
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
    rx.contextHandles.clear();
    rx.contextHandles.resize(port->GetBufferCount(), -1);
    rx.state = {};
    rx.state.isEnabled = true;

    for (rx.state.hardwareIndex = 0; rx.state.hardwareIndex < port->GetBufferCount(); rx.state.hardwareIndex++)
    {
        constexpr TRXDir direction{ TRXDir::Rx };
        SetContextHandle(direction,
            port->BeginDataXfer(GetIndexAddress(direction, GetTransferArrayIndexFromState(direction)),
                GetBufferSize(),
                GetEndpointAddress(direction)));
    }
}

void USBDMA::TxEnable()
{
    tx.contextHandles.clear();
    tx.contextHandles.resize(port->GetBufferCount(), -1);
    tx.state = {};
    tx.state.isEnabled = true;
}

void USBDMA::Disable(TRXDir direction)
{
    auto& dirState{ GetDirectionState(direction) };
    dirState.state.isEnabled = false;

    port->AbortEndpointXfers(GetEndpointAddress(direction));

    dirState.contextHandles.clear();
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
    return port->GetBufferCount();
}

uint8_t* const USBDMA::GetMemoryAddress(TRXDir direction) const
{
    return GetDirectionState(direction).buffer;
}

std::size_t USBDMA::GetTransferArrayIndexFromState(TRXDir direction)
{
    return GetDirectionState(direction).state.hardwareIndex % GetBufferCount();
}

std::size_t USBDMA::GetTransferArrayIndex(uint16_t index)
{
    return index % GetBufferCount();
}

int USBDMA::GetContextHandle(TRXDir direction)
{
    return GetDirectionState(direction).contextHandles.at(GetTransferArrayIndexFromState(direction));
}

int USBDMA::GetContextHandleFromIndex(TRXDir direction, uint16_t index)
{
    return GetDirectionState(direction).contextHandles.at(GetTransferArrayIndex(index));
}

void USBDMA::SetContextHandle(TRXDir direction, int handle)
{
    GetDirectionState(direction).contextHandles.at(GetTransferArrayIndexFromState(direction)) = handle;
}

uint8_t USBDMA::GetEndpointAddress(TRXDir direction)
{
    return GetDirectionState(direction).endpoint;
}

uint8_t* USBDMA::GetIndexAddress(TRXDir direction, uint16_t index)
{
    return GetDirectionState(direction).buffer + GetBufferSize() * GetTransferArrayIndex(index);
}

IDMA::DMAState USBDMA::GetState(TRXDir direction)
{
    return GetDirectionState(direction).state;
}

int USBDMA::SetStateReceive(DMAState state)
{
    constexpr auto direction{ TRXDir::Rx };

    if (!rx.state.isEnabled)
    {
        return 0;
    }

    while (state.softwareIndex != rx.state.softwareIndex)
    {
        rx.state.softwareIndex++;

        if (GetContextHandle(direction) != -1)
        {
            throw std::runtime_error("Asking for a transfer when not all transfers have not been completed"s);
        }

        SetContextHandle(direction,
            port->BeginDataXfer(GetIndexAddress(direction, GetTransferArrayIndexFromState(direction)),
                GetBufferSize(),
                GetEndpointAddress(direction)));
        rx.state.hardwareIndex++;
        rx.state.hardwareIndex &= 0xFFFF;
    }

    return 0;
}

int USBDMA::SetStateTransmit(DMAState state)
{
    constexpr TRXDir direction{ TRXDir::Tx };

    GetDirectionState(direction).state.softwareIndex = state.softwareIndex;

    if (!tx.state.isEnabled)
    {
        return 0;
    }

    while (tx.state.hardwareIndex != tx.state.softwareIndex)
    {
        if (!Wait(direction))
        {
            throw std::runtime_error("Asking for a transfer when not all transfers have not been completed"s);
        }

        SetContextHandle(direction,
            port->BeginDataXfer(GetIndexAddress(direction, GetTransferArrayIndexFromState(direction)),
                GetBufferSize(),
                GetEndpointAddress(direction)));
        tx.state.hardwareIndex++;
        tx.state.hardwareIndex &= 0xFFFF;
    }

    return 0;
}

int USBDMA::SetState(TRXDir direction, DMAState state)
{
    switch (direction)
    {
    case TRXDir::Rx:
        return SetStateReceive(state);
    case TRXDir::Tx:
        return SetStateTransmit(state);
    }
}

bool USBDMA::Wait(TRXDir direction)
{
    const auto index{ GetDirectionState(direction).state.softwareIndex };
    const auto handle{ GetContextHandleFromIndex(direction, index) };

    if (handle == -1) // No transfer here
    {
        return true;
    }

    if (!port->WaitForXfer(handle))
    {
        return false;
    }

    const auto address{ GetIndexAddress(direction, index) };
    const auto received{ port->FinishDataXfer(reinterpret_cast<uint8_t*>(address), GetBufferSize(), handle) };

    if (received != GetBufferSize())
    {
        throw std::runtime_error("Did not transfer all bytes"s);
    }

    GetDirectionState(direction).contextHandles.at(GetTransferArrayIndex(index)) = -1;

    return true;
}

void USBDMA::CacheFlush(
    [[maybe_unused]] TRXDir samplesDirection, [[maybe_unused]] DataTransferDirection dataDirection, [[maybe_unused]] uint16_t index)
{
    return;
}

} // namespace lime
