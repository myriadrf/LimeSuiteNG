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
    isRunning.store(false);
}

USBDMA::DirectionState::~DirectionState()
{
    std::unique_lock lck{ mutex };

    isRunning.store(false);
    cv.notify_all();

    if (sendThread.joinable())
    {
        sendThread.join();
    }

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
    std::unique_lock lock{ GetStateMutex(TRXDir::Rx) };

    rx.contextHandles.fill(-1);
    rx.state = {};
    rx.state.isEnabled = true;
    rx.isRunning.store(true);
    rx.sendThread = std::thread(&USBDMA::RxStartTransferThread, this);
}

void USBDMA::TxEnable()
{
    std::unique_lock lock{ GetStateMutex(TRXDir::Tx) };

    tx.contextHandles.fill(-1);
    tx.state = {};
    tx.state.isEnabled = true;
    tx.isRunning.store(true);
    tx.sendThread = std::thread(&USBDMA::TxStartTransferThread, this);
}

void USBDMA::Disable(TRXDir direction)
{
    auto& state{ GetDirectionState(direction) };
    state.state.isEnabled = false;
    state.isRunning.store(false);

    state.cv.notify_all();

    if (state.sendThread.joinable())
    {
        state.sendThread.join();
    }
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

std::mutex& USBDMA::GetStateMutex(TRXDir direction)
{
    return GetDirectionState(direction).mutex;
}

std::condition_variable& USBDMA::GetStateCV(TRXDir direction)
{
    return GetDirectionState(direction).cv;
}

std::size_t USBDMA::GetTransferArrayIndexFromState(TRXDir direction)
{
    return GetDirectionState(direction).state.hardwareIndex % USBGeneric::GetBufferCount();
}

std::size_t USBDMA::GetTransferArrayIndex(uint16_t index)
{
    return index % USBGeneric::GetBufferCount();
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

std::byte* USBDMA::GetIndexAddress(TRXDir direction, uint16_t index)
{
    return GetDirectionState(direction).buffer + GetBufferSize() * GetTransferArrayIndex(index);
}

IDMA::DMAState USBDMA::GetState(TRXDir direction)
{
    std::unique_lock lock{ GetStateMutex(direction) };

    return GetDirectionState(direction).state;
}

int USBDMA::SetState(TRXDir direction, DMAState state)
{
    std::unique_lock lock{ GetStateMutex(direction) };

    GetDirectionState(direction).state.softwareIndex = state.softwareIndex;

    GetStateCV(direction).notify_all();

    return 0;
}

bool USBDMA::Wait(TRXDir direction)
{
    std::unique_lock lck{ GetStateMutex(direction) };
    const auto handle{ GetContextHandle(direction) };

    if (handle == -1)
    {
        return false;
    }

    return !port->WaitForXfer(handle, 0);
}

static constexpr bool DoDirectionsMatch(const TRXDir samplesDirection, const DataTransferDirection dataDirection)
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
    std::unique_lock lck{ GetStateMutex(samplesDirection) };

    // logic: directions match - finish, directions mismatch - done using buffer send more data please
    if (!DoDirectionsMatch(samplesDirection, dataDirection))
    {
        GetDirectionState(samplesDirection).contextHandles.at(GetTransferArrayIndex(index)) = -1;
        GetStateCV(samplesDirection).notify_all();
        return;
    }

    const auto address{ GetIndexAddress(samplesDirection, index) };

    if (address == nullptr)
    {
        throw std::runtime_error("Address is null"s);
    }

    if (!port->WaitForXfer(GetContextHandleFromIndex(samplesDirection, index)))
    {
        throw std::runtime_error("Communication timeout"s);
    }

    const auto received{ port->FinishDataXfer(
        reinterpret_cast<uint8_t*>(address), GetBufferSize(), GetContextHandleFromIndex(samplesDirection, index)) };

    if (received != GetBufferSize())
    {
        throw std::runtime_error("Did not transfer all bytes"s);
    }
}

void USBDMA::RxStartTransferThread()
{
    constexpr auto direction{ TRXDir::Rx };
    while (rx.isRunning.load())
    {
        std::unique_lock lck{ rx.mutex };

        if (GetContextHandle(direction) != -1)
        {
            rx.cv.wait(lck);
            continue;
        }

        SetContextHandle(direction,
            port->BeginDataXfer(reinterpret_cast<uint8_t*>(GetIndexAddress(direction, GetTransferArrayIndexFromState(direction))),
                GetBufferSize(),
                GetEndpointAddress(direction)));
        rx.state.hardwareIndex++; // TODO: deal with potential overflow?
        std::this_thread::yield();
    }

    port->AbortEndpointXfers(GetEndpointAddress(direction));
}

void USBDMA::TxStartTransferThread()
{
    constexpr auto direction{ TRXDir::Tx };

    while (tx.isRunning.load())
    {
        std::unique_lock lck{ tx.mutex };

        if (GetContextHandle(direction) != -1 && tx.state.hardwareIndex == tx.state.softwareIndex)
        {
            tx.cv.wait(lck);
            continue;
        }

        SetContextHandle(direction,
            port->BeginDataXfer(reinterpret_cast<uint8_t*>(GetIndexAddress(direction, GetTransferArrayIndexFromState(direction))),
                GetBufferSize(),
                GetEndpointAddress(direction)));
        tx.state.hardwareIndex++; // TODO: deal with potential overflow?
        std::this_thread::yield();
    }

    port->AbortEndpointXfers(GetEndpointAddress(direction));
}

} // namespace lime
