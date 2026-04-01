#include "CSR.h"
#include "comms/PCIe/LA9310_PCIe.h"

namespace lime {

LA9310_CSR::LA9310_CSR(std::shared_ptr<LA9310_PCIe> port)
    : port(port)
    , base_addr(reinterpret_cast<volatile uint32_t*>(port->GetBar(LA9310_WINDOW_BAR0).vaddr))
    , region_size(port->GetBar(LA9310_WINDOW_BAR0).size)
{
}

LA9310_CSR::~LA9310_CSR()
{
}

OpStatus LA9310_CSR::ioWrite64(uint64_t address, uint64_t value)
{
    if (!base_addr)
        return OpStatus::Error;

    if (address >= region_size)
        return OpStatus::OutOfRange;

    size_t wordIndex = address >> 2;
    base_addr[wordIndex] = value;
    return OpStatus::Success;
}

uint64_t LA9310_CSR::ioRead64(uint64_t address, OpStatus* status)
{
    constexpr uint64_t invalidValue = ~0x0;
    if (!base_addr)
    {
        if (status)
            *status = OpStatus::Error;
        return invalidValue;
    }

    if (address >= region_size)
    {
        if (status)
            *status = OpStatus::OutOfRange;
        return invalidValue;
    }

    size_t wordIndex = address >> 2;
    if (status)
        *status = OpStatus::Success;
    return base_addr[wordIndex];
}

} // namespace lime
