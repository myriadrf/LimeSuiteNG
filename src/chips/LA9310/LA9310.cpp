#include "LA9310.h"

#include "comms/PCIe/LA9310_PCIe.h"

namespace lime {

LA9310::LA9310(std::shared_ptr<LA9310_PCIe> port)
    : pcie(port)
    , vspa(port)
    , phytimer(reinterpret_cast<uint64_t>(port->GetBar(LA9310_WINDOW_BAR0).vaddr) + 0x1020000)
{
}

LA9310::~LA9310()
{
}

} // namespace lime