#include "LA9310.h"

#include "comms/PCIe/LA9310_PCIe.h"

namespace lime {

LA9310::LA9310(std::shared_ptr<LA9310_PCIe> port)
    : phytimer(port)
    , vspa(port)
    , pcie(port)
{
}

LA9310::~LA9310()
{
}

bool LA9310::IsM4CoreProgrammed()
{
    volatile uint8_t* BAR0_addr = reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR0).vaddr);
    volatile uint32_t* SCRATCHRW2 = reinterpret_cast<volatile uint32_t*>(BAR0_addr + 0x1E00000 + 0x204);
    return *SCRATCHRW2 != 0; // firmware booting stage
}

} // namespace lime