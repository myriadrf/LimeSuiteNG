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

} // namespace lime