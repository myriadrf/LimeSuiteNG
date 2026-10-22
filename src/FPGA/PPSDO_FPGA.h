#pragma once

#include "limesuiteng/PPSDO.h"

#include <cstdint>
#include <memory>

namespace lime {

class ICSR;

/// PPSDO implemented in FPGA gateware, controlled through Configuration Space Registers.
class PPSDO_FPGA : public PPSDO
{
  public:
    /// @param csr Interface to the device's Configuration Space Registers.
    /// @param baseAddress Address of the PPSDO register block.
    PPSDO_FPGA(std::shared_ptr<ICSR> csr, uint64_t baseAddress);

    OpStatus Configure(const PPSDO::Config& config) override;
    OpStatus Enable(bool enable) override;
    OpStatus GetStatus(PPSDO::Status& status) override;

  private:
    uint64_t ReadRegister(uint32_t offset, OpStatus& status);
    OpStatus WriteRegister(uint32_t offset, uint64_t value);

    std::shared_ptr<ICSR> csr;
    uint64_t baseAddress;
};

} // namespace lime
