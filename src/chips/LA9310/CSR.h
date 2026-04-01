#ifndef LIME_LA9310_CSR_H
#define LIME_LA9310_CSR_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include "comms/ICSR.h"

#include <cstdint>
#include <memory>

namespace lime {

class LA9310_PCIe;

class LA9310_CSR : public lime::ICSR
{
  public:
    LA9310_CSR() = delete;
    LA9310_CSR(std::shared_ptr<LA9310_PCIe> port);
    virtual ~LA9310_CSR();
    virtual OpStatus ioWrite64(uint64_t address, uint64_t value) override;
    virtual uint64_t ioRead64(uint64_t address, OpStatus* status = nullptr) override;

  private:
    std::shared_ptr<LA9310_PCIe> port;
    volatile uint32_t* base_addr;
    const uint32_t region_size;
};

} // namespace lime

#endif // LIME_LA9310_CSR_H
