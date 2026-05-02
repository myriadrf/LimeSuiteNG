#ifndef LIME_VSPA_mailbox_H
#define LIME_VSPA_mailbox_H

#include <memory>
#include <mutex>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/config.h"

namespace lime {

class LA9310_PCIe;
class PCIe_CSR_Access;

class LIME_API VSPA_mailbox
{
  public:
    VSPA_mailbox(std::shared_ptr<LA9310_PCIe> port);

    void Send(uint32_t core_id, uint32_t mbox_id, uint64_t value);
    OpStatus Receive(uint32_t core_id, uint32_t mbox_id, uint64_t* value = nullptr);

    void Clear(uint32_t core_idx, uint32_t mbox_id);

    OpStatus Message(uint32_t core_id, uint32_t mbox_id, uint64_t message, uint64_t* response = nullptr);

  private:
    std::shared_ptr<LA9310_PCIe> port;
    std::recursive_mutex mailbox_mutex;
    std::shared_ptr<PCIe_CSR_Access> vspa_ccsr_base;
};

} // namespace lime

#endif