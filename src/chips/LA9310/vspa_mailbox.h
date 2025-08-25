#ifndef LIME_VSPA_MAILBOX_H
#define LIME_VSPA_MAILBOX_H

#include "comms/shiva/shiva_lime.h"

namespace lime {

class VSPA_MailBox
{
  public:
    VSPA_MailBox(std::shared_ptr<ShivaPCIE_lime> port);
    ~VSPA_MailBox();

    int Send(uint32_t core_id, uint32_t mbox_id, uint64_t value);
    int Receive(uint32_t core_id, uint32_t mbox_id, uint64_t* value = nullptr);

    void Clear(uint32_t core_idx, uint32_t mbox_id);

  private:
    void iowrite32(uint32_t value, uint64_t addr);
    uint32_t ioread32(uint64_t addr);
    std::shared_ptr<ShivaPCIE_lime> port;
    uint64_t vspa_addr;
};

} // namespace lime

#endif