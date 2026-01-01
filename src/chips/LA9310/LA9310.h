#ifndef LIME_LA9310_H
#define LIME_LA9310_H

#include <string>
#include <memory>

#include "PHYTimer.h"
#include "VSPA_iqplayer.h"

namespace lime {

class LA9310_PCIe;

class LIME_API LA9310
{
  public:
    LA9310(std::shared_ptr<LA9310_PCIe> port);
    ~LA9310();

    bool IsM4CoreProgrammed();

    PHYTimer phytimer;
    VSPA_iqplayer vspa;

  private:
    std::shared_ptr<LA9310_PCIe> pcie;
};

} // namespace lime

#endif // LIME_LA9310_H
