#ifndef LIME_LA9310_H
#define LIME_LA9310_H

#include <string>
#include <memory>

#include "PHYTimer.h"
#include "VSPA_iqplayer.h"

struct la9310_hif;

namespace lime {

class LA9310_PCIe;

class LIME_API LA9310
{
  public:
    LA9310(std::shared_ptr<LA9310_PCIe> port);
    ~LA9310();

    OpStatus SetSystemClock(double sysClk_Hz, uint8_t adc_rate_mask, uint8_t dac_rate_mask);
    void GetADCDACRates(uint8_t* adc_rate_mask, uint8_t* dac_rate_mask);
    bool IsM4CoreProgrammed();

    PHYTimer phytimer;
    VSPA_iqplayer vspa;

  private:
    std::shared_ptr<LA9310_PCIe> pcie;
    volatile struct la9310_hif* hif; // host interface
};

} // namespace lime

#endif // LIME_LA9310_H
