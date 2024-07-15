#ifndef FPGA_5G_H
#define FPGA_5G_H

#include "FPGA/FPGA_common.h"

namespace lime {

class ISPI;

/** @brief Class for communicating with the Xilinx XC7A50T-2CPG236I FPGA in the LimeSDR XTRX. */
class FPGA_XTRX : public FPGA
{
  public:
    FPGA_XTRX(std::shared_ptr<ISPI> fpgaSPI, std::shared_ptr<ISPI> lms7002mSPI);
    OpStatus SetInterfaceFreq(double txRate_Hz, double rxRate_Hz, int chipIndex = 0) override;

  private:
    OpStatus EnableDirectClocking(bool enabled);
    OpStatus SetInterfaceFreq(double f_Tx_Hz, double f_Rx_Hz, double txPhase, double rxPhase, int chipIndex = 0) override;
    OpStatus SetPllFrequency(const uint8_t pllIndex, const double inputFreq, std::vector<FPGA_PLL_clock>& clocks) override;
};

} // namespace lime
#endif // FPGA_Q_H
