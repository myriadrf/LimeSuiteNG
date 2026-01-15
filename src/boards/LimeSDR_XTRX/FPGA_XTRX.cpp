#include "FPGA_XTRX.h"
#include "limesuiteng/Logger.h"
#include <ciso646>
#include <vector>
#include <map>
#include <cmath>
#include <iostream>

using namespace std::literals::string_literals;

namespace lime {

FPGA_XTRX::FPGA_XTRX(std::shared_ptr<ISPI> fpgaSPI, std::shared_ptr<ISPI> lms7002mSPI)
    : FPGA(fpgaSPI, lms7002mSPI)
{
    GatewareFeatures f;
    f.hasConfigurableStreamPacketSize = true;
    SetFeatures(f);
}

OpStatus FPGA_XTRX::EnableDirectClocking(bool rxDirectClocking, bool txDirectClocking)
{
    const bool isFairwavesRev5 = mHardwareVersion == 0;
    const bool noDirectClocking =
        mGatewareVersion == 1 && ((isFairwavesRev5 && mGatewareRevision < 4) || (!isFairwavesRev5 && mGatewareRevision < 15));
    if ((rxDirectClocking || txDirectClocking) && noDirectClocking)
    {
        return ReportError(
            OpStatus::NotSupported, "FPGA_XTRX: current gateware does not support sample rates <5 MHz, please update gateware."s);
    }
    uint16_t reg = (rxDirectClocking << 1) | txDirectClocking;
    return WriteRegister(0x0005, reg);
}

OpStatus FPGA_XTRX::SetInterfaceFreq(double txRate_Hz, double rxRate_Hz, int chipIndex)
{
    const bool rxDirectClock = rxRate_Hz < 5e6;
    const bool txDirectClock = txRate_Hz < 5e6;
    OpStatus status = EnableDirectClocking(rxDirectClock, txDirectClock);
    if (rxDirectClock)
    {
        lime::info("FPGA Rx direct clocking: %g", rxRate_Hz);
        rxRate_Hz = 0;
    }
    if (txDirectClock)
    {
        lime::info("FPGA Tx direct clocking: %g", txRate_Hz);
        txRate_Hz = 0;
    }
    if (rxDirectClock && txDirectClock)
        return status;

    return FPGA::SetInterfaceFreq(txRate_Hz, rxRate_Hz, chipIndex);
}

OpStatus FPGA_XTRX::SetPllFrequency(const uint8_t pllIndex, const double inputFreq, std::vector<FPGA_PLL_clock>& clocks)
{
    //Xilinx boards have different phase control mechanism
    double phase = clocks.at(1).phaseShift_deg;
    WriteRegister(0x0020, phase);
    return FPGA::SetPllFrequency(pllIndex, inputFreq, clocks);
}

} //namespace lime
