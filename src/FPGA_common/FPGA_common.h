/**
@file FPGA_common.h
@author Lime Microsystems
@brief Common functions used to work with FPGA
*/

#ifndef LIME_FPGA_H
#define LIME_FPGA_H

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "protocols/SamplesPacket.h"
#include "protocols/DataPacket.h"
#include "protocols/TRXLooper.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/OpStatus.h"

using namespace std::literals::string_literals;

namespace lime {
class ISPI;

/** @brief Class for interfacing with a field-programmable gate array (FPGA). */
class FPGA
{
  public:
    FPGA(std::shared_ptr<ISPI> fpgaSPI, std::shared_ptr<ISPI> lms7002mSPI);
    virtual ~FPGA(){};

    OpStatus StartStreaming();
    OpStatus StopStreaming();
    OpStatus ResetTimestamp();

    /** @brief Structure for holding FPGA's Phase-Locked Loop (PLL) clock information. */
    struct FPGA_PLL_clock {
        FPGA_PLL_clock()
            : outFrequency(0)
            , phaseShift_deg(0)
            , index(0)
            , bypass(false)
            , findPhase(false)
            , rd_actualFrequency(0)
        {
        }
        double outFrequency;
        double phaseShift_deg;
        uint8_t index;
        bool bypass;
        bool findPhase;
        double rd_actualFrequency;
    };

    virtual OpStatus SetInterfaceFreq(double f_Tx_Hz, double f_Rx_Hz, double txPhase, double rxPhase, int chipIndex = 0);
    virtual OpStatus SetInterfaceFreq(double f_Tx_Hz, double f_Rx_Hz, int chipIndex = 0);
    double DetectRefClk(double fx3Clk = 100e6);

    static int FPGAPacketPayload2Samples(
        const uint8_t* buffer, int bufLen, bool mimo, bool compressed, complex16_t* const* samples);
    static int FPGAPacketPayload2SamplesFloat(
        const uint8_t* buffer, int bufLen, bool mimo, bool compressed, complex32f_t* const* samples);
    static int Samples2FPGAPacketPayload(
        const complex16_t* const* samples, int samplesCount, bool mimo, bool compressed, uint8_t* buffer);
    static int Samples2FPGAPacketPayloadFloat(
        const complex32f_t* const* samples, int samplesCount, bool mimo, bool compressed, uint8_t* buffer);
    virtual void EnableValuesCache(bool enabled);
    virtual OpStatus WriteRegisters(const uint32_t* addrs, const uint32_t* data, unsigned cnt);
    virtual OpStatus ReadRegisters(const uint32_t* addrs, uint32_t* data, unsigned cnt);
    OpStatus WriteRegister(uint32_t addr, uint32_t val);
    int ReadRegister(uint32_t addr);
    OpStatus WriteLMS7002MSPI(const uint32_t* addr, uint32_t length);
    OpStatus ReadLMS7002MSPI(const uint32_t* addr, uint32_t* values, uint32_t length);

    uint32_t SetUpVariableRxSize(uint32_t packetSize, int payloadSize, int sampleSize, uint8_t chipId);

    /** @brief Structure containing the gateware information of the FPGA */
    struct GatewareInfo {
        int boardID; ///< The ID of the board.
        int version; ///< The version of the gateware.
        int revision; ///< The revision of the gateware.
        int hardwareVersion; ///< The version of the hardware of the device.
    };
    GatewareInfo GetGatewareInfo();
    static void GatewareToDescriptor(const FPGA::GatewareInfo& gw, SDRDescriptor& desc);

    enum class TestID {
        HostReferenceClock = 1 << 0,
        Si5351C = 1 << 1,
        VCTCXO = 1 << 2,
        ADF4002 = 1 << 3,
        GNSS = 1 << 4,
    };
    virtual OpStatus OEMTestSetup(TestID test, double timeout);

    OpStatus ConfigureSamplesStream(uint32_t channelsEnableMask, lime::DataFormat samplesFormat, bool ddr, bool trxiqpulse);
    OpStatus ResetPacketCounters(uint16_t chipId);
    OpStatus StopWaveformPlayback();
    OpStatus ReadTxPacketCounters(uint16_t chipId, uint32_t* fpgaTxPktIngressCount, uint32_t* fpgaTxPktDropCounter);

    OpStatus SelectModule(uint8_t chipIndex);
    OpStatus SubmoduleSPIEnableMask(uint16_t enableMask);

  protected:
    OpStatus WaitTillDone(uint16_t pollAddr, uint16_t doneMask, uint16_t errorMask, const std::string& title = ""s);
    virtual OpStatus SetPllFrequency(uint8_t pllIndex, double inputFreq, std::vector<FPGA_PLL_clock>& outputs);
    OpStatus SetDirectClocking(int clockIndex);
    std::shared_ptr<ISPI> fpgaPort;
    std::shared_ptr<ISPI> lms7002mPort;

  private:
    virtual int ReadRawStreamData(char* buffer, unsigned length, int epIndex, int timeout_ms);
    OpStatus SetPllClock(uint8_t clockIndex, int nSteps, bool waitLock, bool doPhaseSearch, uint16_t& reg23val);
    bool useCache;
    std::map<uint16_t, uint16_t> regsCache;
};

} // namespace lime
#endif // LIME_FPGA_H
