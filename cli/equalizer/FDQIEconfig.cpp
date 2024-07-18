/* ************************************************************************
   FILE:	FDQIEconfig.c
   COMMENT:	Connection with gateware
   CONTENT:
   AUTHOR:	Lime Microsystems
   DATE:	May 18, 2020
   REVISION:
   ************************************************************************ */
// #define VERBOSE

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <chrono>
#include "fft.h"
#include "qfft.h"

#include "INI.h"

#include "FDQIdesign.h"
#include "limesuiteng/limesuiteng.hpp"
#include "limesuiteng/LMS7002M.h"
#include "FPGA/FPGA_common.h"
#include "DSP/CFR/CrestFactorReduction.h"

static constexpr double eps = 0.0005;
using namespace std;
using namespace lime;

static constexpr std::array<CSRegister, 36> CFRarray = { CFR::EN_RXTSP,
    CFR::EN_TXTSP,
    CFR::RX_DCCORR_BYP,
    CFR::RX_PHCORR_BYP,
    CFR::RX_GCORR_BYP,
    CFR::RX_EQU_BYP,
    CFR::RX_DCLOOP_BYP,
    CFR::RX_DCLOOP_AVG,
    CFR::TX_HB_BYP,
    CFR::TX_HB_DEL,
    CFR::SLEEP_CFR,
    CFR::BYPASS_CFR,
    CFR::ODD_CFR,
    CFR::BYPASSGAIN_CFR,
    CFR::SLEEP_FIR,
    CFR::BYPASS_FIR,
    CFR::ODD_FIR,
    CFR::TX_PHCORR_BYP,
    CFR::TX_GCORR_BYP,
    CFR::TX_DCCORR_BYP,
    CFR::TX_ISINC_BYP,
    CFR::TX_EQU_BYP,
    CFR::TX_INVERTQ,
    CFR::TX_GCORRQ,
    CFR::TX_GCORRI,
    CFR::TX_PHCORR,
    CFR::TX_DCCORRI,
    CFR::TX_DCCORRQ,
    CFR::thresholdSpin,
    CFR::thresholdGain,
    CFR::CFR_ORDER,
    CFR::RX_GCORRQ,
    CFR::RX_GCORRI,
    CFR::RX_PHCORR,
    CFR::cmbInsel,
    CFR::MAC };

static std::string stringFormat [[gnu::format(printf, 1, 2)]] (const char *format, ...)
{
    char buff[4096];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);
    return std::string(buff);
}

int terminateProgram(lime::SDRDevice *device)
{
    if (device != NULL)
        DeviceRegistry::freeDevice(device);

    printf("Error\n");
    exit(-1);
}

lime::OpStatus SetRegValue(lime::ISPI *fpga_spi, const uint16_t addr, uint16_t value)
{
    const uint32_t mosi = (1 << 31) | addr << 16 | value;
    return fpga_spi->SPI(&mosi, nullptr, 1);
}

lime::OpStatus SetRegValue(lime::ISPI *fpga_spi, const Register &reg, uint16_t value)
{
    uint32_t mosi = reg.address;
    uint32_t miso = 0;
    lime::OpStatus status = fpga_spi->SPI(&mosi, &miso, 1);
    if (status != lime::OpStatus::Success)
        return status;
    const uint16_t regMask = bitMask(reg.msb, reg.lsb);

    uint32_t regValue = (miso & ~regMask);
    regValue |= ((value << reg.lsb) & regMask);
    mosi = (1 << 31) | reg.address << 16 | regValue;
    return fpga_spi->SPI(&mosi, nullptr, 1);
}

uint16_t GetRegValue(lime::ISPI *fpga_spi, uint16_t addr)
{
    const uint32_t mosi = addr;
    uint32_t miso = 0;
    fpga_spi->SPI(&mosi, &miso, 1);
    return miso & 0xFFFF;
}

void FDQIE_LoopbackSettings(lime::SDRDevice *sdrdev, lime::ISPI *fpga_spi, int chA, int operationPhase) // FDQIE_SetupMeasurement
{
    SetRegValue(fpga_spi, 0xFFFF, (chA == 1) ? 0x1 : 0x2);

    Register cmbLms2Trx1T, cmbLms2Trx1, cmbLms2Rx1In, cmbLms2Rx1C, cmbLms3Rx1, chkLms2Tx1, chkLms2Lna1;

    // RF2 (log. 0), RF1 (log. 1)
    // LMS#2 channel A

    cmbLms2Trx1T = Register(0x00D1, 7, 7);
    //  cmbLms2Trx1T->Append("TX1_1(LNA)->RFSW_TX1OUT");  // RF2 (log. 0)
    //  cmbLms2Trx1T->Append("TX1_1(LNA)->Ground"); // RF1 (log. 1)

    cmbLms2Trx1 = Register(0x00D1, 6, 6);
    // cmbLms2Trx1->Append("RFSW_RX1IN<-TRX1(J8)"); // RF2 (log. 0)
    // cmbLms2Trx1->Append("RFSW_TX1OUT->TRX1(J8)"); // RF1 (log. 1)

    cmbLms2Rx1In = Register(0x00D1, 3, 3);
    // cmbLms2Rx1In->Append("RFSW_RX1<-RX1(J9)"); // RF2 (log. 0)
    // cmbLms2Rx1In->Append("RFSW_RX1<-RFSW_RX1IN"); // RF1 (log. 1)

    cmbLms2Rx1C = Register(0x00D1, 2, 2);
    // cmbLms2Rx1C->Append("RX1_H<-RFSW_RX1(LNA)");  // RF2 (log. 0)
    // cmbLms2Rx1C->Append("RX1_H<-TX1_1(LMS3)"); // RF1 (log. 1)

    cmbLms3Rx1 = Register(0x00D1, 0, 0);
    // cmbLms3Rx1->Append("RX1_H(LMS3)<-RX1(J4)"); // RF2 (log. 0)
    // cmbLms3Rx1->Append("RX1_H(LMS3)<-TX1_1(LMS2)");  // RF1 (log. 1)

    chkLms2Tx1 = Register(0x00D2, 3, 3); // positive logic
    chkLms2Lna1 = Register(0x00D2, 1, 1); // negative logic

    Register cmbLms2Trx2T, cmbLms2Trx2, cmbLms2Rx2In, cmbLms2Rx2C, cmbLms3Rx2, chkLms2Tx2, chkLms2Lna2;

    // LMS#2 channelB

    cmbLms2Trx2T = Register(0x00D1, 9, 9);
    // cmbLms2Trx2T->Append("TX2_1(LNA)->Ground"); // RF2 (log. 0)
    // cmbLms2Trx2T->Append("TX2_1(LNA)->RFSW_TX2OUT"); // RF1 (log. 1)

    cmbLms2Trx2 = Register(0x00D1, 8, 8);
    // cmbLms2Trx2->Append("RFSW_RX2IN<-TRX2(J10)"); // RF2 (log. 0)
    // cmbLms2Trx2->Append("RFSW_TX2OUT->TRX2(J10)"); // RF1 (log. 1)

    cmbLms2Rx2In = Register(0x00D1, 5, 5);
    // cmbLms2Rx2In->Append("RFSW_RX2<-RX2(J11)"); // RF2 (log. 0)
    // cmbLms2Rx2In->Append("RFSW_RX2<-RFSW_RX2IN"); // RF1 (log. 1)

    cmbLms2Rx2C = Register(0x00D1, 4, 4);
    // cmbLms2Rx2C->Append("RX2_H<-RFSW_RX2(LNA)"); // RF2 (log. 0)
    // cmbLms2Rx2C->Append("RX2_H<-TX2_1(LMS3)"); // RF1 (log. 1)

    cmbLms3Rx2 = Register(0x00D1, 1, 1);
    // cmbLms3Rx2->Append("RX2_H(LMS3)<-RX2(J5)");  // RF2 (log. 0)
    // cmbLms3Rx2->Append("RX1_H(LMS3)<-TX2_1(LMS2)");  // RF1 (log. 1)

    chkLms2Tx2 = Register(0x00D2, 2, 2); // positive logic
    chkLms2Lna2 = Register(0x00D2, 0, 0); // negative logic

    // default  channel A
    SetRegValue(fpga_spi, cmbLms2Trx1T, 0); // TX1_1(LNA)->RFSW_TX1OUT
    SetRegValue(fpga_spi, cmbLms2Trx1, 1); // RFSW_TX1OUT->TRX1(J8) ???
    SetRegValue(fpga_spi, cmbLms2Rx1In, 0); // RFSW_RX1<-RX1(J9)
    SetRegValue(fpga_spi, cmbLms2Rx1C, 0); // RX1_H<-RFSW_RX1
    SetRegValue(fpga_spi, cmbLms3Rx1, 1); // RX1_H(LMS3)<-RX1(J4)
    SetRegValue(fpga_spi, chkLms2Tx1, 1); // ON
    SetRegValue(fpga_spi, chkLms2Lna1, 0); // ON

    // default  channel B
    SetRegValue(fpga_spi, cmbLms2Trx2T, 1); // TX2_1(LNA)->RFSW_TX2OUT
    SetRegValue(fpga_spi, cmbLms2Trx2, 1); // RFSW_TX2OUT->TRX2(J10)
    SetRegValue(fpga_spi, cmbLms2Rx2In, 0); // RFSW_RX2<-RX2(J11)
    SetRegValue(fpga_spi, cmbLms2Rx2C, 0); // RX2_H<-RFSW_RX2
    SetRegValue(fpga_spi, cmbLms3Rx2, 1); // RX2_H(LMS3)<-RX2(J5)
    SetRegValue(fpga_spi, chkLms2Tx2, 1); // ON
    SetRegValue(fpga_spi, chkLms2Lna2, 0); // ON

    Register reg;
    reg = Register(0x0080, 2, 2); // NCO switch
    SetRegValue(fpga_spi, reg, 0); // signal

    if (operationPhase == 0)
    {
        FDQIE_SetupTransmitterTone(sdrdev, 0);
    }

    if (chA == 1)
    {
        if ((operationPhase == 1) || (operationPhase == 2))
        {
            SetRegValue(fpga_spi, cmbLms2Trx1T, 1); // TX1_1(LNA)->Ground (50 Ohm)
            SetRegValue(fpga_spi, cmbLms2Trx1, 0); // RFSW_RX1IN<-TRX1(J8)
            SetRegValue(fpga_spi, cmbLms3Rx1, 1); // RX1_H(LMS3)<-TX1_1(LMS2)

            SetRegValue(fpga_spi, chkLms2Tx2, 0); // OFF
            SetRegValue(fpga_spi, chkLms2Lna1, 1); // OFF
            SetRegValue(fpga_spi, chkLms2Lna2, 1); // OFF

            // in operationPhase 1 LMS#2 SXT LO is changed, LMS#3 is receiving
            // in operationPhase 2 LMS#2 NCO is changed, LMS#3 is receiving
            SetRegValue(fpga_spi, reg, 1);
        }
        else if (operationPhase == 3)
        {
            // LMS#3 SXT LO is changed, LMS#2 is receiving  signal
            SetRegValue(fpga_spi, cmbLms2Trx1T, 1); // TX1_1(LNA)->Ground (50 Ohm)
            SetRegValue(fpga_spi, cmbLms2Rx1C, 1); // RX1_H<-TX1_1(LMS3)

            SetRegValue(fpga_spi, chkLms2Lna1, 1); // OFF
            SetRegValue(fpga_spi, chkLms2Lna2, 1); // OFF
            SetRegValue(fpga_spi, chkLms2Tx1, 0); // OFF
            SetRegValue(fpga_spi, chkLms2Tx2, 0); // OFF
        }
    }
    else
    {
        if ((operationPhase == 1) || (operationPhase == 2))
        {
            SetRegValue(fpga_spi, cmbLms2Trx2T, 0); // TX2_1(LNA)->Ground (50 Ohm)
            SetRegValue(fpga_spi, cmbLms2Trx2, 0); // RFSW_RX2IN<-TRX2(J10)
            SetRegValue(fpga_spi, cmbLms3Rx2, 1); // RX2_H(LMS3)<-TX2_1(LMS2)

            SetRegValue(fpga_spi, chkLms2Tx1, 0); // OFF
            SetRegValue(fpga_spi, chkLms2Lna1, 1); // OFF
            SetRegValue(fpga_spi, chkLms2Lna2, 1); // OFF

            // in operationPhase 1 LMS#2 SXT LO is changed, LMS#3 is receiving
            // in operationPhase 2 LMS#2 NCO is changed, LMS#3 is receiving
            SetRegValue(fpga_spi, reg, 1);
        }
        else if (operationPhase == 3)
        {
            // LMS#3 SXT LO is changed, LMS#2 is receiving  signal
            SetRegValue(fpga_spi, cmbLms2Trx2T, 0); // TX2_1(LNA)->Ground (50 Ohm)
            SetRegValue(fpga_spi, cmbLms2Rx2C, 1); // RX1_H<-TX1_1(LMS3)

            SetRegValue(fpga_spi, chkLms2Lna1, 1); // OFF
            SetRegValue(fpga_spi, chkLms2Lna2, 1); // OFF
            SetRegValue(fpga_spi, chkLms2Tx1, 0); // OFF
            SetRegValue(fpga_spi, chkLms2Tx2, 0); // OFF
        }
    }
}

void FDQIE_SetNCO(lime::ISPI *fpga_spi, double refClk_MHz, double value, bool chA)
{

    double ncoFreq_MHz;
    ncoFreq_MHz = value;

    SetRegValue(fpga_spi, 0xFFFF, (chA == true) ? 0x1 : 0x2);

    uint32_t fcw = (uint32_t)((ncoFreq_MHz / refClk_MHz) * 4294967296);
    vector<uint32_t> addrs = { 0x008E, 0x008F };
    vector<uint32_t> values = { (fcw >> 16) & 0xFFFF, fcw & 0xFFFF };

    for (size_t i = 0; i < values.size(); i++)
    {
        if (SetRegValue(fpga_spi, addrs[i], values[i]) != OpStatus::Success)
        {
            printf("[Warn] Cannot write data to device.\n");
            return;
        }
    }
}

void FDQIE_ReadData(lime::SDRDevice *sdrdev,
    int sampleCnt,
    double refClk,
    double freq,
    double *pAmplI,
    double *pPhaseI,
    double *pAmplQ,
    double *pPhaseQ,
    double *freq_I,
    double *freq_Q,
    bool chA,
    int operationPhase)
{

    int channel = 0;

    if ((operationPhase == 1) || (operationPhase == 2))
    {
        if (chA == true)
            channel = 4; // LMS#3 chA
        else
            channel = 5; // LMS#3 chB
    }
    else
    {
        if (chA == true)
            channel = 2; // LMS#2 chA
        else
            channel = 3; // LMS#2 chB
    }

    const uint8_t chipIndex = channel / 2;
    const uint8_t channelIndex = channel % 2;
    StreamConfig stream;
    stream.channels[TRXDir::Rx] = { channelIndex };
    stream.channels[TRXDir::Tx] = { channelIndex };
    stream.format = DataFormat::I16;
    stream.linkFormat = DataFormat::I16;

    if (sdrdev->StreamSetup(stream, chipIndex) != OpStatus::Success)
        terminateProgram(sdrdev);

    int16_t datai[sampleCnt + 4096];
    int16_t dataq[sampleCnt + 4096];
    complex16_t buffer[(sampleCnt + 4096)];

    // Start streaming
    sdrdev->StreamStart(chipIndex);

    int kk;
    if (chA == true)
        kk = 1;
    else
        kk = 2;

    // StreamSetup already selects the active channel
    // SetRegValue(sdrdev, 0xFFFF, kk); // ?
    // SetRegValue(sdrdev, 0xA, 1); // ?

    int samplesRead = 0;
    int old_samplesRead = 0;

    auto t1 = chrono::high_resolution_clock::now();
    int num = 0;

    while ((chrono::high_resolution_clock::now() - t1 < chrono::seconds(10)) &&
           (old_samplesRead < (sampleCnt + 4096))) // run for 5 seconds
    {
        // Receive samples
        complex16_t *dest[2] = { reinterpret_cast<complex16_t *>(buffer), nullptr };
        samplesRead = sdrdev->StreamRx(chipIndex, dest, sampleCnt + 4096, NULL);
        // I and Q samples are interleaved in buffer: IQIQIQ...
        for (int j = 0; j < samplesRead; ++j)
        {
            if ((num < (sampleCnt + 4096)) && (num >= 4096))
            {
                datai[num - 4096] = buffer[j].real();
                dataq[num - 4096] = buffer[j].imag();
            }
            num++;
        }
        old_samplesRead += samplesRead;
    }

    fft<double> afft;
    afft.Nfloor = -120.0;
    afft.fs = refClk / 2.0; // 122.88;
    afft.Rb = 0;
    afft.Amax = 2 << 16;
    afft.skip = 0;
    afft.ssize = 15;
    afft.fname = "mI.fft";

    fft<double> afft2;
    afft2.Nfloor = -120.0;
    afft2.fs = refClk / 2.0; // 122.88;
    afft2.Rb = 0;
    afft2.Amax = 2 << 16;
    afft2.skip = 0;
    afft2.ssize = 15;
    afft2.fname = "mQ.fft";

    double amplI = 0.0;
    double phaseI = 0.0;
    double freqI = 0.0;

    double I_Re = 0.0;
    double I_Im = 0.0;

    double amplQ = 0.0;
    double phaseQ = 0.0;
    double freqQ = 0.0;

    double Q_Re = 0.0;
    double Q_Im = 0.0;

    afft.init();
    for (int j = 0; j < sampleCnt; ++j)
    {
        afft.always((double)datai[j]);
    }
    afft.finish();

    afft.getData(fabs(freq), &freqI, &amplI, &phaseI, &I_Re, &I_Im);
    *freq_I = freqI;
    *pAmplI = amplI;
    *pPhaseI = phaseI;

    afft2.init();
    for (int j = 0; j < sampleCnt; ++j)
    {
        afft2.always((double)dataq[j]);
    }
    afft2.finish();
    afft2.getData(fabs(freq), &freqQ, &amplQ, &phaseQ, &Q_Re, &Q_Im);
    *freq_Q = freqQ;
    *pAmplQ = amplQ;
    *pPhaseQ = phaseQ;

    // Stop streaming
    sdrdev->StreamStop(chipIndex);
    sdrdev->StreamDestroy(chipIndex);
}

void FDQIE_SetupTransmitterIQ(lime::ISPI *fpga_spi, int16_t codeI, int16_t codeQ, int16_t codeAlpha, bool chA)
{
    int kk = 1;
    if (chA == true)
        kk = 1;
    else
        kk = 2;

    if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
    {
        printf("[Warn] Cannot write data to device.\n");
        return;
    }

    if (SetRegValue(fpga_spi, CFR::TX_GCORRI.address, codeI) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (SetRegValue(fpga_spi, CFR::TX_GCORRQ.address, codeQ) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (SetRegValue(fpga_spi, CFR::TX_PHCORR.address, codeAlpha) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
}

static int16_t amd = 0;

int findmin(int N, double *maxi)
{
    if (N <= 0)
        return 0;
    double max = maxi[0];
    int tempi = 0;

    for (int i = 1; i < N; i++)
        if (maxi[i] < max)
        {
            tempi = i;
            max = maxi[i];
        }
    return tempi;
}

void CaliAmplitudeBegin(lime::SDRDevice *sdrdev,
    lime::ISPI *fpga_spi,
    int sampleCnt,
    double refClk,
    double freq,
    int16_t *codeI,
    int16_t *codeQ,
    int16_t *codeAlpha,
    double *gain_error,
    double *phase_error,
    bool chA)
{

    double amplI = 0.0;
    double phaseI = 0.0;
    double amplQ = 0.0;
    double phaseQ = 0.0;
    double freqI, freqQ = 0.0;
    double phase = 0.0;

    double phase_errori[6];
    double gain_errori[6];
    double maxi[6];
    int16_t code_I[6];
    int16_t code_Q[6];
    int16_t code_alphai[6];

    double max = 0.0;

    FDQIE_SetupTransmitterIQ(fpga_spi, *codeI, *codeQ, *codeAlpha, chA);
    FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, 2);
    phase = phaseI - phaseQ;
    if (freq < 0)
        phase *= -1.0;
    if (phase < -1.0)
        phase += 2.0;
    else if (phase > 1.0)
        phase -= 2.0;
    *phase_error = (phase - 0.5);
    *gain_error = atan(amplQ / amplI - 1.0) / M_PI;

#ifdef VERBOSE
    printf("\n[Info] chA=%d, freq=%8.5f, amplI=%8.5f, phaseI=%8.5f, amplQ=%8.5f, phaseQ=%8.5f",
        chA,
        freqI,
        amplI,
        phaseI,
        amplQ,
        phaseQ);
#endif
    // exit(0);

    max = fabs(*phase_error) + fabs(*gain_error);
    int16_t code_alpha = (int)(fabs(*phase_error) * M_PI * 1024.0);

    if (amplI > amplQ)
    {
        code_I[0] = (int)(amplQ / amplI * 2047.0);
        code_Q[0] = 2047;
        code_alphai[0] = code_alpha;

        code_Q[1] = (int)(amplQ / amplI * 2047.0);
        code_I[1] = 2047;
        code_alphai[1] = code_alpha;

        code_I[2] = (int)(amplQ / amplI * 2047.0);
        code_Q[2] = 2047;
        code_alphai[2] = -code_alpha;

        code_Q[3] = (int)(amplQ / amplI * 2047.0);
        code_I[3] = 2047;
        code_alphai[3] = -code_alpha;

        code_I[4] = 2047;
        code_Q[4] = (int)(amplQ / amplI * 2047.0);
        code_alphai[4] = 0;

        code_Q[5] = 2047;
        code_I[5] = (int)(amplQ / amplI * 2047.0);
        code_alphai[5] = 0;
    }
    else
    {
        code_Q[0] = 2047;
        code_I[0] = (int)(amplI / amplQ * 2047.0);
        code_alphai[0] = code_alpha;

        code_I[1] = 2047;
        code_Q[1] = (int)(amplI / amplQ * 2047.0);
        code_alphai[1] = code_alpha;

        code_Q[2] = 2047;
        code_I[2] = (int)(amplI / amplQ * 2047.0);
        code_alphai[2] = -code_alpha;

        code_I[3] = 2047;
        code_Q[3] = (int)(amplI / amplQ * 2047.0);
        code_alphai[3] = -code_alpha;

        code_I[4] = 2047;
        code_Q[4] = (int)(amplI / amplQ * 2047.0);
        code_alphai[4] = 0;

        code_Q[5] = 2047;
        code_I[5] = (int)(amplI / amplQ * 2047.0);
        code_alphai[5] = 0;
    }

    for (int i = 0; i < 6; i++)
    {
        FDQIE_SetupTransmitterIQ(fpga_spi, code_I[i], code_Q[i], code_alphai[i], chA);
        FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, 2);
        phase = phaseI - phaseQ;
        if (freq < 0)
            phase *= -1.0;
        if (phase < -1.0)
            phase += 2.0;
        else if (phase > 1.0)
            phase -= 2.0;
        phase_errori[i] = (phase - 0.5);
        gain_errori[i] = (amplQ / amplI - 1.0);
        maxi[i] = fabs(phase_errori[i]) + fabs(gain_errori[i]);
    }

    int i = findmin(6, maxi);
    // if ((max >= maxi[i]))
    {
        *codeI = code_I[i];
        *codeQ = code_Q[i];
        *codeAlpha = code_alphai[i];
        *gain_error = gain_errori[i];
        *phase_error = phase_errori[i];

#ifdef VERBOSE
        printf("\n[Info] gainI=%4d, gainQ=%4d, codeAlpha=%4d, phase error=%8.5f, ampl. error=%8.5f,",
            code_I[i],
            code_Q[i],
            code_alphai[i],
            phase_errori[i],
            gain_errori[i]);
#endif
    }

    FDQIE_SetupTransmitterIQ(fpga_spi, *codeI, *codeQ, *codeAlpha, chA); // upload
}

double Norm(double phase_error, double gain_error)
{

    double gerr = gain_error + 1.0;
    double perr = phase_error;

    double temp = 0.0;

    temp = fabs(1.0 + gerr * gerr - 2 * gerr * cos(perr * M_PI));
    //temp = fabs(phase_error) + fabs(gain_error);

    return temp;
}

int CaliAmplitude(lime::SDRDevice *sdrdev,
    lime::ISPI *fpga_spi,
    int sampleCnt,
    double refClk,
    double freq,
    int16_t *codeI,
    int16_t *codeQ,
    int16_t codeAlpha,
    double *gain_error,
    double *phase_error,
    int *pdelta,
    bool chA)
{

    double amplI = 0.0;
    double phaseI = 0.0;
    double amplQ = 0.0;
    double phaseQ = 0.0;
    double freqI, freqQ = 0.0;
    double phase = 0.0;

    double phase_errori[6];
    double gain_errori[6];
    double maxi[6];
    int16_t code_I[6];
    int16_t code_Q[6];
    double max = 0.0;

    FDQIE_SetupTransmitterIQ(fpga_spi, *codeI, *codeQ, codeAlpha, chA);
    FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, 2);
    phase = phaseI - phaseQ;
    if (freq < 0)
        phase *= -1.0;
    if (phase < -1.0)
        phase += 2.0;
    else if (phase > 1.0)
        phase -= 2.0;
    *phase_error = (phase - 0.5);
    *gain_error = (amplQ / amplI - 1.0);

    max = fabs(*phase_error) + fabs(*gain_error);
    int16_t delta = (int16_t)(4.0 * 1024.0 * max / pow(2.0, amd) + 0.5);
    int cnt = 0;

    max = Norm(*phase_error, *gain_error); // !!!

    if (delta > 0)
    {
        code_Q[cnt] = (int)2047.0;
        code_I[cnt] = *codeI + delta;
        if (code_I[cnt] > 2047)
            code_I[cnt] = 2047;
        if (code_I[cnt] < 0)
            code_I[cnt] = 0;

        if (code_I[cnt] != (*codeI))
            cnt++;

        code_I[cnt] = (int)2047.0;
        code_Q[cnt] = *codeQ + delta;
        if (code_Q[cnt] > 2047)
            code_Q[cnt] = 2047;
        if (code_Q[cnt] < 0)
            code_Q[cnt] = 0;

        if (code_Q[cnt] != (*codeQ))
            cnt++;

        code_Q[cnt] = (int)2047.0;
        code_I[cnt] = *codeI - delta;
        if (code_I[cnt] > 2047)
            code_I[cnt] = 2047;
        if (code_I[cnt] < 0)
            code_I[cnt] = 0;

        if (code_I[cnt] != (*codeI))
            cnt++;

        code_I[cnt] = (int)2047.0;
        code_Q[cnt] = *codeQ - delta;
        if (code_Q[cnt] > 2047)
            code_Q[cnt] = 2047;
        if (code_Q[cnt] < 0)
            code_Q[cnt] = 0;

        if (code_Q[cnt] != (*codeQ))
            cnt++;
    }

    for (int i = 0; i < cnt; i++)
    {
        FDQIE_SetupTransmitterIQ(fpga_spi, code_I[i], code_Q[i], codeAlpha, chA);
        FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, 2);
        phase = phaseI - phaseQ;
        if (freq < 0)
            phase *= -1.0;
        if (phase < -1.0)
            phase += 2.0;
        else if (phase > 1.0)
            phase -= 2.0;
        phase_errori[i] = (phase - 0.5);
        gain_errori[i] = (amplQ / amplI - 1.0);

        //maxi[i] = fabs(phase_errori[i]) + fabs(gain_errori[i]);
        maxi[i] = Norm(phase_errori[i], gain_errori[i]);
    }

    int i = 0;
    if (cnt > 0)
        i = findmin(cnt, maxi);
    else
        i = 0;

    if ((cnt > 0) && ((max >= maxi[i]) || (max == 0.0)))
    {
        amd = 0;
        *codeI = code_I[i];
        *codeQ = code_Q[i];
        *gain_error = gain_errori[i];
        *phase_error = phase_errori[i];

#ifdef VERBOSE
        printf("\n[Info] gainI=%4d, gainQ=%4d, codeAlpha=%4d, phase error=%8.5f, ampl. error=%8.5f, ampl. delta=%3d,",
            code_I[i],
            code_Q[i],
            codeAlpha,
            phase_errori[i],
            gain_errori[i],
            delta);
#endif
    }
    else
    {
        amd++;
    }

    FDQIE_SetupTransmitterIQ(fpga_spi, *codeI, *codeQ, codeAlpha, chA);
    if (delta == 0)
        amd = 0;

    *pdelta = delta;
    return amd;
}

int phd = 0;

int CaliPhase(lime::SDRDevice *sdrdev,
    lime::ISPI *fpga_spi,
    int sampleCnt,
    double refClk,
    double freq,
    int16_t codeI,
    int16_t codeQ,
    int16_t *codeAlpha,
    double *gain_error,
    double *phase_error,
    int *pdelta,
    bool chA)
{

    double amplI = 0.0;
    double phaseI = 0.0;
    double amplQ = 0.0;
    double phaseQ = 0.0;
    double freqI, freqQ = 0.0;

    double phase1 = 0.0;
    double phase2 = 0.0;
    double phase_error1 = 0.0;
    double phase_error2 = 0.0;
    double gain_error1 = 0.0;
    double gain_error2 = 0.0;

    int16_t code_I1 = 2047;
    int16_t code_Q1 = 2047;
    int16_t code_I2 = 2047;
    int16_t code_Q2 = 2047;

    int16_t code_alpha = *codeAlpha;
    double max1, max2;
    double max;

    FDQIE_SetupTransmitterIQ(fpga_spi, codeI, codeQ, code_alpha, chA);
    FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, 2);

    phase1 = phaseI - phaseQ;
    if (freq < 0)
        phase1 *= -1.0;
    if (phase1 < -1.0)
        phase1 += 2.0;
    else if (phase1 > 1.0)
        phase1 -= 2.0;
    phase_error1 = (phase1 - 0.5);
    *phase_error = phase_error1;
    gain_error1 = (amplQ / amplI - 1.0);
    *gain_error = gain_error1;

    max = fabs(phase_error1) + fabs(gain_error1);
    int16_t delta = (int16_t)(1.5 * 1024.0 * max / pow(2.0, phd + 0.5)); // bilo 1.0 umesto 2.0

    max = Norm(phase_error1, gain_error1); ///!!!

    if (delta > 0)
    {
        int16_t code_alpha1 = code_alpha - delta;
        FDQIE_SetupTransmitterIQ(fpga_spi, codeI, codeQ, code_alpha1, chA);
        FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, 2);
        phase1 = phaseI - phaseQ;
        if (freq < 0)
            phase1 *= -1.0;
        if (phase1 < -1.0)
            phase1 += 2.0;
        else if (phase1 > 1.0)
            phase1 -= 2.0;
        phase_error1 = (phase1 - 0.5);
        gain_error1 = (amplQ / amplI - 1.0);

        //max1 = fabs(phase_error1) + fabs(gain_error1);
        max1 = Norm(phase_error1, gain_error1);

        int16_t code_alpha2 = code_alpha + delta;
        FDQIE_SetupTransmitterIQ(fpga_spi, codeI, codeQ, code_alpha2, chA);
        FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, 2);
        phase2 = phaseI - phaseQ;
        if (freq < 0)
            phase2 *= -1.0;
        if (phase2 < -1.0)
            phase2 += 2.0;
        else if (phase2 > 1.0)
            phase2 -= 2.0;
        phase_error2 = (phase2 - 0.5);
        gain_error2 = (amplQ / amplI - 1.0);

        //max2 = fabs(phase_error2) + fabs(gain_error2);
        max2 = Norm(phase_error2, gain_error2);

        if ((max1 <= max) || (max2 <= max) || (max == 0.0))
        {
            if ((max1 < max2) && (abs(code_alpha1) < 450))
            {
                *codeAlpha = code_alpha1;
                *gain_error = gain_error1;
                *phase_error = phase_error1;
#ifdef VERBOSE
                printf("\n[Info] gainI=%4d, gainQ=%4d, codeAlpha=%4d, phase error=%8.5f, ampl. error=%8.5f, phase delta=%3d,",
                    codeI,
                    codeQ,
                    *codeAlpha,
                    *phase_error,
                    *gain_error,
                    delta);
#endif
                phd = 0;
            }
            else if (abs(code_alpha2) < 450)
            {
                *codeAlpha = code_alpha2;
                *gain_error = gain_error2;
                *phase_error = phase_error2;
#ifdef VERBOSE
                printf("\n[Info] gainI=%4d, gainQ=%4d, codeAlpha=%4d, phase error=%8.5f, ampl. error=%8.5f, phase delta=%3d,",
                    codeI,
                    codeQ,
                    *codeAlpha,
                    *phase_error,
                    *gain_error,
                    delta);

#endif
                phd = 0;
            }
        }
        else
        {
            phd++;
        }
    }
    FDQIE_SetupTransmitterIQ(fpga_spi, codeI, codeQ, *codeAlpha, chA);
    if (delta == 0)
        phd = 0;

    *pdelta = delta;
    return phd;
}

void FDQIE_CalibrateTransmitterSingleFreq(FILE *fid,
    lime::SDRDevice *sdrdev,
    lime::ISPI *fpga_spi,
    int sampleCnt,
    double LOFreq,
    double refClk,
    double freq,
    int16_t *codeI,
    int16_t *codeQ,
    int16_t *codeAlpha,
    double *t_amp,
    double *t_gerr,
    double *t_perr,
    bool chA)
{

    int16_t code_I, code_Q = 0;
    int16_t code_alpha = 0;
    double phase_error = 1.0;
    double gain_error = 1.0;

#ifdef VERBOSE
    printf("\n[Info] Transmitter calibration: freq=%lg, refClk=%lg, ... ", freq, refClk);
#endif

    FDQIE_SetNCO(fpga_spi, refClk, freq, chA);

    // exit(0);

    int k = 0;
    int j = 0;
    int found = 0;

    gain_error = 0.0;
    phase_error = 0.0;

    int Pdelta = 0;
    int Adelta = 0;

    double eps1 = 0.0;

    if ((*codeI == 2047) && (*codeQ == 2047) && (*codeAlpha == 0))
    {
        code_I = 2047;
        code_Q = 2047;
        code_alpha = 0;
        CaliAmplitudeBegin(
            sdrdev, fpga_spi, sampleCnt, refClk, freq, &code_I, &code_Q, &code_alpha, &gain_error, &phase_error, chA);
    }
    else
    {
        code_I = (*codeI);
        code_Q = (*codeQ);
        code_alpha = (*codeAlpha);
    }

    while ((found == 0) && (j < 100))
    {
        k = 0;
        found = 0;
        phd = 0;

        eps1 = eps * (1.0 + ((double)(j)) / 200.0);

        while ((found == 0) && (k < 5))
        {

#ifdef VERBOSE
            printf(" p=%d,", phd);
#endif
            if ((phd == 4) && (Pdelta > 5))
            {
#ifdef VERBOSE
                printf(" CHANGE(%d),", Pdelta);
#endif
                int16_t temp = code_I;
                code_I = code_Q;
                code_Q = temp;
                code_alpha = 0;
                phd = 0;
            }

            if (CaliPhase(sdrdev,
                    fpga_spi,
                    sampleCnt,
                    refClk,
                    freq,
                    code_I,
                    code_Q,
                    &code_alpha,
                    &gain_error,
                    &phase_error,
                    &Pdelta,
                    chA) == 0)
                found = 1;
            k++;
        }

        //if ((fabs(phase_error) < eps) && (fabs(gain_error) < eps))
        if ((phase_error * phase_error + gain_error * gain_error) < (2 * eps1 * eps1))
            found = 1;
        else
            found = 0;

        if (found == 0)
        {

            k = 0;
            found = 0;
            amd = 0;
            while ((found == 0) && (k < 5))
            {
#ifdef VERBOSE
                printf(" a=%d,", amd);
#endif

                if ((amd == 4) && (Adelta > 15))
                {
#ifdef VERBOSE
                    printf(" CHANGE(%d),", Adelta);
#endif
                    int16_t temp = code_I;
                    code_I = code_Q;
                    code_Q = temp;
                    code_alpha = 0;
                    amd = 0;
                }
                else if (CaliAmplitude(sdrdev,
                             fpga_spi,
                             sampleCnt,
                             refClk,
                             freq,
                             &code_I,
                             &code_Q,
                             code_alpha,
                             &gain_error,
                             &phase_error,
                             &Adelta,
                             chA) == 0)
                    found = 1;
                k++;
            }

            //if ((fabs(phase_error) < eps) && (fabs(gain_error) < eps))
            if ((phase_error * phase_error + gain_error * gain_error) < (2 * eps1 * eps1))
                found = 1;
            else
                found = 0;
        }
        j++;
    }

    if ((j >= 100) && (found == 0) && ((fabs(phase_error) + fabs(gain_error)) > 10 * eps))
    {
        // Failure
        code_I = 2047;
        code_Q = 2047;
        code_alpha = 0;
    }

    FDQIE_SetupTransmitterIQ(fpga_spi, code_I, code_Q, code_alpha, chA);
    double amplI = 0.0;
    double phaseI = 0.0;
    double amplQ = 0.0;
    double phaseQ = 0.0;
    double freqI, freqQ = 0.0;

    FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, 2);
    double phase = phaseI - phaseQ;
    if (freq < 0)
        phase *= -1.0;
    if (phase < -1.0)
        phase += 2.0;
    else if (phase > 1.0)
        phase -= 2.0;
    phase_error = (phase - 0.5);
    gain_error = (amplQ / amplI - 1.0);

    printf("\n[Info] freq=%8.5f, gainI=%4d, gainQ=%4d, codeAlpha=%4d, phase error=%8.5f, ampl. error=%8.5f",
        freq,
        code_I,
        code_Q,
        code_alpha,
        phase_error,
        gain_error);

    *codeI = code_I;
    *codeQ = code_Q;
    *codeAlpha = code_alpha;

    *t_amp = amplI;
    *t_gerr = ((double)code_Q) / ((double)code_I);
    *t_perr = 360.0 * atan(((double)(code_alpha)) / 2048.0) / M_PI;

    if (fid != NULL)
        fprintf(fid,
            "%lg, %lg, %lg, %lg, %lg, %lg, %lg, %d, %d, %d, %lg, %lg\n",
            freq,
            amplI,
            ((double)(*codeQ)) / ((double)(*codeI)),
            360.0 * atan(((double)(*codeAlpha)) / 2048.0) / M_PI,
            phaseI,
            amplQ,
            phaseQ,
            code_I,
            code_Q,
            code_alpha,
            phase_error,
            gain_error);
}

void FDQIE_SetupReceiverIQ(lime::ISPI *fpga_spi, int16_t codeI, int16_t codeQ, int16_t codeAlpha, bool chA, int operationPhase)
{

    Register reg1, reg2, reg3;

    int kk = 1;
    if (chA == true)
        kk = 1;
    else
        kk = 2;

    if ((operationPhase == 1) || (operationPhase == 2))
    {
        // LMS#3 RxChain registers
        reg2 = Register(0x0162, 11, 0); // GCORRQ
        reg1 = Register(0x0161, 11, 0); // GCORRI
        reg3 = Register(0x0163, 11, 0); // PHCORR
    }
    else // (operationPhase == 3)
    {
        // LMS#2 RxChain
        reg2 = Register(0x00A1, 11, 0); // GCORRQ
        reg1 = Register(0x00A2, 11, 0); // GCORRI
        reg3 = Register(0x00A3, 11, 0); // PHCORR
    }

    if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
    {
        printf("[Warn] Cannot write data to device.\n");
        return;
    }

    if (SetRegValue(fpga_spi, reg1.address, codeI) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (SetRegValue(fpga_spi, reg2.address, codeQ) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (SetRegValue(fpga_spi, reg3.address, codeAlpha) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
}

void FDQIE_CalibrateReceiverSingleFreq(FILE *fid,
    lime::SDRDevice *sdrdev,
    lime::ISPI *fpga_spi,
    int sampleCnt,
    double LOFreq,
    double refClk,
    double freq,
    int16_t *codeI,
    int16_t *codeQ,
    int16_t *codeAlpha,
    double *amp,
    double *gerr,
    double *perr,
    bool chA,
    int operationPhase) // bool nco,
{

    double amplI = 0.0;
    double phaseI = 0.0;
    double amplQ = 0.0;
    double phaseQ = 0.0;
    double freqI, freqQ = 0.0;
    double phase_error = 0.0;
    double gain_error = 0.0;

    lime::LMS7002M *lms;

    if (operationPhase == 1)
    {
        // LMS#2 SXT LO is used
        lms = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(1));
    }
    else if (operationPhase == 3)
    {
        // LMS#3 SXT LO is used
        lms = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(2));
    }
    else
        return;

    int numAttempts = 0;
    double phase = 0.0;
    double err = 1.0;

    while ((numAttempts < 10) && (err >= 0.01))
    {

        // the API function behaves strange, please check
        // if (LMS_SetLOFrequency(sdrdev, LMS_CH_TX, 0, (LOFreq + freq) * 1000000.0) != 0)
        //    terminateProgram(sdrdev);
        if (lms->SetFrequencySX(TRXDir::Tx, (LOFreq + freq) * 1e6) != OpStatus::Success)
            terminateProgram(sdrdev);

        // if (lms->SetFrequencySX(LMS_CH_TX, (LOFreq + freq) * 1e6) != 0)
        //     terminateProgram(sdrdev);

        // program codeAlpha=0, codeI=2047, codeQ=2047
        FDQIE_SetupReceiverIQ(fpga_spi, 2047, 2047, 0, chA, operationPhase);
        FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, operationPhase);

        phase = phaseI - phaseQ;
        if (phase < -1.0)
            phase += 2.0;
        else if (phase > 1.0)
            phase -= 2.0;

        if (amplI > amplQ)
        {
            if (operationPhase == 3)
            {
                // menjano
                *codeI = (int)(amplQ / amplI * 2047.0); // turn over
                *codeQ = 2047;
            }
            else
            {
                // menjano
                *codeQ = (int)(amplQ / amplI * 2047.0); // turn over
                *codeI = 2047;
            }
        }
        else
        {
            if (operationPhase == 3)
            {
                // menjano
                *codeI = 2047;
                *codeQ = (int)(amplI / amplQ * 2047.0); // turn over
            }
            else
            {
                // menjano
                *codeQ = 2047;
                *codeI = (int)(amplI / amplQ * 2047.0); // turn over
            }
        }

        if (freq > 0)
            *codeAlpha = (int)((phase - 0.5) * M_PI * 1024.0);
        else
            *codeAlpha = (int)((0.0 - phase - 0.5) * M_PI * (1024.0));

        // program codeAlpha, codeI, codeQ
        FDQIE_SetupReceiverIQ(fpga_spi, *codeI, *codeQ, *codeAlpha, chA, operationPhase);
        FDQIE_ReadData(sdrdev, sampleCnt, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, operationPhase);
        phase = phaseI - phaseQ;
        if (freq < 0)
            phase *= -1.0;
        if (phase < -1.0)
            phase += 2.0;
        else if (phase > 1.0)
            phase -= 2.0;

        phase_error = (phase - 0.5);
        gain_error = (amplQ / amplI - 1.0);

        *amp = amplI;
        *gerr = ((double)(*codeQ)) / ((double)(*codeI));
        *perr = 360.0 * atan(((double)(*codeAlpha)) / 2048.0) / M_PI;

        err = fabs(phase_error) + fabs(gain_error);
        numAttempts++;
    }
    //printf("[Info] Num Attempts:%d\n", numAttempts);

    if (fid != NULL)
        fprintf(fid,
            "%lg, %lg, %lg, %lg, %lg, %lg, %lg, %d, %d, %d, %lg, %lg\n",
            freq,
            amplI,
            ((double)(*codeQ)) / ((double)(*codeI)),
            360.0 * atan(((double)(*codeAlpha)) / 2048.0) / M_PI,
            phaseI,
            amplQ,
            phaseQ,
            *codeI,
            *codeQ,
            *codeAlpha,
            phase_error,
            gain_error);

#ifdef VERBOSE
    printf("[Info] freq=%lg, gainI=%d, gainQ=%d, codeAlpha=%d, phase error=%lg, ampl. error=%lg\n",
        freq,
        *codeI,
        *codeQ,
        *codeAlpha,
        phase_error,
        gain_error);
#endif

#ifndef VERBOSE
    printf("[Info] freq=%lg, gainI=%d, gainQ=%d, codeAlpha=%d\n", freq, *codeI, *codeQ, *codeAlpha);
#endif
}

int16_t Rcode_I[50];
int16_t Rcode_Q[50];
int16_t Rcode_Alpha[50];
int16_t Tcode_I[50];
int16_t Tcode_Q[50];
int16_t Tcode_Alpha[50];
double FREQ[50];

void FDQIE_ReceiverMeasurement(lime::SDRDevice *sdrdev,
    lime::ISPI *fpga_spi,
    int32_t numSamples,
    double LOfreq,
    double fs,
    bool chA,
    int operationPhase) // int slaveDevice,
{
    int16_t code_I;
    int16_t code_Q;
    int16_t code_Alpha;
    double amp, gerr, perr;
    double refClk = 2.0 * fs;

    for (int i = 0; i < 50; i++)
    {
        Rcode_I[i] = 2047;
        Rcode_Q[i] = 2047;
        Rcode_Alpha[i] = 0;
    }

    // printf("[Info] Receiver I/Q measurement\n");

    lime::LMS7002M *lms;

    if (operationPhase == 1)
    {
        // LMS#2 SXT LO is used
        lms = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(1));
    }
    else if (operationPhase == 3)
    {
        // LMS#3 SXT LO is used
        lms = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(2));
    }
    else
        return;

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, operationPhase); // enable the loopback
    if (operationPhase == 1)
    {
        printf("[Info] LMS#3 Receiver I/Q measurement\n");
        FDQIE_SetNCO(fpga_spi, refClk, 0.0, chA);
        FDQIE_SetupTransmitterIQ(fpga_spi, 2047, 2047, 0, chA);
    }
    else if (operationPhase == 3)
    {
        printf("[Info] LMS#2 Receiver I/Q measurement\n");
        FDQIE_SetupTransmitterTone(sdrdev, 1); // set max DC value
        //exit(0);
    }
    else
        return;

    int32_t Np = numSamples / 2;
    int i = 0;
    double f[Np];
    for (i = 0; i < Np; i++)
        f[i] = 0.5 * fs * (double)(i) / (double)(Np);

    FILE *fid = NULL;
    if (operationPhase == 1) // LMS#3 receiver measurement
    {
        if (chA == true)
            fid = fopen("ReceiverMeasurement_phase1_A.csv", "w");
        else
            fid = fopen("ReceiverMeasurement_phase1_B.csv", "w");
    }
    else if (operationPhase == 3) // LMS#2 receiver measurement
    {
        if (chA == true)
            fid = fopen("ReceiverMeasurement_phase3_A.csv", "w");
        else
            fid = fopen("ReceiverMeasurement_phase3_B.csv", "w");
    }
    else
        return;

    fprintf(fid, "freq, ampl, gaincor, phasecor, phaseI, amplQ, phaseQ, codeI, codeQ, codeAlpha, phaseError, amplError\n");

    double freq = 0.0;
    int cnt = 0;

    for (freq = -50.0; freq <= 50.0; freq = freq + 5.0)
    {
        if ((freq <= -5.0) || (freq >= 5.0))
        {
            i = (int)(fabs(freq) / (fs / 2.0) * Np);
            if (fabs(f[i + 1] - fabs(freq)) < fabs(f[i] - fabs(freq)))
                i++;
            if (freq < 0)
                f[i] *= -1.0;

            FDQIE_CalibrateReceiverSingleFreq(fid,
                sdrdev,
                fpga_spi,
                numSamples,
                LOfreq,
                refClk,
                f[i],
                &code_I,
                &code_Q,
                &code_Alpha,
                &amp,
                &gerr,
                &perr,
                chA,
                operationPhase);
            Rcode_I[cnt] = code_I;
            Rcode_Q[cnt] = code_Q;
            Rcode_Alpha[cnt] = code_Alpha;
            cnt++;
        }
        else // 0
        {
            double freq1 = -1.0;
            i = (int)(fabs(freq1) / (fs / 2.0) * Np);
            if (fabs(f[i + 1] - fabs(freq1)) < fabs(f[i] - fabs(freq1)))
                i++;
            if (freq1 < 0)
                f[i] *= -1.0;

            FDQIE_CalibrateReceiverSingleFreq(fid,
                sdrdev,
                fpga_spi,
                numSamples,
                LOfreq,
                refClk,
                f[i],
                &code_I,
                &code_Q,
                &code_Alpha,
                &amp,
                &gerr,
                &perr,
                chA,
                operationPhase);
            Rcode_I[cnt] = code_I;
            Rcode_Q[cnt] = code_Q;
            Rcode_Alpha[cnt] = code_Alpha;
            cnt++;

            freq1 = 1.0;
            i = (int)(fabs(freq1) / (fs / 2.0) * Np);
            if (fabs(f[i + 1] - fabs(freq1)) < fabs(f[i] - fabs(freq1)))
                i++;
            if (freq1 < 0)
                f[i] *= -1.0;

            FDQIE_CalibrateReceiverSingleFreq(fid,
                sdrdev,
                fpga_spi,
                numSamples,
                LOfreq,
                refClk,
                f[i],
                &code_I,
                &code_Q,
                &code_Alpha,
                &amp,
                &gerr,
                &perr,
                chA,
                operationPhase);
            Rcode_I[cnt] = code_I;
            Rcode_Q[cnt] = code_Q;
            Rcode_Alpha[cnt] = code_Alpha;
            cnt++;
        }
    }

    if (lms->SetFrequencySX(TRXDir::Tx, LOfreq * 1e6) != OpStatus::Success)
        terminateProgram(sdrdev);

    if (operationPhase == 3)
        FDQIE_SetupTransmitterTone(sdrdev, 0);

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0); // disable the loopback

    fclose(fid);
}

void FDQIE_ReceiverCalibration(lime::SDRDevice *sdrdev,
    lime::ISPI *fpga_spi,
    int32_t numSamples,
    double LOfreq,
    double fs,
    int N,
    double *freq,
    double *r_amp,
    double *r_gerr,
    double *r_perr,
    bool chA,
    int operationPhase) // int slaveDevice, bool nco
{

    int16_t code_I;
    int16_t code_Q;
    int16_t code_Alpha;
    double amp, gerr, perr;
    double refClk = 2.0 * fs;

    lime::LMS7002M *lms;

    if (operationPhase == 1)
    {
        // LMS#2 SXT LO is used
        lms = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(1));
    }
    else if (operationPhase == 3)
    {
        // LMS#3 SXT LO is used
        lms = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(2));
    }
    else
        return;

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, operationPhase); // enable the loopback

    if (operationPhase == 1)
    {
        printf("[Info] LMS#3 Receiver I/Q calibration\n");
        FDQIE_SetNCO(fpga_spi, refClk, 0.0, chA);
        FDQIE_SetupTransmitterIQ(fpga_spi, 2047, 2047, 0, chA);
    }
    else if (operationPhase == 3)
    {
        printf("[Info] LMS#2 Receiver I/Q calibration\n");
        FDQIE_SetupTransmitterTone(sdrdev, 1); // set max DC value
        //exit(1);
    }
    else
        return;

    for (int i = 0; i < 50; i++)
    {
        Rcode_I[i] = 2047;
        Rcode_Q[i] = 2047;
        Rcode_Alpha[i] = 0;
    }

    int32_t Np = numSamples / 2;
    int i = 0;
    double f[Np];
    for (i = 0; i < Np; i++)
        f[i] = 0.5 * fs * (double)(i) / (double)(Np);

    FILE *fid = NULL;

    if (operationPhase == 1) // LMS#3 receiver measurement
    {
        if (chA == true)
            fid = fopen("ReceiverCalibration_phase1_A.csv", "w");
        else
            fid = fopen("ReceiverCalibration_phase1_B.csv", "w");
    }
    else if (operationPhase == 3) // LMS#2 receiver measurement
    {
        if (chA == true)
            fid = fopen("ReceiverCalibration_phase3_A.csv", "w");
        else
            fid = fopen("ReceiverCalibration_phase3_B.csv", "w");
    }
    else
        return;

    fprintf(fid, "freq, ampl, gaincor, phasecor, phaseI, amplQ, phaseQ, codeI, codeQ, codeAlpha, phaseError, amplError\n");

    double freq1 = 0.0;
    for (int k = 0; k < N; k++)
    {
        // if (k == 4)
        {
            freq1 = freq[k];
            i = (int)(fabs(freq1) / (fs / 2.0) * Np);
            if (fabs(f[i + 1] - fabs(freq1)) < fabs(f[i] - fabs(freq1)))
                i++;
            if (freq1 < 0)
                f[i] *= -1.0;

            FDQIE_CalibrateReceiverSingleFreq(fid,
                sdrdev,
                fpga_spi,
                numSamples,
                LOfreq,
                refClk,
                f[i],
                &code_I,
                &code_Q,
                &code_Alpha,
                &amp,
                &gerr,
                &perr,
                chA,
                operationPhase);
            Rcode_I[k] = code_I;
            Rcode_Q[k] = code_Q;
            Rcode_Alpha[k] = code_Alpha;
            r_amp[k] = amp;
            r_gerr[k] = gerr;
            r_perr[k] = perr;
        }
    }

    if (lms->SetFrequencySX(TRXDir::Tx, LOfreq * 1e6) != OpStatus::Success)
        terminateProgram(sdrdev);

    if (operationPhase == 3)
        FDQIE_SetupTransmitterTone(sdrdev, 0);

    // exit(0);

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0); // disable the loopback
    fclose(fid);
}

void FDQIE_TransmitterMeasurement(
    lime::SDRDevice *sdrdev, lime::ISPI *fpga_spi, int32_t numSamples, double LOfreq, double fs, bool chA, int operationPhase)
{

    int16_t code_I;
    int16_t code_Q;
    int16_t code_Alpha;
    double amp, gerr, perr;
    double refClk = 2.0 * fs;

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, operationPhase); // enable the loopback
    printf("[Info] LMS#2 Transmitter I/Q measurement\n");

    for (int i = 0; i < 50; i++)
    {
        Tcode_I[i] = 2047;
        Tcode_Q[i] = 2047;
        Tcode_Alpha[i] = 0;
        FREQ[i] = 0.0;
    }

    int32_t Np = numSamples / 2;
    int i = 0;
    double f[Np];
    for (i = 0; i < Np; i++)
        f[i] = 0.5 * fs * (double)(i) / (double)(Np);

    FILE *fid = NULL;
    if (chA == true)
        fid = fopen("TransmitterMeasurement_A.csv", "w");
    else
        fid = fopen("TransmitterMeasurement_B.csv", "w");

    fprintf(fid, "freq, ampl, gaincor, phasecor, phaseI, amplQ, phaseQ, codeI, codeQ, codeAlpha, phaseError, amplError\n");

    double freq = 0.0;
    int cnt = 0;

    code_I = 2047;
    code_Q = 2047;
    code_Alpha = 0;

    for (freq = -50.0; freq <= 50.0; freq = freq + 5.0)
    {
        if ((freq <= -5.0) || (freq >= 5.0))
        {
            i = (int)(fabs(freq) / (fs / 2.0) * Np);
            if (fabs(f[i + 1] - fabs(freq)) < fabs(f[i] - fabs(freq)))
                i++;
            if (freq < 0)
                f[i] *= -1.0;

            FDQIE_SetupReceiverIQ(fpga_spi, Rcode_I[cnt], Rcode_Q[cnt], Rcode_Alpha[cnt], chA, operationPhase);
            FDQIE_CalibrateTransmitterSingleFreq(
                fid, sdrdev, fpga_spi, numSamples, LOfreq, refClk, f[i], &code_I, &code_Q, &code_Alpha, &amp, &gerr, &perr, chA);
            Tcode_I[cnt] = code_I;
            Tcode_Q[cnt] = code_Q;
            Tcode_Alpha[cnt] = code_Alpha;
            FREQ[cnt] = f[i];
            cnt++;
        }
        else // 0
        {
            double freq1 = -1.0;
            i = (int)(fabs(freq1) / (fs / 2.0) * Np);
            if (fabs(f[i + 1] - fabs(freq1)) < fabs(f[i] - fabs(freq1)))
                i++;
            if (freq1 < 0)
                f[i] *= -1.0;

            FDQIE_SetupReceiverIQ(fpga_spi, Rcode_I[cnt], Rcode_Q[cnt], Rcode_Alpha[cnt], chA, operationPhase);
            FDQIE_CalibrateTransmitterSingleFreq(
                fid, sdrdev, fpga_spi, numSamples, LOfreq, refClk, f[i], &code_I, &code_Q, &code_Alpha, &amp, &gerr, &perr, chA);
            Tcode_I[cnt] = code_I;
            Tcode_Q[cnt] = code_Q;
            Tcode_Alpha[cnt] = code_Alpha;
            FREQ[cnt] = f[i];
            cnt++;

            freq1 = 1.0;
            i = (int)(fabs(freq1) / (fs / 2.0) * Np);
            if (fabs(f[i + 1] - fabs(freq1)) < fabs(f[i] - fabs(freq1)))
                i++;
            if (freq1 < 0)
                f[i] *= -1.0;

            FDQIE_SetupReceiverIQ(fpga_spi, Rcode_I[cnt], Rcode_Q[cnt], Rcode_Alpha[cnt], operationPhase, chA);
            FDQIE_CalibrateTransmitterSingleFreq(
                fid, sdrdev, fpga_spi, numSamples, LOfreq, refClk, f[i], &code_I, &code_Q, &code_Alpha, &amp, &gerr, &perr, chA);
            Tcode_I[cnt] = code_I;
            Tcode_Q[cnt] = code_Q;
            Tcode_Alpha[cnt] = code_Alpha;
            FREQ[cnt] = f[i];
            cnt++;
        }
    }

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0); // disable the loopback
    FDQIE_SetNCO(fpga_spi, refClk, 0.0, chA);
    fclose(fid);
}

void FDQIE_TransmitterCalibration(lime::SDRDevice *sdrdev,
    lime::ISPI *fpga_spi,
    int32_t numSamples,
    double LOfreq,
    double fs,
    int N,
    double *freq,
    double *t_amp,
    double *t_gerr,
    double *t_perr,
    bool chA,
    int operationPhase)
{

    int16_t code_I;
    int16_t code_Q;
    int16_t code_Alpha;
    double refClk = 2.0 * fs;

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, operationPhase); // enable the loopback
    printf("\n[Info] LMS#2 Transmitter I/Q calibration");

    for (int i = 0; i < N; i++)
    {
        Tcode_I[i] = 2047;
        Tcode_Q[i] = 2047;
        Tcode_Alpha[i] = 0;
        FREQ[i] = 0.0;
    }

    int32_t Np = numSamples / 2;
    double f[Np];
    for (int i = 0; i < Np; i++)
        f[i] = 0.5 * fs * (double)(i) / (double)(Np);

    FILE *fid = NULL;

    if (chA == true)
        fid = fopen("TransmitterCalibration_A.csv", "w");
    else
        fid = fopen("TransmitterCalibration_B.csv", "w");

    fprintf(fid, "freq, ampl, gaincor, phasecor, phaseI, amplQ, phaseQ, codeI, codeQ, codeAlpha, phaseError, amplError\n");

    double amp, gerr, perr;

    code_I = 2047;
    code_Q = 2047;
    code_Alpha = 0;

    for (int k = 0; k < N; k++)
    {
        // if (k == 3)
        {
            double freq1 = freq[k];
            int i = (int)(fabs(freq1) / (fs / 2.0) * Np);
            if (fabs(f[i + 1] - fabs(freq1)) < fabs(f[i] - fabs(freq1)))
                i++;
            if (freq1 < 0)
                f[i] *= -1.0;

            FDQIE_SetupReceiverIQ(fpga_spi, Rcode_I[k], Rcode_Q[k], Rcode_Alpha[k], chA, operationPhase);
            // exit(0);
            FDQIE_CalibrateTransmitterSingleFreq(
                fid, sdrdev, fpga_spi, numSamples, LOfreq, refClk, f[i], &code_I, &code_Q, &code_Alpha, &amp, &gerr, &perr, chA);
            Tcode_I[k] = code_I;
            Tcode_Q[k] = code_Q;
            Tcode_Alpha[k] = code_Alpha;
            t_amp[k] = amp;
            t_gerr[k] = gerr;
            t_perr[k] = perr;
            FREQ[k] = f[i];
        }
    }
    printf("\n");

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0); // disable the loopback
    FDQIE_SetNCO(fpga_spi, refClk, 0.0, chA);
    fclose(fid);
}

void FDQIE_SaveEqualiser(lime::ISPI *fpga_spi, int Ntaps, std::string m_sConfigFilename)
{
    FDQIE_SaveEqualiser(fpga_spi, Ntaps, m_sConfigFilename);
}

void FDQIE_ResetEqualiser(lime::ISPI *fpga_spi, bool chA)
{
    const int maxCoefCount = 16;
    uint16_t maddressT_hI = 0x2C0; // address of transmitter's coeffs
    uint16_t maddressT_hQ = 0x2D0;
    uint16_t maddressR_hI = 0x2E0; // address of receiver's coeffs
    uint16_t maddressR_hQ = 0x2F0;
    int16_t regValue = 0;

    int kk = 1;
    if (chA == true)
        kk = 1;
    else
        kk = 2;

    if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
    {
        printf("[Warn] Device cannot be opened.\n");
        return;
    }

    for (int i = 0; i < maxCoefCount; i++)
    {
        if (i == 4)
            regValue = 32767;
        else
            regValue = 0;

        if (SetRegValue(fpga_spi, maddressT_hI + i, regValue) != OpStatus::Success)
            printf("[Warn] Cannot read data from file.\n");
        if (SetRegValue(fpga_spi, maddressT_hQ + i, regValue) != OpStatus::Success)
            printf("[Warn] Cannot read data from file.\n");
        if (SetRegValue(fpga_spi, maddressR_hI + i, regValue) != OpStatus::Success)
            printf("[Warn] Cannot read data from file.\n");
        if (SetRegValue(fpga_spi, maddressR_hQ + i, regValue) != OpStatus::Success)
            printf("[Warn] Cannot read data from file.\n");
    }

    if (SetRegValue(fpga_spi, CFR::TX_GCORRI, 2047) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    if (SetRegValue(fpga_spi, CFR::TX_GCORRQ, 2047) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    if (SetRegValue(fpga_spi, CFR::TX_PHCORR, 0) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (SetRegValue(fpga_spi, CFR::RX_GCORRI, 2047) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    if (SetRegValue(fpga_spi, CFR::RX_GCORRQ, 2047) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    if (SetRegValue(fpga_spi, CFR::RX_PHCORR, 0) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    printf("[Info] Equaliser is in reset state.\n");
}

static void SaveRegisterRangesToIni(
    lime::ISPI *fpga_spi, INI *m_options, uint8_t chipId, const std::vector<Range<uint16_t>> &ranges)
{
    uint16_t regValue;
    for (const auto &range : ranges)
    {
        for (size_t address = range.min; address <= range.max; address += range.step)
        {
            regValue = GetRegValue(fpga_spi, address);
            std::string key = stringFormat("Ch%d_Reg:0x%04X", chipId, static_cast<uint16_t>(address));
            m_options->set(key, std::to_string(regValue));
        }
    }
}

void FDQIE_SaveDC(lime::ISPI *fpga_spi, double RefClk, const std::string &m_sConfigFilename)
{
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");

    const std::vector<Range<uint16_t>> addrRanges = {
        { CFR::TX_DCCORRI.address, CFR::TX_DCCORRI.address },
        { CFR::TX_DCCORRQ.address, CFR::TX_DCCORRQ.address },
    };

    for (int chipId = 1; chipId <= 2; chipId++)
    {
        if (SetRegValue(fpga_spi, 0xFFFF, chipId) != OpStatus::Success)
        {
            printf("[Warn] Device cannot be opened.\n");
            return;
        }

        SaveRegisterRangesToIni(fpga_spi, &m_options, chipId, addrRanges);
    }

    m_options.save(m_sConfigFilename);
    printf("[Info] DC calibration results are saved in file %s.\n", m_sConfigFilename.c_str());
}

void FDQIE_LoadDC(lime::ISPI *fpga_spi, const std::string &m_sConfigFilename)
{
    uint16_t regValue = 0;
    // Transmitter DC
    constexpr Register reg1{ CFR::TX_DCCORRI };
    constexpr Register reg2{ CFR::TX_DCCORRQ };

    const char *config_filename = m_sConfigFilename.c_str();
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");

    for (int kk = 1; kk <= 2; kk++)
    {

        if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
        {
            printf("[Warn] Device cannot be opened.\n");
            return;
        }
        regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg1.address).c_str(), 0);
        if (SetRegValue(fpga_spi, reg1.address, regValue) != OpStatus::Success)
            printf("[Warn] Cannot write data to device.\n");

        regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg2.address).c_str(), 0);
        if (SetRegValue(fpga_spi, reg2.address, regValue) != OpStatus::Success)
            printf("[Warn] Cannot write data to device.\n");
    }
    printf("[Info] DC calibration results are loaded from file %s.\n", config_filename);
}

void FDQIE_LoadEqualiser(lime::ISPI *fpga_spi, const std::string &m_sConfigFilename)
{

    // load configuration from file
    uint16_t regValue = 0;
    const char *config_filename = m_sConfigFilename.c_str();
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");

    constexpr int maxCoefCount = 16;
    constexpr uint16_t maddressT_hI = 0x2C0; // address of TX EQU coeffs
    constexpr uint16_t maddressT_hQ = 0x2D0;
    constexpr uint16_t maddressR_hI = 0x2E0; // address of RX EQU coeffs
    constexpr uint16_t maddressR_hQ = 0x2F0;

    for (int kk = 1; kk <= 2; kk++)
    {

        if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
        {
            printf("[Warn] Device cannot be opened.\n");
            return;
        }
        for (int i = 0; i < maxCoefCount; i++)
        {
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressT_hI + i).c_str(), 0);
            if (SetRegValue(fpga_spi, maddressT_hI + i, regValue) != OpStatus::Success)
                printf("[Warn] Cannot read data from file.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressT_hQ + i).c_str(), 0);
            if (SetRegValue(fpga_spi, maddressT_hQ + i, regValue) != OpStatus::Success)
                printf("[Warn] Cannot read data from file.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressR_hI + i).c_str(), 0);
            if (SetRegValue(fpga_spi, maddressR_hI + i, regValue) != OpStatus::Success)
                printf("[Warn] Cannot read data from file.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressR_hQ + i).c_str(), 0);
            if (SetRegValue(fpga_spi, maddressR_hQ + i, regValue) != OpStatus::Success)
                printf("[Warn] Cannot read data from file.\n");
        }
        // Static transmitter I/Q corrector
        {
            constexpr Register reg1{ CFR::TX_GCORRI };
            constexpr Register reg2{ CFR::TX_GCORRQ };
            constexpr Register reg3{ CFR::TX_PHCORR };

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg1.address).c_str(), 0);
            if (SetRegValue(fpga_spi, reg1.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg2.address).c_str(), 0);
            if (SetRegValue(fpga_spi, reg2.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg3.address).c_str(), 0);
            if (SetRegValue(fpga_spi, reg3.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");
        }

        // Static RX I/Q corrector
        {
            constexpr Register reg1{ CFR::RX_GCORRI };
            constexpr Register reg2{ CFR::RX_GCORRQ };
            constexpr Register reg3{ CFR::RX_PHCORR };

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg1.address).c_str(), 0);
            if (SetRegValue(fpga_spi, reg1.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg2.address).c_str(), 0);
            if (SetRegValue(fpga_spi, reg2.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg3.address).c_str(), 0);
            if (SetRegValue(fpga_spi, reg3.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");
        }
    }

    if (SetRegValue(fpga_spi, 0xFFFF, 1) != OpStatus::Success)
    {
        printf("[Warn] Cannot write data to device.\n");
        return;
    }
    printf("[Info] Equaliser configuration is loaded from file %s.\n", config_filename);
}

void FDQIE_SaveConfig(lime::ISPI *fpga_spi,
    int Ntaps,
    int codeI,
    int codeQ,
    int codeAlpha,
    int *hI,
    int *hQ,
    int codeI2,
    int codeQ2,
    int codeAlpha2,
    int *hI2,
    int *hQ2,
    bool chA)
{

    // save previously found calibrated values into FPGA
    // one channel only

#ifdef VERBOSE
    printf("Equaliser HW PARAMETERS:\n");
    printf("Transmitter:    CodeI = %d, CodeQ = %d, CodeAlpha = %d\n", codeI, codeQ, codeAlpha);
    for (int i = 0; i < Ntaps; i++)
        printf("    hI[%2d] = %7d, hQ[%2d] = %7d\n", i, hI[i], i, hQ[i]);
    printf("Receiver:    CodeI = %d, CodeQ = %d, CodeAlpha = %d\n", codeI2, codeQ2, codeAlpha2);
    for (int i = 0; i < Ntaps; i++)
        printf("    hI[%2d] = %7d, hQ[%2d] = %7d\n", i, hI2[i], i, hQ2[i]);
#endif

    const int maxCoefCount = 16;
    uint16_t maddressT_hI = 0x2C0; // address of transmitter's coeffs
    uint16_t maddressT_hQ = 0x2D0;
    uint16_t maddressR_hI = 0x2E0; // address of receiver's coeffs
    uint16_t maddressR_hQ = 0x2F0;
    Register reg1, reg2, reg3;

    int kk = 1;
    if (chA == true)
        kk = 1;
    else
        kk = 2;

    if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
    {
        printf("[Warn] Device cannot be opened.\n");
        return;
    }

    for (int i = 0; i < maxCoefCount; i++)
    {
        if (i < Ntaps)
            SetRegValue(fpga_spi, maddressT_hI + i, hI[i]);
        else
            SetRegValue(fpga_spi, maddressT_hI + i, 0);

        if (i < Ntaps)
            SetRegValue(fpga_spi, maddressT_hQ + i, hQ[i]);
        else
            SetRegValue(fpga_spi, maddressT_hQ + i, 0);

        if (i < Ntaps)
            SetRegValue(fpga_spi, maddressR_hI + i, hI2[i]);
        else
            SetRegValue(fpga_spi, maddressR_hI + i, 0);

        if (i < Ntaps)
            SetRegValue(fpga_spi, maddressR_hQ + i, hQ2[i]);
        else
            SetRegValue(fpga_spi, maddressR_hQ + i, 0);
    }

    uint16_t regValue = 0;

    // Static Transmitter I/Q corrector
    reg1 = Register(0x0082, 11, 0); // GCORRI
    reg2 = Register(0x0081, 11, 0); // GCORRQ
    reg3 = Register(0x0083, 11, 0); // PHCORR

    regValue = codeI;
    if (SetRegValue(fpga_spi, reg1.address, regValue) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    regValue = codeQ;
    if (SetRegValue(fpga_spi, reg2.address, regValue) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    regValue = codeAlpha;
    if (SetRegValue(fpga_spi, reg3.address, regValue) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    // Static receiver I/Q corrector
    reg1 = Register(0x00A2, 11, 0); // GCORRI
    reg2 = Register(0x00A1, 11, 0); // GCORRQ
    reg3 = Register(0x00A3, 11, 0); // PHCORR

    regValue = codeI2;
    if (SetRegValue(fpga_spi, reg1.address, regValue) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    regValue = codeQ2;
    if (SetRegValue(fpga_spi, reg2.address, regValue) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    regValue = codeAlpha2;
    if (SetRegValue(fpga_spi, reg3.address, regValue) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (SetRegValue(fpga_spi, 0xFFFF, 1) != OpStatus::Success)
    {
        printf("[Warn] Cannot write data to device.\n");
        return;
    }

    printf("[Info] Equaliser is programmed.\n");
}

void SaveCFRFIR(lime::ISPI *fpga_spi, const std::string &m_sConfigFilename)
{
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");

    // FIR registers start address
    const int maxCoefCount = 32;
    const std::vector<Range<uint16_t>> addrRanges = {
        { 768, 768 + maxCoefCount - 1 }, // address of CFR
        { 832, 832 + maxCoefCount - 1 }, // CFR
        { 896, 896 + maxCoefCount - 1 }, // Post-CFR FIR filter coefficients
        { 960, 960 + maxCoefCount - 1 }, // FIR
    };

    std::set<uint16_t> CFRadresses;
    for (const auto &csr : CFRarray)
        CFRadresses.insert(csr.address);

    for (int kk = 1; kk <= 2; ++kk)
    {
        if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
        {
            printf("[Warn] Device cannot be opened.\n");
            return;
        }

        // Save various board configuration
        for (const auto addr : CFRadresses)
        {
            uint16_t value = GetRegValue(fpga_spi, addr);
            m_options.set(stringFormat("Ch%d_Reg:0x%04X", kk, addr), std::to_string(value));
        }

        SaveRegisterRangesToIni(fpga_spi, &m_options, kk, addrRanges);
    }
    m_options.save(m_sConfigFilename);
    printf("[Info] CFRFIR configuration is saved to file %s.\n", m_sConfigFilename.c_str());
}

void LoadCFRFIR(lime::ISPI *fpga_spi, const std::string &m_sConfigFilename)
{
    // load configuration from file
    uint16_t regValue = 0;
    const char *config_filename = m_sConfigFilename.c_str();
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");
    // std::map<wxObject*, CSRegister>::iterator iter;

    for (int kk = 1; kk <= 2; kk++)
    {
        if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
        {
            lime::error("[Warn] Device cannot be opened.\n");
            return;
        }
        // for (iter = controlsPtr2Registers.begin(); iter != controlsPtr2Registers.end(); ++iter)
        // {
        //     Register reg = iter->second;
        //     regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg.address), 0); // read data from file
        //     SetRegValue(fpga_spi,reg.address, regValue);
        // }
        // Load CFR FIR filter coefficients
        const int maxCoefCount = 32;
        uint16_t maddressf0 = 768; // CFR
        uint16_t maddressf1 = 832; // CFR
        uint16_t maddressf2 = 896; // FIR
        uint16_t maddressf3 = 960; // FIR
        for (int i = 0; i < maxCoefCount; i++)
        {
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressf0 + i), 0);
            SetRegValue(fpga_spi, maddressf0 + i, regValue);

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressf1 + i), 0);
            SetRegValue(fpga_spi, maddressf1 + i, regValue);

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressf2 + i), 0);
            SetRegValue(fpga_spi, maddressf2 + i, regValue);

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressf3 + i), 0);
            SetRegValue(fpga_spi, maddressf3 + i, regValue);
        }
    }

    if (SetRegValue(fpga_spi, 0xFFFF, 0x1) != OpStatus::Success)
    {
        lime::error("[Warn] Cannot write data to device.\n");
        return;
    }

    lime::info("[Info] CFR & CFRFIR configuration is loaded from file %s.\n", config_filename);
}
