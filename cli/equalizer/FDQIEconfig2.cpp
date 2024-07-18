#include <stdlib.h>
#include <stdio.h>
#include <string>
#include "INI.h"
#include <unistd.h>
#include <chrono>
#include "limesuiteng/Register.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/LMS7002M.h"

#include "DSP/CFR/CrestFactorReduction.h"
#include "FPGA/FPGA_common.h"

#include "FDQIdesign.h"

using namespace std;
using namespace lime;

void FDQIE_SetupTransmitterTone(lime::SDRDevice *sdrdev, bool m_bMax)
{
    LMS7002M *lms = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(2)); // LMS#3 is now transmitter
    long value = 0;
    if (m_bMax == true)
        value = (int)32767 / 2;
    else
        value = 0;

    lms->Modify_SPI_Reg_bits(LMS7002MCSR::DC_REG_TXTSP, value);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGDCLDI_TXTSP, 0);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGDCLDI_TXTSP, 1);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGDCLDI_TXTSP, 0);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::DC_REG_TXTSP, value);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGDCLDQ_TXTSP, 0);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGDCLDQ_TXTSP, 1);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGDCLDQ_TXTSP, 0);
}

void FDQIE_SetupTransmitterGain(lime::ISPI *fpga_spi, int16_t codeI, int16_t codeQ, bool chA, int operationPhase)
{

    // codeAlpha stays unchanged
    int kk;
    if (chA == true)
        kk = 1;
    else
        kk = 2;

    if ((operationPhase == 1) || (operationPhase == 2))
    {
    }
    else
        return;

    constexpr Register reg2 = CFR::TX_GCORRQ;
    constexpr Register reg1 = CFR::TX_GCORRI;
    if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
    {
        printf("[Warn] Cannot write data to device.\n");
        return;
    }

    if (SetRegValue(fpga_spi, reg1, codeI) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (SetRegValue(fpga_spi, reg2, codeQ) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
}

void FDQIE_SetupTransmitterDC(lime::ISPI *fpga_spi, int16_t codeI, int16_t codeQ, bool chA)
{

    int kk;
    constexpr Register reg1 = CFR::TX_DCCORRI;
    constexpr Register reg2 = CFR::TX_DCCORRQ;

    if (chA == true)
        kk = 1;
    else
        kk = 2;

    if (SetRegValue(fpga_spi, 0xFFFF, kk) != OpStatus::Success)
    {
        printf("[Warn] Cannot write data to device.\n");
        return;
    }

    if (SetRegValue(fpga_spi, reg1, codeI) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (SetRegValue(fpga_spi, reg2, codeQ) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
}

void FDQIE_SetupReceiverDC(SDRDevice *sdrdev, lime::ISPI *fpga_spi, int16_t codeI, int16_t codeQ, bool chA, int operationPhase)
{
    lime::LMS7002M *rfsoc = nullptr;
    if ((operationPhase == 1) || (operationPhase == 2))
        // LMS#3 receive
        rfsoc = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(2)); // LMS#3
    else
        // LMS#2 receive
        rfsoc = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(1)); // LMS#2

    int kk;
    if (chA == true)
        kk = 1;
    else
        kk = 2;
    rfsoc->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, kk);
    rfsoc->Modify_SPI_Reg_bits(LMS7002MCSR::EN_DCOFF_RXFE_RFE, 1);

    uint16_t regValue = 0;
    if (codeI < 0)
        regValue = 64 - codeI;
    else
        regValue = codeI;

    rfsoc->Modify_SPI_Reg_bits(LMS7002MCSR::DCOFFI_RFE, regValue);

    if (codeQ < 0)
        regValue = 64 - codeQ;
    else
        regValue = codeQ;

    rfsoc->Modify_SPI_Reg_bits(LMS7002MCSR::DCOFFQ_RFE, regValue);
}

void FDQIE_DCCorrSettings(lime::ISPI *fpga_spi, bool dccorrbypass, bool dcloopbypass, bool chA, int operationPhase)
{

    SetRegValue(fpga_spi, 0xFFFF, (chA == 1) ? 0x1 : 0x2);
    Register reg1, reg2;

    if ((operationPhase == 1) || (operationPhase == 2))
    {
        // LMS#3 RxChain registers
        // chkRX_DCLOOP_BYP
        reg1 = Register(0x016C, 8, 8);
        // chkRX_DCCORR_BYP
        reg2 = Register(0x016C, 2, 2);
    }
    else // (operationPhase == 3)
    {
        // LMS#2 RxChain
        // chkRX_DCLOOP_BYP
        reg1 = Register(0x00AC, 8, 8);
        // chkRX_DCCORR_BYP
        reg2 = Register(0x00AC, 2, 2);
    }

    if (dcloopbypass == true)
        SetRegValue(fpga_spi, reg1, 1);
    else
        SetRegValue(fpga_spi, reg1, 0);

    if (dccorrbypass == true)
        SetRegValue(fpga_spi, reg2, 1);
    else
        SetRegValue(fpga_spi, reg2, 0);
}

void FDQIE_ReadData2(lime::SDRDevice *sdrdev, int sampleCnt, double *pAveI, double *pAveQ, bool chA, int operationPhase)
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

    // StreamSetup already select the channel
    // SetRegValue(sdrdev, 0xFFFF, 1 << (channel >> 1)); // ????
    // SetRegValue(sdrdev, 0xA, 1); // ????

    int samplesRead = 0;
    int old_samplesRead = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    int num = 0;

    while ((std::chrono::high_resolution_clock::now() - t1 < std::chrono::seconds(10)) &&
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

    double sumI = 0;
    double sumQ = 0;

    for (int j = 0; j < sampleCnt; ++j)
    {
        sumI = sumI + ((double)datai[j]);
        sumQ = sumQ + ((double)dataq[j]);
    }
    *pAveI = (double)(sumI / sampleCnt);
    *pAveQ = (double)(sumQ / sampleCnt);

    // Stop streaming
    sdrdev->StreamStop(chipIndex);
    sdrdev->StreamDestroy(chipIndex);
}

void FDQIE_CalibrateReceiverDC(
    lime::SDRDevice *sdrdev, lime::ISPI *fpga_spi, int numSamples, int *R_codeI, int *R_codeQ, bool chA, int operationPhase)
{
    double avgI = 0.0;
    double avgQ = 0.0;
    double avgI1 = 0.0;
    double avgQ1 = 0.0;
    double avgI2 = 0.0;
    double avgQ2 = 0.0;

    int16_t code_I = 0;
    int16_t code_Q = 0;
    int16_t code_I1 = 0;
    int16_t code_Q1 = 0;

    int loopback = 1;
    bool dccorrbypass = 1;
    bool dcloopstop = 1;

    // prepare
    if (operationPhase == 3)
    {
        printf("\n[Info] LMS#2 Receiver DC calibration\n");
        FDQIE_SetupTransmitterTone(sdrdev, 0);
    }
    else if (operationPhase == 1)
    {
        printf("\n[Info] LMS#3 Receiver DC calibration\n");
        FDQIE_SetupTransmitterGain(fpga_spi, 0, 0, chA, 1); // transmitter gain corrections to zero
    }
    else
        return;

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, operationPhase); // disable the loopback
    FDQIE_DCCorrSettings(fpga_spi, dccorrbypass, dcloopstop, chA, operationPhase); // bypass DC correction, stop DC correction track

    FDQIE_SetupReceiverDC(sdrdev, fpga_spi, 0, 0, chA, operationPhase); // 1. run
    FDQIE_ReadData2(sdrdev, numSamples, &avgI, &avgQ, chA, operationPhase);

#ifdef VERBOSE
    // printf("1. avgI = %+.5f, avgQ = %.5f\n", avgI, avgQ);
#endif

    FDQIE_SetupReceiverDC(sdrdev, fpga_spi, 63, 0, chA, operationPhase); // 2. run
    FDQIE_ReadData2(sdrdev, numSamples, &avgI1, &avgQ1, chA, operationPhase);

#ifdef VERBOSE
    // printf("2. avgI = %+.5f, avgQ = %.5f\n", avgI1, avgQ1);
#endif

    FDQIE_SetupReceiverDC(sdrdev, fpga_spi, 0, 63, chA, operationPhase); // 3. run
    FDQIE_ReadData2(sdrdev, numSamples, &avgI2, &avgQ2, chA, operationPhase);

#ifdef VERBOSE
    // printf("3. avgI = %+.5f, avgQ = %.5f\n", avgI2, avgQ2);
#endif

    code_I = (int16_t)(-63.0 * (avgI * (avgQ2 - avgQ) - avgQ * (avgI2 - avgI)) /
                           ((avgI1 - avgI) * (avgQ2 - avgQ) - (avgQ1 - avgQ) * (avgI2 - avgI)) +
                       0.5);

    code_Q = (int16_t)(-63.0 * (avgQ * (avgI1 - avgI) - avgI * (avgQ1 - avgQ)) /
                           ((avgI1 - avgI) * (avgQ2 - avgQ) - (avgQ1 - avgQ) * (avgI2 - avgI)) +
                       0.5);

    if (code_I > 63)
        code_I = 63;
    else if (code_I < -63)
        code_I = -63;

    if (code_Q > 63)
        code_Q = 63;
    else if (code_Q < -63)
        code_Q = -63;

#ifdef VERBOSE
    printf("[Info] Receiver DC codes: codeI=%d, codeQ=%d,", code_I, code_Q);
#endif

    FDQIE_SetupReceiverDC(sdrdev, fpga_spi, code_I, code_Q, chA, operationPhase);
    FDQIE_ReadData2(sdrdev, numSamples, &avgI, &avgQ, chA, operationPhase);

#ifdef VERBOSE
    printf(" avgI=%8.5f, avgQ=%8.5f\n", avgI, avgQ);
#endif

    code_I1 = code_I;
    code_Q1 = code_Q;

    int Limit = 1;

    for (int i = -Limit; i <= Limit; i++)
        for (int j = -Limit; j <= Limit; j++)
        {
            if ((abs(code_I + i) < 64) && (abs(code_Q + j) < 64))
            {
                FDQIE_SetupReceiverDC(sdrdev, fpga_spi, code_I + i, code_Q + j, chA, operationPhase);
                FDQIE_ReadData2(sdrdev, numSamples, &avgI1, &avgQ1, chA, operationPhase);
                if ((abs(avgI1) + abs(avgQ1)) < (abs(avgI) + abs(avgQ)))
                {
                    avgI = avgI1; // new min
                    avgQ = avgQ1;
                    code_I1 = code_I + i;
                    code_Q1 = code_Q + j;
                }
            }
        }

    printf("[Info] Receiver DC codes: codeI=%d, codeQ=%d\n", code_I1, code_Q1);
    printf("\n");
    FDQIE_SetupReceiverDC(sdrdev, fpga_spi, code_I1, code_Q1, chA, operationPhase);

    *R_codeI = code_I1;
    *R_codeQ = code_Q1;

    FDQIE_ReadData2(sdrdev, numSamples, &avgI, &avgQ, chA, operationPhase);

    sleep(1);
    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, operationPhase); // enable the loopback
    sleep(3);
    FDQIE_DCCorrSettings(fpga_spi, !dccorrbypass, !dcloopstop, chA, operationPhase); // bypass DC correction, start DC tracking loop
    sleep(3);
    FDQIE_DCCorrSettings(fpga_spi, !dccorrbypass, dcloopstop, chA, operationPhase); // bypass DC correction, stop DC tracking loop
    sleep(3);

    if (operationPhase == 3)
        FDQIE_SetupTransmitterTone(sdrdev, 1); // set max DC value
    else if (operationPhase == 1)
        FDQIE_SetupTransmitterGain(fpga_spi, 2047, 2047, chA, 1);
    else
        return;

    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0); // disable the loopback
}

void FDQIE_CalibrateTransmitterDC(lime::SDRDevice *sdrdev,
    lime::ISPI *fpga_spi,
    int numSamples,
    double LOFreq,
    double refClk,
    double freq,
    int *T_codeI,
    int *T_codeQ,
    bool chA,
    int operationPhase,
    bool nco)
{

    double amplI = 0.0;
    double phaseI = 0.0;
    double amplQ = 0.0;
    double phaseQ = 0.0;
    double freqI, freqQ = 0.0;

    int loopback = 1;

    printf("\n[Info] LMS#2 Transmitter DC calibration\n");

    lime::LMS7002M *lms = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(1)); // get LMS#2

    if (lms->SetFrequencySX(TRXDir::Tx, (LOFreq + freq) * 1e6) != OpStatus::Success)
        terminateProgram(sdrdev);

    if (nco)
        FDQIE_SetNCO(fpga_spi, refClk, 25, chA);
    // FDQIE_SetNCO(sdrdev, refClk, 5, chA);

    // prepare
    if (nco)
        FDQIE_SetupTransmitterGain(fpga_spi, 2047, 2047, chA, operationPhase); // transmitter gain corrections to zero
    else
        FDQIE_SetupTransmitterGain(fpga_spi, 0, 0, chA, operationPhase); // transmitter gain corrections to zero

    int16_t code_I = 0;
    int16_t code_Q = 0;
    FDQIE_SetupTransmitterDC(fpga_spi, code_I, code_Q, chA); // default
    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, operationPhase); // enable the loopback

    double dc = 0.0; // 4 * amplI * amplI + amplQ * amplQ;
    double sum = 0.0;
    int N = 2;

    for (int k = 0; k < N; k++)
    {
        FDQIE_ReadData(sdrdev, numSamples, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, operationPhase);
        sum = sum + amplI * amplI + amplQ * amplQ;
    }
    dc = sum;

    int delta = 128;
    bool stop = false;
    bool stop2 = false;
    int16_t max = 4096;

    while (delta > 0)
    {
        stop = false;
        stop2 = false;
        while ((!stop) && (abs(code_I) < max))
        {
            FDQIE_SetupTransmitterDC(fpga_spi, code_I + delta, code_Q, chA);

            sum = 0.0;
            for (int k = 0; k < N; k++)
            {
                FDQIE_ReadData(
                    sdrdev, numSamples, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, operationPhase);
                sum = sum + amplI * amplI + amplQ * amplQ;
            }

            if (sum < dc)
            {
                dc = sum;
                code_I = code_I + delta;
                stop2 = true;
#ifdef VERBOSE
                printf("[Info] codeI=%d, codeQ=%d, ", code_I, code_Q);
                printf("delta=%d, dc=%8.5f\n", delta, dc);
#endif
            }
            else
                stop = true;
        }

        stop = false;
        while ((!stop) && (!stop2) && (abs(code_I) < max))
        {
            FDQIE_SetupTransmitterDC(fpga_spi, code_I - delta, code_Q, chA);

            sum = 0.0;
            for (int k = 0; k < N; k++)
            {
                FDQIE_ReadData(
                    sdrdev, numSamples, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, operationPhase);
                sum = sum + amplI * amplI + amplQ * amplQ;
            }

            if (sum < dc)
            {
                dc = sum;
                code_I = code_I - delta;
#ifdef VERBOSE
                printf("[Info] codeI=%d, codeQ=%d, ", code_I, code_Q);
                printf("delta= %d, dc=%8.5f\n", -delta, dc);
#endif
            }
            else
                stop = true;
        }

        stop = false;
        stop2 = false;
        while ((!stop) && (abs(code_Q) < max))
        {
            FDQIE_SetupTransmitterDC(fpga_spi, code_I, code_Q + delta, chA);

            sum = 0.0;
            for (int k = 0; k < N; k++)
            {
                FDQIE_ReadData(
                    sdrdev, numSamples, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, operationPhase);
                sum = sum + amplI * amplI + amplQ * amplQ;
            }

            if (sum < dc)
            {
                dc = sum;
                code_Q = code_Q + delta;
                stop2 = true;
#ifdef VERBOSE
                printf("[Info] codeI=%d, codeQ=%d, ", code_I, code_Q);
                printf("delta=%d, dc=%8.5f\n", delta, dc);
#endif
            }
            else
                stop = true;
        }

        stop = false;
        while ((!stop) && (!stop2) && (abs(code_Q) < max))
        {
            FDQIE_SetupTransmitterDC(fpga_spi, code_I, code_Q - delta, chA);

            sum = 0.0;
            for (int k = 0; k < N; k++)
            {
                FDQIE_ReadData(
                    sdrdev, numSamples, refClk, freq, &amplI, &phaseI, &amplQ, &phaseQ, &freqI, &freqQ, chA, operationPhase);
                sum = sum + amplI * amplI + amplQ * amplQ;
            }

            if (sum < dc)
            {
                dc = sum;
                code_Q = code_Q - delta;
#ifdef VERBOSE
                printf("[Info] codeI=%d, codeQ=%d, ", code_I, code_Q);
                printf("delta=%d, dc=%8.5f\n", -delta, dc);
#endif
            }
            else
                stop = true;
        }

        // printf("delta= %d\n", delta);
        delta = delta / 2;
        if (((abs(code_I)) >= max) || (abs(code_Q) >= max))
            delta = 0;
    }

    FDQIE_SetupTransmitterDC(fpga_spi, code_I, code_Q, chA);
    printf("[Info] Transmitter DC codes: codeI=%d, codeQ=%d\n", code_I, code_Q);

    *T_codeI = code_I;
    *T_codeQ = code_Q;

    if (lms->SetFrequencySX(TRXDir::Tx, (LOFreq)*1e6) != OpStatus::Success)
        terminateProgram(sdrdev);

    FDQIE_SetNCO(fpga_spi, refClk, 0.0, chA);

    FDQIE_SetupTransmitterGain(fpga_spi, 2047, 2047, chA, operationPhase);
    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0); // disable the loopback
}

void FDQIE_AdjustGain(lime::SDRDevice *sdrdev, int incRXGain, int incTxGain, bool chA)
{
    lime::LMS7002M *lms;

    // LMS#2 receive
    lms = reinterpret_cast<LMS7002M *>(sdrdev->GetInternalChip(1)); // LMS#2
    int gain = 0;
    int kk = 0;
    if (chA == true)
        kk = 1;
    else
        kk = 2;
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, kk);

    gain = lms->Get_SPI_Reg_bits(LMS7002MCSR::G_PGA_RBB);

#ifdef VERBOSE
    printf("[Info] Old value for LMS#2 RBB PGA gain was %ddB\n", gain - 12);
#endif

    gain = gain + incRXGain;
    if (gain < 0)
    { // starts with -12dB and increases
        gain = 0;
        printf("[Warning] LMS#2 RBB PGA gain cannot be adjusted.\n");
    }
    else if (gain > 31)
    {
        gain = 31;
        printf("[Warning] LMS#2 RBB PGA gain cannot be adjusted.\n");
    }
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::G_PGA_RBB, gain);
    gain = lms->Get_SPI_Reg_bits(LMS7002MCSR::G_PGA_RBB);

#ifdef VERBOSE
    printf("[Info] LMS#2 RBB PGA gain is increased to %ddB \n", int(gain - 12));
#endif

    gain = lms->Get_SPI_Reg_bits(LMS7002MCSR::LOSS_MAIN_TXPAD_TRF);

#ifdef VERBOSE
    printf("[Info] Old value for LMS#2 TRF TXPAD gain was %d \n", gain);
#endif

    gain = gain - incTxGain;
    if (gain < 0)
    { // starts with 0dB and decreases
        gain = 0;
        printf("[Warning] LMS#2 TRF TXPAD gain cannot be adjusted as calculated.\n");
    }
    else if (gain > 31)
    {
        gain = 31;
        printf("[Warning] LMS#2 TRF TXPAD gain cannot be adjusted as calculated.\n");
    }

    lms->Modify_SPI_Reg_bits(LMS7002MCSR::LOSS_MAIN_TXPAD_TRF, gain);
    gain = lms->Get_SPI_Reg_bits(LMS7002MCSR::LOSS_MAIN_TXPAD_TRF);

#ifdef VERBOSE
    printf("[Info] LMS#2 TRF TXPAD gain is increased to %d\n", gain);
#endif
}
