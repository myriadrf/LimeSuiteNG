/* ************************************************************************
   FILE:	equAPI.c
   COMMENT:	Test bench for FDQI Equaliser
   CONTENT:
   AUTHOR:	Lime Microsystems
   DATE:	May 18, 2020
   REVISION:
   ************************************************************************ */

/* Include section */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

#include "common.h"
#include "limesuiteng/limesuiteng.hpp"
#include "limesuiteng/LMS7002M.h"

#include "DSP/CFR/CrestFactorReduction.h"

using namespace lime;

#define CALIBRATION

#include "FDQIdesign.h"

extern int16_t Tcode_I[30];
extern int16_t Tcode_Q[30];

bool chA = true;
bool m_bList = false;
bool m_bSave = false;
bool m_bLoad = false;
bool m_bCalibration = false;
bool m_bReset = false;
bool m_bHelp = false;
bool m_bTdd = false;
bool m_bMeasurement = false;
bool m_bAdjustGain = false;
bool m_bTXDC = false;

std::string m_sConfigFilename = "EqualizerSettings_M.ini2";

void parse(int argc, char *argv[])
{

    std::string m_sTemp;
    std::string str_extension = ".ini2";
    chA = true;
    m_bSave = false;
    m_bCalibration = false;
    m_bReset = false;
    m_bLoad = false;
    m_bHelp = false;
    m_bList = false;
    m_bTdd = false;
    m_bMeasurement = false;
    m_bAdjustGain = false;
    m_bTXDC = false;

    if (argc > 1)
    {
        if (atoi(argv[1]) == 0)
            chA = 1;
        else
            chA = 0;
    }
    else
    {
        m_bList = true; // no second argument
    }

    for (int i = 1; i < argc; i++)
    {
        m_sTemp = argv[i];
        // if (i < argc - 1) // not the last argument
        {
            if ((m_sTemp.compare("--save") == 0) || (m_sTemp.compare("-s") == 0))
            {
                m_bSave = true;
                if (i < argc - 1)
                {
                    m_sConfigFilename = argv[i + 1];

                    if (m_sConfigFilename.compare(m_sConfigFilename.length() - 5, m_sConfigFilename.length() - 1, str_extension) !=
                        0)
                    {
                        printf("[Error] Syntax error. Filename should contain extension .ini2\n");
                        m_bSave = false;
                    }
                }
                else
                {
                    printf("[Error] Syntax error. Filename is missing\n");
                    m_bSave = false;
                }
            }

            else if ((m_sTemp.compare("--load") == 0) || (m_sTemp.compare("-l") == 0))
            {
                m_bLoad = true;
                if (i < argc - 1)
                {
                    m_sConfigFilename = argv[i + 1];

                    if (m_sConfigFilename.compare(m_sConfigFilename.length() - 5, m_sConfigFilename.length() - 1, str_extension) !=
                        0)
                    {
                        printf("[Error] Syntax error. Filename should contain extension .ini2\n");
                        m_bLoad = false;
                    }
                }
                else
                {
                    printf("[Error] Syntax error. Filename is missing\n");
                    m_bLoad = false;
                }
            }
        }

        if ((m_sTemp.compare("--calibration") == 0) || (m_sTemp.compare("-c") == 0))
        {
            m_bCalibration = true;
        }

        if ((m_sTemp.compare("--reset") == 0) || (m_sTemp.compare("-r") == 0))
        {
            m_bReset = true;
        }

        if ((m_sTemp.compare("--help") == 0) || (m_sTemp.compare("-h") == 0))
        {
            m_bHelp = true;
        }

        if (m_sTemp.compare("--list") == 0)
        {
            m_bList = true;
        }

        if ((m_sTemp.compare("--tdd") == 0) || (m_sTemp.compare("-t") == 0))
        {
            m_bTdd = true;
        }

        if ((m_sTemp.compare("--measurement") == 0) || (m_sTemp.compare("-m") == 0))
        {
            m_bMeasurement = true;
        }

        if ((m_sTemp.compare("--adjustgain") == 0) || (m_sTemp.compare("-a") == 0))
        {
            m_bAdjustGain = true;
        }

        if ((m_sTemp.compare("--dc") == 0) || (m_sTemp.compare("-d") == 0))
        {
            m_bTXDC = true;
        }
    }
}

int FDQIE_Check(lime::SDRDevice *sdrdev, FDQI_SPI *fpga_spi, int MeasPoints, double refClk_MHz, double *perr, bool chA)
{
    int i = 0;
    int m_bFound = 0;

    const int kk = chA == true ? 1 : 2;

    while ((i < MeasPoints) && (m_bFound == 0))
    {
        if (fabs(perr[i]) > 50.0)
        {
            m_bFound = 1;

            FDQIE_ResetEqualiser(fpga_spi, chA);
            FDQIE_SetupTransmitterDC(fpga_spi, 0, 0, chA); // default
            // TODO: FDQIE_SetupReceiverDC(sdr, fpga_spi, 0, 0, kk); // default
            FDQIE_SetNCO(fpga_spi, refClk_MHz, 0.0, chA); // reset NCO
            FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0);
            printf("[Error] Calibration error occured. Please check the LMS7002m configuration files.\n");
            printf("[Error] Equaliser is now in reset state.\n");
            return 1;
        }
        else
            i++;
    }
    return 0;
}

static LogLevel logVerbosity = LogLevel::Error;
static void LogCallback(LogLevel lvl, const std::string &msg)
{
    if (lvl > logVerbosity)
        return;
    std::cerr << msg << std::endl;
}

int main(int argc, char *argv[])
{

    double fs = 122.88; // ADC sampling frequancy
    double refClk_MHz = 2.0 * fs; // DAC sampling frequancy [MHz] - 245.76

    double r_clk = refClk_MHz / 2.0; // RX Equilizer sampling frequency
    double r_wp = 50.0 / r_clk; // RX Equilizer pass band
    double t_clk = refClk_MHz; // TX Equilizer sampling frequency
    double t_wp = 50.0 / t_clk; // TX Equilizer pass band

    double f[6 * 2], r_w[6 * 2], t_w[6 * 2]; // Frequency (MHz) and normalized ones for RX and TX
    double r_amp1[6 * 2], r_gerr1[6 * 2], r_perr1[6 * 2]; // RX measured amplitude, gain error and phase error, Phase 1
    double t_amp[6 * 2], t_gerr[6 * 2], t_perr[6 * 2]; // TX measured amplitude, gain error and phase error
    double r_amp[6 * 2], r_gerr[6 * 2], r_perr[6 * 2]; // RX measured amplitude, gain error and phase error, Phase 2

    int r_codeI, r_codeQ, r_codeAlpha; // RX equilizer
    int r_Ntaps = 15;
    int r_hI[r_Ntaps], r_hQ[r_Ntaps];
    int t_codeI, t_codeQ, t_codeAlpha; // TX equilizer
    int t_Ntaps = 15;
    int t_hI[t_Ntaps], t_hQ[t_Ntaps];
    bool tx = true;
    double amp, gerr, perr;

    int TcodeI = 0;
    int TcodeQ = 0;
    int RcodeI = 0;
    int RcodeQ = 0;

    FILE *fid = NULL;

    for (int i = 0; i < 12; i++)
        f[i] = 0.0;

    int MeasPoints = 12;

    if (MeasPoints == 12)
    {
        f[0] = -50.0;
        f[1] = -45.0;
        f[2] = -35.0;
        f[3] = -25.0;
        f[4] = -15.0;
        f[5] = -5.0;
        f[6] = 5.0;
        f[7] = 15.0;
        f[8] = 25.0;
        f[9] = 35.0;
        f[10] = 45.0;
        f[11] = 50.0;
    }
    else
    {
        f[0] = -50.0;
        f[1] = -25.0;
        f[2] = -5.0;
        f[3] = 5.0;
        f[4] = 25.0;
        f[5] = 50.0;
    }

    int16_t code_I, code_Q, code_Alpha; // temporary data
    int32_t numSamples = 32 * 1024; // FFT points

    parse(argc, argv);

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        std::cerr << "No devices found" << std::endl;
        return EXIT_FAILURE;
    }

    if (m_bList == true)
    {
        for (const auto &h : handles)
            std::cerr << "[Info]" << h.Serialize() << std::endl;
        return 0;
    }
    else if (m_bHelp == true)
    {
        printf("EQUAPI - the manual\n\n");
        printf("NAME\n");
        printf("   equAPI - for Equaliser for LimeSDR-5GRadio board\n\n");
        printf("SYNOPSYS\n");
        printf("   sudo ./equAPI [--list]\n");
        printf("   sudo ./equAPI X [-r]\n");
        printf("   sudo ./equAPI X [-c]\n");
        printf("   sudo ./equAPI X [-m]\n");
        printf("   sudo ./equAPI X [-c] [-a] [-s]<filename>\n");
        printf("   sudo ./equAPI X [-s] <filename>\n");
        printf("   sudo ./equAPI X [-l] <filename>\n\n");
        printf("OPTIONS\n");
        printf("   X                      channel number A or B, X={0,1}\n");
        printf("   <filename>             filename with extension .ini2 \n");
        printf("   --list                 get the list of all available LimeSDR-5GRadio boards\n");
        printf("   -r, --reset            reset the Equaliser configuration\n");
        printf("   -c, --calibrate        calibrate Equaliser of target board\n");
        printf("   -s, --save <filename>  save the Equaliser configuration of target board into the file\n");
        printf("   -l, --load <filename>  load the configuration from specified file into target board \n");
        printf("   -t, --tdd              TDD mode is selected\n\n");
        printf("   -a, --adjustgain       adjust analogue gain after calibration process\n\n");
        printf("   -d, --dc               calibrate separatelly TX DC offset\n\n");
        printf("   -h, --help             help\n\n");
        printf("AUTHORS\n");
        printf("    Srdjan Milenkovic,      s.milenkovic@limemicro.com\n");
        printf("    Borisav Jovanovic,      b.jovanovic@limemicro.com\n\n");
        return 0;
    }

    const char *devName = "X3";
    lime::SDRDevice *sdrdev = ConnectToFilteredOrDefaultDevice(devName);
    if (!sdrdev)
        return EXIT_FAILURE;

    sdrdev->SetMessageLogCallback(LogCallback);
    lime::registerLogHandler(LogCallback);

    std::string m_sTemp;
    m_sTemp = argv[1]; // chip index

    if ((m_bLoad == true) || (m_bCalibration == true) || (m_bReset == true))
        if ((m_sTemp.compare("0") != 0) && (m_sTemp.compare("1") != 0))
        {
            printf("[Warn] The function argument with appropriate index is required.\n");
            return 0;
        }

    std::string str1;
    std::string str2;

    double TxLOfreq = 2140.0; // TX LO frequency
    double RxLOfreq = 2140.0; // RX LO frequency
    double freq = 0.0;

    lime::LMS7002M *lms2 = reinterpret_cast<lime::LMS7002M *>(sdrdev->GetInternalChip(1));
    lime::LMS7002M *lms3 = reinterpret_cast<lime::LMS7002M *>(sdrdev->GetInternalChip(2));

    // check if device has CFR/equalizer
    const int32_t fpga_chipSelect = GetChipSelectByName(sdrdev, "FPGA");
    if (fpga_chipSelect < 0)
    {
        printf("Can't find FPGA SPI\n");
        terminateProgram(sdrdev);
    }

    FDQI_SPI *fpga_spi = new FDQI_SPI(sdrdev, fpga_chipSelect);

    freq = lms2->GetFrequencySX(TRXDir::Tx) / 1e6;
    printf("[Info] LMS#2 TX center frequency: %+5.1f MHz\n", freq);
    TxLOfreq = freq;

    if (m_bTdd == false)
    {
        freq = lms2->GetFrequencySX(TRXDir::Rx) / 1e6;
        printf("[Info] FDD mode - LMS#2 RX center frequency: %+5.1f MHz\n", freq);
        RxLOfreq = freq;
    }
    else
    {
        RxLOfreq = TxLOfreq;
        printf("[Info] TDD mode - LMS#2 RX center frequency: %+5.1f MHz\n", RxLOfreq);
    }

    // Setup default board settings
    FDQIE_DCCorrSettings(fpga_spi, 0, 1, 0, chA); // enable DC correction, don't track dc correction loop
    FDQIE_SetNCO(fpga_spi, refClk_MHz, 0.0, chA);
    FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0); // disable the loopback

    if (m_bLoad == true)
    {
        FDQIE_LoadDC(fpga_spi, m_sConfigFilename); // read DC configuration from file and load to FPGA
        FDQIE_LoadEqualiser(fpga_spi, m_sConfigFilename); // read Equaliser configuration from file and load to FPGA
    }
    else if (m_bReset == true)
    {
        FDQIE_ResetEqualiser(fpga_spi, chA);
        FDQIE_SetupTransmitterDC(fpga_spi, 0, 0, chA); // default
        FDQIE_SetupReceiverDC(sdrdev, fpga_spi, 0, 0, 0, chA); // default
    }
    else if ((m_bCalibration == true) || (m_bMeasurement == true))
    {

        int operationPhase = 0;
        int incRXGain = 0;
        int incTxGain = 0;

        FDQIE_ResetEqualiser(fpga_spi, chA); // reset all filter coefficients, including gain and phase corrections
        FDQIE_SetupTransmitterDC(fpga_spi, 0, 0, chA); // default
        FDQIE_SetupReceiverDC(sdrdev, fpga_spi, 0, 0, 0, chA); // default
        FDQIE_SetNCO(fpga_spi, refClk_MHz, 0.0, chA);
        FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0);
        // exit (0);
        operationPhase = 1; // Receiver measurement, phase 1
        // LMS#2 SXT LO is changed, LMS#3 is the Receiver now
        if (lms3->SetFrequencySX(TRXDir::Tx, TxLOfreq * 1e6) != OpStatus::Success)
            return EXIT_FAILURE;
        if (lms3->SetFrequencySX(TRXDir::Rx, TxLOfreq * 1e6) != OpStatus::Success)
            return EXIT_FAILURE;

        FDQIE_CalibrateReceiverDC(sdrdev, fpga_spi, numSamples, &RcodeI, &RcodeQ, chA, operationPhase);
        if (m_bMeasurement)
            FDQIE_ReceiverMeasurement(sdrdev, fpga_spi, numSamples, TxLOfreq, fs, chA, operationPhase); // SXT LO is changed
        else
        {
            FDQIE_ReceiverCalibration(sdrdev,
                fpga_spi,
                numSamples,
                TxLOfreq,
                fs,
                MeasPoints,
                f,
                r_amp1,
                r_gerr1,
                r_perr1,
                chA,
                operationPhase); // SXT LO is changed
            if (FDQIE_Check(sdrdev, fpga_spi, MeasPoints, refClk_MHz, r_perr1, chA) == 1)
                exit(0); // exit (0);
        }

        //        exit (0);
        operationPhase = 2; // Transmitter measurement, phase 2
        // LMS#2 SXT LO is fixed. The NCO is changed.  LMS#3 is receiver
        if (m_bMeasurement)
            FDQIE_TransmitterMeasurement(sdrdev, fpga_spi, numSamples, TxLOfreq, fs, chA, operationPhase); // NCO is changed
        else
        {
            FDQIE_TransmitterCalibration(sdrdev,
                fpga_spi,
                numSamples,
                TxLOfreq,
                fs,
                MeasPoints,
                f,
                t_amp,
                t_gerr,
                t_perr,
                chA,
                operationPhase); // NCO is changed
            if (FDQIE_Check(sdrdev, fpga_spi, MeasPoints, refClk_MHz, t_perr, chA) == 1)
                exit(0);
        }

        //removed DC offset here
        //FDQIE_CalibrateTransmitterDC(device, numSamples, TxLOfreq, refClk_MHz, 0.9975, &TcodeI, &TcodeQ, chA, operationPhase, true);

        // exit(0);
        //   Receiver measurement, phase 3
        operationPhase = 3;
        // LMS#3 SXT LO is changed.
        // if (lms2->SetFrequencySX(LMS_CH_RX, RxLOfreq * 1e6) != 0) //// LMS#2 is the Receiver now
        //	error(device);
        FDQIE_CalibrateReceiverDC(sdrdev, fpga_spi, numSamples, &RcodeI, &RcodeQ, chA, operationPhase);

        if (m_bMeasurement)
            FDQIE_ReceiverMeasurement(sdrdev, fpga_spi, numSamples, RxLOfreq, fs, chA, operationPhase);
        else
        {
            FDQIE_ReceiverCalibration(
                sdrdev, fpga_spi, numSamples, RxLOfreq, fs, MeasPoints, f, r_amp, r_gerr, r_perr, chA, operationPhase);
            if (FDQIE_Check(sdrdev, fpga_spi, MeasPoints, refClk_MHz, r_perr, chA) == 1)
                exit(0);
        }

        if (m_bMeasurement)
            exit(0);

        FDQImeasure(f, r_amp, r_gerr, r_perr, t_amp, t_gerr, t_perr, MeasPoints); // doing nothing, just prints

        FDQIpreproc(f,
            r_amp1,
            r_gerr1,
            r_perr1,
            t_amp,
            t_gerr,
            t_perr, // Preprocessing (normalization, de-embeding, ...)
            r_amp,
            r_gerr,
            r_perr,
            r_clk,
            t_clk,
            r_w,
            t_w,
            &incRXGain,
            &incTxGain,
            MeasPoints);

        printf("[Info] LMS#2 RECEIVER MATH MODEL\n");
        FDQIEdesign(r_wp,
            r_w,
            r_amp,
            r_gerr,
            r_perr, // RX Equaliser design
            &r_codeI,
            &r_codeQ,
            &r_codeAlpha,
            r_Ntaps,
            r_hI,
            r_hQ,
            !tx,
            chA,
            MeasPoints);
        printf("[Info] LMS#2 TRANSMITTER MATH MODEL\n");
        FDQIEdesign(t_wp,
            t_w,
            t_amp,
            t_gerr,
            t_perr, // TX Equaliser design
            &t_codeI,
            &t_codeQ,
            &t_codeAlpha,
            t_Ntaps,
            t_hI,
            t_hQ,
            tx,
            chA,
            MeasPoints);

        FDQIE_SaveConfig(fpga_spi,
            t_Ntaps,
            t_codeI,
            t_codeQ,
            t_codeAlpha,
            t_hI,
            t_hQ,
            r_codeI,
            r_codeQ,
            r_codeAlpha,
            r_hI,
            r_hQ,
            chA); // store equaliser configuration into FPGA

        if (m_bAdjustGain)
            FDQIE_AdjustGain(sdrdev, incRXGain, incTxGain, chA);

        if (m_bSave == true)
        {
            FDQIE_SaveEqualiser(fpga_spi, t_Ntaps, m_sConfigFilename); // read configuration from FPGA and store to file
            FDQIE_SaveDC(fpga_spi, refClk_MHz, m_sConfigFilename); // read DC configuration from FPGA and LMS7 and store to file
        }

        FDQIE_SetNCO(fpga_spi, refClk_MHz, 0.0, chA); // reset NCO
        FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0); // disable the loopback
            // Terminate50Ohm(device, 0);						// Release the Receiver input
    }

    else if (m_bTXDC)
    {

        int operationPhase = 2; // Transmitter measurement, phase 2
        FDQIE_CalibrateTransmitterDC(
            sdrdev, fpga_spi, numSamples, TxLOfreq, refClk_MHz, 0.9975, &TcodeI, &TcodeQ, chA, operationPhase, false);

        FDQIE_SetNCO(fpga_spi, refClk_MHz, 0.0, chA); // reset NCO
        FDQIE_LoopbackSettings(sdrdev, fpga_spi, chA, 0); // disable the loopback
    }
    else if (m_bSave == true)
    {

        FDQIE_SaveEqualiser(fpga_spi, t_Ntaps, m_sConfigFilename); // read configuration from FPGA and store to file
        FDQIE_SaveDC(fpga_spi, refClk_MHz, m_sConfigFilename); // read DC configuration from FPGA and LMS7 and store to file
    }

    DeviceRegistry::freeDevice(sdrdev);
}
