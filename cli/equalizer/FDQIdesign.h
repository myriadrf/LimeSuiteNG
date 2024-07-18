#pragma once

#include "limesuiteng/limesuiteng.hpp"
#include "FDQIspi.h"

lime::OpStatus SetRegValue(lime::ISPI *fpga_spi, const lime::Register &reg, uint16_t value);
lime::OpStatus SetRegValue(lime::ISPI *fpga_spi, uint16_t addr, uint16_t value);
uint16_t GetRegValue(lime::ISPI *fpga_spi, uint16_t addr);
void FDQIE_SetupTransmitterTone(lime::SDRDevice *pLmsControl, bool m_bMax);
int terminateProgram(lime::SDRDevice *device);
void FDQIE_ReadData(lime::SDRDevice *pLmsControl,
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
    int operationPhase);

/* Function declarations */
void FDQImeasure(
    double *f, double *r_amp, double *r_gerr, double *r_perr, double *t_amp, double *t_gerr, double *t_perr, int MeasPoints);
void FDQIEdesign(double wp,
    double *w,
    double *amp,
    double *gerr,
    double *perr,
    int *codeI,
    int *codeQ,
    int *codeAlpha,
    int Ntaps,
    int *hI,
    int *hQ,
    bool tx,
    int slaveDevice,
    int MeasPoints);
void FDQIpreproc(double *f,
    double *r_amp1,
    double *r_gerr1,
    double *r_perr1,
    double *t_amp,
    double *t_gerr,
    double *t_perr,
    double *r_amp,
    double *r_gerr,
    double *r_perr,
    double r_clk,
    double t_clk,
    double *r_w,
    double *t_w,
    int *incRXGain,
    int *incTxGain,
    int MeasPoints);

// load/store functions
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
    bool chA);
void FDQIE_ResetEqualiser(lime::ISPI *fpga_spi, bool chA);

void FDQIE_SaveEqualiser(lime::ISPI *fpga_spi, int Ntaps, std::string m_sConfigFilename);
void FDQIE_LoadEqualiser(lime::ISPI *fpga_spi, const std::string &m_sConfigFilename);

void FDQIE_SaveDC(lime::ISPI *fpga_spi, double RefClk, const std::string &m_sConfigFilename);
void FDQIE_LoadDC(lime::ISPI *fpga_spi, const std::string &m_sConfigFilename);

void FDQIE_SetNCO(lime::ISPI *fpga_spi, double refClk_MHz, double value, bool chA);
void FDQIE_LoopbackSettings(lime::SDRDevice *sdrdev, lime::ISPI *fpga_spi, int chA, int operationPhase);
void FDQIE_DCCorrSettings(lime::ISPI *fpga_spi, bool dccorrbypass, bool dcloopbypass, bool chA, int operationPhase);
void FDQIE_SetupReceiverDC(lime::SDRDevice *sdr, lime::ISPI *fpga_spi, int16_t codeI, int16_t codeQ, bool chA, int operationPhase);
void FDQIE_SetupTransmitterDC(lime::ISPI *fpga_spi, int16_t codeI, int16_t codeQ, bool chA);

// calibration functions
void FDQIE_CalibrateReceiverDC(
    lime::SDRDevice *sdrdev, lime::ISPI *fpga_spi, int numSamples, int *R_codeI, int *R_codeQ, bool chA, int operationPhase);
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
    bool nco);
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
    int operationPhase);
void FDQIE_ReceiverMeasurement(
    lime::SDRDevice *sdrdev, lime::ISPI *fpga_spi, int32_t numSamples, double LOfreq, double fs, bool chA, int operationPhase);
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
    int operationPhase);
void FDQIE_TransmitterMeasurement(
    lime::SDRDevice *sdrdev, lime::ISPI *fpga_spi, int32_t numSamples, double LOfreq, double fs, bool chA, int operationPhase);
void FDQIE_AdjustGain(lime::SDRDevice *sdrdev, int incRXGain, int incTxGain, bool chA);
