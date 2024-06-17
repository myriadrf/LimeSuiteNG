#include "LMS7002MCSR_Data.h"
#include "limesuiteng/LMS7002MCSR.h"

#include <functional>
#include <vector>

namespace lime {
namespace LMS7002MCSR_Data {
const CSRegister LRST_TX_B = {
    0x0020, 15, 15, 1, "LRST_TX_B", "Resets all the logic registers to the default state for Tx MIMO channel B"
};
const CSRegister MRST_TX_B = {
    0x0020, 14, 14, 1, "MRST_TX_B", "Resets all the configuration memory to the default state for Tx MIMO channel B"
};
const CSRegister LRST_TX_A = {
    0x0020, 13, 13, 1, "LRST_TX_A", "Resets all the logic registers to the default state for Tx MIMO channel A"
};
const CSRegister MRST_TX_A = {
    0x0020, 12, 12, 1, "MRST_TX_A", "Resets all the configuration memory to the default state for Tx MIMO channel A"
};
const CSRegister LRST_RX_B = {
    0x0020, 11, 11, 1, "LRST_RX_B", "Resets all the logic registers to the default state for Rx MIMO channel B"
};
const CSRegister MRST_RX_B = {
    0x0020, 10, 10, 1, "MRST_RX_B", "Resets all the configuration memory to the default state for Rx MIMO channel B"
};
const CSRegister LRST_RX_A = {
    0x0020, 9, 9, 1, "LRST_RX_A", "Resets all the logic registers to the default state for Rx MIMO channel A"
};
const CSRegister MRST_RX_A = {
    0x0020, 8, 8, 1, "MRST_RX_A", "Resets all the configuration memory to the default state for Rx MIMO channel A"
};
const CSRegister SRST_RXFIFO = { 0x0020, 7, 7, 1, "SRST_RXFIFO", "RX FIFO soft reset (LimeLight Interface)" };
const CSRegister SRST_TXFIFO = { 0x0020, 6, 6, 1, "SRST_TXFIFO", "TX FIFO soft reset (LimeLight Interface)" };
const CSRegister RXEN_B = { 0x0020, 5, 5, 1, "RXEN_B", "Power control for Rx MIMO channel B" };
const CSRegister RXEN_A = { 0x0020, 4, 4, 1, "RXEN_A", "Power control for Rx MIMO channel A" };
const CSRegister TXEN_B = { 0x0020, 3, 3, 1, "TXEN_B", "Power control for Tx MIMO channel B" };
const CSRegister TXEN_A = { 0x0020, 2, 2, 1, "TXEN_A", "Power control for Tx MIMO channel A" };
const CSRegister MAC = { 0x0020, 1, 0, 3, "MAC", "Selects MIMO channel for communication" };
const CSRegister TX_CLK_PE = { 0x0021, 11, 11, 1, "TX_CLK_PE", "Pull up control of TX_CLK pad" };
const CSRegister RX_CLK_PE = { 0x0021, 10, 10, 1, "RX_CLK_PE", "Pull up control of RX_CLK pad" };
const CSRegister SDA_PE = { 0x0021, 9, 9, 1, "SDA_PE", "Pull up control of SDA pad" };
const CSRegister SDA_DS = { 0x0021, 8, 8, 0, "SDA_DS", "Driver strength of SDA pad" };
const CSRegister SCL_PE = { 0x0021, 7, 7, 1, "SCL_PE", "Pull up control of SCL pad" };
const CSRegister SCL_DS = { 0x0021, 6, 6, 0, "SCL_DS", "Driver strength of SCL pad" };
const CSRegister SDIO_DS = { 0x0021, 5, 5, 0, "SDIO_DS", "Driver strength of SDIO pad" };
const CSRegister SDIO_PE = { 0x0021, 4, 4, 1, "SDIO_PE", "Pull up control of SDIO pad" };
const CSRegister SDO_PE = { 0x0021, 3, 3, 1, "SDO_PE", "Pull up control of SDO pad" };
const CSRegister SCLK_PE = { 0x0021, 2, 2, 1, "SCLK_PE", "Pull up control of SCLK pad" };
const CSRegister SEN_PE = { 0x0021, 1, 1, 1, "SEN_PE", "Pull up control of SEN pad" };
const CSRegister SPIMODE = { 0x0021, 0, 0, 1, "SPIMODE", "SPI communication mode" };
const CSRegister DIQ2_DS = { 0x0022, 11, 11, 0, "DIQ2_DS", "Driver strength of DIQ2 pad" };
const CSRegister DIQ2_PE = { 0x0022, 10, 10, 1, "DIQ2_PE", "Pull up control of DIQ2 pad" };
const CSRegister IQ_SEL_EN_2_PE = { 0x0022, 9, 9, 1, "IQ_SEL_EN_2_PE", "Pull up control of IQ_SEL_EN_2 pad" };
const CSRegister TXNRX2_PE = { 0x0022, 8, 8, 1, "TXNRX2_PE", "Pull up control of TXNRX2 pad" };
const CSRegister FCLK2_PE = { 0x0022, 7, 7, 1, "FCLK2_PE", "Pull up control of FCLK2 pad" };
const CSRegister MCLK2_PE = { 0x0022, 6, 6, 1, "MCLK2_PE", "Pull up control of MCLK2 pad" };
const CSRegister DIQ1_DS = { 0x0022, 5, 5, 0, "DIQ1_DS", "Pull up control of MCLK2 pad" };
const CSRegister DIQ1_PE = { 0x0022, 4, 4, 1, "DIQ1_PE", "Pull up control of DIQ1 pad" };
const CSRegister IQ_SEL_EN_1_PE = { 0x0022, 3, 3, 1, "IQ_SEL_EN_1_PE", "Pull up control of IQ_SEL_EN_1 pad" };
const CSRegister TXNRX1_PE = { 0x0022, 2, 2, 1, "TXNRX1_PE", "Pull up control of TXNRX1 pad" };
const CSRegister FCLK1_PE = { 0x0022, 1, 1, 1, "FCLK1_PE", "Pull up control of FCLK1 pad" };
const CSRegister MCLK1_PE = { 0x0022, 0, 0, 1, "MCLK1_PE", "Pull up control of MCLK1 pad" };
const CSRegister DIQDIRCTR2 = { 0x0023, 15, 15, 0, "DIQDIRCTR2", "DIQ2 direction control mode" };
const CSRegister DIQDIR2 = { 0x0023, 14, 14, 1, "DIQDIR2", "DIQ2 direction" };
const CSRegister DIQDIRCTR1 = { 0x0023, 13, 13, 0, "DIQDIRCTR1", "DIQ1 direction control mode" };
const CSRegister DIQDIR1 = { 0x0023, 12, 12, 1, "DIQDIR1", "DIQ1 direction" };
const CSRegister ENABLEDIRCTR2 = { 0x0023, 11, 11, 0, "ENABLEDIRCTR2", "ENABLE2 direction control mode" };
const CSRegister ENABLEDIR2 = { 0x0023, 10, 10, 1, "ENABLEDIR2", "ENABLE2 direction" };
const CSRegister ENABLEDIRCTR1 = { 0x0023, 9, 9, 0, "ENABLEDIRCTR1", "ENABLE1 direction control mode" };
const CSRegister ENABLEDIR1 = { 0x0023, 8, 8, 1, "ENABLEDIR1", "ENABLE1 direction." };
const CSRegister MOD_EN = { 0x0023, 6, 6, 1, "MOD_EN", "LimeLight interface enable" };
const CSRegister LML2_FIDM = { 0x0023, 5, 5, 0, "LML2_FIDM", "Frame start ID selection for Port 2 when LML_MODE2 = 0" };
const CSRegister LML2_TXNRXIQ = { 0x0023, 4, 4, 1, "LML2_TXNRXIQ", "TXIQ/RXIQ mode selection for Port 2 when LML_MODE2 = 0" };
const CSRegister LML2_MODE = { 0x0023, 3, 3, 1, "LML2_MODE", "Mode of LimeLight Port 2" };
const CSRegister LML1_FIDM = { 0x0023, 2, 2, 0, "LML1_FIDM", "Frame start ID selection for Port 1 when LML_MODE1 = 0" };
const CSRegister LML1_TXNRXIQ = { 0x0023, 1, 1, 0, "LML1_TXNRXIQ", "TXIQ/RXIQ mode selection for Port 1 when LML_MODE1 = 0" };
const CSRegister LML1_MODE = { 0x0023, 0, 0, 1, "LML1_MODE", "Mode of LimeLight Port 1" };
const CSRegister LML1_S3S = { 0x0024, 15, 14, 3, "LML1_S3S", "Sample source in position 3, when Port 1 is RF2BB" };
const CSRegister LML1_S2S = { 0x0024, 13, 12, 2, "LML1_S2S", "Sample source in position 2, when Port 1 is RF2BB" };
const CSRegister LML1_S1S = { 0x0024, 11, 10, 1, "LML1_S1S", "Sample source in position 1, when Port 1 is RF2BB" };
const CSRegister LML1_S0S = { 0x0024, 9, 8, 0, "LML1_S0S", "Sample source in position 0, when Port 1 is RF2BB" };
const CSRegister LML1_BQP = { 0x0024, 7, 6, 3, "LML1_BQP", "BQ sample position in frame, when Port 1 is BB2RF" };
const CSRegister LML1_BIP = { 0x0024, 5, 4, 2, "LML1_BIP", "BI sample position in frame, when Port 1 is BB2RF" };
const CSRegister LML1_AQP = { 0x0024, 3, 2, 1, "LML1_AQP", "AQ sample position in frame, when Port 1 is BB2RF" };
const CSRegister LML1_AIP = { 0x0024, 1, 0, 0, "LML1_AIP", "AI sample position in frame, when Port 1 is BB2RF" };
const CSRegister LML1_BB2RF_PST = {
    0x0025,
    15,
    8,
    1,
    "LML1_BB2RF_PST",
    "Number of clock cycles to wait before data drive stop after burst stop is detected in JESD207 mode on Port 1 and Port 1 BB2RF"
};
const CSRegister LML1_BB2RF_PRE = { 0x0025,
    7,
    0,
    1,
    "LML1_BB2RF_PRE",
    "Number of clock cycles to wait before data drive start after burst start is detected in JESD207 mode on Port 1 and Port 1 "
    "BB2RF" };
const CSRegister LML1_RF2BB_PST = { 0x0026,
    15,
    8,
    1,
    "LML1_RF2BB_PST",
    "Number of clock cycles to wait before data capture stop after burst stop is detected in JESD207 mode on Port 1 and Port 1 is "
    "RF2BB" };
const CSRegister LML1_RF2BB_PRE = { 0x0026,
    7,
    0,
    1,
    "LML1_RF2BB_PRE",
    "Number of clock cycles to wait before data capture start after burst start is detected in JESD207 mode on Port 1 and Port 1 "
    "RF2BB" };
const CSRegister LML2_S3S = { 0x0027, 15, 14, 3, "LML2_S3S", "Sample source in position 3, when Port 2 is RF2BB" };
const CSRegister LML2_S2S = { 0x0027, 13, 12, 2, "LML2_S2S", "Sample source in position 2, when Port 2 is RF2BB" };
const CSRegister LML2_S1S = { 0x0027, 11, 10, 1, "LML2_S1S", "Sample source in position 1, when Port 2 is RF2BB" };
const CSRegister LML2_S0S = { 0x0027, 9, 8, 0, "LML2_S0S", "Sample source in position 0, when Port 2 is RF2BB" };
const CSRegister LML2_BQP = { 0x0027, 7, 6, 3, "LML2_BQP", "BQ sample position in frame, when Port 2 is BB2RF" };
const CSRegister LML2_BIP = { 0x0027, 5, 4, 2, "LML2_BIP", "BI sample position in frame, when Port 2 is BB2RF" };
const CSRegister LML2_AQP = { 0x0027, 3, 2, 1, "LML2_AQP", "AQ sample position in frame, when Port 2 is BB2RF" };
const CSRegister LML2_AIP = { 0x0027, 1, 0, 0, "LML2_AIP", "AI sample position in frame, when Port 2 is BB2RF" };
const CSRegister LML2_BB2RF_PST = {
    0x0028,
    15,
    8,
    1,
    "LML2_BB2RF_PST",
    "Number of clock cycles to wait before data drive stop after burst stop is detected in JESD207 mode on Port 2 and Port 2 BB2RF"
};
const CSRegister LML2_BB2RF_PRE = { 0x0028,
    7,
    0,
    1,
    "LML2_BB2RF_PRE",
    "Number of clock cycles to wait before data drive start after burst start is detected in JESD207 mode on Port 2 and Port 2 "
    "BB2RF" };
const CSRegister LML2_RF2BB_PST = { 0x0029,
    15,
    8,
    1,
    "LML2_RF2BB_PST",
    "Number of clock cycles to wait before data capture stop after burst stop is detected in JESD207 mode on Port 2 and Port 2 "
    "RF2BB" };
const CSRegister LML2_RF2BB_PRE = { 0x0029,
    7,
    0,
    1,
    "LML2_RF2BB_PRE",
    "Number of clock cycles to wait before data capture start after burst start is detected in JESD207 mode on Port 2 and Port 2 "
    "RF2BB" };
const CSRegister FCLK2_DLY = { 0x002A, 15, 14, 0, "FCLK2_DLY", "" };
const CSRegister FCLK1_DLY = { 0x002A, 13, 12, 0, "FCLK1_DLY", "" };
const CSRegister RX_MUX = { 0x002A, 11, 10, 0, "RX_MUX", "RxFIFO data source selection" };
const CSRegister TX_MUX = { 0x002A, 9, 8, 0, "TX_MUX", "Port selection for data transmit to TSP" };
const CSRegister TXRDCLK_MUX = { 0x002A, 7, 6, 2, "TXRDCLK_MUX", "TX FIFO read clock selection" };
const CSRegister TXWRCLK_MUX = { 0x002A, 5, 4, 0, "TXWRCLK_MUX", "TX FIFO write clock selection" };
const CSRegister RXRDCLK_MUX = { 0x002A, 3, 2, 1, "RXRDCLK_MUX", "RX FIFO read clock selection" };
const CSRegister RXWRCLK_MUX = { 0x002A, 1, 0, 2, "RXWRCLK_MUX", "RX FIFO write clock selection" };
const CSRegister FCLK2_INV = { 0x002B, 15, 15, 0, "FCLK2_INV", "FCLK2 clock inversion" };
const CSRegister FCLK1_INV = { 0x002B, 14, 14, 0, "FCLK1_INV", "FCLK1 clock inversion" };
const CSRegister MCLK2DLY = { 0x002B, 13, 12, 0, "MCLK2DLY", "MCLK2 clock internal delay" };
const CSRegister MCLK1DLY = { 0x002B, 11, 10, 0, "MCLK1DLY", "MCLK1 clock internal delay" };
const CSRegister MCLK2SRC = { 0x002B, 5, 4, 1, "MCLK2SRC", "MCLK2 clock source" };
const CSRegister MCLK1SRC = { 0x002B, 3, 2, 0, "MCLK1SRC", "MCLK1 clock source" };
const CSRegister TXDIVEN = { 0x002B, 1, 1, 0, "TXDIVEN", "TX clock divider enable" };
const CSRegister RXDIVEN = { 0x002B, 0, 0, 0, "RXDIVEN", "RX clock divider enable" };
const CSRegister TXTSPCLKA_DIV = { 0x002C,
    15,
    8,
    255,
    "TXTSPCLKA_DIV",
    "TxTSP clock divider, used to produce MCLK(1/2). Clock division ratio is 2(TXTSPCLKA_DIV + 1)" };
const CSRegister RXTSPCLKA_DIV = { 0x002C,
    7,
    0,
    255,
    "RXTSPCLKA_DIV",
    "RxTSP clock divider, used to produce MCLK(1/2). Clock division ratio is 2(RXTSPCLKA_DIV + 1)" };
const CSRegister MIMO_SISO = { 0x002E, 15, 15, 0, "MIMO_SISO", "MIMO channel B enable control" };
const CSRegister VER = { 0x002F, 15, 11, 7, "VER", "" };
const CSRegister REV = { 0x002F, 10, 6, 1, "REV", "" };
const CSRegister MASK = { 0x002F, 5, 0, 1, "MASK", "" };
const CSRegister EN_DIR_LDO = { 0x0081, 3, 3, 0, "EN_DIR_LDO", "" };
const CSRegister EN_DIR_CGEN = { 0x0081, 2, 2, 0, "EN_DIR_CGEN", "" };
const CSRegister EN_DIR_XBUF = { 0x0081, 1, 1, 0, "EN_DIR_XBUF", "" };
const CSRegister EN_DIR_AFE = { 0x0081, 0, 0, 0, "EN_DIR_AFE", "" };
const CSRegister ISEL_DAC_AFE = { 0x0082, 15, 13, 4, "ISEL_DAC_AFE", "Controls the peak current of the DAC output current" };
const CSRegister MODE_INTERLEAVE_AFE = { 0x0082, 12, 12, 0, "MODE_INTERLEAVE_AFE", "time interleaves the two ADCs into one ADC" };
const CSRegister MUX_AFE_1 = { 0x0082, 11, 10, 0, "MUX_AFE_1", "Controls the MUX at the input of the ADC channel 1" };
const CSRegister MUX_AFE_2 = { 0x0082, 9, 8, 0, "MUX_AFE_2", "Controls the MUX at the input of the ADC channel 2" };
const CSRegister PD_AFE = { 0x0082, 5, 5, 0, "PD_AFE", "Power down control for the AFE current mirror in BIAS_TOP" };
const CSRegister PD_RX_AFE1 = { 0x0082, 4, 4, 0, "PD_RX_AFE1", "Power down control for the ADC of channel 1" };
const CSRegister PD_RX_AFE2 = { 0x0082, 3, 3, 1, "PD_RX_AFE2", "Power down control for the ADC of channel 2" };
const CSRegister PD_TX_AFE1 = { 0x0082, 2, 2, 0, "PD_TX_AFE1", "Power down control for the DAC of channel 1" };
const CSRegister PD_TX_AFE2 = { 0x0082, 1, 1, 1, "PD_TX_AFE2", "Power down control for the DAC of channel 2" };
const CSRegister EN_G_AFE = { 0x0082, 0, 0, 1, "EN_G_AFE", "Enable control for all the AFE power downs" };
const CSRegister MUX_BIAS_OUT = { 0x0084, 12, 11, 0, "MUX_BIAS_OUT", "Test mode of the BIAS_TOP" };
const CSRegister RP_CALIB_BIAS = { 0x0084,
    10,
    6,
    16,
    "RP_CALIB_BIAS",
    "Calibration code for rppolywo. This code is set by the calibration algorithm: BIAS_RPPOLY_calibration" };
const CSRegister PD_FRP_BIAS = { 0x0084, 4, 4, 0, "PD_FRP_BIAS", "Power down signal for Fix/RP block" };
const CSRegister PD_F_BIAS = { 0x0084, 3, 3, 0, "PD_F_BIAS", "Power down signal for Fix" };
const CSRegister PD_PTRP_BIAS = { 0x0084, 2, 2, 0, "PD_PTRP_BIAS", "Power down signal for PTAT/RP block" };
const CSRegister PD_PT_BIAS = { 0x0084, 1, 1, 0, "PD_PT_BIAS", "Power down signal for PTAT block" };
const CSRegister PD_BIAS_MASTER = { 0x0084, 0, 0, 0, "PD_BIAS_MASTER", "Enable signal for central bias block" };
const CSRegister SLFB_XBUF_RX = { 0x0085, 8, 8, 0, "SLFB_XBUF_RX", "Self biasing digital contol SLFB_XBUF_RX" };
const CSRegister SLFB_XBUF_TX = { 0x0085, 7, 7, 0, "SLFB_XBUF_TX", "Self biasing digital contol SLFB_XBUF_TX" };
const CSRegister BYP_XBUF_RX = { 0x0085, 6, 6, 0, "BYP_XBUF_RX", "Shorts the Input 3.3V buffer in XBUF" };
const CSRegister BYP_XBUF_TX = { 0x0085, 5, 5, 0, "BYP_XBUF_TX", "Shorts the Input 3.3V buffer in XBUF" };
const CSRegister EN_OUT2_XBUF_TX = { 0x0085,
    4,
    4,
    0,
    "EN_OUT2_XBUF_TX",
    "Enables the 2nd output of TX XBUF. This 2nd buffer goes to XBUF_RX. This should be active when only 1 XBUF is to be used" };
const CSRegister EN_TBUFIN_XBUF_RX = { 0x0085,
    3,
    3,
    0,
    "EN_TBUFIN_XBUF_RX",
    "Disables the input from the external XOSC and buffers the 2nd input signal (from TX XBUF 2nd output) to the RX. This should "
    "be active when only 1 XBUF is to be used" };
const CSRegister PD_XBUF_RX = { 0x0085, 2, 2, 0, "PD_XBUF_RX", "Power down signal PD_XBUF_RX" };
const CSRegister PD_XBUF_TX = { 0x0085, 1, 1, 0, "PD_XBUF_TX", "Power down signal PD_XBUF_TX" };
const CSRegister EN_G_XBUF = { 0x0085, 0, 0, 1, "EN_G_XBUF", "Enable control for all the XBUF power downs" };
const CSRegister SPDUP_VCO_CGEN = { 0x0086,
    15,
    15,
    0,
    "SPDUP_VCO_CGEN",
    "Bypasses the noise filter resistor for fast setlling time. It should be connected to a 1us pulse" };
const CSRegister RESET_N_CGEN = {
    0x0086, 14, 14, 1, "RESET_N_CGEN", "A pulse should be used in the start-up to reset ( 1-normal operation)"
};
const CSRegister EN_ADCCLKH_CLKGN = {
    0x0086, 11, 11, 1, "EN_ADCCLKH_CLKGN", "Selects if F_CLKH or F_CLKL is connected to FCLK_ADC"
};
const CSRegister EN_COARSE_CKLGEN = { 0x0086, 10, 10, 0, "EN_COARSE_CKLGEN", "Enable signal for coarse tuning block" };
const CSRegister EN_INTONLY_SDM_CGEN = { 0x0086, 9, 9, 0, "EN_INTONLY_SDM_CGEN", "Enables INTEGER-N mode of the SX " };
const CSRegister EN_SDM_CLK_CGEN = {
    0x0086, 8, 8, 1, "EN_SDM_CLK_CGEN", "Enables/Disables SDM clock. In INT-N mode or for noise testing, SDM clock can be disabled"
};
const CSRegister PD_CP_CGEN = { 0x0086, 6, 6, 0, "PD_CP_CGEN", "Power down for Charge Pump" };
const CSRegister PD_FDIV_FB_CGEN = { 0x0086, 5, 5, 0, "PD_FDIV_FB_CGEN", "Power down for feedback frequency divider" };
const CSRegister PD_FDIV_O_CGEN = {
    0x0086, 4, 4, 0, "PD_FDIV_O_CGEN", "Power down for forward frequency divider of the CGEN block"
};
const CSRegister PD_SDM_CGEN = { 0x0086, 3, 3, 0, "PD_SDM_CGEN", "Power down for SDM" };
const CSRegister PD_VCO_CGEN = { 0x0086, 2, 2, 0, "PD_VCO_CGEN", "Power down for VCO" };
const CSRegister PD_VCO_COMP_CGEN = { 0x0086, 1, 1, 0, "PD_VCO_COMP_CGEN", "Power down for VCO comparator" };
const CSRegister EN_G_CGEN = { 0x0086, 0, 0, 1, "EN_G_CGEN", "Enable control for all the CGEN power downs" };
const CSRegister FRAC_SDM_CGEN_LSB = { 0x0087, 15, 0, 0x0400, "FRAC_SDM_CGEN_LSB", "" };
const CSRegister INT_SDM_CGEN = { 0x0088, 13, 4, 120, "INT_SDM_CGEN", "" };
const CSRegister FRAC_SDM_CGEN_MSB = { 0x0088, 3, 0, 0, "FRAC_SDM_CGEN_MSB", "" };
const CSRegister REV_SDMCLK_CGEN = { 0x0089, 15, 15, 0, "REV_SDMCLK_CGEN", "Reverses the SDM clock" };
const CSRegister SEL_SDMCLK_CGEN = {
    0x0089, 14, 14, 0, "SEL_SDMCLK_CGEN", "Selects between the feedback divider output and Fref for SDM"
};
const CSRegister SX_DITHER_EN_CGEN = { 0x0089, 13, 13, 0, "SX_DITHER_EN_CGEN", "Enabled dithering in SDM" };
const CSRegister CLKH_OV_CLKL_CGEN = { 0x0089,
    12,
    11,
    0,
    "CLKH_OV_CLKL_CGEN",
    "FCLKL here is ADC clock. FCLKH is the clock to the DAC and if no division is added to the ADC as well" };
const CSRegister DIV_OUTCH_CGEN = { 0x0089, 10, 3, 4, "DIV_OUTCH_CGEN", "" };
const CSRegister TST_CGEN = { 0x0089, 2, 0, 0, "TST_CGEN", "Controls the test mode of the SX\n\
    0 - TST disabled; RSSI analog outputs enabled if RSSI blocks active and when all PLL test signals are off\n\
    1 - tstdo[0] = ADC clock; tstdo[1] = DAC clock; tstao = High impedance;\n\
    2 - tstdo[0] = SDM clock; tstdo[1] = feedback divider output; tstao = VCO tune through a 60kOhm resistor;\n\
    3 - tstdo[0] = Reference clock; tstdo[1] = feedback divider output; tstao = VCO tune through a 10kOhm resistor;\n\
    4 - tstdo[0] = High impedance; tstdo[1] = High impedance; tstao = High impedance;\n\
    5 - tstdo[0] = Charge pump Down signal; tstdo[1] = Charge pump Up signal; tstao = High impedance;\n\
    6 - tstdo[0] = High impedance; tstdo[1] = High impedance; tstao = VCO tune through a 60kOhm resistor;\n\
    7 - tstdo[0] = High impedance; tstdo[1] = High impedance; tstao = VCO tune through a 10kOhm resistor;\n\
    if TST_SX[2] = 1 --> VCO_TSTBUF active generating VCO_TST_DIV20 and VCO_TST_DIV40" };
const CSRegister REV_CLKDAC_CGEN = { 0x008A, 14, 14, 0, "REV_CLKDAC_CGEN", "Inverts the clock F_CLKL" };
const CSRegister CMPLO_CTRL_CGEN = { 0x008B, 14, 14, 0, "CMPLO_CTRL_CGEN", "" };
const CSRegister REV_CLKADC_CGEN = { 0x008A, 13, 13, 0, "REV_CLKADC_CGEN", "Inverts the clock F_CLKL" };
const CSRegister REVPH_PFD_CGEN = { 0x008A,
    12,
    12,
    0,
    "REVPH_PFD_CGEN",
    "Reverse the pulses of PFD. It can be used to reverse the polarity of the PLL loop (positive feedback to negative feedback)" };
const CSRegister IOFFSET_CP_CGEN = { 0x008A,
    11,
    6,
    20,
    "IOFFSET_CP_CGEN",
    "Scales the offset current of the charge pump, 0-->63. This current is used in Fran-N mode to create an offset in the CP "
    "response and avoide the non-linear section" };
const CSRegister IPULSE_CP_CGEN = { 0x008A, 5, 0, 20, "IPULSE_CP_CGEN", "Scales the pulse current of the charge pump" };
const CSRegister ICT_VCO_CGEN = { 0x008B, 13, 9, 16, "ICT_VCO_CGEN", "Scales the VCO bias current from 0 to 2.5xInom" };
const CSRegister CSW_VCO_CGEN = { 0x008B, 8, 1, 128, "CSW_VCO_CGEN", "" };
const CSRegister COARSE_START_CGEN = {
    0x008B, 0, 0, 0, "COARSE_START_CGEN", "Control signal for coarse tuning algorithm (SX_SWC_calibration)"
};
const CSRegister COARSE_STEPDONE_CGEN = { 0x008C, 15, 15, 0, "COARSE_STEPDONE_CGEN", "" };
const CSRegister COARSEPLL_COMPO_CGEN = { 0x008C, 14, 14, 0, "COARSEPLL_COMPO_CGEN", "" };
const CSRegister VCO_CMPHO_CGEN = { 0x008C, 13, 13, 0, "VCO_CMPHO_CGEN", "" };
const CSRegister VCO_CMPLO_CGEN = { 0x008C, 12, 12, 0, "VCO_CMPLO_CGEN", "" };
const CSRegister CP2_CGEN = {
    0x008C, 11, 8, 6, "CP2_CGEN", "Controls the value of CP2 (cap from CP output to GND) in the PLL filter"
};
const CSRegister CP3_CGEN = {
    0x008C, 7, 4, 7, "CP3_CGEN", "Controls the value of CP3 (cap from VCO Vtune input to GND) in the PLL filter"
};
const CSRegister CZ_CGEN = { 0x008C, 3, 0, 11, "CZ_CGEN", "Controls the value of CZ (Zero capacitor) in the PLL filter" };
const CSRegister EN_LDO_DIG = { 0x0092, 15, 15, 0, "EN_LDO_DIG", "Enables the LDO" };
const CSRegister EN_LDO_DIGGN = { 0x0092, 14, 14, 0, "EN_LDO_DIGGN", "Enables the LDO" };
const CSRegister EN_LDO_DIGSXR = { 0x0092, 13, 13, 0, "EN_LDO_DIGSXR", "Enables the LDO" };
const CSRegister EN_LDO_DIGSXT = { 0x0092, 12, 12, 0, "EN_LDO_DIGSXT", "Enables the LDO" };
const CSRegister EN_LDO_DIVGN = { 0x0092, 11, 11, 0, "EN_LDO_DIVGN", "Enables the LDO" };
const CSRegister EN_LDO_DIVSXR = { 0x0092, 10, 10, 0, "EN_LDO_DIVSXR", "Enables the LDO" };
const CSRegister EN_LDO_DIVSXT = { 0x0092, 9, 9, 0, "EN_LDO_DIVSXT", "Enables the LDO" };
const CSRegister EN_LDO_LNA12 = { 0x0092, 8, 8, 0, "EN_LDO_LNA12", "Enables the LDO" };
const CSRegister EN_LDO_LNA14 = { 0x0092, 7, 7, 0, "EN_LDO_LNA14", "Enables the LDO" };
const CSRegister EN_LDO_MXRFE = { 0x0092, 6, 6, 0, "EN_LDO_MXRFE", "Enables the LDO" };
const CSRegister EN_LDO_RBB = { 0x0092, 5, 5, 0, "EN_LDO_RBB", "Enables the LDO" };
const CSRegister EN_LDO_RXBUF = { 0x0092, 4, 4, 0, "EN_LDO_RXBUF", "Enables the LDO" };
const CSRegister EN_LDO_TBB = { 0x0092, 3, 3, 0, "EN_LDO_TBB", "Enables the LDO" };
const CSRegister EN_LDO_TIA12 = { 0x0092, 2, 2, 0, "EN_LDO_TIA12", "Enables the LDO" };
const CSRegister EN_LDO_TIA14 = { 0x0092, 1, 1, 0, "EN_LDO_TIA14", "Enables the LDO" };
const CSRegister EN_G_LDO = { 0x0092, 0, 0, 1, "EN_G_LDO", "Enable control for all the LDO power downs" };
const CSRegister EN_LOADIMP_LDO_TLOB = {
    0x0093, 15, 15, 0, "EN_LOADIMP_LDO_TLOB", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_TPAD = {
    0x0093, 14, 14, 0, "EN_LOADIMP_LDO_TPAD", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_TXBUF = {
    0x0093, 13, 13, 0, "EN_LOADIMP_LDO_TXBUF", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_VCOGN = {
    0x0093, 12, 12, 0, "EN_LOADIMP_LDO_VCOGN", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_VCOSXR = {
    0x0093, 11, 11, 0, "EN_LOADIMP_LDO_VCOSXR", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_VCOSXT = {
    0x0093, 10, 10, 0, "EN_LOADIMP_LDO_VCOSXT", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LDO_AFE = { 0x0093, 9, 9, 0, "EN_LDO_AFE", "Enables the LDO" };
const CSRegister EN_LDO_CPGN = { 0x0093, 8, 8, 0, "EN_LDO_CPGN", "Enables the LDO" };
const CSRegister EN_LDO_CPSXR = { 0x0093, 7, 7, 0, "EN_LDO_CPSXR", "Enables the LDO" };
const CSRegister EN_LDO_TLOB = { 0x0093, 6, 6, 0, "EN_LDO_TLOB", "Enables the LDO" };
const CSRegister EN_LDO_TPAD = { 0x0093, 5, 5, 0, "EN_LDO_TPAD", "Enables the LDO" };
const CSRegister EN_LDO_TXBUF = { 0x0093, 4, 4, 0, "EN_LDO_TXBUF", "Enables the LDO" };
const CSRegister EN_LDO_VCOGN = { 0x0093, 3, 3, 0, "EN_LDO_VCOGN", "Enables the LDO" };
const CSRegister EN_LDO_VCOSXR = { 0x0093, 2, 2, 0, "EN_LDO_VCOSXR", "Enables the LDO" };
const CSRegister EN_LDO_VCOSXT = { 0x0093, 1, 1, 0, "EN_LDO_VCOSXT", "Enables the LDO" };
const CSRegister EN_LDO_CPSXT = { 0x0093, 0, 0, 0, "EN_LDO_CPSXT", "Enables the LDO" };
const CSRegister EN_LOADIMP_LDO_CPSXT = {
    0x0094, 15, 15, 0, "EN_LOADIMP_LDO_CPSXT", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_DIG = {
    0x0094, 14, 14, 0, "EN_LOADIMP_LDO_DIG", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_DIGGN = {
    0x0094, 13, 13, 0, "EN_LOADIMP_LDO_DIGGN", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_DIGSXR = {
    0x0094, 12, 12, 0, "EN_LOADIMP_LDO_DIGSXR", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_DIGSXT = {
    0x0094, 11, 11, 0, "EN_LOADIMP_LDO_DIGSXT", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_DIVGN = {
    0x0094, 10, 10, 0, "EN_LOADIMP_LDO_DIVGN", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_DIVSXR = {
    0x0094, 9, 9, 0, "EN_LOADIMP_LDO_DIVSXR", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_DIVSXT = {
    0x0094, 8, 8, 0, "EN_LOADIMP_LDO_DIVSXT", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_LNA12 = {
    0x0094, 7, 7, 0, "EN_LOADIMP_LDO_LNA12", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_LNA14 = {
    0x0094, 6, 6, 0, "EN_LOADIMP_LDO_LNA14", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_MXRFE = {
    0x0094, 5, 5, 0, "EN_LOADIMP_LDO_MXRFE", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_RBB = {
    0x0094, 4, 4, 0, "EN_LOADIMP_LDO_RBB", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_RXBUF = {
    0x0094, 3, 3, 0, "EN_LOADIMP_LDO_RXBUF", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_TBB = {
    0x0094, 2, 2, 0, "EN_LOADIMP_LDO_TBB", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_TIA12 = {
    0x0094, 1, 1, 0, "EN_LOADIMP_LDO_TIA12", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_TIA14 = {
    0x0094, 0, 0, 0, "EN_LOADIMP_LDO_TIA14", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister BYP_LDO_TBB = { 0x0095, 15, 15, 0, "BYP_LDO_TBB", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_TIA12 = { 0x0095, 14, 14, 0, "BYP_LDO_TIA12", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_TIA14 = { 0x0095, 13, 13, 0, "BYP_LDO_TIA14", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_TLOB = { 0x0095, 12, 12, 0, "BYP_LDO_TLOB", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_TPAD = { 0x0095, 11, 11, 0, "BYP_LDO_TPAD", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_TXBUF = { 0x0095, 10, 10, 0, "BYP_LDO_TXBUF", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_VCOGN = { 0x0095, 9, 9, 0, "BYP_LDO_VCOGN", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_VCOSXR = { 0x0095, 8, 8, 0, "BYP_LDO_VCOSXR", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_VCOSXT = { 0x0095, 7, 7, 0, "BYP_LDO_VCOSXT", "Bypass signal for the LDO" };
const CSRegister EN_LOADIMP_LDO_AFE = {
    0x0095, 2, 2, 0, "EN_LOADIMP_LDO_AFE", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_CPGN = {
    0x0095, 1, 1, 0, "EN_LOADIMP_LDO_CPGN", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_CPSXR = {
    0x0095, 0, 0, 0, "EN_LOADIMP_LDO_CPSXR", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister BYP_LDO_AFE = { 0x0096, 15, 15, 0, "BYP_LDO_AFE", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_CPGN = { 0x0096, 14, 14, 0, "BYP_LDO_CPGN", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_CPSXR = { 0x0096, 13, 13, 0, "BYP_LDO_CPSXR", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_CPSXT = { 0x0096, 12, 12, 0, "BYP_LDO_CPSXT", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_DIG = { 0x0096, 11, 11, 0, "BYP_LDO_DIG", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_DIGGN = { 0x0096, 10, 10, 0, "BYP_LDO_DIGGN", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_DIGSXR = { 0x0096, 9, 9, 0, "BYP_LDO_DIGSXR", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_DIGSXT = { 0x0096, 8, 8, 0, "BYP_LDO_DIGSXT", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_DIVGN = { 0x0096, 7, 7, 0, "BYP_LDO_DIVGN", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_DIVSXR = { 0x0096, 6, 6, 0, "BYP_LDO_DIVSXR", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_DIVSXT = { 0x0096, 5, 5, 0, "BYP_LDO_DIVSXT", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_LNA12 = { 0x0096, 4, 4, 0, "BYP_LDO_LNA12", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_LNA14 = { 0x0096, 3, 3, 0, "BYP_LDO_LNA14", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_MXRFE = { 0x0096, 2, 2, 0, "BYP_LDO_MXRFE", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_RBB = { 0x0096, 1, 1, 0, "BYP_LDO_RBB", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_RXBUF = { 0x0096, 0, 0, 0, "BYP_LDO_RXBUF", "Bypass signal for the LDO" };
const CSRegister SPDUP_LDO_DIVSXR = {
    0x0097, 15, 15, 0, "SPDUP_LDO_DIVSXR", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_DIVSXT = {
    0x0097, 14, 14, 0, "SPDUP_LDO_DIVSXT", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_LNA12 = {
    0x0097, 13, 13, 0, "SPDUP_LDO_LNA12", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_LNA14 = {
    0x0097, 12, 12, 0, "SPDUP_LDO_LNA14", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_MXRFE = {
    0x0097, 11, 11, 0, "SPDUP_LDO_MXRFE", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_RBB = {
    0x0097, 10, 10, 0, "SPDUP_LDO_RBB", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_RXBUF = {
    0x0097, 9, 9, 0, "SPDUP_LDO_RXBUF", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_TBB = {
    0x0097, 8, 8, 0, "SPDUP_LDO_TBB", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_TIA12 = {
    0x0097, 7, 7, 0, "SPDUP_LDO_TIA12", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_TIA14 = {
    0x0097, 6, 6, 0, "SPDUP_LDO_TIA14", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_TLOB = {
    0x0097, 5, 5, 0, "SPDUP_LDO_TLOB", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_TPAD = {
    0x0097, 4, 4, 0, "SPDUP_LDO_TPAD", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_TXBUF = {
    0x0097, 3, 3, 0, "SPDUP_LDO_TXBUF", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_VCOGN = {
    0x0097, 2, 2, 0, "SPDUP_LDO_VCOGN", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_VCOSXR = {
    0x0097, 1, 1, 0, "SPDUP_LDO_VCOSXR", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_VCOSXT = {
    0x0097, 0, 0, 0, "SPDUP_LDO_VCOSXT", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_AFE = {
    0x0098, 8, 8, 0, "SPDUP_LDO_AFE", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_CPGN = {
    0x0098, 7, 7, 0, "SPDUP_LDO_CPGN", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_CPSXR = {
    0x0098, 6, 6, 0, "SPDUP_LDO_CPSXR", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_CPSXT = {
    0x0098, 5, 5, 0, "SPDUP_LDO_CPSXT", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_DIG = {
    0x0098, 4, 4, 0, "SPDUP_LDO_DIG", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_DIGGN = {
    0x0098, 3, 3, 0, "SPDUP_LDO_DIGGN", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_DIGSXR = {
    0x0098, 2, 2, 0, "SPDUP_LDO_DIGSXR", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_DIGSXT = {
    0x0098, 1, 1, 0, "SPDUP_LDO_DIGSXT", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_DIVGN = {
    0x0098, 0, 0, 0, "SPDUP_LDO_DIVGN", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister RDIV_VCOSXR = {
    0x0099, 15, 8, 101, "RDIV_VCOSXR", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_VCOSXT = {
    0x0099, 7, 0, 101, "RDIV_VCOSXT", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_TXBUF = {
    0x009A, 15, 8, 101, "RDIV_TXBUF", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_VCOGN = {
    0x009A, 7, 0, 140, "RDIV_VCOGN", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_TLOB = {
    0x009B, 15, 8, 101, "RDIV_TLOB", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_TPAD = {
    0x009B, 7, 0, 101, "RDIV_TPAD", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_TIA12 = {
    0x009C, 15, 8, 101, "RDIV_TIA12", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_TIA14 = {
    0x009C, 7, 0, 140, "RDIV_TIA14", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_RXBUF = {
    0x009D, 15, 8, 101, "RDIV_RXBUF", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_TBB = {
    0x009D, 7, 0, 101, "RDIV_TBB", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_MXRFE = {
    0x009E, 15, 8, 101, "RDIV_MXRFE", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_RBB = {
    0x009E, 7, 0, 140, "RDIV_RBB", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_LNA12 = {
    0x009F, 15, 8, 101, "RDIV_LNA12", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_LNA14 = {
    0x009F, 7, 0, 140, "RDIV_LNA14", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_DIVSXR = {
    0x00A0, 15, 8, 101, "RDIV_DIVSXR", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_DIVSXT = {
    0x00A0, 7, 0, 101, "RDIV_DIVSXT", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_DIGSXT = {
    0x00A1, 15, 8, 101, "RDIV_DIGSXT", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_DIVGN = {
    0x00A1, 7, 0, 101, "RDIV_DIVGN", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_DIGGN = {
    0x00A2, 15, 8, 101, "RDIV_DIGGN", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_DIGSXR = {
    0x00A2, 7, 0, 101, "RDIV_DIGSXR", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_CPSXT = {
    0x00A3, 15, 8, 101, "RDIV_CPSXT", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_DIG = {
    0x00A3, 7, 0, 101, "RDIV_DIG", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_CPGN = {
    0x00A4, 15, 8, 101, "RDIV_CPGN", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_CPSXR = {
    0x00A4, 7, 0, 101, "RDIV_CPSXR", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_SPIBUF = {
    0x00A5, 15, 8, 101, "RDIV_SPIBUF", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_AFE = {
    0x00A5, 7, 0, 101, "RDIV_AFE", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister SPDUP_LDO_SPIBUF = {
    0x00A6, 12, 12, 0, "SPDUP_LDO_SPIBUF", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_DIGIp2 = {
    0x00A6, 11, 11, 0, "SPDUP_LDO_DIGIp2", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister SPDUP_LDO_DIGIp1 = {
    0x00A6, 10, 10, 0, "SPDUP_LDO_DIGIp1", "Short the noise filter resistor to speed up the settling time"
};
const CSRegister BYP_LDO_SPIBUF = { 0x00A6, 9, 9, 0, "BYP_LDO_SPIBUF", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_DIGIp2 = { 0x00A6, 8, 8, 0, "BYP_LDO_DIGIp2", "Bypass signal for the LDO" };
const CSRegister BYP_LDO_DIGIp1 = { 0x00A6, 7, 7, 0, "BYP_LDO_DIGIp1", "Bypass signal for the LDO" };
const CSRegister EN_LOADIMP_LDO_SPIBUF = {
    0x00A6, 6, 6, 0, "EN_LOADIMP_LDO_SPIBUF", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_DIGIp2 = {
    0x00A6, 5, 5, 0, "EN_LOADIMP_LDO_DIGIp2", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister EN_LOADIMP_LDO_DIGIp1 = {
    0x00A6, 4, 4, 0, "EN_LOADIMP_LDO_DIGIp1", "Enables the load dependent bias to optimize the load regulation"
};
const CSRegister PD_LDO_SPIBUF = { 0x00A6, 3, 3, 1, "PD_LDO_SPIBUF", "Enables the LDO" };
const CSRegister PD_LDO_DIGIp2 = { 0x00A6, 2, 2, 1, "PD_LDO_DIGIp2", "Enables the LDO" };
const CSRegister PD_LDO_DIGIp1 = { 0x00A6, 1, 1, 1, "PD_LDO_DIGIp1", "Enables the LDO" };
const CSRegister EN_G_LDOP = { 0x00A6, 0, 0, 1, "EN_G_LDOP", "Enable control for all the LDO power downs" };
const CSRegister RDIV_DIGIp2 = {
    0x00A7, 15, 8, 101, "RDIV_DIGIp2", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister RDIV_DIGIp1 = {
    0x00A7, 7, 0, 101, "RDIV_DIGIp1", "Controls the output voltage of the LDO by setting the resistive voltage divider ratio"
};
const CSRegister BSIGT = { 0x00A8, 31, 9, 0, "BSIGT", "" };
const CSRegister BSTATE = { 0x00A8, 8, 8, 0, "BSTATE", "" };
const CSRegister EN_SDM_TSTO_SXT = { 0x00A8, 6, 6, 0, "EN_SDM_TSTO_SXT", "Enables the SDM_TSTO_SXT" };
const CSRegister EN_SDM_TSTO_SXR = { 0x00A8, 5, 5, 0, "EN_SDM_TSTO_SXR", "Enables the SDM_TSTO_SXR" };
const CSRegister EN_SDM_TSTO_CGEN = { 0x00A8, 4, 4, 0, "EN_SDM_TSTO_CGEN", "Enables the SDM_TSTO_CGEN" };
const CSRegister BENC = { 0x00A8, 3, 3, 0, "BENC", "enables CGEN BIST" };
const CSRegister BENR = { 0x00A8, 2, 2, 0, "BENR", "enables SXR BIST" };
const CSRegister BENT = { 0x00A8, 1, 1, 0, "BENT", "enables SXT BIST" };
const CSRegister BSTART = {
    0x00A8, 0, 0, 0, "BSTART", "Starts delta sigma built in self test. Keep it at 1 one at least three clock cycles"
};
const CSRegister BSIGR = { 0x00AA, 22, 0, 0, "BSIGR", "" };
const CSRegister BSIGC = { 0x00AB, 29, 7, 0, "BSIGC", "" };
const CSRegister CDS_MCLK2 = { 0x00AD, 15, 14, 0, "CDS_MCLK2", "MCLK2 clock delay" };
const CSRegister CDS_MCLK1 = { 0x00AD, 13, 12, 0, "CDS_MCLK1", "MCLK1 clock delay" };
const CSRegister CDSN_TXBTSP = { 0x00AD, 9, 9, 1, "CDSN_TXBTSP", "TX TSPB clock inversion control" };
const CSRegister CDSN_TXATSP = { 0x00AD, 8, 8, 1, "CDSN_TXATSP", "TX TSPA clock inversion control" };
const CSRegister CDSN_RXBTSP = { 0x00AD, 7, 7, 1, "CDSN_RXBTSP", "RX TSPB clock inversion control" };
const CSRegister CDSN_RXATSP = { 0x00AD, 6, 6, 1, "CDSN_RXATSP", "RX TSPA clock inversion control" };
const CSRegister CDSN_TXBLML = { 0x00AD, 5, 5, 1, "CDSN_TXBLML", "TX LMLB clock inversion control" };
const CSRegister CDSN_TXALML = { 0x00AD, 4, 4, 1, "CDSN_TXALML", "TX LMLA clock inversion control" };
const CSRegister CDSN_RXBLML = { 0x00AD, 3, 3, 1, "CDSN_RXBLML", "RX LMLB clock inversion control" };
const CSRegister CDSN_RXALML = { 0x00AD, 2, 2, 1, "CDSN_RXALML", "RX LMLA clock inversion control" };
const CSRegister CDSN_MCLK2 = { 0x00AD, 1, 1, 1, "CDSN_MCLK2", "MCLK2 clock inversion control" };
const CSRegister CDSN_MCLK1 = { 0x00AD, 0, 0, 1, "CDSN_MCLK1", "MCLK1 clock inversion control" };
const CSRegister CDS_TXBTSP = { 0x00AE, 15, 14, 3, "CDS_TXBTSP", "TX TSP B clock delay" };
const CSRegister CDS_TXATSP = { 0x00AE, 13, 12, 3, "CDS_TXATSP", "TX TSP A clock delay" };
const CSRegister CDS_RXBTSP = { 0x00AE, 11, 10, 0, "CDS_RXBTSP", "RX TSP B clock delay" };
const CSRegister CDS_RXATSP = { 0x00AE, 9, 8, 0, "CDS_RXATSP", "RX TSP A clock delay" };
const CSRegister CDS_TXBLML = { 0x00AE, 7, 6, 0, "CDS_TXBLML", "TX LML B clock delay" };
const CSRegister CDS_TXALML = { 0x00AE, 5, 4, 0, "CDS_TXALML", "TX LML A clock delay" };
const CSRegister CDS_RXBLML = { 0x00AE, 3, 2, 0, "CDS_RXBLML", "RX LML B clock delay" };
const CSRegister CDS_RXALML = { 0x00AE, 1, 0, 0, "CDS_RXALML", "RX LML A clock delay" };
const CSRegister EN_LOWBWLOMX_TMX_TRF = { 0x0100,
    15,
    15,
    0,
    "EN_LOWBWLOMX_TMX_TRF",
    "Controls the high pass pole frequency of the RC biasing the gate of the mixer switches" };
const CSRegister EN_NEXTTX_TRF = {
    0x0100, 14, 14, 0, "EN_NEXTTX_TRF", "Enables the daisy change LO buffer going from TRF_1 to TRF2"
};
const CSRegister EN_AMPHF_PDET_TRF = { 0x0100, 13, 12, 3, "EN_AMPHF_PDET_TRF", "Enables the TXPAD power detector preamplifier" };
const CSRegister LOADR_PDET_TRF = { 0x0100, 11, 10, 1, "LOADR_PDET_TRF", "Controls the resistive load of the Power detector" };
const CSRegister PD_PDET_TRF = { 0x0100, 3, 3, 1, "PD_PDET_TRF", "Powerdown signal for Power Detector" };
const CSRegister PD_TLOBUF_TRF = { 0x0100, 2, 2, 0, "PD_TLOBUF_TRF", "Powerdown signal for TX LO buffer" };
const CSRegister PD_TXPAD_TRF = { 0x0100, 1, 1, 0, "PD_TXPAD_TRF", "Powerdown signal for TXPAD" };
const CSRegister EN_G_TRF = { 0x0100, 0, 0, 1, "EN_G_TRF", "Enable control for all the TRF_1 power downs" };
const CSRegister F_TXPAD_TRF = { 0x0101,
    15,
    13,
    3,
    "F_TXPAD_TRF",
    "Controls the switched capacitor at the TXPAD output. Is used for fine tuning of the TXPAD output" };
const CSRegister L_LOOPB_TXPAD_TRF = {
    0x0101, 12, 11, 3, "L_LOOPB_TXPAD_TRF", "Controls the loss of the of the loopback path at the TX side"
};
const CSRegister LOSS_LIN_TXPAD_TRF = {
    0x0101, 10, 6, 0, "LOSS_LIN_TXPAD_TRF", "Controls the gain of the linearizing part of of the TXPAD"
};
const CSRegister LOSS_MAIN_TXPAD_TRF = { 0x0101, 5, 1, 0, "LOSS_MAIN_TXPAD_TRF", "Controls the gain output power of the TXPAD" };
const CSRegister EN_LOOPB_TXPAD_TRF = { 0x0101, 0, 0, 0, "EN_LOOPB_TXPAD_TRF", "Enables the TXPAD loopback path" };
const CSRegister GCAS_GNDREF_TXPAD_TRF = {
    0x0102, 15, 15, 0, "GCAS_GNDREF_TXPAD_TRF", "Controls if the TXPAD cascode transistor gate bias is referred to VDD or GND"
};
const CSRegister ICT_LIN_TXPAD_TRF = {
    0x0102, 14, 10, 12, "ICT_LIN_TXPAD_TRF", "Control the bias current of the linearization section of the TXPAD"
};
const CSRegister ICT_MAIN_TXPAD_TRF = {
    0x0102, 9, 5, 12, "ICT_MAIN_TXPAD_TRF", "Control the bias current of the main gm section of the TXPAD"
};
const CSRegister VGCAS_TXPAD_TRF = { 0x0102, 4, 0, 0, "VGCAS_TXPAD_TRF", "Controls the bias voltage at the gate of TXPAD cascade" };
const CSRegister SEL_BAND1_TRF = { 0x0103, 11, 11, 1, "SEL_BAND1_TRF", "" };
const CSRegister SEL_BAND2_TRF = { 0x0103, 10, 10, 0, "SEL_BAND2_TRF", "" };
const CSRegister LOBIASN_TXM_TRF = { 0x0103, 9, 5, 16, "LOBIASN_TXM_TRF", "Controls the bias at the gate of the mixer NMOS" };
const CSRegister LOBIASP_TXX_TRF = { 0x0103, 4, 0, 18, "LOBIASP_TXX_TRF", "Controls the bias at the gate of the mixer PMOS" };
const CSRegister CDC_I_TRF = { 0x0104, 7, 4, 8, "CDC_I_TRF", "" };
const CSRegister CDC_Q_TRF = { 0x0104, 3, 0, 8, "CDC_Q_TRF", "" };
const CSRegister STATPULSE_TBB = { 0x0105, 15, 15, 0, "STATPULSE_TBB", "" };
const CSRegister LOOPB_TBB = { 0x0105,
    14,
    12,
    0,
    "LOOPB_TBB",
    "This controls which signal is connected to the loopback output pins. Note: when both the lowpass ladder and real pole are "
    "powered down, the output of the active highband biquad is routed to the loopb output" };
const CSRegister PD_LPFH_TBB = { 0x0105, 4, 4, 0, "PD_LPFH_TBB", "This selectively powers down the LPFH_TBB biquad" };
const CSRegister PD_LPFIAMP_TBB = {
    0x0105, 3, 3, 0, "PD_LPFIAMP_TBB", "selectively powers down the LPFIAMP_TBB front-end current amp of the transmitter baseband"
};
const CSRegister PD_LPFLAD_TBB = { 0x0105,
    2,
    2,
    1,
    "PD_LPFLAD_TBB",
    "This selectively powers down the LPFLAD_TBB low pass ladder filter of the transmitter baseband" };
const CSRegister PD_LPFS5_TBB = { 0x0105,
    1,
    1,
    1,
    "PD_LPFS5_TBB",
    "This selectively powers down the LPFS5_TBB low pass real-pole filter of the transmitter baseband" };
const CSRegister EN_G_TBB = { 0x0105, 0, 0, 1, "EN_G_TBB", "Enable control for all the TBB_TOP power downs" };
const CSRegister ICT_LPFS5_F_TBB = { 0x0106,
    14,
    10,
    12,
    "ICT_LPFS5_F_TBB",
    "This controls the operational amplifier's output stage bias current of the low band real pole filter of the transmitter's "
    "baseband" };
const CSRegister ICT_LPFS5_PT_TBB = { 0x0106,
    9,
    5,
    12,
    "ICT_LPFS5_PT_TBB",
    "This controls the operational amplifier's input stage bias current of the low band real pole filter of the transmitter's "
    "baseband" };
const CSRegister ICT_LPF_H_PT_TBB = { 0x0106,
    4,
    0,
    12,
    "ICT_LPF_H_PT_TBB",
    "This controls the operational amplifiers input stage bias reference current of the high band low pass filter of the "
    "transmitter's baseband " };
const CSRegister ICT_LPFH_F_TBB = { 0x0107,
    14,
    10,
    12,
    "ICT_LPFH_F_TBB",
    "controls the operational amplifiers output stage bias reference current of the high band low pass filter of the transmitter's "
    "baseband (LPFH_TBB)" };
const CSRegister ICT_LPFLAD_F_TBB = { 0x0107,
    9,
    5,
    12,
    "ICT_LPFLAD_F_TBB",
    "This controls the operational amplfiers' output stages bias reference current of the low band ladder filter of the "
    "transmisster's baseband" };
const CSRegister ICT_LPFLAD_PT_TBB = { 0x0107,
    4,
    0,
    12,
    "ICT_LPFLAD_PT_TBB",
    "This controls the operational amplifers' input stages bias reference current of the low band ladder filter of the "
    "transmitter's baseband" };
const CSRegister CG_IAMP_TBB = { 0x0108,
    15,
    10,
    37,
    "CG_IAMP_TBB",
    "This controls the frontend gain of the TBB. For a given gain value, this control value varies with the set TX mode. After "
    "resistance calibration, the following table gives the nominal values for each frequency setting. However, this table is to be "
    "updated and corrected after calibration" };
const CSRegister ICT_IAMP_FRP_TBB = {
    0x0108, 9, 5, 12, "ICT_IAMP_FRP_TBB", "This controls the reference bias current of the IAMP main bias current sources"
};
const CSRegister ICT_IAMP_GG_FRP_TBB = { 0x0108,
    4,
    0,
    12,
    "ICT_IAMP_GG_FRP_TBB",
    "This controls the reference bias current of the IAMP's cascode transistors gate voltages that set the IAMP's input voltage "
    "level. The IAMP's input is connected to the DAC output" };
const CSRegister RCAL_LPFH_TBB = { 0x0109,
    15,
    8,
    97,
    "RCAL_LPFH_TBB",
    "This controls the value of the equivalent resistance of the resistor banks of the biquad filter stage (of the high band "
    "section) of the transmitter baseband(TBB)" };
const CSRegister RCAL_LPFLAD_TBB = { 0x0109,
    7,
    0,
    193,
    "RCAL_LPFLAD_TBB",
    "This controls the value of the equivalent resistance of the resistor banks of the low pass filter ladder (of the low band "
    "section) of the transmitter baseband(TBB)" };
const CSRegister TSTIN_TBB = { 0x010A,
    15,
    14,
    0,
    "TSTIN_TBB",
    "This control selects where the input test signal (vinp/n_aux_bbq/i) is routed to as well as disabling the route." };
const CSRegister BYPLADDER_TBB = { 0x010A,
    13,
    13,
    0,
    "BYPLADDER_TBB",
    "This signal bypasses the LPF ladder of TBB and directly connects the output of current amplifier to the null port of the real "
    "pole stage of TBB low pass filter" };
const CSRegister CCAL_LPFLAD_TBB = { 0x010A,
    12,
    8,
    16,
    "CCAL_LPFLAD_TBB",
    "A common control signal for all the capacitor banks of TBB filters (including the ladder, real pole, and the high band "
    "biquad). It is the calibrated value of the banks control that sets the value of the banks' equivalent capacitor to their "
    "respective nominal values" };
const CSRegister RCAL_LPFS5_TBB = { 0x010A,
    7,
    0,
    76,
    "RCAL_LPFS5_TBB",
    "This controls the value of the equivalent resistance of the resistor banks of the real pole filter stage (of the low band "
    "section) of the transmitter baseband (TBB). Following is a nominal values table that are corrected for any process variation "
    "after calibration" };
const CSRegister CDC_I_RFE = { 0x010C, 15, 12, 8, "CDC_I_RFE", "" };
const CSRegister CDC_Q_RFE = { 0x010C, 11, 8, 8, "CDC_Q_RFE", "" };
const CSRegister PD_LNA_RFE = { 0x010C, 7, 7, 1, "PD_LNA_RFE", "Power control signal for LNA_RFE" };
const CSRegister PD_RLOOPB_1_RFE = { 0x010C, 6, 6, 1, "PD_RLOOPB_1_RFE", "Power control signal for RXFE loopback 1" };
const CSRegister PD_RLOOPB_2_RFE = { 0x010C, 5, 5, 1, "PD_RLOOPB_2_RFE", "Power control signal for RXFE loopback 2" };
const CSRegister PD_MXLOBUF_RFE = { 0x010C, 4, 4, 1, "PD_MXLOBUF_RFE", "Power control signal for RXFE mixer lo buffer" };
const CSRegister PD_QGEN_RFE = { 0x010C, 3, 3, 1, "PD_QGEN_RFE", "Power control signal for RXFE Quadrature LO generator" };
const CSRegister PD_RSSI_RFE = { 0x010C, 2, 2, 1, "PD_RSSI_RFE", "Power control signal for RXFE RSSI" };
const CSRegister PD_TIA_RFE = { 0x010C, 1, 1, 0, "PD_TIA_RFE", "Power control signal for RXFE TIA" };
const CSRegister EN_G_RFE = { 0x010C, 0, 0, 1, "EN_G_RFE", "Enable control for all the RFE_1 power downs" };
const CSRegister SEL_PATH_RFE = { 0x010D, 8, 7, 1, "SEL_PATH_RFE", "Selects the active path of the RXFE" };
const CSRegister EN_DCOFF_RXFE_RFE = { 0x010D, 6, 6, 0, "EN_DCOFF_RXFE_RFE", "Enables the DCOFFSET block for the RXFE" };
const CSRegister EN_INSHSW_LB1_RFE = { 0x010D,
    4,
    4,
    1,
    "EN_INSHSW_LB1_RFE",
    "Enables the input shorting switch at the input of the loopback 1 (in parallel with LNAL mixer)" };
const CSRegister EN_INSHSW_LB2_RFE = { 0x010D,
    3,
    3,
    1,
    "EN_INSHSW_LB2_RFE",
    "Enables the input shorting switch at the input of the loopback 2 (in parallel with LNAW mixer)" };
const CSRegister EN_INSHSW_L_RFE = {
    0x010D, 2, 2, 1, "EN_INSHSW_L_RFE", "Enables the input shorting switch at the input of the LNAL"
};
const CSRegister EN_INSHSW_W_RFE = {
    0x010D, 1, 1, 1, "EN_INSHSW_W_RFE", "Enables the input shorting switch at the input of the LNAW"
};
const CSRegister EN_NEXTRX_RFE = {
    0x010D, 0, 0, 0, "EN_NEXTRX_RFE", "Enables the daisy chain LO buffer going from RXFE1 to RXFE2"
};
const CSRegister DCOFFI_RFE = { 0x010E,
    13,
    7,
    64,
    "DCOFFI_RFE",
    "Controls DC offset at the output of the TIA by injecting current to the input of the TIA (I side)" };
const CSRegister DCOFFQ_RFE = { 0x010E,
    6,
    0,
    64,
    "DCOFFQ_RFE",
    "Controls DC offset at the output of the TIA by injecting current to the input of the TIA (Q side)" };
const CSRegister ICT_LOOPB_RFE = {
    0x010F, 14, 10, 12, "ICT_LOOPB_RFE", "Controls the reference current of the RXFE loopback amplifier"
};
const CSRegister ICT_TIAMAIN_RFE = {
    0x010F, 9, 5, 6, "ICT_TIAMAIN_RFE", "Controls the reference current of the RXFE TIA first stage"
};
const CSRegister ICT_TIAOUT_RFE = { 0x010F, 4, 0, 6, "ICT_TIAOUT_RFE", "Controls the reference current of the RXFE TIA 2nd stage" };
const CSRegister ICT_LNACMO_RFE = {
    0x0110, 14, 10, 2, "ICT_LNACMO_RFE", "Controls the current generating LNA output common mode voltage"
};
const CSRegister ICT_LNA_RFE = { 0x0110, 9, 5, 12, "ICT_LNA_RFE", "Controls the current of the LNA core" };
const CSRegister ICT_LODC_RFE = {
    0x0110, 4, 0, 20, "ICT_LODC_RFE", "Controls the DC of the mixer LO signal at the gate of the mixer switches"
};
const CSRegister CAP_RXMXO_RFE = { 0x0111, 9, 5, 4, "CAP_RXMXO_RFE", "Control the decoupling cap at the output of the RX Mixer" };
const CSRegister CGSIN_LNA_RFE = { 0x0111,
    4,
    0,
    3,
    "CGSIN_LNA_RFE",
    "Controls the cap parallel with the LNA input input NMOS CGS to control the Q of the matching circuit and provides trade off "
    "between gain/NF and IIP. The higher the frequency, the lower CGSIN_LNA_RFE should be. Also, the higher CGSIN, the lower the "
    "Q, The lower the gain, the higher the NF, and the higher the IIP3" };
const CSRegister CCOMP_TIA_RFE = { 0x0112, 15, 12, 12, "CCOMP_TIA_RFE", "Compensation capacitor for TIA" };
const CSRegister CFB_TIA_RFE = { 0x0112,
    11,
    0,
    230,
    "CFB_TIA_RFE",
    "Feeback capacitor for TIA. Controls the 3dB BW of the TIA. Should be set with calibration through digital baseband" };
const CSRegister G_LNA_RFE = { 0x0113, 9, 6, 15, "G_LNA_RFE", "Controls the gain of the LNA" };
const CSRegister G_RXLOOPB_RFE = { 0x0113, 5, 2, 0, "G_RXLOOPB_RFE", "Controls RXFE loopback gain" };
const CSRegister G_TIA_RFE = { 0x0113, 1, 0, 3, "G_TIA_RFE", "Controls the Gain of the TIA" };
const CSRegister RCOMP_TIA_RFE = { 0x0114, 8, 5, 4, "RCOMP_TIA_RFE", "Controls the compensation resistors of the TIA opamp" };
const CSRegister RFB_TIA_RFE = { 0x0114, 4, 0, 13, "RFB_TIA_RFE", "Sets the feedback resistor to the nominal value" };
const CSRegister EN_LB_LPFH_RBB = { 0x0115, 15, 15, 0, "EN_LB_LPFH_RBB", "" };
const CSRegister EN_LB_LPFL_RBB = { 0x0115, 14, 14, 0, "EN_LB_LPFL_RBB", "" };
const CSRegister PD_LPFH_RBB = { 0x0115, 3, 3, 1, "PD_LPFH_RBB", "Power down of the LPFH block" };
const CSRegister PD_LPFL_RBB = { 0x0115, 2, 2, 0, "PD_LPFL_RBB", "Power down of the LPFL block" };
const CSRegister PD_PGA_RBB = { 0x0115, 1, 1, 0, "PD_PGA_RBB", "Power down of the PGA block" };
const CSRegister EN_G_RBB = { 0x0115, 0, 0, 1, "EN_G_RBB", "Enable control for all the RBB_1 power downs" };
const CSRegister R_CTL_LPF_RBB = { 0x0116,
    15,
    11,
    16,
    "R_CTL_LPF_RBB",
    "Controls the absolute value of the resistance of the RC time constant of the RBB_LPF blocks (both Low and High)" };
const CSRegister RCC_CTL_LPFH_RBB = {
    0x0116, 10, 8, 1, "RCC_CTL_LPFH_RBB", "Controls the stability passive compensation of the LPFH_RBB operational amplifier"
};
const CSRegister C_CTL_LPFH_RBB = { 0x0116,
    7,
    0,
    128,
    "C_CTL_LPFH_RBB",
    "Controls the capacitance value of the RC time constant of RBB_LPFH and it varies with the respective rxMode from 37MHz to "
    "108MHz" };
const CSRegister RCC_CTL_LPFL_RBB = {
    0x0117, 13, 11, 5, "RCC_CTL_LPFL_RBB", "Controls the stability passive compensation of the LPFL_RBB operational amplifier"
};
const CSRegister C_CTL_LPFL_RBB = { 0x0117,
    10,
    0,
    12,
    "C_CTL_LPFL_RBB",
    "Controls the capacitance value of the RC time constant of RBB_LPFH and it varies with the respective rxMode from 1.4MHz to "
    "20MHz." };
const CSRegister INPUT_CTL_PGA_RBB = { 0x0118,
    15,
    13,
    0,
    "INPUT_CTL_PGA_RBB",
    "There are a total of four different differential inputs to the PGA. Only one of them is active at a time" };
const CSRegister ICT_LPF_IN_RBB = { 0x0118,
    9,
    5,
    12,
    "ICT_LPF_IN_RBB",
    "Controls the reference bias current of the input stage of the operational amplifier used in RBB_LPF blocks (Low or High). " };
const CSRegister ICT_LPF_OUT_RBB = { 0x0118,
    4,
    0,
    12,
    "ICT_LPF_OUT_RBB",
    "The reference bias current of the output stage of the operational amplifier used in RBB_LPF blocks (low or High)" };
const CSRegister OSW_PGA_RBB = { 0x0119,
    15,
    15,
    0,
    "OSW_PGA_RBB",
    "There are two instances of the PGA circuit in the design. The output of the RBB_LPF blocks are connected the input of these "
    "PGA blocks (common). The output of one of them is connected to two pads pgaoutn and pgaoutp and the output of the other PGA "
    "is connected directly to the ADC input" };
const CSRegister ICT_PGA_OUT_RBB = { 0x0119,
    14,
    10,
    20,
    "ICT_PGA_OUT_RBB",
    "Controls the output stage reference bias current of the operational amplifier used in the PGA circuit" };
const CSRegister ICT_PGA_IN_RBB = { 0x0119,
    9,
    5,
    20,
    "ICT_PGA_IN_RBB",
    "Controls the input stage reference bias current of the operational amplifier used in the PGA circuit" };
const CSRegister G_PGA_RBB = { 0x0119, 4, 0, 11, "G_PGA_RBB", "This is the gain of the PGA" };
const CSRegister RCC_CTL_PGA_RBB = {
    0x011A, 13, 9, 23, "RCC_CTL_PGA_RBB", "Controls the stability passive compensation of the PGA_RBB operational amplifier"
};
const CSRegister C_CTL_PGA_RBB = { 0x011A,
    6,
    0,
    2,
    "C_CTL_PGA_RBB",
    "Control the value of the feedback capacitor of the PGA that is used to help against the parasitic cap at the virtual node for "
    "stability" };
const CSRegister RESET_N = { 0x011C, 15, 15, 1, "RESET_N", "Resets SX. A pulse should be used in the start-up to reset" };
const CSRegister SPDUP_VCO = { 0x011C,
    14,
    14,
    0,
    "SPDUP_VCO",
    "Bypasses the noise filter resistor for fast settling time. It should be connected to a 1uS pulse" };
const CSRegister BYPLDO_VCO = { 0x011C, 13, 13, 1, "BYPLDO_VCO", "Controls the bypass signal for the SX LDO" };
const CSRegister EN_COARSEPLL = { 0x011C, 12, 12, 0, "EN_COARSEPLL", "Enable signal for coarse tuning block" };
const CSRegister CURLIM_VCO = { 0x011C, 11, 11, 1, "CURLIM_VCO", "Enables the output current limitation in the VCO regulator" };
const CSRegister EN_DIV2_DIVPROG = { 0x011C, 10, 10, 1, "EN_DIV2_DIVPROG", "" };
const CSRegister EN_INTONLY_SDM = { 0x011C, 9, 9, 0, "EN_INTONLY_SDM", "Enables INTEGER-N mode of the SX " };
const CSRegister EN_SDM_CLK = {
    0x011C, 8, 8, 1, "EN_SDM_CLK", "Enables/Disables SDM clock. In INT-N mode or for noise testing, SDM clock can be disabled"
};
const CSRegister PD_FBDIV = { 0x011C, 7, 7, 0, "PD_FBDIV", "Power down the feedback divider block" };
const CSRegister PD_LOCH_T2RBUF = {
    0x011C, 6, 6, 1, "PD_LOCH_T2RBUF", "Power down for LO buffer from SXT to SXR. To be active only in the TDD mode"
};
const CSRegister PD_CP = { 0x011C, 5, 5, 0, "PD_CP", "Power down for Charge Pump" };
const CSRegister PD_FDIV = { 0x011C, 4, 4, 0, "PD_FDIV", "Power down for feedback frequency and forward dividers" };
const CSRegister PD_SDM = { 0x011C, 3, 3, 0, "PD_SDM", "Power down for SDM" };
const CSRegister PD_VCO_COMP = { 0x011C, 2, 2, 0, "PD_VCO_COMP", "Power down for VCO comparator" };
const CSRegister PD_VCO = { 0x011C, 1, 1, 1, "PD_VCO", "Power down for VCO" };
const CSRegister EN_G = { 0x011C, 0, 0, 1, "EN_G", "Enable control for all the SXT power downs" };
const CSRegister FRAC_SDM_LSB = { 0x011D, 15, 0, 0x0400, "FRAC_SDM_LSB", "" };
const CSRegister INT_SDM = { 0x011E, 13, 4, 120, "INT_SDM", "" };
const CSRegister FRAC_SDM_MSB = { 0x011E, 3, 0, 0, "FRAC_SDM_MSB", "" };
const CSRegister PW_DIV2_LOCH = { 0x011F,
    14,
    12,
    3,
    "PW_DIV2_LOCH",
    "trims the duty cycle of DIV2 LOCH. Only works when forward divider is dividing by at least 2 (excluding quadrature block "
    "division). If in bypass mode, this does not work" };
const CSRegister PW_DIV4_LOCH = { 0x011F,
    11,
    9,
    3,
    "PW_DIV4_LOCH",
    "trims the duty cycle of DIV4 LOCH. Only works when forward divider is dividing by at least 4 (excluding quadrature block "
    "division). If in bypass mode, this does not work" };
const CSRegister DIV_LOCH = { 0x011F, 8, 6, 1, "DIV_LOCH", "Controls the division ratio in the LOCH_DIV" };
const CSRegister TST_SX = { 0x011F,
    5,
    3,
    0,
    "TST_SX",
    "Controls the test mode of PLLs. TST signal lines are shared between all PLLs (CGEN, RX and TX). Only one TST signal of any PLL should be active at a given time.\n\
    0 - TST disabled; RSSI analog outputs enabled if RSSI blocks active and when all PLL test signals are off(default)\n\
    1 - tstdo[0] = VCO / 20 clock*; tstdo[1] = VCO / 40 clock*; tstao = High impedance;\n\
    2 - tstdo[0] = SDM clock; tstdo[1] = feedback divider output; tstao = VCO tune through a 60kOhm resistor;\n\
    3 - tstdo[0] = Reference clock; tstdo[1] = feedback divider output; tstao = VCO tune through a 10kOhm resistor;\n\
    4 - tstdo[0] = High impedance; tstdo[1] = High impedance; tstao = High impedance;\n\
    5 - tstdo[0] = Charge pump Down signal; tstdo[1] = Charge pump Up signal; tstao = High impedance;\n\
    6 - tstdo[0] = High impedance; tstdo[1] = High impedance; tstao = VCO tune through a 60kOhm resistor;\n\
    7 - tstdo[0] = High impedance; tstdo[1] = High impedance; tstao = VCO tune through a 10kOhm resistor;\n\
    if TST_SX[2] = 1 --> VCO_TSTBUF active generating VCO_TST_DIV20 and VCO_TST_DIV40\n\
    * When EN_DIV2_DIVPROG_(SXR, SXT) is active, the division ratio must be multiplied by 2 (40 / 80)" };
const CSRegister SEL_SDMCLK = { 0x011F, 2, 2, 0, "SEL_SDMCLK", "Selects between the feedback divider output and Fref for SDM" };
const CSRegister SX_DITHER_EN = { 0x011F, 1, 1, 0, "SX_DITHER_EN", "Enabled dithering in SDM" };
const CSRegister REV_SDMCLK = { 0x011F, 0, 0, 0, "REV_SDMCLK", "Reverses the SDM clock" };
const CSRegister VDIV_VCO = { 0x0120, 15, 8, 185, "VDIV_VCO", "Controls VCO LDO output voltage" };
const CSRegister ICT_VCO = { 0x0120, 7, 0, 255, "ICT_VCO", "Scales the VCO bias current from 0 to 2.5xInom" };
const CSRegister RSEL_LDO_VCO = {
    0x0121, 15, 11, 6, "RSEL_LDO_VCO", "Set the reference voltage that supplies bias voltage of switch-cap array and varactor"
};
const CSRegister CSW_VCO = { 0x0121,
    10,
    3,
    128,
    "CSW_VCO",
    "Coarse control of VCO frequency, 0 for lowest frequency and 255 for highest. This control is set by SX_SWC_calibration" };
const CSRegister SEL_VCO = { 0x0121, 2, 1, 2, "SEL_VCO", "Selects the active VCO. It is set by SX_SWC_calibration" };
const CSRegister COARSE_START = { 0x0121, 0, 0, 0, "COARSE_START", "" };
const CSRegister REVPH_PFD = { 0x0122,
    12,
    12,
    0,
    "REVPH_PFD",
    "Reverse the pulses of PFD. It can be used to reverse the polarity of the PLL loop (positive feedback to negative feedback)" };
const CSRegister IOFFSET_CP = { 0x0122,
    11,
    6,
    12,
    "IOFFSET_CP",
    "Scales the offset current of the charge pump, 0-->63. This current is used in Fran-N mode to create an offset in the CP "
    "response and avoid the non-linear section" };
const CSRegister IPULSE_CP = { 0x0122, 5, 0, 63, "IPULSE_CP", "Scales the pulse current of the charge pump" };
const CSRegister COARSE_STEPDONE = { 0x0123, 15, 15, 0, "COARSE_STEPDONE", "" };
const CSRegister COARSEPLL_COMPO = { 0x0123, 14, 14, 0, "COARSEPLL_COMPO", "" };
const CSRegister VCO_CMPHO = { 0x0123, 13, 13, 0, "VCO_CMPHO", "" };
const CSRegister VCO_CMPLO = { 0x0123, 12, 12, 0, "VCO_CMPLO", "" };
const CSRegister CP2_PLL = {
    0x0123, 11, 8, 6, "CP2_PLL", "Controls the value of CP2 (cap from CP output to GND) in the PLL filter"
};
const CSRegister CP3_PLL = {
    0x0123, 7, 4, 7, "CP3_PLL", "Controls the value of CP3 (cap from VCO Vtune input to GND) in the PLL filter"
};
const CSRegister CZ = { 0x0123, 3, 0, 11, "CZ", "Controls the value of CZ (Zero capacitor) in the PLL filter" };
const CSRegister EN_DIR_SXRSXT = { 0x0124, 4, 4, 0, "EN_DIR_SXRSXT", "Enables direct control of PDs and ENs for SXR/SXT module" };
const CSRegister EN_DIR_RBB = { 0x0124, 3, 3, 0, "EN_DIR_RBB", "Enables direct control of PDs and ENs for RBB module" };
const CSRegister EN_DIR_RFE = { 0x0124, 2, 2, 0, "EN_DIR_RFE", "Enables direct control of PDs and ENs for RFE module" };
const CSRegister EN_DIR_TBB = { 0x0124, 1, 1, 0, "EN_DIR_TBB", "Enables direct control of PDs and ENs for TBB module" };
const CSRegister EN_DIR_TRF = { 0x0124, 0, 0, 0, "EN_DIR_TRF", "Enables direct control of PDs and ENs for TRF module" };
const CSRegister TSGFC_TXTSP = { 0x0200, 9, 9, 0, "TSGFC_TXTSP", "TSG full scale control" };
const CSRegister TSGFCW_TXTSP = { 0x0200, 8, 7, 1, "TSGFCW_TXTSP", "Set frequency of TSG's NCO" };
const CSRegister TSGDCLDQ_TXTSP = { 0x0200, 6, 6, 0, "TSGDCLDQ_TXTSP", "" };
const CSRegister TSGDCLDI_TXTSP = { 0x0200, 5, 5, 0, "TSGDCLDI_TXTSP", "" };
const CSRegister TSGSWAPIQ_TXTSP = { 0x0200, 4, 4, 0, "TSGSWAPIQ_TXTSP", "Swap signals at test signal generator's output" };
const CSRegister TSGMODE_TXTSP = { 0x0200, 3, 3, 0, "TSGMODE_TXTSP", "Test signal generator mode" };
const CSRegister INSEL_TXTSP = { 0x0200, 2, 2, 0, "INSEL_TXTSP", "Input source of TxTSP" };
const CSRegister BSTART_TXTSP = {
    0x0200, 1, 1, 0, "BSTART_TXTSP", "Starts delta sigma built in self test. Keep it at 1 one at least three clock cycles"
};
const CSRegister EN_TXTSP = { 0x0200, 0, 0, 1, "EN_TXTSP", "TxTSP modules enable" };
const CSRegister GCORRQ_TXTSP = { 0x0201, 10, 0, 2047, "GCORRQ_TXTSP", "corrector value, channel Q Unsigned integer" };
const CSRegister GCORRI_TXTSP = { 0x0202, 10, 0, 2047, "GCORRI_TXTSP", "corrector value, channel I Unsigned integer" };
const CSRegister HBI_OVR_TXTSP = { 0x0203, 14, 12, 0, "HBI_OVR_TXTSP", "HBI interpolation ratio" };
const CSRegister IQCORR_TXTSP = {
    0x0203, 11, 0, 0, "IQCORR_TXTSP", "Phase corrector value (tan(Alpha/2)). Integer, 2's complement"
};
const CSRegister DCCORRI_TXTSP = { 0x0204, 15, 8, 0, "DCCORRI_TXTSP", "DC corrector value, channel I. Integer, 2's complement" };
const CSRegister DCCORRQ_TXTSP = { 0x0204, 7, 0, 0, "DCCORRQ_TXTSP", "DC corrector value, channel Q. Integer, 2's complement" };
const CSRegister GFIR1_L_TXTSP = {
    0x0205, 10, 8, 0, "GFIR1_L_TXTSP", "Parameter l of GFIR1 (l = roundUp(CoeffN/5)-1). Unsigned integer"
};
const CSRegister GFIR1_N_TXTSP = {
    0x0205, 7, 0, 0, "GFIR1_N_TXTSP", "Clock division ratio of GFIR1 is GFIR1_N + 1. Unsigned integer"
};
const CSRegister GFIR2_L_TXTSP = {
    0x0206, 10, 8, 0, "GFIR2_L_TXTSP", "Parameter l of GFIR2 (l = roundUp(CoeffN/5)-1). Unsigned integer"
};
const CSRegister GFIR2_N_TXTSP = {
    0x0206, 7, 0, 0, "GFIR2_N_TXTSP", "Clock division ratio of GFIR2 is GFIR2_N + 1. Unsigned integer"
};
const CSRegister GFIR3_L_TXTSP = {
    0x0207, 10, 8, 0, "GFIR3_L_TXTSP", "Parameter l of GFIR3 (l = roundUp(CoeffN/5)-1). Unsigned integer"
};
const CSRegister GFIR3_N_TXTSP = {
    0x0207, 7, 0, 0, "GFIR3_N_TXTSP", "Clock division ratio of GFIR3 is GFIR3_N + 1. Unsigned integer"
};
const CSRegister CMIX_GAIN_TXTSP = { 0x0208, 15, 14, 0, "CMIX_GAIN_TXTSP", "Gain of CMIX output" };
const CSRegister CMIX_SC_TXTSP = { 0x0208, 13, 13, 0, "CMIX_SC_TXTSP", "Spectrum control of CMIX" };
const CSRegister CMIX_BYP_TXTSP = { 0x0208, 8, 8, 0, "CMIX_BYP_TXTSP", "CMIX bypass" };
const CSRegister ISINC_BYP_TXTSP = { 0x0208, 7, 7, 0, "ISINC_BYP_TXTSP", "ISINC bypass" };
const CSRegister GFIR3_BYP_TXTSP = { 0x0208, 6, 6, 1, "GFIR3_BYP_TXTSP", "GFIR3 bypass" };
const CSRegister GFIR2_BYP_TXTSP = { 0x0208, 5, 5, 1, "GFIR2_BYP_TXTSP", "GFIR2 bypass" };
const CSRegister GFIR1_BYP_TXTSP = { 0x0208, 4, 4, 1, "GFIR1_BYP_TXTSP", "GFIR1 bypass" };
const CSRegister DC_BYP_TXTSP = { 0x0208, 3, 3, 0, "DC_BYP_TXTSP", "DC corrector bypass" };
const CSRegister GC_BYP_TXTSP = { 0x0208, 1, 1, 0, "GC_BYP_TXTSP", "Gain corrector bypass" };
const CSRegister PH_BYP_TXTSP = { 0x0208, 0, 0, 0, "PH_BYP_TXTSP", "Phase corrector bypass" };
const CSRegister BSIGI_TXTSP = { 0x0209, 23, 1, 0, "BSIGI_TXTSP", "" };
const CSRegister BSTATE_TXTSP = { 0x0209, 0, 0, 0, "BSTATE_TXTSP", "" };
const CSRegister BSIGQ_TXTSP = { 0x020A, 30, 8, 0, "BSIGQ_TXTSP", "" };
const CSRegister DC_REG_TXTSP = { 0x020C, 15, 0, 0, "DC_REG_TXTSP", "" };
const CSRegister DTHBIT_TX = { 0x0240, 8, 5, 1, "DTHBIT_TX", "NCO bits to dither" };
const CSRegister SEL_TX = { 0x0240, 4, 1, 0, "SEL_TX", "" };
const CSRegister MODE_TX = { 0x0240, 0, 0, 0, "MODE_TX", "" };
const CSRegister PHO_TX = { 0x0241, 15, 0, 0, "PHO_TX", "" };
const CSRegister CAPTURE = { 0x0400, 15, 15, 0, "CAPTURE", "" };
const CSRegister CAPSEL = { 0x0400, 14, 13, 0, "CAPSEL", "" };
const CSRegister CAPSEL_ADC = { 0x0400, 12, 12, 0, "CAPSEL_ADC", "Selects ADC value source to be captured" };
const CSRegister TSGFC_RXTSP = { 0x0400, 9, 9, 0, "TSGFC_RXTSP", "TSG full scale control" };
const CSRegister TSGFCW_RXTSP = { 0x0400, 8, 7, 1, "TSGFCW_RXTSP", "Set frequency of TSG's NCO" };
const CSRegister TSGDCLDQ_RXTSP = { 0x0400, 6, 6, 0, "TSGDCLDQ_RXTSP", "" };
const CSRegister TSGDCLDI_RXTSP = { 0x0400, 5, 5, 0, "TSGDCLDI_RXTSP", "" };
const CSRegister TSGSWAPIQ_RXTSP = { 0x0400, 4, 4, 0, "TSGSWAPIQ_RXTSP", "Swap signals at test signal generator's output" };
const CSRegister TSGMODE_RXTSP = { 0x0400, 3, 3, 0, "TSGMODE_RXTSP", "Test signal generator mode" };
const CSRegister INSEL_RXTSP = { 0x0400, 2, 2, 0, "INSEL_RXTSP", "Input source of RxTSP" };
const CSRegister BSTART_RXTSP = {
    0x0400, 1, 1, 0, "BSTART_RXTSP", "Starts delta sigma built in self test. Keep it at 1 one at least three clock cycles"
};
const CSRegister EN_RXTSP = { 0x0400, 0, 0, 1, "EN_RXTSP", "RxTSP modules enable" };
const CSRegister GCORRQ_RXTSP = { 0x0401, 10, 0, 2047, "GCORRQ_RXTSP", "corrector value, channel Q Unsigned integer" };
const CSRegister GCORRI_RXTSP = { 0x0402, 10, 0, 2047, "GCORRI_RXTSP", "corrector value, channel I Unsigned integer" };
const CSRegister HBD_OVR_RXTSP = {
    0x0403, 14, 12, 0, "HBD_OVR_RXTSP", "HBD interpolation ratio. Interpolation ratio is 2HBD_OVR+1"
};
const CSRegister IQCORR_RXTSP = {
    0x0403, 11, 0, 0, "IQCORR_RXTSP", "Phase corrector value (tan(Alpha/2)). Integer, 2's complement"
};
const CSRegister HBD_DLY = { 0x0404, 15, 13, 0, "HBD_DLY", "" };
const CSRegister DCCORR_AVG_RXTSP = { 0x0404,
    2,
    0,
    0,
    "DCCORR_AVG_RXTSP",
    "Number of samples to average for Automatic DC corrector. Number of samples to average is 2DCCORR_AVG + 12" };
const CSRegister GFIR1_L_RXTSP = {
    0x0405, 10, 8, 0, "GFIR1_L_RXTSP", "Parameter l of GFIR1 (l = roundUp(CoeffN/5)-1). Unsigned integer"
};
const CSRegister GFIR1_N_RXTSP = {
    0x0405, 7, 0, 0, "GFIR1_N_RXTSP", "Clock division ratio of GFIR1 is GFIR1_N + 1. Unsigned integer"
};
const CSRegister GFIR2_L_RXTSP = {
    0x0406, 10, 8, 0, "GFIR2_L_RXTSP", "Parameter l of GFIR2 (l = roundUp(CoeffN/5)-1). Unsigned integer"
};
const CSRegister GFIR2_N_RXTSP = {
    0x0406, 7, 0, 0, "GFIR2_N_RXTSP", "Clock division ratio of GFIR2 is GFIR2_N + 1. Unsigned integer"
};
const CSRegister GFIR3_L_RXTSP = {
    0x0407, 10, 8, 0, "GFIR3_L_RXTSP", "Parameter l of GFIR3 (l = roundUp(CoeffN/5)-1). Unsigned integer"
};
const CSRegister GFIR3_N_RXTSP = {
    0x0407, 7, 0, 0, "GFIR3_N_RXTSP", "Clock division ratio of GFIR3 is GFIR3_N + 1. Unsigned integer"
};
const CSRegister AGC_K_RXTSP = { 0x0408, 17, 0, 0, "AGC_K_RXTSP", "AGC loop gain" };
const CSRegister AGC_ADESIRED_RXTSP = { 0x0409, 15, 4, 0, "AGC_ADESIRED_RXTSP", "AGC_ADESIRED" };
const CSRegister RSSI_MODE = { 0x040A, 15, 14, 0, "RSSI_MODE", "" };
const CSRegister AGC_MODE_RXTSP = { 0x040A, 13, 12, 1, "AGC_MODE_RXTSP", "" };
const CSRegister AGC_AVG_RXTSP = { 0x040A, 2, 0, 0, "AGC_AVG_RXTSP", "AGC Averaging window size" };
const CSRegister DC_REG_RXTSP = { 0x040B, 15, 0, 0, "DC_REG_RXTSP", "" };
const CSRegister CMIX_GAIN_RXTSP = { 0x040C, 15, 14, 0, "CMIX_GAIN_RXTSP", "Gain of CMIX output" };
const CSRegister CMIX_SC_RXTSP = { 0x040C, 13, 13, 0, "CMIX_SC_RXTSP", "Spectrum control of CMIX" };
const CSRegister CMIX_BYP_RXTSP = { 0x040C, 7, 7, 0, "CMIX_BYP_RXTSP", "CMIX bypass" };
const CSRegister AGC_BYP_RXTSP = { 0x040C, 6, 6, 0, "AGC_BYP_RXTSP", "AGC bypass" };
const CSRegister GFIR3_BYP_RXTSP = { 0x040C, 5, 5, 1, "GFIR3_BYP_RXTSP", "GFIR3 bypass" };
const CSRegister GFIR2_BYP_RXTSP = { 0x040C, 4, 4, 1, "GFIR2_BYP_RXTSP", "GFIR2 bypass" };
const CSRegister GFIR1_BYP_RXTSP = { 0x040C, 3, 3, 1, "GFIR1_BYP_RXTSP", "GFIR1 bypass" };
const CSRegister DC_BYP_RXTSP = { 0x040C, 2, 2, 0, "DC_BYP_RXTSP", "DC corrector bypass" };
const CSRegister GC_BYP_RXTSP = { 0x040C, 1, 1, 0, "GC_BYP_RXTSP", "Gain corrector bypass" };
const CSRegister PH_BYP_RXTSP = { 0x040C, 0, 0, 0, "PH_BYP_RXTSP", "Phase corrector bypass" };
const CSRegister CAPD = { 0x040E, 31, 0, 0, "CAPD", "" };
const CSRegister DTHBIT_RX = { 0x0440, 8, 5, 1, "DTHBIT_RX", "NCO bits to dither" };
const CSRegister SEL_RX = { 0x0440, 4, 1, 0, "SEL_RX", "" };
const CSRegister MODE_RX = { 0x0440, 0, 0, 0, "MODE_RX", "" };
const CSRegister PHO_RX = { 0x0441, 15, 0, 0, "PHO_RX", "" };
const CSRegister TRX_GAIN_SRC = { 0x0081, 15, 15, 0, "TRX_GAIN_SRC", "Alternative TRX gain source select" };

const CSRegister LML2_TRXIQPULSE = { 0x0022, 15, 15, 0, "LML2_TRXIQPULSE", "TRXIQPULSE mode selection for LML Port 2" };
const CSRegister LML2_SISODDR = { 0x0022, 14, 14, 0, "LML2_SISODDR", "SISODDR mode selection for LML Port 2" };
const CSRegister LML1_TRXIQPULSE = { 0x0022, 13, 13, 0, "LML1_TRXIQPULSE", "TRXIQPULSE mode selection for LML Port 1" };
const CSRegister LML1_SISODDR = { 0x0022, 12, 12, 0, "LML1_SISODDR", "SISODDR mode selection for LML Port 1" };
const CSRegister MCLK2_DLY = { 0x002B, 13, 12, 0, "MCLK2_DLY", "MCLK2 clock internal delay." };
const CSRegister MCLK1_DLY = { 0x002B, 11, 10, 0, "MCLK1_DLY", "MCLK1 clock internal delay." };
const CSRegister MCLK2_INV = { 0x002B, 9, 9, 0, "MCLK2_INV", "MCLK2 clock inversion." };
const CSRegister MCLK1_INV = { 0x002B, 8, 8, 0, "MCLK1_INV", "MCLK1 clock inversion." };

const CSRegister CMIX_GAIN_TXTSP_R3 = { 0x0208, 12, 12, 0, "CMIX_GAIN_TXTSP_R3", "Gain of CMIX output, most significant part" };
const CSRegister CMIX_GAIN_RXTSP_R3 = { 0x040C, 12, 12, 0, "CMIX_GAIN_RXTSP_R3", "Gain of CMIX output, most significant part" };
const CSRegister R5_LPF_BYP_TBB = {
    0x010B, 0, 0, 0, "R5_LPF_BYP_TBB", "Bypasses LPFS5_TBB low pass real-pole filter capacitor banks"
};
const CSRegister CG_IAMP_TBB_R3 = { 0x0125,
    15,
    10,
    37,
    "CG_IAMP_TBB_R3",
    "[Alternative control] This controls the reference bias current of the IAMP's cascode transistors gate voltages that set the "
    "IAMP's input voltage level. The IAMP's input is connected to the DAC output" };
const CSRegister LOSS_LIN_TXPAD_R3 = {
    0x0125, 9, 5, 0, "LOSS_LIN_TXPAD_R3", "[Alternative control] Controls the gain of the linearizing part of of the TXPAD"
};
const CSRegister LOSS_MAIN_TXPAD_R3 = {
    0x0125, 4, 0, 0, "LOSS_MAIN_TXPAD_R3", "[Alternative control] Controls the gain output power of the TXPAD"
};
const CSRegister C_CTL_PGA_RBB_R3 = { 0x0126,
    12,
    11,
    2,
    "C_CTL_PGA_RBB_R3",
    "[Alternative control] Control the value of the feedback capacitor of the PGA that is used to help against the parasitic cap "
    "at the virtual node for stability" };
const CSRegister G_PGA_RBB_R3 = { 0x0126, 10, 6, 11, "G_PGA_RBB_R3", "[Alternative control] Gain of the PGA" };
const CSRegister G_LNA_RFE_R3 = { 0x0126, 5, 2, 15, "G_LNA_RFE_R3", "[Alternative control] Controls the gain of the LNA" };
const CSRegister G_TIA_RFE_R3 = { 0x0126, 1, 0, 3, "G_TIA_RFE_R3", "[Alternative control] Controls the Gain of the TIA" };
const CSRegister RZ_CTRL = { 0x0122, 15, 14, 0, "RZ_CTRL", "Controls the PLL LPF zero resistor values" };
const CSRegister CMPLO_CTRL_SX = {
    0x0122, 13, 13, 0, "CMPLO_CTRL_SX", "Controls the SXR/SXT PLL VCO comparator low threshold value"
};
const CSRegister CMPLO_CTRL_CGEN_R3 = {
    0x008B, 14, 14, 0, "CMPLO_CTRL_CGEN", "Controls the CGEN PLL VCO comparator low threshold value"
};
const CSRegister ISINK_SPIBUFF = { 0x00A6, 15, 13, 0, "ISINK_SPI_BUFF", "Controls the SPIBUF LDO output resistive load" };

const CSRegister DCMODE = { 0x05C0, 15, 15, 0, "DCMODE", "DC calibration mode" };
const CSRegister PD_DCDAC_RXB = { 0x05C0, 7, 7, 1, "PD_DCDAC_RXB", "" };
const CSRegister PD_DCDAC_RXA = { 0x05C0, 6, 6, 1, "PD_DCDAC_RXA", "" };
const CSRegister PD_DCDAC_TXB = { 0x05C0, 5, 5, 1, "PD_DCDAC_TXB", "" };
const CSRegister PD_DCDAC_TXA = { 0x05C0, 4, 4, 1, "PD_DCDAC_TXA", "" };
const CSRegister PD_DCCMP_RXB = { 0x05C0, 3, 3, 1, "PD_DCCMP_RXB", "" };
const CSRegister PD_DCCMP_RXA = { 0x05C0, 2, 2, 1, "PD_DCCMP_RXA", "" };
const CSRegister PD_DCCMP_TXB = { 0x05C0, 1, 1, 1, "PD_DCCMP_TXB", "" };
const CSRegister PD_DCCMP_TXA = { 0x05C0, 0, 0, 1, "PD_DCCMP_TXA", "" };

const CSRegister DCCAL_CALSTATUS_RXBQ = { 0x05C1, 15, 15, 0, "DCCAL_CALSTATUS_RXBQ", "" };
const CSRegister DCCAL_CALSTATUS_RXBI = { 0x05C1, 14, 14, 0, "DCCAL_CALSTATUS_RXBI", "" };
const CSRegister DCCAL_CALSTATUS_RXAQ = { 0x05C1, 13, 13, 0, "DCCAL_CALSTATUS_RXAQ", "" };
const CSRegister DCCAL_CALSTATUS_RXAI = { 0x05C1, 12, 12, 0, "DCCAL_CALSTATUS_RXAI", "" };
const CSRegister DCCAL_CALSTATUS_TXBQ = { 0x05C1, 11, 11, 0, "DCCAL_CALSTATUS_TXBQ", "" };
const CSRegister DCCAL_CALSTATUS_TXBI = { 0x05C1, 10, 10, 0, "DCCAL_CALSTATUS_TXBI", "" };
const CSRegister DCCAL_CALSTATUS_TXAQ = { 0x05C1, 9, 9, 0, "DCCAL_CALSTATUS_TXAQ", "" };
const CSRegister DCCAL_CALSTATUS_TXAI = { 0x05C1, 8, 8, 0, "DCCAL_CALSTATUS_TXAI", "" };

const CSRegister DCCAL_CMPSTATUS_RXBQ = { 0x05C1, 7, 7, 0, "DCCAL_CMPSTATUS_RXBQ", "" };
const CSRegister DCCAL_CMPSTATUS_RXBI = { 0x05C1, 6, 6, 0, "DCCAL_CMPSTATUS_RXBI", "" };
const CSRegister DCCAL_CMPSTATUS_RXAQ = { 0x05C1, 5, 5, 0, "DCCAL_CMPSTATUS_RXAQ", "" };
const CSRegister DCCAL_CMPSTATUS_RXAI = { 0x05C1, 4, 4, 0, "DCCAL_CMPSTATUS_RXAI", "" };
const CSRegister DCCAL_CMPSTATUS_TXBQ = { 0x05C1, 3, 3, 0, "DCCAL_CMPSTATUS_TXBQ", "" };
const CSRegister DCCAL_CMPSTATUS_TXBI = { 0x05C1, 2, 2, 0, "DCCAL_CMPSTATUS_TXBI", "" };
const CSRegister DCCAL_CMPSTATUS_TXAQ = { 0x05C1, 1, 1, 0, "DCCAL_CMPSTATUS_TXAQ", "" };
const CSRegister DCCAL_CMPSTATUS_TXAI = { 0x05C1, 0, 0, 0, "DCCAL_CMPSTATUS_TXAI", "" };

const CSRegister DCCAL_CMPCFG_RXBQ = { 0x05C2, 15, 15, 0, "DCCAL_CMPCFG_RXBQ", "" };
const CSRegister DCCAL_CMPCFG_RXBI = { 0x05C2, 14, 14, 0, "DCCAL_CMPCFG_RXBI", "" };
const CSRegister DCCAL_CMPCFG_RXAQ = { 0x05C2, 13, 13, 0, "DCCAL_CMPCFG_RXAQ", "" };
const CSRegister DCCAL_CMPCFG_RXAI = { 0x05C2, 12, 12, 0, "DCCAL_CMPCFG_RXAI", "" };
const CSRegister DCCAL_CMPCFG_TXBQ = { 0x05C2, 11, 11, 0, "DCCAL_CMPCFG_TXBQ", "" };
const CSRegister DCCAL_CMPCFG_TXBI = { 0x05C2, 10, 10, 0, "DCCAL_CMPCFG_TXBI", "" };
const CSRegister DCCAL_CMPCFG_TXAQ = { 0x05C2, 9, 9, 0, "DCCAL_CMPCFG_TXAQ", "" };
const CSRegister DCCAL_CMPCFG_TXAI = { 0x05C2, 8, 8, 0, "DCCAL_CMPCFG_TXAI", "" };

const CSRegister DCCAL_START_RXBQ = { 0x05C2, 7, 7, 0, "DCCAL_START_RXBQ", "" };
const CSRegister DCCAL_START_RXBI = { 0x05C2, 6, 6, 0, "DCCAL_START_RXBI", "" };
const CSRegister DCCAL_START_RXAQ = { 0x05C2, 5, 5, 0, "DCCAL_START_RXAQ", "" };
const CSRegister DCCAL_START_RXAI = { 0x05C2, 4, 4, 0, "DCCAL_START_RXAI", "" };
const CSRegister DCCAL_START_TXBQ = { 0x05C2, 3, 3, 0, "DCCAL_START_TXBQ", "" };
const CSRegister DCCAL_START_TXBI = { 0x05C2, 2, 2, 0, "DCCAL_START_TXBI", "" };
const CSRegister DCCAL_START_TXAQ = { 0x05C2, 1, 1, 0, "DCCAL_START_TXAQ", "" };
const CSRegister DCCAL_START_TXAI = { 0x05C2, 0, 0, 0, "DCCAL_START_TXAI", "" };

const CSRegister DCWR_TXAI = { 0x05C3, 15, 15, 0, "DCWR_TXAI", "" };
const CSRegister DCRD_TXAI = { 0x05C3, 14, 14, 0, "DCRD_TXAI", "" };
const CSRegister DC_TXAI = { 0x05C3, 10, 0, 0, "DC_TXAI", "" };
const CSRegister DCWR_TXAQ = { 0x05C4, 15, 15, 0, "DCWR_TXAQ", "" };
const CSRegister DCRD_TXAQ = { 0x05C4, 14, 14, 0, "DCRD_TXAQ", "" };
const CSRegister DC_TXAQ = { 0x05C4, 10, 0, 0, "DC_TXAQ", "" };
const CSRegister DCWR_TXBI = { 0x05C5, 15, 15, 0, "DCWR_TXBI", "" };
const CSRegister DCRD_TXBI = { 0x05C5, 14, 14, 0, "DCRD_TXBI", "" };
const CSRegister DC_TXBI = { 0x05C5, 10, 0, 0, "DC_TXBI", "" };
const CSRegister DCWR_TXBQ = { 0x05C6, 15, 15, 0, "DCWR_TXBQ", "" };
const CSRegister DCRD_TXBQ = { 0x05C6, 14, 14, 0, "DCRD_TXBQ", "" };
const CSRegister DC_TXBQ = { 0x05C6, 10, 0, 0, "DC_TXBQ", "" };
const CSRegister DCWR_RXAI = { 0x05C7, 15, 15, 0, "DCWR_RXAI", "" };
const CSRegister DCRD_RXAI = { 0x05C7, 14, 14, 0, "DCRD_RXAI", "" };
const CSRegister DC_RXAI = { 0x05C7, 6, 0, 0, "DC_RXAI", "" };
const CSRegister DCWR_RXAQ = { 0x05C8, 15, 15, 0, "DCWR_RXAQ", "" };
const CSRegister DCRD_RXAQ = { 0x05C8, 14, 14, 0, "DCRD_RXAQ", "" };
const CSRegister DC_RXAQ = { 0x05C8, 6, 0, 0, "DC_RXAQ", "" };
const CSRegister DCWR_RXBI = { 0x05C9, 15, 15, 0, "DCWR_RXBI", "" };
const CSRegister DCRD_RXBI = { 0x05C9, 14, 14, 0, "DCRD_RXBI", "" };
const CSRegister DC_RXBI = { 0x05C9, 6, 0, 0, "DC_RXBI", "" };
const CSRegister DCWR_RXBQ = { 0x05CA, 15, 15, 0, "DCWR_RXBQ", "" };
const CSRegister DCRD_RXBQ = { 0x05CA, 14, 14, 0, "DCRD_RXBQ", "" };
const CSRegister DC_RXBQ = { 0x05CA, 6, 0, 0, "DC_RXBQ", "" };
const CSRegister DC_RXCDIV = { 0x05CB, 15, 8, 32, "DC_RXCDIV", "Clock division ratio for Rx DC calibration loop" };
const CSRegister DC_TXCDIV = { 0x05CB, 7, 0, 32, "DC_TXCDIV", "Clock division ratio for Tx DC calibration loop" };
const CSRegister HYSCMP_RXB = { 0x05CC, 11, 9, 0, "HYSCMP_RXB", "Comparator hysteresis control, RXB channel" };
const CSRegister HYSCMP_RXA = { 0x05CC, 8, 6, 0, "HYSCMP_RXA", "Comparator hysteresis control, RXA channel" };
const CSRegister HYSCMP_TXB = { 0x05CC, 5, 3, 0, "HYSCMP_TXB", "Comparator hysteresis control, TXB channel" };
const CSRegister HYSCMP_TXA = { 0x05CC, 2, 0, 0, "HYSCMP_TXA", "Comparator hysteresis control, TXA channel" };
const CSRegister DAC_CLKDIV = { 0x0600, 15, 8, 15, "DAC_CLKDIV", "Clock division ratio for measurement loop" };
const CSRegister RSSI_RSSIMODE = { 0x0600, 1, 1, 0, "RSSI_RSSIMODE", "" };
const CSRegister RSSI_PD = { 0x0600, 0, 0, 0, "RSSI_PD", "" };
const CSRegister INTADC_CMPSTATUS_TEMPREF = { 0x0601, 5, 5, 0, "INTADC_CMPSTATUS_TEMPREF", "" };
const CSRegister INTADC_CMPSTATUS_TEMPVPTAT = { 0x0601, 4, 4, 0, "INTADC_CMPSTATUS_TEMPVPTAT", "" };
const CSRegister INTADC_CMPSTATUS_RSSI2 = { 0x0601, 3, 3, 0, "INTADC_CMPSTATUS_RSSI2", "" };
const CSRegister INTADC_CMPSTATUS_RSSI1 = { 0x0601, 2, 2, 0, "INTADC_CMPSTATUS_RSSI1", "" };
const CSRegister INTADC_CMPSTATUS_PDET2 = { 0x0601, 1, 1, 0, "INTADC_CMPSTATUS_PDET2", "" };
const CSRegister INTADC_CMPSTATUS_PDET1 = { 0x0601, 0, 0, 0, "INTADC_CMPSTATUS_PDET1", "" };
const CSRegister RSSI_BIAS = { 0x0602, 13, 9, 1 << 4, "RSSI_BIAS", "Controls the reference bias current of the test ADC" };
const CSRegister RSSI_HYSCMP = { 0x0602, 8, 6, 0, "RSSI_HYSCMP", "Comparator hysteresis control" };
const CSRegister INTADC_CMPCFG_TEMPREF = { 0x0602, 5, 5, 0, "INTADC_CMPCFG_TEMPREF", "" };
const CSRegister INTADC_CMPCFG_TEMPVPTAT = { 0x0602, 4, 4, 0, "INTADC_CMPCFG_TEMPVPTAT", "" };
const CSRegister INTADC_CMPCFG_RSSI2 = { 0x0602, 3, 3, 0, "INTADC_CMPCFG_RSSI2", "" };
const CSRegister INTADC_CMPCFG_RSSI1 = { 0x0602, 2, 2, 0, "INTADC_CMPCFG_RSSI1", "" };
const CSRegister INTADC_CMPCFG_PDET2 = { 0x0602, 1, 1, 0, "INTADC_CMPCFG_PDET2", "" };
const CSRegister INTADC_CMPCFG_PDET1 = { 0x0602, 0, 0, 0, "INTADC_CMPCFG_PDET1", "" };
const CSRegister RSSI_DAC_VAL = { 0x0603, 7, 0, 0, "RSSI_DAC_VAL", "Stores the value to be written to the DAC, when MODE = 1" };
const CSRegister RSSI_PDET2_VAL = { 0x0604, 15, 8, 0, "RSSI_PDET2_VAL", "Stores the value of Power Detector 2" };
const CSRegister RSSI_PDET1_VAL = { 0x0604, 7, 0, 0, "RSSI_PDET1_VAL", "Stores the value of Power Detector 1" };
const CSRegister RSSI_RSSI2_VAL = { 0x0605, 15, 8, 0, "RSSI_RSSI2_VAL", "Stores the value of RSSI 2" };
const CSRegister RSSI_RSSI1_VAL = { 0x0605, 7, 0, 0, "RSSI_RSSI1_VAL", "Stores the value of RSSI 1" };
const CSRegister RSSI_TREF_VAL = { 0x0606, 15, 8, 0, "RSSI_TREF_VAL", "Stores the temperature reference value" };
const CSRegister RSSI_TVPTAT_VAL = {
    0x0606, 7, 0, 0, "RSSI_TVPTAT_VAL", "Stores the voltage proportional to absolute temperature value"
};
const CSRegister RSSIDC_CMPSTATUS = { 0x0640, 15, 15, 0, "RSSIDC_CMPSTATUS", "" };
const CSRegister RSSIDC_RSEL = { 0x0640, 8, 4, 10, "RSSIDC_RSEL", "Reference voltage for the RSSI output comparator" };
const CSRegister RSSIDC_HYSCMP = { 0x0640, 3, 1, 0, "RSSIDC_HYSCMP", "" };
const CSRegister RSSIDC_PD = { 0x0640, 0, 0, 0, "RSSIDC_PD", "Power down modules" };
const CSRegister RSSIDC_DCO2 = { 0x0641, 13, 7, 32, "RSSIDC_DCO2", "Value of RSSI offset DAC2" };
const CSRegister RSSIDC_DCO1 = { 0x0641, 6, 0, 32, "RSSIDC_DCO1", "Value of RSSI offset DAC1" };
const CSRegister DCLOOP_STOP = { 0x040C, 8, 8, 0, "DCLOOP_STOP", "Stops RxDC tracking loop" };
} // namespace LMS7002MCSR_Data
} // namespace lime