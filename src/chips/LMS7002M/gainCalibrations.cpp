#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"
#include "LMS7002MCSR_Data.h"
#include <algorithm>
#include <cmath>

using namespace lime;
using namespace lime::LMS7002MCSR_Data;
using namespace std::literals::string_literals;

OpStatus LMS7002M::CalibrateTxGainSetup()
{
    OpStatus status;
    int ch = Get_SPI_Reg_bits(LMS7002MCSR::MAC);

    uint16_t value = SPI_read(0x0020);
    if ((value & 3) == 1)
        value = value | 0x0014;
    else
        value = value | 0x0028;
    SPI_write(0x0020, value);

    //RxTSP
    SetDefaults(MemorySection::RxTSP);
    SetDefaults(MemorySection::RxNCO);
    Modify_SPI_Reg_bits(LMS7002MCSR::AGC_MODE_RXTSP, 1);
    Modify_SPI_Reg_bits(LMS7002MCSR::AGC_AVG_RXTSP, 1);
    Modify_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP, 1);
    Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_BYP_RXTSP, 1);
    Modify_SPI_Reg_bits(LMS7002MCSR::AGC_BYP_RXTSP, 0);

    //TBB
    Modify_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, 1);
    Modify_SPI_Reg_bits(LMS7002MCSR::LOOPB_TBB, 3);

    //RFE
    Modify_SPI_Reg_bits(LMS7002MCSR::EN_G_RFE, 0);
    Modify_SPI_Reg_bits(0x010D, 4, 1, 0xF);

    //RBB
    SetDefaults(MemorySection::RBB);
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_LPFL_RBB, 1);
    Modify_SPI_Reg_bits(LMS7002MCSR::INPUT_CTL_PGA_RBB, 3);
    Modify_SPI_Reg_bits(LMS7002MCSR::G_PGA_RBB, 12);
    Modify_SPI_Reg_bits(LMS7002MCSR::RCC_CTL_PGA_RBB, 23);

    //TRF
    Modify_SPI_Reg_bits(LMS7002MCSR::EN_G_TRF, 0);

    //AFE
    const int isel_dac_afe = Get_SPI_Reg_bits(LMS7002MCSR::ISEL_DAC_AFE);
    SetDefaults(MemorySection::AFE);
    Modify_SPI_Reg_bits(LMS7002MCSR::ISEL_DAC_AFE, isel_dac_afe);
    if (ch == 2)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::PD_RX_AFE2, 0);
        Modify_SPI_Reg_bits(LMS7002MCSR::PD_TX_AFE2, 0);
    }

    //BIAS
    const int rp_calib_bias = Get_SPI_Reg_bits(LMS7002MCSR::RP_CALIB_BIAS);
    SetDefaults(MemorySection::BIAS);
    Modify_SPI_Reg_bits(LMS7002MCSR::RP_CALIB_BIAS, rp_calib_bias);

    //LDO
    //do nothing

    //XBUF
    //use configured xbuf settings

    //CGEN
    SetDefaults(MemorySection::CGEN);
    // Don't trigger FPGA interface update, samples streaming is not required for this calibration, and because it would have to be restored.
    // After calibration, CGEN registers will be restored manually, that won't trigger FPGA update.
    skipExternalDataInterfaceUpdate = true;
    status = SetFrequencyCGEN(61.44e6);
    skipExternalDataInterfaceUpdate = false;
    if (status != OpStatus::Success)
        return status;

    //SXR
    Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 1);
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_VCO, 1);

    Modify_SPI_Reg_bits(LMS7002MCSR::MAC, ch);

    //TxTSP
    const int isinc = Get_SPI_Reg_bits(LMS7002MCSR::ISINC_BYP_TXTSP);
    const int txcmixGainLSB = Get_SPI_Reg_bits(LMS7002MCSR::CMIX_GAIN_TXTSP);
    const int txcmixGainMSB = Get_SPI_Reg_bits(LMS7002MCSR::CMIX_GAIN_TXTSP_R3);
    SetDefaults(MemorySection::TxTSP);
    SetDefaults(MemorySection::TxNCO);
    Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_GAIN_TXTSP, txcmixGainLSB);
    Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_GAIN_TXTSP_R3, txcmixGainMSB);
    Modify_SPI_Reg_bits(LMS7002MCSR::ISINC_BYP_TXTSP, isinc);
    Modify_SPI_Reg_bits(LMS7002MCSR::TSGMODE_TXTSP, 1);
    Modify_SPI_Reg_bits(LMS7002MCSR::INSEL_TXTSP, 1);
    int16_t tsgValue = 0x7FFF;
    if (txcmixGainMSB == 0 && txcmixGainLSB == 1)
        tsgValue = 0x3FFF;
    else if (txcmixGainMSB == 1 && txcmixGainLSB == 0)
        tsgValue = 0x5A85;
    else
        tsgValue = 0x7FFF;
    LoadDC_REG_IQ(TRXDir::Tx, tsgValue, tsgValue);
    SetNCOFrequency(TRXDir::Tx, 0, 0.5e6);
    Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_BYP_TXTSP, 0);

    return OpStatus::Success;
}

///APPROXIMATE conversion
static constexpr uint32_t maxRSSI = 0x10669;
static float chip_rssi_to_dbfs(uint32_t rssi)
{
    if (rssi == 0)
        rssi = 1;
    return 20 * log10(float(rssi) / maxRSSI);
}

OpStatus LMS7002M::CalibrateTxGain()
{
    if (!controlPort)
    {
        lime::error("No device connected"s);
        return OpStatus::IOFailure;
    }
    OpStatus status;
    int cg_iamp = 1;
    auto registersBackup = BackupRegisterMap();
    status = CalibrateTxGainSetup();
    if (status == OpStatus::Success)
    {
        lime::debug("Calibrating CG_IAMP_TBB:"s);
        Modify_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, cg_iamp);
        uint32_t previousRSSI = GetRSSI();
        lime::debug("CG_IAMP_TBB(%i) RSSI:0x%08X  approx. %+2.2f dBFS", cg_iamp, previousRSSI, chip_rssi_to_dbfs(previousRSSI));

        while (GetRSSI() < 0xFD00)
        {
            ++cg_iamp;
            if (cg_iamp > 63)
                break;

            Modify_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, cg_iamp);
            const uint32_t rssi = GetRSSI();
            lime::debug("CG_IAMP_TBB(%i) RSSI:0x%08X  approx. %+2.2f dBFS", cg_iamp, rssi, chip_rssi_to_dbfs(rssi));
            if (rssi < previousRSSI)
            {
                // drop in RSSI indicates oversaturation
                --cg_iamp;
                break;
            }
            previousRSSI = rssi;
        }
    }
    RestoreRegisterMap(registersBackup);

    int ind = GetActiveChannelIndex() % 2;

    if (status == OpStatus::Success)
    {
        opt_gain_tbb[ind] = std::clamp(cg_iamp, 1, 63); // can't allow opt_gain_tbb to be 0, it's used in division
        Modify_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, cg_iamp);
    }
    //logic reset
    Modify_SPI_Reg_bits(LMS7002MCSR::LRST_TX_A, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::LRST_TX_B, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::LRST_TX_A, 1);
    Modify_SPI_Reg_bits(LMS7002MCSR::LRST_TX_B, 1);

    return status;
}
