#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#include "csr.h"
#include "lms7002m_context.h"
#include "privates.h"

#include <stdint.h>

lime_Result lms7002m_calibrate_internal_adc(lms7002m_context* self, int clkDiv)
{
    if (lms7002m_spi_read_csr(self, LMS7002M_MASK) == 0)
        return lms7002m_report_error(self, lime_Result_NotSupported, "Operation not supported");

    const uint16_t biasMux = lms7002m_spi_read_csr(self, LMS7002M_MUX_BIAS_OUT);
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, 1);

    lms7002m_spi_write(self, 0x0600, 0x0F01);
    lms7002m_spi_write(self, 0x0602, 0x2000);
    lms7002m_spi_write(self, 0x0603, 0x0000);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_PD, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_RSSIMODE, 1);
    lms7002m_spi_modify_csr(self, LMS7002M_DAC_CLKDIV, clkDiv);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_BIAS, 8);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_DAC_VAL, 170);

    uint8_t bias = lms7002m_spi_read_csr(self, LMS7002M_RSSI_BIAS);
    uint16_t regValue = lms7002m_spi_read(self, 0x0601);
    while (((regValue >> 5) & 0x1) != 1)
    {
        if (bias > 31)
            return lms7002m_report_error(self, lime_Result_Error, "Temperature internal ADC calibration failed");
        ++bias;
        lms7002m_spi_modify_csr(self, LMS7002M_RSSI_BIAS, bias);
        regValue = lms7002m_spi_read(self, 0x0601);
    }
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_PD, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, biasMux);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_RSSIMODE, 0);
    return lime_Result_Success;
}

lime_Result lms7002m_calibrate_rp_bias(lms7002m_context* self)
{
    if (lms7002m_spi_read_csr(self, LMS7002M_MASK) == 0)
        return lms7002m_report_error(self, lime_Result_NotSupported, "Operation not supported");

    lms7002m_calibrate_internal_adc(self, 32);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_PD, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_RSSIMODE, 0);

    const uint16_t biasMux = lms7002m_spi_read_csr(self, LMS7002M_MUX_BIAS_OUT);
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, 1);

    lms7002m_sleep(250);

    uint16_t reg606 = lms7002m_spi_read(self, 0x0606);
    uint16_t Vref = (reg606 >> 8) & 0xFF;
    uint16_t Vptat = reg606 & 0xFF;

    if (Vref > Vptat)
    {
        uint16_t rpCalib = lms7002m_spi_read_csr(self, LMS7002M_RP_CALIB_BIAS);
        while (Vref > Vptat)
        {
            --rpCalib;
            lms7002m_spi_modify_csr(self, LMS7002M_RP_CALIB_BIAS, rpCalib);
            reg606 = lms7002m_spi_read(self, 0x0606);
            Vref = (reg606 >> 8) & 0xFF;
            Vptat = reg606 & 0xFF;
        }
    }
    if (Vref < Vptat)
    {
        uint16_t rpCalib = lms7002m_spi_read_csr(self, LMS7002M_RP_CALIB_BIAS);
        while (Vref < Vptat)
        {
            ++rpCalib;
            lms7002m_spi_modify_csr(self, LMS7002M_RP_CALIB_BIAS, rpCalib);
            reg606 = lms7002m_spi_read(self, 0x0606);
            Vref = (reg606 >> 8) & 0xFF;
            Vptat = reg606 & 0xFF;
        }
    }
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, biasMux);
    return lime_Result_Success;
}
