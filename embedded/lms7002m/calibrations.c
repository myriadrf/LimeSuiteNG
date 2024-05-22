#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#include "csr.h"
#include "lms7002m_context.h"
#include "privates.h"

#include <stdint.h>
#include <stdlib.h>

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

lime_Result lms7002m_calibrate_analog_rssi_dc_offset(lms7002m_context* self)
{
    lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_W_RFE, 1);
    lms7002m_calibrate_internal_adc(self, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_PD_RSSI_RFE, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_PD_TIA_RFE, 0);

    lms7002m_spi_write(self, 0x0640, 22 << 4);

    lms7002m_spi_modify_csr(self, LMS7002M_RSSIDC_DCO2, 0);

    int value = -63;
    uint8_t wrValue = abs(value);
    if (value < 0)
        wrValue |= 0x40;
    lms7002m_spi_modify_csr(self, LMS7002M_RSSIDC_DCO1, wrValue);
    uint8_t cmp = lms7002m_spi_read_csr(self, LMS7002M_RSSIDC_CMPSTATUS);
    uint8_t cmpPrev = cmp;
    int8_t edges[2];
    uint8_t edgesIndex = 0;
    for (value = -63; value < 64; ++value)
    {
        wrValue = abs(value);
        if (value < 0)
            wrValue |= 0x40;
        lms7002m_spi_modify_csr(self, LMS7002M_RSSIDC_DCO1, wrValue);
        lms7002m_sleep(5);
        cmp = lms7002m_spi_read_csr(self, LMS7002M_RSSIDC_CMPSTATUS);
        if (cmp != cmpPrev)
        {
            edges[edgesIndex++] = value;
            cmpPrev = cmp;

            if (edgesIndex > 1)
            {
                break;
            }
        }
    }
    if (edgesIndex != 2)
    {
        LOG_D(self, "%s", "Not found");
        return lms7002m_report_error(self, lime_Result_InvalidValue, "%s", "Failed to find value");
    }
    const int8_t found = (edges[0] + edges[1]) / 2;
    wrValue = abs(found);
    if (found < 0)
        wrValue |= 0x40;
    lms7002m_spi_modify_csr(self, LMS7002M_RSSIDC_DCO1, wrValue);
    LOG_D(self, "Found %i", found);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_W_RFE, 0);
    return lime_Result_Success;
}
