#include "DCCorrector.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#include "embedded/lms7002m/csr_data.h"
#include "embedded/lms7002m/spi.h"

namespace lime {

LMS7002M_DC::LMS7002M_DC(lms7002m_context* rfsoc, TRXDir dir, uint32_t channel)
    : rfsoc(rfsoc)
{
    base_reg = (dir == TRXDir::Tx) ? 0x05C3 : 0x05C7;
    base_reg += channel * 2;
}

OpStatus LMS7002M_DC::Enabled(bool enable)
{
    switch (base_reg)
    {
    case 0x05C3:
        return static_cast<lime::OpStatus>(lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_TXA, !enable));
    case 0x05C5:
        return static_cast<lime::OpStatus>(lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_TXB, !enable));
    case 0x05C7:
        return static_cast<lime::OpStatus>(lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXA, !enable));
    case 0x05C9:
        return static_cast<lime::OpStatus>(lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXB, !enable));
    default:
        return OpStatus::Error;
    }
}

OpStatus LMS7002M_DC::SetDCOffset(complex16_t offset)
{
    SetDCI(offset.real());
    return SetDCQ(offset.imag());
}

OpStatus LMS7002M_DC::SetDCI(int16_t offset)
{
    lms7002m_write_analog_dc(rfsoc, base_reg, offset);
    return OpStatus::Success;
}

OpStatus LMS7002M_DC::SetDCQ(int16_t offset)
{
    lms7002m_write_analog_dc(rfsoc, base_reg + 1, offset);
    return OpStatus::Success;
}

complex16_t LMS7002M_DC::GetDCOffset()
{
    return complex16_t(lms7002m_read_analog_dc(rfsoc, base_reg), lms7002m_read_analog_dc(rfsoc, base_reg + 1));
}

lime::Range<float> LMS7002M_DC::GetRange()
{
    if (base_reg < 0x05C7) // Tx
        return lime::Range<float>(-1023, 1023, 1.0);
    else // Rx
        return lime::Range<float>(-63, 63, 1.0);
}

IDCCorrector::Type LMS7002M_DC::GetType() const
{
    return IDCCorrector::Type::Analog;
}

} // namespace lime