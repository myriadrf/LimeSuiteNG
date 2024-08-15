#include "SPI_utilities.h"

#include "limesuiteng/Register.h"
#include "comms/ISPI.h"

namespace lime {

OpStatus WriteSPI(ISPI* impl, uint16_t address, uint16_t value)
{
    const uint32_t mosi = (1 << 31) | address << 16 | value;
    return impl->SPI(&mosi, nullptr, 1);
}

uint16_t ReadSPI(ISPI* impl, uint16_t address, OpStatus* status)
{
    const uint32_t mosi = address;
    uint32_t miso = 0;
    const OpStatus ret = impl->SPI(&mosi, &miso, 1);
    if (status)
        *status = ret;
    return miso & 0xFFFF;
}

OpStatus ModifyRegister(ISPI* impl, const Register& reg, uint16_t value)
{
    uint32_t mosi = reg.address;
    uint32_t miso = 0;

    // Read
    lime::OpStatus status = impl->SPI(&mosi, &miso, 1);
    if (status != lime::OpStatus::Success)
        return status;
    const uint16_t regMask = bitMask(reg.msb, reg.lsb);

    // Modify
    uint32_t regValue = (miso & ~regMask);
    regValue |= ((value << reg.lsb) & regMask);

    // Write
    mosi = (1 << 31) | reg.address << 16 | regValue;
    return impl->SPI(&mosi, nullptr, 1);
}

uint16_t ReadRegister(ISPI* impl, const Register& reg, OpStatus* status)
{
    uint32_t mosi = reg.address;
    uint32_t miso = 0;
    const uint16_t regMask = bitMask(reg.msb, reg.lsb);
    // Read
    const OpStatus ret = impl->SPI(&mosi, &miso, 1);
    if (status)
        *status = ret;
    return (miso & regMask) >> reg.lsb;
}

} // namespace lime
