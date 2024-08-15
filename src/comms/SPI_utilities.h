#ifndef LIMESUITENG_SPI_UTILITIES_H
#define LIMESUITENG_SPI_UTILITIES_H

#include <cstdint>

#include "limesuiteng/OpStatus.h"

namespace lime {

class ISPI;
class Register;

OpStatus WriteSPI(ISPI* spi, uint16_t address, uint16_t value);
uint16_t ReadSPI(ISPI* spi, uint16_t address, OpStatus* status = nullptr);

OpStatus ModifyRegister(ISPI* spi, const Register& reg, uint16_t value);
uint16_t ReadRegister(ISPI* spi, const Register& reg, OpStatus* status = nullptr);

} // namespace lime

#endif // LIMESUITENG_SPI_UTILITIES_H