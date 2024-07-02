#ifndef LIME_LMS64C_FPGA_OVER_USB_H
#define LIME_LMS64C_FPGA_OVER_USB_H

#include "comms/IComms.h"
#include "USB_CSR_Pipe.h"
#include <memory>

namespace lime {

/** @brief A class for communicating with a device's FPGA over a USB interface. */
class LMS64C_FPGA_Over_USB : public IComms
{
  public:
    /**
      @brief Constructs a new LMS64C_FPGA_Over_USB object.
      @param dataPort The USB communications pipe to use.
     */
    LMS64C_FPGA_Over_USB(std::shared_ptr<USB_CSR_Pipe> dataPort);

    OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    OpStatus SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    OpStatus GPIODirRead(uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIORead(uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIODirWrite(const uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIOWrite(const uint8_t* buffer, const size_t bufLength) override;

    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    OpStatus ProgramWrite(const char* data, size_t length, int prog_mode, int target, ProgressCallback callback = nullptr) override;

    OpStatus MemoryWrite(uint32_t address, const void* data, uint32_t dataLength) override;
    OpStatus MemoryRead(uint32_t address, void* data, uint32_t dataLength) override;

  private:
    std::shared_ptr<USB_CSR_Pipe> pipe;
};

} // namespace lime

#endif // LIME_LMS64C_FPGA_OVER_USB_H
