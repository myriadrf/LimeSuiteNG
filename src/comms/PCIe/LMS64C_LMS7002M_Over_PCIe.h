#ifndef LIME_LMS64C_LMS7002M_OVER_PCIE_H
#define LIME_LMS64C_LMS7002M_OVER_PCIE_H

#include "comms/IComms.h"
#include "LitePCIe.h"
#include "PCIE_CSR_Pipe.h"

#include <cstdint>
#include <memory>

namespace lime {

/** @brief A class for communicating with a device's LMS7002M chip over a PCIe interface. */
class LMS64C_LMS7002M_Over_PCIe : public IComms
{
  public:
    /**
      @brief Constructs a new LMS64C_LMS7002M_Over_PCIe object
      @param dataPort The PCIe data connection to use.
     */
    LMS64C_LMS7002M_Over_PCIe(std::shared_ptr<LitePCIe> dataPort);
    virtual OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    virtual OpStatus SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    OpStatus ResetDevice(int chipSelect) override;

    OpStatus GPIOWrite(const uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIORead(uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIODirWrite(const uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIODirRead(uint8_t* buffer, const size_t bufLength) override;
    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;
    OpStatus ProgramWrite(const char* data, size_t length, int prog_mode, int target, ProgressCallback callback = nullptr) override;
    OpStatus MemoryWrite(uint32_t address, const void* data, uint32_t dataLength) override;
    OpStatus MemoryRead(uint32_t address, void* data, uint32_t dataLength) override;

  private:
    PCIE_CSR_Pipe pipe;
};

} // namespace lime

#endif // LIME_LMS64C_LMS7002M_OVER_PCIE_H
