#ifndef LIME_LMS8001_H
#define LIME_LMS8001_H

// #define REF_CLK_MHZ 30.72
#define REF_CLK_MHZ 40.00

// Temperature Sensor Default Coefficients
#define TEMPSENS_T0 -105.45
#define TEMPSENS_T1 1.2646
#define TEMPSENS_T2 -0.000548

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/config.h"
#include "LMS8001_parameters.h"

#include <string>
#include <memory>

namespace lime {

class ISPI;
class LMS8001_RegistersMap;

class LIME_API LMS8001
{
  public:
    LMS8001(std::shared_ptr<ISPI> port);
    virtual ~LMS8001();

    OpStatus UploadAll();
    OpStatus DownloadAll();
    bool IsSynced();

    OpStatus ResetChip();
    OpStatus LoadConfig(const std::string& filename);
    OpStatus SaveConfig(const std::string& filename);

    uint16_t Get_SPI_Reg_bits(
        const LMS8Parameter& param, bool fromChip = true, uint8_t channel = 0, uint8_t PLLprofile = 0, OpStatus* status = nullptr);
    uint16_t Get_SPI_Reg_bits(uint16_t address, uint8_t msb, uint8_t lsb, bool fromChip = true, OpStatus* status = nullptr);

    OpStatus Modify_SPI_Reg_bits(
        const LMS8Parameter& param, const uint16_t value, bool fromChip = true, uint8_t channel = 0, uint8_t PLLprofile = 0);
    OpStatus Modify_SPI_Reg_bits(uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value, bool fromChip = true);
    OpStatus SPI_write(uint16_t address, uint16_t data);
    uint16_t SPI_read(uint16_t address, bool fromChip = true, OpStatus* status = 0);

    LMS8Parameter findLMS8Param(char* field);

    uint8_t channel;
    uint8_t PLLprofile;

    enum MemorySection {
        MS_ChipConfig = 0,
        MS_BiasLDOConfig,
        MS_Channel,
        MS_HLMIX,
        MS_PLLConfig,
        MS_PLLProfiles,
        MEMORY_SECTIONS_COUNT
    };
    virtual OpStatus SetDefaults(MemorySection module);

    double tempSens_T0;
    double mRefClk_MHz;

  private:
    uint16_t MemorySectionAddresses[MEMORY_SECTIONS_COUNT][2];
    void BackupAllRegisters();
    void RestoreAllRegisters();

    OpStatus SPI_write_batch(const uint16_t* spiAddr, const uint16_t* spiData, uint16_t cnt);
    OpStatus SPI_read_batch(const uint16_t* spiAddr, uint16_t* spiData, uint16_t cnt);
    OpStatus Modify_SPI_Reg_mask(const uint16_t* addr, const uint16_t* masks, const uint16_t* values, uint8_t start, uint8_t stop);

    std::shared_ptr<lime::ISPI> controlPort;
    LMS8001_RegistersMap* mRegistersMap;
};

} // namespace lime

#endif // LIME_LMS8001_H
