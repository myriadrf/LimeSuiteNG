/**
    @file LMS64CProtocol.h
    @author Lime Microsystems
    @brief Implementation of LMS64C protocol.
*/

#pragma once
#include <functional>
#include <mutex>
#include <thread>

#include "limesuiteng/OpStatus.h"
#include "LMSBoards.h"

namespace lime {

class ISerialPort;
struct SDRDescriptor;
struct CustomParameterIO;

struct LMS64CPacket {
    static constexpr int size = 64;
    static constexpr int payloadSize = 56;
    static constexpr int headerSize = size - payloadSize;
    LMS64CPacket();
    uint8_t cmd;
    uint8_t status;
    uint8_t blockCount;
    uint8_t periphID;
    uint8_t subDevice;
    uint8_t reserved[3];
    uint8_t payload[payloadSize];
};

namespace LMS64CProtocol {

enum eCMD_LMS {
    CMD_GET_INFO = 0x00,
    CMD_LMS6002_RST = 0x10,
    ///Writes data to SI5356 synthesizer via I2C
    CMD_SI5356_WR = 0x11,
    ///Reads data from SI5356 synthesizer via I2C
    CMD_SI5356_RD = 0x12,
    ///Writes data to SI5351 synthesizer via I2C
    CMD_SI5351_WR = 0x13,
    ///Reads data from SI5351 synthesizer via I2C
    CMD_SI5351_RD = 0x14,
    ///PanelBus DVI (HDMI) Transmitter control
    CMD_TFP410_WR = 0x15,
    ///PanelBus DVI (HDMI) Transmitter control
    CMD_TFP410_RD = 0x16,
    ///Sets new LMS7002M chip’s RESET pin level (0, 1, pulse)
    CMD_LMS7002_RST = 0x20,
    ///Writes data to LMS7002M chip via SPI
    CMD_LMS7002_WR = 0x21,
    ///Reads data from LMS7002M chip via SPI
    CMD_LMS7002_RD = 0x22,
    ///Writes data to LMS6002 chip via SPI
    CMD_LMS6002_WR = 0x23,
    ///Reads data from LMS6002 chip via SPI
    CMD_LMS6002_RD = 0x24,

    CMD_LMS_LNA = 0x2A,
    CMD_LMS_PA = 0x2B,

    CMD_PROG_MCU = 0x2C,
    ///Writes data to ADF4002 chip via SPI
    CMD_ADF4002_WR = 0x31,

    CMD_USB_FIFO_RST = 0x40,
    CMD_PE636040_WR = 0x41,
    CMD_PE636040_RD = 0x42,

    CMD_GPIO_DIR_WR = 0x4F,
    CMD_GPIO_DIR_RD = 0x50,
    CMD_GPIO_WR = 0x51,
    CMD_GPIO_RD = 0x52,

    CMD_ALTERA_FPGA_GW_WR = 0x53,
    CMD_ALTERA_FPGA_GW_RD = 0x54,

    CMD_BRDSPI_WR = 0x55, //16 bit spi for stream, dataspark control
    CMD_BRDSPI_RD = 0x56, //16 bit spi for stream, dataspark control
    CMD_BRDSPI8_WR = 0x57, //8 + 8 bit spi for stream, dataspark control
    CMD_BRDSPI8_RD = 0x58, //8 + 8 bit spi for stream, dataspark control

    CMD_BRDCONF_WR = 0x5D, //write config data to board
    CMD_BRDCONF_RD = 0x5E, //read config data from board

    CMD_ANALOG_VAL_WR = 0x61, //write analog value
    CMD_ANALOG_VAL_RD = 0x62, //read analog value

    CMD_MYRIAD_RST = 0x80,
    CMD_MYRIAD_WR = 0x81,
    CMD_MYRIAD_RD = 0x82,
    CMD_MEMORY_WR = 0x8C,
    CMD_MEMORY_RD = 0x8D
};

enum eCMD_STATUS {
    STATUS_UNDEFINED,
    STATUS_COMPLETED_CMD,
    STATUS_UNKNOWN_CMD,
    STATUS_BUSY_CMD,
    STATUS_MANY_BLOCKS_CMD,
    STATUS_ERROR_CMD,
    STATUS_WRONG_ORDER_CMD,
    STATUS_RESOURCE_DENIED_CMD,
    STATUS_COUNT
};

enum class ProgramWriteTarget : uint8_t {
    HPM,
    FX3,
    FPGA = 3u,
};

struct FirmwareInfo {
    int deviceId;
    int expansionBoardId;
    int firmware;
    int hardware;
    int protocol;
    uint64_t boardSerialNumber;
};
OpStatus GetFirmwareInfo(ISerialPort& port, FirmwareInfo& info, uint32_t subDevice = 0);
void FirmwareToDescriptor(const FirmwareInfo& info, SDRDescriptor& descriptor);

OpStatus LMS7002M_SPI(
    ISerialPort& port, uint8_t chipSelect, const uint32_t* mosi, uint32_t* miso, size_t count, uint32_t subDevice = 0);
OpStatus FPGA_SPI(ISerialPort& port, const uint32_t* mosi, uint32_t* miso, size_t count, uint32_t subDevice = 0);
OpStatus ADF4002_SPI(ISerialPort& port, const uint32_t* mosi, size_t count, uint32_t subDevice = 0);

OpStatus I2C_Write(ISerialPort& port, uint32_t address, const uint8_t* data, size_t count);
OpStatus I2C_Read(ISerialPort& port, uint32_t address, uint8_t* data, size_t count);

OpStatus GPIODirRead(ISerialPort& port, uint8_t* buffer, const size_t bufLength);
OpStatus GPIORead(ISerialPort& port, uint8_t* buffer, const size_t bufLength);
OpStatus GPIODirWrite(ISerialPort& port, const uint8_t* buffer, const size_t bufLength);
OpStatus GPIOWrite(ISerialPort& port, const uint8_t* buffer, const size_t bufLength);

OpStatus CustomParameterWrite(ISerialPort& port, const std::vector<CustomParameterIO>& parameters, uint32_t subDevice = 0);
OpStatus CustomParameterRead(ISerialPort& port, std::vector<CustomParameterIO>& parameters, uint32_t subDevice = 0);

typedef std::function<bool(std::size_t bsent, std::size_t btotal, const std::string&)> ProgressCallback;

OpStatus ProgramWrite(ISerialPort& port,
    const char* data,
    size_t length,
    int prog_mode,
    ProgramWriteTarget device,
    ProgressCallback callback = nullptr,
    uint32_t subDevice = 0);

OpStatus DeviceReset(ISerialPort& port, uint32_t socIndex, uint32_t subDevice = 0);
OpStatus MemoryWrite(ISerialPort& port, uint32_t address, const void* data, size_t dataLen, uint32_t subDevice = 0);
OpStatus MemoryRead(ISerialPort& port, uint32_t address, void* data, size_t dataLen, uint32_t subDevice = 0);

} // namespace LMS64CProtocol

/** @brief Class for interacting with the EEPROM management packets */
class LMS64CPacketMemoryWriteView
{
  public:
    LMS64CPacketMemoryWriteView(LMS64CPacket* pkt);
    void SetMode(int mode);
    void SetChunkIndex(int index);
    void SetChunkSize(int size);
    void SetAddress(int size);
    void SetDevice(LMS64CProtocol::ProgramWriteTarget device);
    void SetData(const uint8_t* src, size_t len);

    void GetData(uint8_t* dest, size_t len) const;
    static constexpr size_t GetMaxDataSize();

  private:
    LMS64CPacketMemoryWriteView() = delete;
    LMS64CPacket* packet;
};

} // namespace lime
