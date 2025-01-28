/**
    @file LMS64CProtocol.h
    @author Lime Microsystems
    @brief Implementation of LMS64C protocol.
*/

#pragma once
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "limesuiteng/OpStatus.h"
#include "LMSBoards.h"

namespace lime {

class ISerialPort;
struct SDRDescriptor;
struct CustomParameterIO;

namespace LMS64CProtocol {

/// @brief The available commands in the LMS64C protocol.
enum class Command : uint8_t {
    GET_INFO = 0x00,
    SERIAL_WR = 0x03, // Write device serial number
    SERIAL_RD = 0x04, // Read device serial number
    LMS6002_RST = 0x10,
    ///Writes data to SI5356 synthesizer via I2C
    SI5356_WR = 0x11,
    ///Reads data from SI5356 synthesizer via I2C
    SI5356_RD = 0x12,
    ///Writes data to SI5351 synthesizer via I2C
    SI5351_WR = 0x13,
    ///Reads data from SI5351 synthesizer via I2C
    SI5351_RD = 0x14,
    ///PanelBus DVI (HDMI) Transmitter control
    TFP410_WR = 0x15,
    ///PanelBus DVI (HDMI) Transmitter control
    TFP410_RD = 0x16,
    ///Sets new LMS7002M chip’s RESET pin level (0, 1, pulse)
    LMS7002_RST = 0x20,
    ///Writes data to LMS7002M chip via SPI
    LMS7002_WR = 0x21,
    ///Reads data from LMS7002M chip via SPI
    LMS7002_RD = 0x22,
    ///Writes data to LMS6002 chip via SPI
    LMS6002_WR = 0x23,
    ///Reads data from LMS6002 chip via SPI
    LMS6002_RD = 0x24,

    LMS_LNA = 0x2A,
    LMS_PA = 0x2B,

    PROG_MCU = 0x2C,
    ///Writes data to ADF4002 chip via SPI
    ADF4002_WR = 0x31,

    USB_FIFO_RST = 0x40,
    PE636040_WR = 0x41,
    PE636040_RD = 0x42,

    GPIO_DIR_WR = 0x4F,
    GPIO_DIR_RD = 0x50,
    GPIO_WR = 0x51,
    GPIO_RD = 0x52,

    ALTERA_FPGA_GW_WR = 0x53,
    ALTERA_FPGA_GW_RD = 0x54,

    BRDSPI_WR = 0x55, //16 bit spi for stream, dataspark control
    BRDSPI_RD = 0x56, //16 bit spi for stream, dataspark control
    BRDSPI8_WR = 0x57, //8 + 8 bit spi for stream, dataspark control
    BRDSPI8_RD = 0x58, //8 + 8 bit spi for stream, dataspark control

    BRDCONF_WR = 0x5D, //write config data to board
    BRDCONF_RD = 0x5E, //read config data from board

    ANALOG_VAL_WR = 0x61, //write analog value
    ANALOG_VAL_RD = 0x62, //read analog value

    MYRIAD_RST = 0x80,
    MYRIAD_WR = 0x81,
    MYRIAD_RD = 0x82,
    MEMORY_WR = 0x8C,
    MEMORY_RD = 0x8D
};

/// @brief The command statuses the device can send back.
enum class CommandStatus : uint8_t { Undefined, Completed, Unknown, Busy, TooManyBlocks, Error, WrongOrder, ResourceDenied, Count };

/// @brief The available targets for firmware/gateware
enum class ALTERA_FPGA_GW_WR_targets : uint8_t {
    HPM = 0u,
    FX3 = 1u,
    FPGA = 3u,
};

/// @brief The available targets of memory usage.
enum class MEMORY_WR_targets : uint8_t {
    HPM = 0u,
    FX3 = 1u,
    FPGA_FLASH = 2u,
    EEPROM = 3u,
};

/// @brief Structure denoting the information about the firmware of the device.
struct FirmwareInfo {
    int deviceId; ///< The ID of the device.
    int expansionBoardId; ///< The ID of the expansion board of the device.
    int firmware; ///< The firmware version of the device.
    int hardware; ///< The hardware version of the device.
    int protocol; ///< The protocol version of the device.
    uint64_t boardSerialNumber; ///< The serial number of the device.
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

/// @brief The function to call on programming progress updates.
typedef std::function<bool(std::size_t bsent, std::size_t btotal, const std::string&)> ProgressCallback;

OpStatus FirmwareWrite(ISerialPort& port,
    const char* data,
    size_t length,
    int prog_mode,
    ALTERA_FPGA_GW_WR_targets device,
    ProgressCallback callback = nullptr,
    uint32_t subDevice = 0);

OpStatus DeviceReset(ISerialPort& port, uint32_t socIndex, uint32_t subDevice = 0);
OpStatus MemoryWrite(
    ISerialPort& port, MEMORY_WR_targets target, uint32_t address, const void* data, size_t dataLen, uint32_t subDevice = 0);
OpStatus MemoryRead(
    ISerialPort& port, MEMORY_WR_targets target, uint32_t address, void* data, size_t dataLen, uint32_t subDevice = 0);

OpStatus WriteSerialNumber(ISerialPort& port, const std::vector<uint8_t>& data);
OpStatus ReadSerialNumber(ISerialPort& port, std::vector<uint8_t>& data);

} // namespace LMS64CProtocol

/// @brief The LMS64C protocol packet structure
struct LMS64CPacket {
    static constexpr int size = 64; ///< The size of the packet.
    static constexpr int payloadSize = 56; ///< The size of the payload in the packet.
    static constexpr int headerSize = size - payloadSize; ///< The size of the header.

    LMS64CProtocol::Command cmd{ LMS64CProtocol::Command::GET_INFO }; ///< The command of the packet.
    LMS64CProtocol::CommandStatus status{ LMS64CProtocol::CommandStatus::Undefined }; ///< The completion status of the packet.
    uint8_t blockCount{}; ///< The count of blocks in the payload.
    uint8_t periphID{}; ///< The ID of the peripheral to use.
    uint8_t subDevice{}; ///< The ID of the subdevice to use.
    uint8_t reserved[3]{}; ///< Currently unused
    uint8_t payload[payloadSize]{}; ///< The information of the payload.
};

static_assert(sizeof(LMS64CPacket) == 64);

/** @brief Class for interacting with the EEPROM management packets */
class LMS64CPacketMemoryWriteView
{
  public:
    /// @brief Constructs the LMS64CPacketMemoryWriteView.
    /// @param pkt The packet to use for the operations.
    LMS64CPacketMemoryWriteView(LMS64CPacket* pkt);

    /// @brief Sets the mode of the programming.
    /// @param mode The mode to set.
    void SetMode(int mode);

    /// @brief Sets the index of the chunk to use.
    /// @param index The index of the chunk.
    void SetChunkIndex(int index);

    /// @brief Sets the size of the chunk.
    /// @param size The size of the chunk.
    void SetChunkSize(int size);

    /// @brief Sets the address of the memory to use.
    /// @param addr The address of the memory.
    void SetAddress(int addr);

    /// @brief Sets the memory device to use.
    /// @param device The memory device to use.
    void SetDevice(LMS64CProtocol::MEMORY_WR_targets device);

    /// @brief Sets the data of the packet.
    /// @param src The data to set in the packet.
    /// @param len The length of the data to set.
    void SetData(const uint8_t* src, size_t len);

    /// @brief Gets the data of the packet.
    /// @param dest The buffer to put the packet data in.
    /// @param len The length of the data to read.
    void GetData(uint8_t* dest, size_t len) const;

    /// @brief Gets the maximum amount of data possible to store in a single packet.
    /// @return The maximum amount of data in a packet.
    static constexpr size_t GetMaxDataSize();

  private:
    LMS64CPacketMemoryWriteView() = delete;
    LMS64CPacket* packet;
};

/** @brief Class for manipulating device serial number */
class LMS64CPacketSerialCommandView
{
  public:
    /// @brief The storage to write the information to.
    enum class Storage : uint8_t { Default = 0, Volatile = 1, NonVolatile = 2, OneTimeProgramable = 3 };

    /// @brief Constructs the LMS64CPacketSerialCommandView.
    /// @param pkt The packet to use for the operations.
    LMS64CPacketSerialCommandView(LMS64CPacket* pkt);

    /// @brief Sets the storage type to use with this packet.
    /// @param type The storage type to use.
    void SetStorageType(Storage type);

    /// @brief Sets the key to unlock the serial key writing functionality.
    /// @param key The key to unlock the serial key writer.
    void SetUnlockKey(uint8_t key);

    /// @brief Sets the serial to set the device to.
    /// @param bytes The bytes of serial to set.
    void SetSerial(const std::vector<uint8_t>& bytes);

    /// @brief Gets the current serial of the device.
    /// @param bytes The array to read the serial bytes of the device to.
    void GetSerial(std::vector<uint8_t>& bytes) const;

    /// @brief Gets the maximum possible length of the serial.
    /// @return The maximum length of the serial.
    static constexpr size_t GetMaxSerialLength();

  private:
    LMS64CPacketSerialCommandView() = delete;
    LMS64CPacket* packet;
};

} // namespace lime
