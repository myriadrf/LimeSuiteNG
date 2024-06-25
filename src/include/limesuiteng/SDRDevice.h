#ifndef LIME_SDRDevice_H
#define LIME_SDRDevice_H

#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "limesuiteng/config.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/OpStatus.h"
#include "limesuiteng/SDRConfig.h"

namespace lime {

struct MemoryDevice;
struct Region;
struct SDRDescriptor;
struct StreamConfig;
struct StreamMeta;
struct StreamStats;
struct DataStorage;
struct Region;
struct CustomParameterIO;
class OEMTestReporter;

enum class eMemoryDevice : uint8_t;
enum class eGainTypes : uint8_t;
enum class LogLevel : uint8_t;
enum class TRXDir : bool;

/// @brief Class for holding information about an SDR (Software Defined Radio) device.
/// SDRDevice can have multiple modules (RF chips), that can operate independently.
class LIME_API SDRDevice
{
  public:
    static constexpr uint8_t MAX_RFSOC_COUNT = 16; ///< Maximum amount of Radio-Frequency System-on-Chips

    /// @brief Describes the status of a global positioning system.
    struct GPS_Lock {
        /// @brief Enumerator describing the possible status of a positioning system.
        enum class LockStatus : uint8_t { Undefined, NotAvailable, Has2D, Has3D };

        LockStatus galileo; ///< Status for the Galileo system (European system).
        LockStatus beidou; ///< Status for the BeiDou system (Chinese system).
        LockStatus glonass; ///< Status for the GLONASS system (Russian system).
        LockStatus gps; ///< Status for the GPS system (American system).
    };

    virtual ~SDRDevice();

    /// @brief Configures the device using the given configuration.
    /// @param config The configuration to set up the device with.
    /// @param moduleIndex The device index to configure.
    /// @return The status of the operation.
    virtual OpStatus Configure(const SDRConfig& config, uint8_t moduleIndex) = 0;

    /// @brief Gets the Descriptor of the SDR Device.
    /// @return The Descriptor of the device.
    virtual const SDRDescriptor& GetDescriptor() const = 0;

    /// @brief Initializes the device with initial settings.
    /// @return The success status of the initialization.
    virtual OpStatus Init() = 0;

    /// @brief Resets the device.
    /// @return The status of the operation.
    virtual OpStatus Reset() = 0;

    /// @brief Gets the current status of the GPS locks.
    /// @param status The pointer to which to output the GPS status.
    /// @return The status of the operation.
    virtual OpStatus GetGPSLock(GPS_Lock* status) = 0;

    /// @brief Enables or disables the specified channel.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The channel to configure.
    /// @param enable Whether to enable the channel or not.
    /// @return The status of the operation.
    virtual OpStatus EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable) = 0;

    /// @brief Gets the frequency of a specified clock.
    /// @param clk_id The clock ID to get the frequency of.
    /// @param channel The channel to get the frequency of.
    /// @return The frequency of the specified clock (in Hz).
    virtual double GetClockFreq(uint8_t clk_id, uint8_t channel) = 0;

    /// @brief Sets the frequency of a specified clock.
    /// @param clk_id The clock ID to set the frequency of.
    /// @param freq The new frequency of the specified clock (in Hz).
    /// @param channel The channel to set the frequency of.
    /// @return The status of the operation.
    virtual OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) = 0;

    /// @brief Gets the current frequency of the given channel.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @return The current radio frequency of the channel (in Hz).
    virtual double GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the radio frequency of the given channel.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param frequency The frequency to set the channel to (in Hz).
    /// @return The status of the operation.
    virtual OpStatus SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double frequency) = 0;

    /// @brief Gets the current frequency of the NCO.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @param index The index of the NCO to read from.
    /// @param phaseOffset [out] The phase offset of the NCO (in degrees)
    /// @return The current frequency of the NCO (in Hz)
    virtual double GetNCOFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double& phaseOffset) = 0;

    /// @brief Sets the frequency and the phase angle of the NCO.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param index The index of the NCO to use.
    /// @param frequency The frequency of the NCO to set (in Hz).
    /// @param phaseOffset Phase offset angle (in degrees)
    /// @return The status of the operation.
    virtual OpStatus SetNCOFrequency(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double frequency, double phaseOffset = -1.0) = 0;

    /// @brief Gets the current offset of the NCO compared to the main frequency.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @return The delta between the current device frequency and the current device NCO frequency (in Hz).
    virtual double GetNCOOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Gets the current index of the NCO.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @return The current index of the NCO [0-15]
    virtual int GetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the index of the NCO.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param index The index of the NCO to use.
    /// @param downconv The spectrum control of the CMIX (true = downconvert, false = upconvert)
    /// @return The status of the operation.
    virtual OpStatus SetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, bool downconv) = 0;

    /// @brief Gets the current sample rate of the device.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @param rf_samplerate [out] RF sampling rate.
    /// @return The currend device sample rate (in Hz)
    virtual double GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint32_t* rf_samplerate = nullptr) = 0;

    /// @brief Sets the sample rate of the device.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param sampleRate The target sample rate (in Hz)
    /// @param oversample The RF oversampling ratio.
    /// @return The status of the operation.
    virtual OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) = 0;

    /// @brief Gets the current value of the specified gain.
    /// @param moduleIndex The device index to read from.
    /// @param direction The direction to read from.
    /// @param channel The channel to read from.
    /// @param gain The type of gain to get the data of.
    /// @param value The value of the gain (in dB).
    /// @return The status code of the operation.
    virtual OpStatus GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double& value) = 0;

    /// @brief Sets the gain level of a specified gain.
    /// @param moduleIndex The device index to configure.
    /// @param direction The direction to configure.
    /// @param channel The channel to configure.
    /// @param gain The type of gain to set.
    /// @param value The amount of gain to set (in dB).
    /// @return The status code of the operation.
    virtual OpStatus SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double value) = 0;

    /// @brief Gets the current frequency of the Low Pass Filter.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @return The current frequency of the Low Pass Filter (in Hz).
    virtual double GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the Low Pass Filter to a specified frequency.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param lpf The bandwidth of the Low Pass Filter to set it to (in Hz).
    /// @return The status of the operation.
    virtual OpStatus SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf) = 0;

    /// @brief Gets the currently set antenna of the device.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @return The ID of the currently set antenna.
    virtual uint8_t GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the current antenna of the device.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param path The ID of the antenna to set the device to use.
    /// @return The status of the operation.
    virtual OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) = 0;

    /// @brief Gets the current status of the test signal mode.
    /// @param moduleIndex The device index to read from.
    /// @param direction The direction to read from.
    /// @param channel The channel to read from.
    /// @return The current status of the test signal mode.
    virtual ChannelConfig::Direction::TestSignal GetTestSignal(uint8_t moduleIndex, TRXDir direction, uint8_t channel) = 0;

    /// @brief Sets the test signal mode.
    /// @param moduleIndex The device index to configure.
    /// @param direction The direction to configure.
    /// @param channel The channel to configure.
    /// @param signalConfiguration The configuration of the test mode to set.
    /// @param dc_i The I value of the test mode to send (0 for defaults)
    /// @param dc_q The Q value of the test mode to send (0 for defaults)
    /// @return The status of the operation.
    virtual OpStatus SetTestSignal(uint8_t moduleIndex,
        TRXDir direction,
        uint8_t channel,
        ChannelConfig::Direction::TestSignal signalConfiguration,
        int16_t dc_i = 0,
        int16_t dc_q = 0) = 0;

    /// @brief Gets if the DC corrector bypass is enabled or not.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @return Whether the DC corrector bypassis enabled or not (false = bypass the corrector, true = use the corrector)
    virtual bool GetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Enables or disables the DC corrector bypass.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param isAutomatic Whether to use the DC corrector bypass or not (false = bypass the corrector, true = use the corrector)
    /// @return The status of the operation.
    virtual OpStatus SetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool isAutomatic) = 0;

    /// @brief Gets the DC I and Q corrector values.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @return The current DC I and Q corrector values.
    virtual complex64f_t GetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the DC I and Q corrector values.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param offset The offsets of the I and Q channels.
    /// @return The status of the operation.
    virtual OpStatus SetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& offset) = 0;

    /// @brief Gets the current I and Q gain corrector values.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @param channel The channel to read from.
    /// @return The current I and Q gain corrector values.
    virtual complex64f_t GetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the I and Q gain corrector values.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param balance The I and Q corrector values to set.
    /// @return The status of the operation.
    virtual OpStatus SetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& balance) = 0;

    /// @brief Gets whether the VCO comparators of the clock generator are locked or not.
    /// @param moduleIndex The device index to read from.
    /// @return A value indicating whether the VCO comparators of the clock generator are locked or not.
    virtual bool GetCGENLocked(uint8_t moduleIndex) = 0;

    /// @brief Gets the temperature of the device.
    /// @param moduleIndex The device index to get the temperature of.
    /// @return The temperature of the device (in degrees Celsius)
    virtual double GetTemperature(uint8_t moduleIndex) = 0;

    /// @brief Gets whether the VCO comparators of the LO synthesizer are locked or not.
    /// @param moduleIndex The device index to read from.
    /// @param trx The direction to read from.
    /// @return A value indicating whether the VCO comparators of the clock generator are locked or not.
    virtual bool GetSXLocked(uint8_t moduleIndex, TRXDir trx) = 0;

    /// @brief Reads the value of the given register.
    /// @param moduleIndex The device index to read from.
    /// @param address The memory address to read from.
    /// @param useFPGA Whether to read memory from the FPGA or not.
    /// @return The value read from the register.
    virtual unsigned int ReadRegister(uint8_t moduleIndex, unsigned int address, bool useFPGA = false) = 0;

    /// @brief Writes the given register value to the given address.
    /// @param moduleIndex The device index to configure.
    /// @param address The address of the memory to write to.
    /// @param value The value to write to the device's memory.
    /// @param useFPGA Whether to write to the FPGA or not (default false)
    /// @return The status of the operation.
    virtual OpStatus WriteRegister(uint8_t moduleIndex, unsigned int address, unsigned int value, bool useFPGA = false) = 0;

    /// @brief Loads the configuration of a device from a given file.
    /// @param moduleIndex The device index to write the configuration into.
    /// @param filename The file to read the data from.
    /// @return The status of the operation.
    virtual OpStatus LoadConfig(uint8_t moduleIndex, const std::string& filename) = 0;

    /// @brief Saves the current configuration of the device into a given file.
    /// @param moduleIndex The device index to save the data from.
    /// @param filename The file to save the information to.
    /// @return The status of the operation.
    virtual OpStatus SaveConfig(uint8_t moduleIndex, const std::string& filename) = 0;

    /// @brief Gets the given parameter from the device.
    /// @param moduleIndex The device index to configure.
    /// @param channel The channel to configure.
    /// @param parameterKey The key of the paremeter to read from.
    /// @return The value read from the parameter.
    virtual uint16_t GetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey) = 0;

    /// @brief Sets the given parameter in the device.
    /// @param moduleIndex The device index to configure.
    /// @param channel The channel to configure.
    /// @param parameterKey The key of the paremeter to write to.
    /// @param value The value to write to the address.
    /// @return The status of the operation.
    virtual OpStatus SetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey, uint16_t value) = 0;

    /// @brief Gets the given parameter from the device.
    /// @param moduleIndex The device index to get the data from.
    /// @param channel The channel to get the data from.
    /// @param address The memory address of the device to read.
    /// @param msb The index of the most significant bit of the address to read. (16-bit register)
    /// @param lsb The index of the least significant bit of the address to read. (16-bit register)
    /// @return The value read from the parameter.
    virtual uint16_t GetParameter(uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb) = 0;

    /// @brief Sets the given parameter in the device.
    /// @param moduleIndex The device index to configure.
    /// @param channel The channel to configure.
    /// @param address The memory address in the device to change.
    /// @param msb The index of the most significant bit of the address to modify. (16-bit register)
    /// @param lsb The index of the least significant bit of the address to modify. (16-bit register)
    /// @param value The value to write to the address.
    /// @return The status of the operation.
    virtual OpStatus SetParameter(
        uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value) = 0;

    /// @brief Calibrates the given channel for a given bandwidth.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The channel to configure.
    /// @param bandwidth The bandwidth of the channel to calibrate for (in Hz).
    /// @return The status of the operation.
    virtual OpStatus Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double bandwidth) = 0;

    /// @brief Configures the GFIR with the settings.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The channel to configure.
    /// @param settings The settings of the GFIR to set.
    /// @return The status of the operation.
    virtual OpStatus ConfigureGFIR(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, ChannelConfig::Direction::GFIRFilter settings) = 0;

    /// @brief Gets the current coefficients of a GFIR.
    /// @param moduleIndex The device index to get the coefficients from.
    /// @param trx The direction of the channel to get the data from.
    /// @param channel The channel to get the data from.
    /// @param gfirID The ID of the GFIR to get the coefficients from.
    /// @return The current coefficients (normalized in the range [-1; 1]) of the GFIR.
    virtual std::vector<double> GetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID) = 0;

    /// @brief Sets the coefficients of a given GFIR
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The channel to set the filter of.
    /// @param gfirID The ID of the GFIR to set.
    /// @param coefficients The coefficients (normalized in the range [-1; 1]) to set the GFIR to.
    /// @return The status of the operation.
    virtual OpStatus SetGFIRCoefficients(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, std::vector<double> coefficients) = 0;

    /// @brief Sets the GFIR to use.
    /// @param moduleIndex The device index to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The channel to set the filter of.
    /// @param gfirID The ID of the GFIR to set.
    /// @param enabled Whether the specifed GFIR should be enabled or disabled.
    /// @return The status of the operation.
    virtual OpStatus SetGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, bool enabled) = 0;

    /// @brief Synchronizes the cached changed register values on the host with the real values on the device.
    /// @param toChip The direction in which to synchronize (true = uploads to the device).
    /// @return The status of the operation.
    virtual OpStatus Synchronize(bool toChip) = 0;

    /// @brief Enable or disable register value caching on the host side.
    /// @param enable Whether to enable or disable the register value caching (true = enabled).
    virtual void EnableCache(bool enable) = 0;

    /// @brief Gets the hardware timestamp with the applied offset.
    /// @param moduleIndex The device index to configure.
    /// @return The current timestamp of the hardware.
    virtual uint64_t GetHardwareTimestamp(uint8_t moduleIndex) = 0;

    /// @brief Sets the hardware timestamp to the provided one by applying a constant offset.
    /// @param moduleIndex The device index to configure.
    /// @param now What the definition of the current time should be.
    /// @return The status of the operation.
    virtual OpStatus SetHardwareTimestamp(uint8_t moduleIndex, const uint64_t now) = 0;

    /// @brief Sets up all the streams on a device.
    /// @param config The configuration to use for setting the streams up.
    /// @param moduleIndex The index of the device to set up.
    /// @return The status code of the operation.
    virtual OpStatus StreamSetup(const StreamConfig& config, uint8_t moduleIndex) = 0;

    /// @brief Starts all the set up streams on the device.
    /// @param moduleIndex The index of the device to start the streams on.
    virtual void StreamStart(uint8_t moduleIndex) = 0;

    /// @brief Starts all the set up streams on the devices.
    /// @param moduleIndexes The indices of the devices to start the streams on.
    virtual void StreamStart(const std::vector<uint8_t> moduleIndexes);

    /// @brief Stops all the set up streams on the device.
    /// @param moduleIndex The index of the device to stop the streams on.
    virtual void StreamStop(uint8_t moduleIndex) = 0;

    /// @brief Stops all the set up streams on the devices.
    /// @param moduleIndexes The indices of the devices to stop the streams on.
    virtual void StreamStop(const std::vector<uint8_t> moduleIndexes);

    /// @brief Deallocate stream resources.
    /// @param moduleIndex The index of the device to stop the streams on.
    virtual void StreamDestroy(uint8_t moduleIndex) = 0;

    /// @brief Reveives samples from all the active streams in the device.
    /// @param moduleIndex The index of the device to receive the samples from.
    /// @param samples The buffer to put the received samples in.
    /// @param count The amount of samples to reveive.
    /// @param meta The metadata of the packets of the stream.
    /// @return The amount of samples received.
    virtual uint32_t StreamRx(uint8_t moduleIndex, lime::complex32f_t* const* samples, uint32_t count, StreamMeta* meta) = 0;
    /// @copydoc SDRDevice::StreamRx()
    virtual uint32_t StreamRx(uint8_t moduleIndex, lime::complex16_t* const* samples, uint32_t count, StreamMeta* meta) = 0;
    /// @copydoc SDRDevice::StreamRx()
    virtual uint32_t StreamRx(uint8_t moduleIndex, lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta) = 0;

    /// @brief Transmits packets from all the active streams in the device.
    /// @param moduleIndex The index of the device to transmit the samples with.
    /// @param samples The buffer of the samples to transmit.
    /// @param count The amount of samples to transmit.
    /// @param meta The metadata of the packets of the stream.
    /// @return The amount of samples transmitted.
    virtual uint32_t StreamTx(
        uint8_t moduleIndex, const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta) = 0;
    /// @copydoc SDRDevice::StreamTx()
    virtual uint32_t StreamTx(
        uint8_t moduleIndex, const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta) = 0;
    /// @copydoc SDRDevice::StreamRx()
    virtual uint32_t StreamTx(
        uint8_t moduleIndex, const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta) = 0;

    /// @brief Retrieves the current stream statistics.
    /// @param moduleIndex The index of the device to retrieve the status from.
    /// @param rx The pointer (or nullptr if not needed) to store the receive statistics to.
    /// @param tx The pointer (or nullptr if not needed) to store the transmit statistics to.
    virtual void StreamStatus(uint8_t moduleIndex, StreamStats* rx, StreamStats* tx) = 0;

    /// @brief Uploads waveform to on board memory for later use.
    /// @param config The configuration of the stream.
    /// @param moduleIndex The index of the device to upload the waveform to.
    /// @param samples The samples to upload to the device.
    /// @param count The amount of samples to upload to the device.
    /// @return Operation status.
    virtual OpStatus UploadTxWaveform(const StreamConfig& config, uint8_t moduleIndex, const void** samples, uint32_t count);

    /// @copydoc ISPI::SPI()
    /// @param spiBusAddress The SPI address of the device to use.
    virtual OpStatus SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count);

    /// @copydoc II2C::I2CWrite()
    virtual OpStatus I2CWrite(int address, const uint8_t* data, uint32_t length);

    /// @copydoc II2C::I2CRead()
    virtual OpStatus I2CRead(int address, uint8_t* dest, uint32_t length);

    /***********************************************************************
     * GPIO API
     **********************************************************************/

    /// @copydoc IComms::GPIOWrite()
    virtual OpStatus GPIOWrite(const uint8_t* buffer, const size_t bufLength);

    /// @copydoc IComms::GPIORead()
    virtual OpStatus GPIORead(uint8_t* buffer, const size_t bufLength);

    /// @copydoc IComms::GPIODirWrite()
    virtual OpStatus GPIODirWrite(const uint8_t* buffer, const size_t bufLength);

    /// @copydoc IComms::GPIODirRead()
    virtual OpStatus GPIODirRead(uint8_t* buffer, const size_t bufLength);

    /***********************************************************************
     * Aribtrary settings API
     **********************************************************************/

    /// @copydoc IComms::CustomParameterWrite()
    virtual OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters);

    /// @copydoc IComms::CustomParameterRead()
    virtual OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters);

    /// @brief The definition of a function to call when a log message is generated.
    typedef std::function<void(LogLevel, const std::string&)> LogCallbackType;

    /// @brief Sets callback function which gets called each a log message is received
    /// @param callback The callback to use from this point onwards.
    virtual void SetMessageLogCallback(LogCallbackType callback);

    /// @brief Gets the pointer to an internal chip of the device.
    /// @param index The index of the device to retreive.
    /// @return The pointer to the internal device.
    virtual void* GetInternalChip(uint32_t index) = 0;

    /// @brief The definition of a function to call whenever memory is being uploaded.
    typedef std::function<bool(std::size_t bsent, std::size_t btotal, const std::string&)> UploadMemoryCallback;

    /// @brief Uploads the given memory into the specified device.
    /// @param device The memory device to upload the memory to.
    /// @param moduleIndex The index of the main device to upload the memory to.
    /// @param data The data to upload to the device.
    /// @param length The length of the memory to upload.
    /// @param callback The callback to call for status updates.
    /// @return The success status of the operation.
    virtual OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback);

    /// @brief Writes given data into a given memory address in EEPROM memory.
    /// @param storage The storage device to write to.
    /// @param region Information of the region in which to write the data to.
    /// @param data The data to write into the specified memory.
    /// @return The operation success state.
    virtual OpStatus MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data);

    /// @brief Reads data from a given memory address in EEPROM memory.
    /// @param storage The storage device to read from.
    /// @param region Information of the region from which to read the memory.
    /// @param data The storage buffer for the data being read.
    /// @return The operation success state.
    virtual OpStatus MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data);

    /// @brief Runs various device specific tests to check functionality
    /// @param reporter Object for handling test results callbacks
    /// @return The operation success state.
    virtual OpStatus OEMTest(OEMTestReporter* reporter);

    /// @brief Writes one time programmable serial number of the device
    /// @param serialNumber Device's serial number
    /// @return The operation success state.
    virtual OpStatus WriteSerialNumber(uint64_t serialNumber);
};

} // namespace lime
#endif
