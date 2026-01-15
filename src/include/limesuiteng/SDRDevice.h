#ifndef LIME_SDRDevice_H
#define LIME_SDRDevice_H

/**
@file limesuiteng/SDRDevice.h
@author Lime Microsystems
@brief Defines SDR device configuration and control interface class
*/

#include <cstring>
#include <chrono>
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
class RFStream;
class GPIO_Interface;

enum class eMemoryDevice : uint8_t;
enum class eGainTypes : uint8_t;
enum class LogLevel : uint8_t;
enum class TRXDir : bool;

/// @brief Class for holding information about an SDR (Software Defined Radio) device.
/// SDRDevice can have multiple modules (RF chips), that can operate independently.
class LIME_API SDRDevice
{
  public:
    static constexpr std::chrono::microseconds DEFAULT_TIMEOUT{ std::chrono::microseconds(1000000) };
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

    /// @brief Configures the device using the given configuration structure.
    /// Configures all device channels with default values or custom values as 
    /// specified in @ref lime::SDRConfig "SDRConfig" structure.
    /// @param config The configuration to set up the device with.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus Configure(const SDRConfig& config, uint8_t moduleIndex) = 0;

    /// @brief Gets the Descriptor of the SDR Device.
    /// @return The Descriptor of the device.
    virtual const SDRDescriptor& GetDescriptor() const = 0;

    /** 
     * @brief Initializes the device with initial settings.
     * This function completely resets SDR device and RF chip configuration to default values.
     * Recommended to use this function if the device is new and is registered for the first time.
     * Initialization is performed by default, if @ref lime::SDRDevice::Configure "Configure"
     * function is called with default configuration on.  
     * @return The success status of the initialization.
     */ 
    virtual OpStatus Init() = 0;

    /**
     * @brief Resets the device.
     * Sends a reset signal to the device RF chip reset pin.
     * @return The @ref lime::OpStatus "status" of the operation.
     */
    virtual OpStatus Reset() = 0;

    /// @brief Gets the current status of the GPS locks.
    /// @param status The pointer to which to output the GPS status.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus GetGPSLock(GPS_Lock* status) = 0;

    /// @brief Enables or disables the specified channel.
    /// This powers on or off device hardware of individual
    /// channel direction. Toggling off a channel direction
    /// reduces the consumption of device current and power.
    /// To completely disable channel, both channel directions Tx
    /// and Rx must be disabled.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param enable Whether to enable the channel or not.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable) = 0;

    /// @brief Gets the frequency of a specified clock.
    /// Can Return device reference clock, TX/RX local oscillator clock, clock generator clock,
    /// RX/TX transceiver signal processor clock. To get local oscillator clock for a specific 
    /// direction of a channel, consider using GetFrequency() function.
    /// @param clk_id The @ref lime::LMS7002M::ClockID "clock ID" to get the frequency of.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to get the frequency of.
    /// @return The frequency of the specified clock (in Hz).
    virtual double GetClockFreq(uint8_t clk_id, uint8_t channel) = 0;

    /// @brief Sets the frequency of a specified clock.
    /// Can set device reference clock, TX/RX local oscillator clock, clock generator 
    /// clock. To set local oscillator clock for a specific direction of a channel, consider 
    /// using SetFrequency() function.
    /// @note TX/RX transceiver signal processor clock values are read-only.
    /// @param clk_id The clock ID to set the frequency of. Supported @ref lime::LMS7002M::ClockID "clock IDs".
    /// @param freq The new frequency of the specified clock (in Hz).
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to set the frequency of.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) = 0;

    ///@brief Gets the current frequency of local oscillator for the selected direction of a channel.
    /// If the device is configured for TDD mode, this will always return current TX local oscillator frequency for all channels. 
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @return The current local oscillator frequency for the selected channel and direction (in Hz).
    virtual double GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets new frequency of local oscillator for the selected direction of a channel.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param frequency The frequency to set the channel to (in Hz).
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double frequency) = 0;

    /// @brief Gets the current frequency of the NCO.
    /// Gets NCO frequency entry from NCO memory table using index.
    /// @note To get the frequency that is currently being used by the NCO, 
    /// first obtain the index using GetNCOIndex() function.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @param index The index of NCO frequency entry to read from NCO memory table [0-15].
    /// @param phaseOffset [out] The phase offset of the NCO (in degrees)
    /// @return The current frequency of the NCO (in Hz)
    virtual double GetNCOFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double& phaseOffset) = 0;

    /// @brief Sets the frequency and the phase angle of the NCO.
    /// Sets NCO frequency/phase entry of NCO memory table using index.
    /// @note To feed the new frequency into NCO, set the new frequency 
    /// entry index as active using SetNCOIndex().
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param index The index of NCO frequency entry to overwrite in NCO memory table [0-15].
    /// @param frequency The frequency of the NCO to set (in Hz).
    /// @param phaseOffset Phase offset angle (in degrees)
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetNCOFrequency(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double frequency, double phaseOffset = -1.0) = 0;

    /// @brief Gets the current offset of the NCO compared to the main frequency.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @return The delta between the current device frequency and the current device NCO frequency (in Hz).
    virtual double GetNCOOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Gets the current index of the NCO.
    /// Returns the index of the NCO memory table entry that contains the frequency currently being used by NCO.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @return The current index of the active NCO [0-15].
    virtual int GetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the index of the NCO.
    /// Selects NCO frequency entry from NCO memory table to be used by NCO.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param index The index of NCO memory table entry to use for NCO [0-15].
    /// @param downconv The spectrum control of the CMIX (true = downconvert, false = upconvert)
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, bool downconv) = 0;

    /// @brief Gets the current sample rate of the device.
    /// Returns device sample rate and optionally, the actual RF sample rate used in AFE ADCs and DACs.
    /// The actual RF sample rate in AFE ADCs and DACs is much higher than the true sample rate due to oversampling ratio.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @param rf_samplerate [out] RF sampling rate. This parameter is optional.
    /// @return The current device sample rate (in Hz)
    virtual double GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint32_t* rf_samplerate = nullptr) = 0;

    /// @brief Sets the sample rate of the device.
    /// Calculates and sets the RF sample rate of ADCs and DACs based on specified sample rate and oversampling ratio.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param sampleRate The target sample rate (in Hz)
    /// @param oversample The RF oversampling ratio. Pass 0 to auto select oversampling ratio based on specified sample rate.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) = 0;

    /// @brief Gets the current gain value of amplifier.
    /// Returns the current gain setting value for the specified amplifier type within device RF chip.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param direction The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @param gain Internal RF chip amplifier type. Check out all possible @ref lime::eGainTypes "amplifier types".
    /// @param value The current gain value of the specified amplifier type (in dB).
    /// @return The @ref lime::OpStatus "status" code of the operation.
    virtual OpStatus GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double& value) = 0;

    /// @brief Sets the gain level for amplifier.
    /// Sets a new gain setting value for specified amplifier type within device RF chip.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param direction The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param gain Internal RF chip amplifier type. Check out all possible @ref lime::eGainTypes "amplifier types".
    /// @param value The new specified amplifier gain value (in dB).
    /// @return The @ref lime::OpStatus "status" code of the operation.
    virtual OpStatus SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double value) = 0;

    /// @brief Gets the current frequency of the Low Pass Filter.
    /// Returns currently active Low Pass filter bandwidth frequency of device RF chip RFE (RXLPF or TXLPF).
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @return The current frequency of the Low Pass Filter (in Hz).
    virtual double GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the Low Pass Filter to a specified frequency.
    /// Sets the new Low Pass filter bandwidth frequency value for device RF chip RFE (RXLPF or TXLPF).
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param lpf The bandwidth of the Low Pass Filter to set it to (in Hz).
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf) = 0;

    /// @brief Gets the currently set antenna of the device.
    /// Returns antenna path ID that identifies currently active antenna type for selected direction of a channel.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @return The ID of the currently set antenna.
    virtual uint8_t GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the current antenna of the device.
    /// Activates a specific antenna type for a selected direction of a channel using antenna path ID.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param path The ID of the antenna to set the device to use.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) = 0;

    /// @brief Gets the current status of the test signal mode.
    /// Provides information about the test signal that is being generated within device RF chip TSP.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param direction The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @return A structure which describes the current status of the test signal mode.
    /// @see To find out more about the test signal parameters, check out 
    /// @ref lime::ChannelConfig::Direction::TestSignal "test signal structure members".
    virtual ChannelConfig::Direction::TestSignal GetTestSignal(uint8_t moduleIndex, TRXDir direction, uint8_t channel) = 0;

    /// @brief Sets the test signal mode.
    /// Updates test signal parameters within device RF chip TSP for selected channel direction.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param direction The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param signalConfiguration The configuration of the test mode to set.
    /// @param dc_i The I value of the test mode to send (0 for defaults)
    /// @param dc_q The Q value of the test mode to send (0 for defaults)
    /// @return The @ref lime::OpStatus "status" of the operation.
    /// @see More about test signal @ref lime::ChannelConfig::Direction::TestSignal "parameters".
    virtual OpStatus SetTestSignal(uint8_t moduleIndex,
        TRXDir direction,
        uint8_t channel,
        ChannelConfig::Direction::TestSignal signalConfiguration,
        int16_t dc_i = 0,
        int16_t dc_q = 0) = 0;

    /// @brief Gets the current DC corrector status from device RF chip TSP.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @return Whether the DC corrector bypass is enabled or not (false = bypass the corrector, true = use the corrector)
    virtual bool GetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Enables or disables the DC corrector bypass of device RF chip TSP.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction to configure.
    /// @param channel The channel to configure.
    /// @param isAutomatic Whether to use the DC corrector bypass or not (false = bypass the corrector, true = use the corrector)
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool isAutomatic) = 0;

    /// @brief Gets the current DC I and Q corrector values from device RF chip TSP.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @return The current DC I and Q corrector values.
    virtual complex64f_t GetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the new DC I and Q corrector values for device RF chip TSP.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param offset The offsets of the I and Q channels.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& offset) = 0;

    /// @brief Gets the current I and Q gain corrector values from device RF chip TSP.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to read from.
    /// @return The current I and Q gain corrector values.
    virtual complex64f_t GetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel) = 0;

    /// @brief Sets the new I and Q gain corrector values for device RF chip TSP.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param balance The I and Q corrector values to set.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& balance) = 0;

    /// @brief Gets whether the VCO comparators of the main clock generator are locked or not.
    /// Returns the value (lock status: true or false) which describes if the main
    /// clock source within device RF chip is generating clock that matches the phase and frequency of reference clock.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @return A value indicating whether the VCO comparators of the clock generator are locked or not.
    virtual bool GetCGENLocked(uint8_t moduleIndex) = 0;

    /// @brief Gets the temperature of the device.
    /// Provides information about the device RF chip internal temperature.
    /// @param moduleIndex The @ref Device_index "device index" to get the temperature of.
    /// @return The temperature of the device (in degrees Celsius)
    virtual double GetTemperature(uint8_t moduleIndex) = 0;

    /// @brief Gets whether the VCO comparators of the LO synthesizer are locked or not.
    /// Returns the value (lock status: true or false) which describes if the synthesizer LO clock
    /// within device RF chip RFE is generating clock that matches the phase and frequency of reference clock.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param trx The direction to read from.
    /// @return A value indicating whether the VCO comparators of the clock generator are locked or not.
    virtual bool GetSXLocked(uint8_t moduleIndex, TRXDir trx) = 0;

    /// @brief Reads the value of the given register.
    /// Allows to read entire register value of RF or FPGA chip.
    /// FPGA is addressed using 32 bit addresses.
    /// RF chip is addressed using 16 bit addresses. Pad the remaining bits with 0.
    /// @param moduleIndex The @ref Device_index "device index" to read from.
    /// @param address The memory address to read from.
    /// @param useFPGA Whether to read memory from the FPGA or not.
    /// @return The value read from the register.
    virtual unsigned int ReadRegister(uint8_t moduleIndex, unsigned int address, bool useFPGA = false) = 0;

    /// @brief Writes the given register value to the given address.
    /// Allows to write entire register value to RF or FPGA chip.
    /// FPGA is addressed using 32 bit addresses.
    /// RF chip is addressed using 16 bit addresses. Pad the remaining bits with 0.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param address The address of the memory to write to.
    /// @param value The value to write to the device's memory.
    /// @param useFPGA Whether to write to the FPGA or not (default false)
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus WriteRegister(uint8_t moduleIndex, unsigned int address, unsigned int value, bool useFPGA = false) = 0;

    /// @brief Loads the configuration of a device from a given file.
    /// Loads device RF chip configuration from a .ini file which can be generated
    /// using LimeSuiteNG GUI configuration software. Supports legacy 
    /// configuration file format.
    /// @param moduleIndex The @ref Device_index "device index" to write the configuration into.
    /// @param filename The file to read the data from.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus LoadConfig(uint8_t moduleIndex, const std::string& filename) = 0;

    /// @brief Saves the current configuration of the device into a given file.
    /// Saves device RF chip configuration to a .ini file, that can be reviewed 
    /// using LimeSuiteNG GUI configuration software or re-used to configure other devices.
    /// @param moduleIndex The @ref Device_index "device index" to save the data from.
    /// @param filename The file to save the information to.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SaveConfig(uint8_t moduleIndex, const std::string& filename) = 0;

    /// @brief Gets the given parameter from the device.
    /// Returns the current value of device parameter from device register space using only parameter name. 
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param parameterKey The key of the parameter to read from.
    /// @return The value read from the parameter.
    virtual uint16_t GetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey) = 0;

    /// @brief Sets the given parameter in the device.
    /// Sets a new value for specified parameter within device register space using only parameter name.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param parameterKey The key of the parameter to write to.
    /// @param value The value to write to the address.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey, uint16_t value) = 0;

    /// @brief Reads specified parameter bit-field from register and returns its value.
    /// @param moduleIndex The @ref Device_index "device index" to get the data from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to get the data from.
    /// @param address The memory address of the device to read.
    /// @param msb The index of the most significant bit of the address to read. (16-bit register)
    /// @param lsb The index of the least significant bit of the address to read. (16-bit register)
    /// @return The value read from the parameter.
    virtual uint16_t GetParameter(uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb) = 0;

    /// @brief Writes new value to specified register parameter bit-field.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param address The memory address in the device to change.
    /// @param msb The index of the most significant bit of the address to modify. (16-bit register)
    /// @param lsb The index of the least significant bit of the address to modify. (16-bit register)
    /// @param value The value to write to the address.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetParameter(
        uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value) = 0;

    /// @brief Calibrates the given channel for a given bandwidth.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param bandwidth The bandwidth of the channel to calibrate for (in Hz).
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double bandwidth) = 0;

    /// @brief Auto toggle and configure all GFIR filters for specified channel direction.
    /// Allows to disable or enable all 3 available GFIR filters for a specified channel direction at once.
    /// If GFIR filters are disabled, the last GFIR filter configurations are saved and the filters are bypassed in processing stage.
    /// If GFIR filters are enabled, new GFIR filter coefficients will be obtained and loaded into device registers. New GFIR filter
    /// values are automatically calculated by using the provided bandwidth setting. Bandwidth setting value is provided through the 
    /// function parameter "settings".
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to configure.
    /// @param settings The settings of the GFIR filter. More about GFIR filter @ref ChannelConfig::Direction::GFIRFilter "settings". 
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus ConfigureGFIR(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, ChannelConfig::Direction::GFIRFilter settings) = 0;

    /// @brief Gets the current coefficients of a single GFIR filter for channel direction.
    /// Each channel direction has 3 GFIR filters. The GFIR filters with IDs 0 and 1 store up to 40 coefficients.
    /// The GFIR filter with ID 2 can store up to 120 coefficients.  
    /// @param moduleIndex The @ref Device_index "device index" to get the coefficients from.
    /// @param trx The direction of the channel to get the data from.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to get the data from.
    /// @param gfirID The ID of the GFIR. Supported ID range [0; 2].
    /// @return The current coefficients (normalized in the range [-1; 1]) of the GFIR.
    virtual std::vector<double> GetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID) = 0;

    /// @brief Sets the coefficients of a single GFIR filter for channel direction.
    /// Each channel direction has 3 GFIR filters. The GFIR filters with IDs 0 and 1 store up to 40 coefficients.
    /// The GFIR filter with ID 2 can store up to 120 coefficients.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to set the filter of.
    /// @param gfirID The ID of the GFIR. Supported ID range [0; 2].
    /// @param coefficients The coefficients (normalized in the range [-1; 1]) to set the GFIR to.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetGFIRCoefficients(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, std::vector<double> coefficients) = 0;

    /// @brief Toggles the use of a single GFIR filter in processing stage for specified channel direction.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param trx The direction of the channel to configure.
    /// @param channel The @ref lime::LMS7002M::Channel "channel" to set the filter of.
    /// @param gfirID The ID of the GFIR to set. Supported ID range [0; 2].
    /// @param enabled Whether the specifed GFIR should be enabled or disabled.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, bool enabled) = 0;

    /// @brief Synchronizes the cached changed register values on the host with the real values on the device.
    /// @param toChip The direction in which to synchronize (true = uploads to the device).
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus Synchronize(bool toChip) = 0;

    /// @brief Enable or disable register value caching on the host side.
    /// @param enable Whether to enable or disable the register value caching (true = enabled).
    virtual void EnableCache(bool enable) = 0;

    /// @brief Gets the hardware timestamp with the applied offset.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @return The current timestamp of the hardware.
    virtual uint64_t GetHardwareTimestamp(uint8_t moduleIndex) = 0;

    /// @brief Sets the hardware timestamp to the provided one by applying a constant offset.
    /// @param moduleIndex The @ref Device_index "device index" to configure.
    /// @param now What the definition of the current time should be.
    /// @return The @ref lime::OpStatus "status" of the operation.
    virtual OpStatus SetHardwareTimestamp(uint8_t moduleIndex, const uint64_t now) = 0;
    
    /// @cond DEPRECATED
    /// @brief Sets up all the streams on a device.
    /// @param config The configuration to use for setting the streams up.
    /// @param moduleIndex The index of the device to set up.
    /// @return The @ref lime::OpStatus "status" code of the operation.
    [[deprecated]] virtual OpStatus StreamSetup(const StreamConfig& config, uint8_t moduleIndex) = 0;

    /// @brief Starts all the set up streams on the device.
    /// @param moduleIndex The index of the device to start the streams on.
    [[deprecated]] virtual void StreamStart(uint8_t moduleIndex) = 0;

    /// @brief Starts all the set up streams on the devices.
    /// @param moduleIndexes The indices of the devices to start the streams on.
    [[deprecated]] virtual void StreamStart(const std::vector<uint8_t>& moduleIndexes);

    /// @brief Stops all the set up streams on the device.
    /// @param moduleIndex The index of the device to stop the streams on.
    [[deprecated]] virtual void StreamStop(uint8_t moduleIndex) = 0;

    /// @brief Stops all the set up streams on the devices.
    /// @param moduleIndexes The indices of the devices to stop the streams on.
    [[deprecated]] virtual void StreamStop(const std::vector<uint8_t>& moduleIndexes);

    /// @brief Deallocate stream resources.
    /// @param moduleIndex The index of the device to stop the streams on.
    [[deprecated]] virtual void StreamDestroy(uint8_t moduleIndex) = 0;

    /// @brief Receives samples from all the active streams in the device.
    /// @param moduleIndex The index of the device to receive the samples from.
    /// @param samples The buffer to put the received samples in.
    /// @param count The amount of samples to receive.
    /// @param meta The metadata of the packets of the stream.
    /// @param timeout Number of microseconds for the operation to complete, function can return early if timeout is shorter than time required to gather requested amount of samples
    /// @return The amount of samples received.
    virtual uint32_t StreamRx(uint8_t moduleIndex,
        lime::complex32f_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;
    /// @copydoc SDRDevice::StreamRx()
    virtual uint32_t StreamRx(uint8_t moduleIndex,
        lime::complex16_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;
    /// @copydoc SDRDevice::StreamRx()
    virtual uint32_t StreamRx(uint8_t moduleIndex,
        lime::complex12_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;

    /// @brief Transmits packets from all the active streams in the device.
    /// @param moduleIndex The index of the device to transmit the samples with.
    /// @param samples The buffer of the samples to transmit.
    /// @param count The amount of samples to transmit.
    /// @param meta The metadata of the packets of the stream.
    /// @param timeout Number of microseconds for the operation to complete, function can return early if timeout is shorter than time required to gather requested amount of samples
    /// @return The amount of samples transmitted.
    [[deprecated]] virtual uint32_t StreamTx(uint8_t moduleIndex,
        const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;
    /// @copydoc SDRDevice::StreamTx()
    [[deprecated]] virtual uint32_t StreamTx(uint8_t moduleIndex,
        const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;
    /// @copydoc SDRDevice::StreamRx()
    [[deprecated]] virtual uint32_t StreamTx(uint8_t moduleIndex,
        const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;

    /// @brief Retrieves the current stream statistics.
    /// @param moduleIndex The index of the device to retrieve the status from.
    /// @param rx The pointer (or nullptr if not needed) to store the receive statistics to.
    /// @param tx The pointer (or nullptr if not needed) to store the transmit statistics to.
    [[deprecated]] virtual void StreamStatus(uint8_t moduleIndex, StreamStats* rx, StreamStats* tx) = 0;
    /// @endcond

    /// @brief Creates a RF data streaming interface for selected RF module.
    /// @param config The configuration to use for setting the streams up.
    /// @param moduleIndex The index of the device to set up.
    /// @return RF data streaming interface object.
    virtual std::unique_ptr<lime::RFStream> StreamCreate(const StreamConfig& config, uint8_t moduleIndex) = 0;

    /// @brief Uploads waveform to on board memory for later use.
    /// @param config The configuration of the stream.
    /// @param moduleIndex The index of the device to upload the waveform to.
    /// @param samples The samples to upload to the device.
    /// @param count The amount of samples to upload to the device.
    /// @return Operation status.
    virtual OpStatus UploadTxWaveform(const StreamConfig& config, uint8_t moduleIndex, const void** samples, uint32_t count);

    /// @copydoc lime::ISPI::Transact()
    /// @param spiBusAddress The SPI address of the device to use.
    virtual OpStatus SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count);

    /// @copydoc lime::II2C::I2CWrite()
    virtual OpStatus I2CWrite(int address, const uint8_t* data, uint32_t length);

    /// @copydoc lime::II2C::I2CRead()
    virtual OpStatus I2CRead(int address, uint8_t* dest, uint32_t length);

    /***********************************************************************
     * GPIO API
     **********************************************************************/

    /// @brief Change device GPIO pin state.
    /// Writes data to a single or multiple device GPIO pins at once.
    /// Single bit in GPIO register controls output of a single GPIO pin.
    /// GPIO indexing starts from register LSB in ascending order.
    /// GPIO0 = bit 0, GPIO1 = bit 1 and etc.
    /// GPIO registers are 8 bit wide. If device has more than one GPIO register,
    /// then the next register continues to index GPIOs in ascending order, 
    /// GPIO8 = bit 0, GPIO9 = bit 1 and etc.
    /// GPIO registers and register count is specific to a device and its GW.
    /// Value - 1 == HIGH state, 0 == LOW state.
    /// @param buffer buffer with new GPIO pin output values.
    /// @param bufLength Number of GPIO registers to update.
    /// @return The @ref lime::OpStatus "status" of operation.
    virtual OpStatus GPIOWrite(const uint8_t* buffer, const size_t bufLength);

    /// @brief Read device GPIO pin state.
    /// Reads the status of all GPIO pins for a specified number of GPIO registers.
    /// Single bit in GPIO register shows status of a single GPIO pin.
    /// GPIO indexing starts from register LSB in ascending order.
    /// GPIO0 = bit 0, GPIO1 = bit 1 and etc.
    /// GPIO registers are 8 bit wide. If device has more than one GPIO register,
    /// then the next register continues to index GPIOs in ascending order, 
    /// GPIO8 = bit 0, GPIO9 = bit 1 and etc.
    /// GPIO registers and register count is specific to a device and its GW.
    /// Value - 1 == HIGH state, 0 == LOW state.
    /// @param buffer empty buffer of sufficient length. 
    /// @param bufLength Number of GPIO registers to read.
    /// @return The @ref lime::OpStatus "status" of operation.
    virtual OpStatus GPIORead(uint8_t* buffer, const size_t bufLength);

    /// @brief Change device GPIO pin direction.
    /// Changes direction of a single or multiple device GPIO pins at once.
    /// Single bit in GPIO register controls a single GPIO pin direction.
    /// GPIO indexing starts from register LSB in ascending order.
    /// GPIO0 = bit 0, GPIO1 = bit 1 and etc.
    /// GPIO registers are 8 bit wide. If device has more than one GPIO register,
    /// then the next register continues to index GPIOs in ascending order, 
    /// GPIO8 = bit 0, GPIO9 = bit 1 and etc.
    /// GPIO registers and register count is specific to a device and its GW.
    /// Value - 1 == OUTPUT direction, 0 == INPUT direction. 
    /// @param buffer buffer with new GPIO pin direction settings.
    /// @param bufLength Number of GPIO pin direction registers to update.
    /// @return The @ref lime::OpStatus "status" of operation.
    virtual OpStatus GPIODirWrite(const uint8_t* buffer, const size_t bufLength);

    /// @brief Read device GPIO pin direction.
    /// Reads the current direction of all GPIO pins for specified number of GPIO direction registers.
    /// Single bit in GPIO register shows direction status of a single GPIO pin.
    /// GPIO indexing starts from register LSB in ascending order.
    /// GPIO0 = bit 0, GPIO1 = bit 1 and etc.
    /// GPIO registers are 8 bit wide. If device has more than one GPIO register,
    /// then the next register continues to index GPIOs in ascending order, 
    /// GPIO8 = bit 0, GPIO9 = bit 1 and etc.
    /// GPIO registers and register count is specific to a device and its GW.
    /// Value - 1 == OUTPUT direction, 0 == INPUT direction. 
    /// @param buffer empty data buffer of sufficient length.
    /// @param bufLength Number of GPIO pin direction registers to read.
    /// @return The @ref lime::OpStatus "status" of operation.
    virtual OpStatus GPIODirRead(uint8_t* buffer, const size_t bufLength);

    /***********************************************************************
     * Arbitrary settings API
     **********************************************************************/

    virtual OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters);

    virtual OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters);

    /// @brief The definition of a function to call when a log message is generated.
    typedef std::function<void(LogLevel, const std::string&)> LogCallbackType;

    /// @brief Sets callback function which gets called each a log message is received
    /// @param callback The callback to use from this point onwards.
    virtual void SetMessageLogCallback(LogCallbackType callback);

    /// @brief Gets the pointer to an internal chip of the device.
    /// @param index The index of the device to retrieve.
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
    /// @return The @ref lime::OpStatus "status" of operation.
    virtual OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback);

    /// @brief Writes given data into a given memory address in EEPROM memory.
    /// @param storage The storage device to write to.
    /// @param region Information of the region in which to write the data to.
    /// @param data The data to write into the specified memory.
    /// @return The @ref lime::OpStatus "status" of operation.
    virtual OpStatus MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data);

    /// @brief Reads data from a given memory address in EEPROM memory.
    /// @param storage The storage device to read from.
    /// @param region Information of the region from which to read the memory.
    /// @param data The storage buffer for the data being read.
    /// @return The @ref lime::OpStatus "status" of operation.
    virtual OpStatus MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data);

    /// @brief Runs various device specific tests to check functionality
    /// @param reporter Object for handling test results callbacks
    /// @return The @ref lime::OpStatus "status" of  operation.
    virtual OpStatus OEMTest(OEMTestReporter* reporter);

    /// @brief Writes one time programmable serial number of the device
    /// @param serialNumber Device's serial number
    /// @return The @ref lime::OpStatus "status" of operation.
    virtual OpStatus WriteSerialNumber(uint64_t serialNumber);

    /// @brief Return GPIO controls interface if available
    /// @return nullptr if not available
    virtual GPIO_Interface* GetGPIOControls();
};

} // namespace lime
#endif
