#ifndef LIMESUITENG_LIMEPLUGIN_H
#define LIMESUITENG_LIMEPLUGIN_H

#include "limesuiteng/config.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamComposite.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/types.h"

#include <map>
#include <vector>
#include <deque>

/// Interface for providing parameters from configuration file
class LIME_API LimeSettingsProvider
{
  public:
    virtual ~LimeSettingsProvider(){};

    /// @brief Gets the given parameter as a string.
    /// @param dest The variable to store the result to.
    /// @param varname The parameter to search for.
    /// @return The success status of the operation (true if found).
    virtual bool GetString(std::string& dest, const char* varname) = 0;

    /// @brief Gets the given parameter as a double-precision floating point number.
    /// @param dest The variable to store the result to.
    /// @param varname The parameter to search for.
    /// @return The success status of the operation (true if found).
    virtual bool GetDouble(double& dest, const char* varname) = 0;
};

// Tx/Rx settings from configuration file
/// @brief Settings for a single direction
struct LIME_API DirectionalSettings {
    std::string antenna; ///< The name of the antenna to use.
    std::string calibration; ///< The mode of calibration to use.
    double lo_override; ///< The overridden frequency of the direction.
    double gfir_bandwidth; ///< The width of the General Finite Impulse Response filter
    int oversample; ///< Oversampling ratio.
    int power_dBm; ///< The maximum power level of the direction.
    bool powerAvailable; ///< Whether the maximum power level value #power_dBm is available
    bool gfir_enable; ///< Whether the General Finite Impulse Response filter is enabled.
};

/// @brief Configuration settings for a device
struct LIME_API ConfigSettings {
    ConfigSettings()
        : lpfBandwidthScale(1.0)
        , maxChannelsToUse(2)
        , linkFormat(lime::DataFormat::I12)
        , double_freq_conversion_to_lower_side(false)
        , syncPPS(false)
    {
    }
    DirectionalSettings rx; ///< The settings for the receive direction.
    DirectionalSettings tx; ///< The settings for the transmit direction.
    std::string iniFilename; ///< The filename of the configuration file to load.

    // For adjusting initially requested LPF bandwidth
    float lpfBandwidthScale; ///< The bandwidth scale for the Low Pass Filter
    int maxChannelsToUse; ///< Maximum amount of channels that can be used from this chip.
    lime::DataFormat linkFormat; ///< The data format to communicate to the device with.
    bool double_freq_conversion_to_lower_side; ///< Denotes whether to invert the Q value or not.
    bool syncPPS; ///< Denotes whether to wait for the next Pulse Per Second impulse before sending the data on supported devices.
};

/// @brief Individual RF SoC device configuration
struct LIME_API DevNode {
  public:
    DevNode();
    ConfigSettings configInputs; ///< The configuration of the device node as retrieved by the configuration file.
    std::string handleString; ///< The handle of the device.
    uint8_t chipIndex; ///< The chip index to use of the device.
    std::vector<uint32_t> fpgaRegisterWrites; ///< The values to write to the FPGA upon initialization.
    lime::SDRDevice* device; ///< The device that owns the chip.
    lime::SDRConfig config; ///< The configuration of the device.
    int portIndex{}; ///< The index of the port to use.
    int devIndex{}; ///< The index of the device.
    bool assignedToPort{}; ///< Whether this device is assigned to a port or not.
};

/// @brief The data for a channel
struct LIME_API ChannelData {
    DevNode* parent; ///< The owner of this ChannelData.
    int chipChannel; ///< The channel the chip is using with this data.
};

// Ports/Cells that can have combined multiple RF devices to act as one
/// @brief The information about the connection to the devices.
struct LIME_API PortData {
    // settings from file
    std::string deviceNames; ///< The names of the devices to connect to.

    std::vector<DevNode*> nodes; ///< The devices connected to.
    lime::StreamComposite* composite; ///< The composite sample stream streamer.
    ConfigSettings configInputs; ///< The configuration settings for the devices.
};

/// @brief The structure to hold all required information for the plug-in to work.
struct LIME_API LimePluginContext {
    LimePluginContext();
    std::vector<ChannelData> rxChannels; ///< The list of receive channels
    std::vector<ChannelData> txChannels; ///< The list of transmit channels
    std::vector<PortData> ports; ///< The port information of the devices to connect to.
    std::vector<DevNode> rfdev; ///< The Radio-frequency device to use.
    std::map<std::string, lime::SDRDevice*> uniqueDevices; ///< The mapping of device handles to actual devices.
    LimeSettingsProvider* config; ///< The interface to get the settings from.
    lime::DataFormat samplesFormat; ///< The format in which to the samples should be presented to the API.

    /* Path of the config file, not terminating by / */
    std::string currentWorkingDirectory; ///< The directory to look for the configuration file.
    lime::SDRDevice::LogCallbackType hostLog; ///< The callback function to call on when a message is getting logged
};

/// @brief The structure holding the current settings of the streams.
struct LIME_API LimeRuntimeParameters {
    /// @brief The structure for storing current settings for the stream of a direction.
    struct LIME_API ChannelParams {
        std::vector<int64_t> freq; ///< The frequencies of the channels (in Hz).
        std::vector<double> gain; ///< The gains of the channels (in dB).
        std::vector<int> bandwidth; ///< The bandwidth of the filters (in Hz).
    };

    ChannelParams rx; ///< Parameters for all the receive channels.
    ChannelParams tx; ///< Parameters for all the transmit channels.

    /// @brief The parameters of the ports.
    struct LIME_API PortParams {
        double sample_rate; ///< The sample rate of the transmissions (in Hz).
        int rx_channel_count; ///< The count of the receive channels.
        int tx_channel_count; ///< The count of the transmit channels.
    };

    std::vector<PortParams> rf_ports; ///< The parameters for each of the devices.
};

LIME_API void LimePlugin_SetTxGain(LimePluginContext* context, double gain, int channel_num);
LIME_API void LimePlugin_SetRxGain(LimePluginContext* context, double gain, int channel_num);

// context should be allocated/freed by the host
LIME_API int LimePlugin_Init(
    LimePluginContext* context, lime::SDRDevice::LogCallbackType logFptr, LimeSettingsProvider* configProvider);
LIME_API int LimePlugin_Setup(LimePluginContext* context, const LimeRuntimeParameters* params);
LIME_API int LimePlugin_Start(LimePluginContext* context);
LIME_API int LimePlugin_Stop(LimePluginContext* context);
LIME_API int LimePlugin_Destroy(LimePluginContext* context);

LIME_API int LimePlugin_Write_complex32f(
    LimePluginContext* context, const lime::complex32f_t* const* samples, int nsamples, int port, lime::StreamMeta& meta);
LIME_API int LimePlugin_Write_complex16(
    LimePluginContext* context, const lime::complex16_t* const* samples, int nsamples, int port, lime::StreamMeta& meta);
LIME_API int LimePlugin_Read_complex32f(
    LimePluginContext* context, lime::complex32f_t* const* samples, int nsamples, int port, lime::StreamMeta& meta);
LIME_API int LimePlugin_Read_complex16(
    LimePluginContext* context, lime::complex16_t* const* samples, int nsamples, int port, lime::StreamMeta& meta);

template<class T> void CopyCArrayToVector(std::vector<T>& vec, const T* arr, size_t count)
{
    vec.resize(count);
    memcpy(vec.data(), arr, count * sizeof(T));
}

template<class T, class S> void AssignCArrayToVector(std::vector<T>& vec, const S* arr, size_t count)
{
    vec.resize(count);
    for (size_t i = 0; i < count; ++i)
        vec[i] = arr[i];
}

#endif // LIMESUITENG_LIMEPLUGIN_H
