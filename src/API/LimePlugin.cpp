#include "limesuiteng/LimePlugin.h"
#include "gainTable.h"

#include <algorithm>
#include <assert.h>
#include <chrono>
#include <iostream>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string_view>

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/SDRDescriptor.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"

#ifdef _MSC_VER
    #define strncasecmp _strnicmp
    #define strcasecmp _stricmp
#endif

using namespace lime;
using namespace lime::LMS7002MCSR_Data;
using namespace std;

static constexpr int LIME_MAX_UNIQUE_DEVICES = 16;
static constexpr int LIME_TRX_MAX_RF_PORT = 16;

std::vector<std::string_view> splitString(std::string_view string, std::string_view delimiter)
{
    std::vector<std::string_view> ret;

    auto position = string.find(delimiter);

    while (position != std::string_view::npos)
    {
        if (position != 0)
        {
            ret.push_back(string.substr(0, position));
        }

        string = string.substr(position + 1);
        position = string.find(delimiter);
    }

    if (string.size() > 0)
    {
        ret.push_back(string);
    }
    return ret;
}

struct StreamStatus {
    lime::StreamStats rx;
    lime::StreamStats tx;
};

static std::mutex gainsMutex;
static std::array<StreamStatus, LIME_TRX_MAX_RF_PORT> portStreamStates;

static lime::SDRDevice::LogCallbackType hostCallback = nullptr;

DevNode::DevNode()
    : chipIndex(0)
    , device(nullptr)
{
}

LimePluginContext::LimePluginContext()
    : rfdev(LIME_MAX_UNIQUE_DEVICES)
    , config(nullptr)
    , samplesFormat(DataFormat::F32)
{
    ports.reserve(LIME_TRX_MAX_RF_PORT);
}

enum CalibrateFlag { None = 0, DCIQ = 1, Filter = 2 };

template<class T>
static bool GetSetting [[gnu::format(printf, 3, 4)]] (LimeSettingsProvider* settings, T* pval, const char* prop_name_format, ...)
{
    char name[256];
    va_list args;
    va_start(args, prop_name_format);
    std::vsnprintf(name, sizeof(name), prop_name_format, args);
    va_end(args);

    double val = 0;
    if (!settings->GetDouble(val, name))
        return false;
    *pval = val;
    return true;
}

template<>
bool GetSetting [[gnu::format(printf, 3, 4)]] (LimeSettingsProvider* settings, std::string* pval, const char* prop_name_format, ...)
{
    char name[256];
    va_list args;
    va_start(args, prop_name_format);
    std::vsnprintf(name, sizeof(name), prop_name_format, args);
    va_end(args);

    return settings->GetString(*pval, name);
}

template<class T>
static bool GetParam [[gnu::format(printf, 3, 4)]] (LimePluginContext* context, T& pval, const char* prop_name_format, ...)
{
    char name[256];
    va_list args;
    va_start(args, prop_name_format);
    std::vsnprintf(name, sizeof(name), prop_name_format, args);
    va_end(args);

    double val = 0;
    if (!context->config->GetDouble(val, name))
        return false;
    pval = val;
    return true;
}

static lime::LogLevel logVerbosity = lime::LogLevel::Debug;

static void Log [[gnu::format(printf, 2, 3)]] (LogLevel lvl, const char* format, ...)
{
    if (lvl > logVerbosity)
        return;
    char msg[512];
    va_list args;
    va_start(args, format);
    std::vsnprintf(msg, sizeof(msg), format, args);
    va_end(args);
    if (hostCallback)
        hostCallback(lvl, msg);
}

static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl > logVerbosity)
        return;
    if (hostCallback)
        hostCallback(lvl, msg);
}

static auto lastStreamUpdate = std::chrono::steady_clock::now();
bool OnStreamStatusChange(bool isTx, const StreamStats* s, void* userData)
{
    StreamStatus& status = *static_cast<StreamStatus*>(userData);
    StreamStats& dest = isTx ? status.tx : status.rx;

    dest.FIFO = s->FIFO;
    dest.dataRate_Bps = s->dataRate_Bps;
    dest.overrun = s->overrun;
    dest.underrun = s->underrun;
    dest.loss = s->loss;
    if (!isTx) // Tx dropped packets are reported from Rx received packet flags
        status.tx.loss = s->late;

    // reporting every dropped packet can be quite spammy, so print info only periodically
    auto now = chrono::steady_clock::now();
    if (chrono::duration_cast<chrono::milliseconds>(now - lastStreamUpdate) > chrono::milliseconds(500) &&
        logVerbosity >= LogLevel::Warning)
    {
        stringstream ss;
        ss << "Rx| Loss: "sv << status.rx.loss << " overrun: "sv << status.rx.overrun << " rate: "sv << status.rx.dataRate_Bps / 1e6
           << " MB/s"sv
           << "\nTx| Late: "sv << status.tx.loss << " underrun: "sv << status.tx.underrun << " rate: "sv
           << status.tx.dataRate_Bps / 1e6 << " MB/s"sv;
        Log(LogLevel::Warning, "%s", ss.str().c_str());
        lastStreamUpdate = now;
    }
    return false;
}

//min gain 0
//max gain ~70-76 (higher will probably degrade signal quality to much)
void LimePlugin_SetTxGain(LimePluginContext* context, double gain, int channel_num)
{
    int row = std::clamp(static_cast<int>(gain), 0, static_cast<int>(txGainTable.size() - 1));

    std::lock_guard<std::mutex> lk(gainsMutex);

    SDRDevice* device = context->txChannels[channel_num].parent->device;
    LMS7002M* chip = static_cast<LMS7002M*>(device->GetInternalChip(context->txChannels[channel_num].parent->chipIndex));
    chip->Modify_SPI_Reg_bits(MAC, context->txChannels[channel_num].chipChannel + 1);
    chip->Modify_SPI_Reg_bits(LOSS_MAIN_TXPAD_TRF, txGainTable[row].main);
    chip->Modify_SPI_Reg_bits(LOSS_LIN_TXPAD_TRF, txGainTable[row].lin);
    chip->Modify_SPI_Reg_bits(MAC, 1);
    Log(LogLevel::Debug,
        "chip%i ch%i Tx gain set MAIN:%i, LIN:%i",
        context->txChannels[channel_num].parent->chipIndex,
        context->txChannels[channel_num].chipChannel,
        txGainTable[row].main,
        txGainTable[row].lin);
}

void LimePlugin_SetRxGain(LimePluginContext* context, double gain, int channel_num)
{
    int row = std::clamp(static_cast<int>(gain), 0, static_cast<int>(rxGainTable.size() - 1));

    std::lock_guard<std::mutex> lk(gainsMutex);

    SDRDevice* device = context->rxChannels[channel_num].parent->device;
    LMS7002M* chip = static_cast<LMS7002M*>(device->GetInternalChip(context->rxChannels[channel_num].parent->chipIndex));
    chip->Modify_SPI_Reg_bits(MAC, context->rxChannels[channel_num].chipChannel + 1);
    chip->Modify_SPI_Reg_bits(G_LNA_RFE, rxGainTable[row].lna);
    chip->Modify_SPI_Reg_bits(G_PGA_RBB, rxGainTable[row].pga);
    chip->Modify_SPI_Reg_bits(MAC, 1);
    Log(LogLevel::Debug,
        "chip%i ch%i Rx gain set LNA:%i, PGA:%i",
        context->rxChannels[channel_num].parent->chipIndex,
        context->rxChannels[channel_num].chipChannel,
        rxGainTable[row].lna,
        rxGainTable[row].pga);
}

static void ConfigGains(
    LimePluginContext* context, const LimeRuntimeParameters* params, std::vector<ChannelData>& channelMapping, bool isTx)
{
    for (size_t ch = 0; ch < channelMapping.size(); ++ch)
    {
        if (isTx)
            LimePlugin_SetTxGain(context, params->tx.gain[ch], ch);
        else
            LimePlugin_SetRxGain(context, params->rx.gain[ch], ch);
    }
}

static OpStatus MapChannelsToDevices(
    std::vector<ChannelData>& channels, std::vector<PortData>& ports, const LimeRuntimeParameters& params, TRXDir dir)
{
    channels.clear();
    for (uint32_t p = 0; p < params.rf_ports.size(); ++p)
    {
        int port_channel_count = dir == TRXDir::Tx ? params.rf_ports[p].tx_channel_count : params.rf_ports[p].rx_channel_count;

        std::deque<DevNode*> assignedDevices;
        copy(ports[p].nodes.begin(), ports[p].nodes.end(), assignedDevices.begin());

        int remainingChannels = assignedDevices.front()->configInputs.maxChannelsToUse;
        int chipRelativeChannelIndex = 0;

        for (int i = 0; i < port_channel_count; ++i)
        {
            if (remainingChannels == 0)
            {
                assignedDevices.pop_front();
                if (assignedDevices.empty())
                {
                    Log(LogLevel::Error, "port%i requires more channels than assigned devices have.", p);
                    return OpStatus::Error;
                }
                remainingChannels = assignedDevices.front()->configInputs.maxChannelsToUse;
                chipRelativeChannelIndex = 0;
            }
            ChannelData lane{};
            lane.chipChannel = chipRelativeChannelIndex;
            ++chipRelativeChannelIndex;
            lane.parent = assignedDevices.front();
            lane.parent->assignedToPort = true;
            --remainingChannels;
            // Log(LogLevel::Debug,
            //     "%s Channel%i : dev%i chipIndex:%i, chipChannel:%i",
            //     ToCString(dir),
            //     static_cast<int>(channels.size()),
            //     lane.parent->devIndex,
            //     lane.parent->chipIndex,
            //     lane.chipChannel);
            channels.push_back(lane);
        }
    }
    return OpStatus::Success;
}

static void ParseFPGARegistersWrites(LimePluginContext* context, int devIndex)
{
    std::string varname = "dev"s + std::to_string(devIndex) + "_writeRegisters"s;
    std::string value;

    if (!context->config->GetString(value, varname.c_str()))
    {
        return;
    }

    std::string_view writeRegistersStr{ value };
    const auto tokens = splitString(writeRegistersStr, ";"sv);
    for (const auto& token : tokens)
    {
        uint32_t spiVal = 0;
        std::stringstream sstream{ std::string{ token } };
        sstream >> std::hex >> spiVal;
        context->rfdev.at(devIndex).fpgaRegisterWrites.push_back(spiVal | (1 << 31)); // adding spi write bit for convenience
    }
}

int LimePlugin_Stop(LimePluginContext* context)
{
    for (auto& port : context->ports)
    {
        if (!port.composite)
            continue;
        port.composite->StreamStop();
    }
    return 0;
}

int LimePlugin_Destroy(LimePluginContext* context)
{
    LimePlugin_Stop(context);
    for (auto& iter : context->uniqueDevices)
        DeviceRegistry::freeDevice(iter.second);
    return 0;
}

static bool FuzzyHandleMatch(const DeviceHandle& handle, const std::string& text)
{
    if (text.empty())
        return true;

    if (!handle.name.empty() && handle.name.find(text) != std::string::npos)
        return true;

    if (!handle.addr.empty() && handle.addr.find(text) != std::string::npos)
        return true;

    if (!handle.serial.empty() && handle.serial.find(text) != std::string::npos)
        return true;

    if (!handle.media.empty() && handle.media.find(text) != std::string::npos)
        return true;

    return false;
}

static int FilterHandles(
    const std::string& text, const std::vector<DeviceHandle>& handles, std::vector<DeviceHandle>& filteredHandles)
{
    if (handles.empty())
        return 0;

    DeviceHandle deserializedHandle(text);
    filteredHandles.clear();
    for (const DeviceHandle& h : handles)
    {
        // compare hint as if it was in serialized handle form.
        // if it's not, compare using basic text search among handle fields
        if (h.IsEqualIgnoringEmpty(deserializedHandle) || FuzzyHandleMatch(h, text))
            filteredHandles.push_back(h);
    }

    return filteredHandles.size();
}

static OpStatus ConnectInitializeDevices(LimePluginContext* context)
{
    // Collect and connect specified unique nodes
    std::vector<DeviceHandle> fullHandles = DeviceRegistry::enumerate();
    for (int i = 0; i < LIME_MAX_UNIQUE_DEVICES; ++i)
    {
        DevNode& node = context->rfdev.at(i);
        if (node.handleString.empty())
            continue;

        std::vector<DeviceHandle> filteredHandles;
        FilterHandles(node.handleString, fullHandles, filteredHandles);

        if (filteredHandles.size() > 1)
        {
            Log(LogLevel::Error, "%s : ambiguous handle, matches multiple devices.", node.handleString.c_str());
            return OpStatus::InvalidValue;
        }
        if (filteredHandles.empty())
        {
            Log(LogLevel::Error, "No device found to match handle: %s.", node.handleString.c_str());
            return OpStatus::InvalidValue;
        }
        auto iter = context->uniqueDevices.find(filteredHandles.at(0).Serialize());
        if (iter != context->uniqueDevices.end())
        {
            node.device = iter->second;
            continue; // skip if the device already exists
        }

        SDRDevice* device = DeviceRegistry::makeDevice(filteredHandles.at(0));
        if (device != nullptr)
            Log(LogLevel::Info, "Connected: %s", filteredHandles.at(0).Serialize().c_str());
        else
        {
            Log(LogLevel::Error, "Failed to connect: %s", filteredHandles.at(0).Serialize().c_str());
            return OpStatus::Error;
        }

        // Initialize device to default settings
        lime::registerLogHandler(LogCallback);
        device->SetMessageLogCallback(LogCallback);
        device->EnableCache(false);
        device->Init();
        context->uniqueDevices[filteredHandles.at(0).Serialize()] = device;
        node.device = device;
    }
    return OpStatus::Success;
}

static OpStatus LoadDevicesConfigurationFile(LimePluginContext* context)
{
    for (size_t i = 0; i < context->rfdev.size(); ++i)
    {
        DevNode& node = context->rfdev.at(i);
        if (node.device == nullptr)
            continue;

        const auto& desc = node.device->GetDescriptor();
        if (node.chipIndex >= desc.rfSOC.size())
        {
            Log(LogLevel::Error, "Invalid chipIndex (%i). dev%li has only %li chips.", node.chipIndex, i, desc.rfSOC.size());
            return OpStatus::OutOfRange;
        }

        if (node.configInputs.iniFilename.empty())
            continue;

        LMS7002M* chip = static_cast<LMS7002M*>(node.device->GetInternalChip(node.chipIndex));

        std::string configFilepath;
        if (node.configInputs.iniFilename[0] != '/') // is not global path
            configFilepath += context->currentWorkingDirectory + "/"s;

        configFilepath += node.configInputs.iniFilename;

        if (chip->LoadConfig(configFilepath, false) != OpStatus::Success)
        {
            Log(LogLevel::Error, "dev%li chip%i Error loading file: %s", i, node.chipIndex, configFilepath.c_str());
            return OpStatus::Error;
        }

        node.config.skipDefaults = true;
        Log(LogLevel::Info, "dev%li chip%i loaded with: %s", i, node.chipIndex, configFilepath.c_str());
    }
    return OpStatus::Success;
}

static OpStatus AssignDevicesToPorts(LimePluginContext* context)
{
    for (int p = 0; p < LIME_TRX_MAX_RF_PORT; ++p)
    {
        PortData& port = context->ports.at(p);

        std::string_view devices{ port.deviceNames };
        const auto tokens = splitString(devices, ","sv);
        for (auto token : tokens)
        {
            if (token.find("dev"sv) != 0) // token.starts_with in C++20
            {
                // invalid device name
                Log(LogLevel::Error, "Port%i assigned invalid (%s) device.", p, std::string{ token }.c_str());
                return OpStatus::InvalidValue;
            }

            int devIndex = 0;
            token.remove_prefix(3);
            std::stringstream sstream{ std::string{ token } };
            sstream >> devIndex;
            DevNode* assignedDevice = &context->rfdev.at(devIndex);
            port.nodes.push_back(assignedDevice);
            assignedDevice->portIndex = p;
            assignedDevice->assignedToPort = true;

            // copy port's config parameters to each assigned device to form base
            // which will later be modified by individual device parameter overrides
            assignedDevice->configInputs = port.configInputs;
        }
    }
    return OpStatus::Success;
}

static void GatherEnvironmentSettings(LimePluginContext* context, LimeSettingsProvider* configProvider)
{
    int val = 0;
    if (GetParam(context, val, "logLevel"))
        logVerbosity = std::min(static_cast<LogLevel>(val), LogLevel::Debug);
}

static void GatherDirectionalSettings(LimeSettingsProvider* settings, DirectionalSettings* dir, const char* varPrefix)
{
    GetSetting(settings, &dir->antenna, "%s_path", varPrefix);
    GetSetting(settings, &dir->lo_override, "%s_lo_override", varPrefix);
    GetSetting(settings, &dir->gfir_enable, "%s_gfir_enable", varPrefix);
    GetSetting(settings, &dir->gfir_bandwidth, "%s_gfir_bandwidth", varPrefix);
    dir->powerAvailable = GetSetting(settings, &dir->power_dBm, "%s_power_dBm", varPrefix);
    GetSetting(settings, &dir->calibration, "%s_calibration", varPrefix);
    GetSetting(settings, &dir->oversample, "%s_oversample", varPrefix);
}

static void GatherConfigSettings(ConfigSettings* param, LimeSettingsProvider* settings, const char* prefix)
{
    GetSetting(settings, &param->iniFilename, "%s_ini", prefix);
    GetSetting(settings, &param->lpfBandwidthScale, "%s_lpf_bandwidth_scale", prefix);
    GetSetting(settings, &param->maxChannelsToUse, "%s_max_channels_to_use", prefix);
    GetSetting(settings, &param->double_freq_conversion_to_lower_side, "%s_double_freq_conversion_to_lower_side", prefix);
    std::string linkFormatStr;
    if (GetSetting(settings, &linkFormatStr, "%s_linkFormat", prefix))
    {
        if (linkFormatStr == "I16"s)
            param->linkFormat = lime::DataFormat::I16;
        else if (linkFormatStr == "I12"s)
            param->linkFormat = lime::DataFormat::I12;
        else
        {
            Log(LogLevel::Warning, "Invalid link format (%s): falling back to I12", linkFormatStr.c_str());
            param->linkFormat = lime::DataFormat::I12;
        }
    }
    GetSetting(settings, &param->syncPPS, "%s_syncPPS", prefix);

    char dirPrefix[32];
    std::snprintf(dirPrefix, sizeof(dirPrefix), "%s_rx", prefix);
    GatherDirectionalSettings(settings, &param->rx, dirPrefix);
    std::snprintf(dirPrefix, sizeof(dirPrefix), "%s_tx", prefix);
    GatherDirectionalSettings(settings, &param->tx, dirPrefix);
}

static void GatherDeviceTreeNodeSettings(LimePluginContext* context, LimeSettingsProvider* settings)
{
    for (uint32_t i = 0; i < context->rfdev.size(); ++i)
    {
        DevNode& dev = context->rfdev.at(i);
        char devPrefix[16];
        std::snprintf(devPrefix, sizeof(devPrefix), "dev%i", i);
        GetSetting(settings, &dev.handleString, "%s", devPrefix);
        GetSetting(settings, &dev.chipIndex, "%s_chip_index", devPrefix);

        GatherConfigSettings(&dev.configInputs, settings, devPrefix);
        ParseFPGARegistersWrites(context, i);
    }
}

static OpStatus GatherPortSettings(LimePluginContext* context, LimeSettingsProvider* settings)
{
    int specifiedPortsCount = 0;
    for (uint32_t i = 0; i < context->ports.size(); ++i)
    {
        PortData& port = context->ports.at(i);
        char portPrefix[16];
        std::snprintf(portPrefix, sizeof(portPrefix), "port%i", i);
        GatherConfigSettings(&port.configInputs, settings, portPrefix);
        if (GetSetting(settings, &port.deviceNames, "%s", portPrefix))
        {
            ++specifiedPortsCount;
            OpStatus status = AssignDevicesToPorts(context);
            if (status != OpStatus::Success)
                return status;
        }
    }

    if (specifiedPortsCount == 0)
    {
        Log(LogLevel::Error, "No ports have been specified.");
        return OpStatus::Error;
    }
    return OpStatus::Success;
}

static OpStatus TransferDeviceDirectionalSettings(
    DevNode& node, const DirectionalSettings& settings, ChannelConfig::Direction& trx, TRXDir dir)
{
    trx.enabled = false;
    trx.oversample = settings.oversample;
    trx.gfir.enabled = settings.gfir_enable;
    trx.gfir.bandwidth = settings.gfir_bandwidth;

    const auto& desc = node.device->GetDescriptor().rfSOC[node.chipIndex];
    const auto& paths = desc.pathNames.at(dir);
    trx.path = 0;

    if (!settings.antenna.empty())
    {
        bool match = false;
        for (size_t j = 0; j < paths.size(); ++j)
        {
            if (strcasecmp(paths[j].c_str(), settings.antenna.c_str()) == 0)
            {
                trx.path = j;
                match = true;
                break;
            }
        }
        if (!match)
        {
            Log(LogLevel::Error, "%s path not found. Available: ", settings.antenna.c_str());
            for (const auto& iter : paths)
                Log(LogLevel::Error, "\"%s\" ", iter.c_str());
            return OpStatus::InvalidValue;
        }
    }

    if (settings.lo_override > 0)
        trx.centerFrequency = settings.lo_override;

    int flag = CalibrateFlag::Filter; // by default calibrate only filters
    if (!settings.calibration.empty())
    {
        const char* value = settings.calibration.c_str();
        // strcasecmp is not a function in the C++ standard
        if (!strcasecmp(value, "none"))
            flag = CalibrateFlag::None;
        else if ((!strcasecmp(value, "force")) || (!strcasecmp(value, "all")))
            flag = CalibrateFlag::Filter | CalibrateFlag::DCIQ;
        else if (!strcasecmp(value, "filter"))
            flag = CalibrateFlag::Filter;
        else if (!strcasecmp(value, "dciq"))
            flag = CalibrateFlag::DCIQ;
    }

    if (flag & DCIQ)
        trx.calibrate = true;

    // copy setting to all channels
    for (int i = 1; i < 2; ++i)
    {
        if (dir == TRXDir::Tx)
            node.config.channel[i].tx = trx;
        else
            node.config.channel[i].rx = trx;
    }
    return OpStatus::Success;
}

static OpStatus TransferSettingsToDevicesConfig(std::vector<DevNode>& nodes)
{
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        DevNode& node = nodes.at(i);
        if (!node.device)
            continue;
        node.devIndex = i;
        node.assignedToPort = false;

        for (int ch = 0; ch < node.configInputs.maxChannelsToUse; ++ch)
        {
            OpStatus status = TransferDeviceDirectionalSettings(node, node.configInputs.rx, node.config.channel[ch].rx, TRXDir::Rx);
            if (status != OpStatus::Success)
                return status;
            status = TransferDeviceDirectionalSettings(node, node.configInputs.tx, node.config.channel[ch].tx, TRXDir::Tx);
            if (status != OpStatus::Success)
                return status;
        }
    }
    return OpStatus::Success;
}

int LimePlugin_Init(LimePluginContext* context, lime::SDRDevice::LogCallbackType logFptr, LimeSettingsProvider* configProvider)
{
    context->hostLog = logFptr;
    hostCallback = logFptr;
    context->config = configProvider;
    context->ports.resize(LIME_TRX_MAX_RF_PORT);
    context->rfdev.resize(LIME_MAX_UNIQUE_DEVICES);

    GatherEnvironmentSettings(context, configProvider);

    // first gather port settings, they will be base for each device
    if (GatherPortSettings(context, configProvider) != OpStatus::Success)
        return -1;

    // gather each device settings and override values set by port
    GatherDeviceTreeNodeSettings(context, configProvider);

    try
    {
        // Enumerate devices
        std::vector<DeviceHandle> devHandles = DeviceRegistry::enumerate();
        if (devHandles.empty())
        {
            Log(LogLevel::Error, "No connected devices discovered.");
            return -1;
        }
        else
        {
            Log(LogLevel::Debug, "Available devices:");
            for (const auto& dev : devHandles)
                Log(LogLevel::Debug, "\t\"%s\"", dev.Serialize().c_str());
        }

        // Collect and initialize specified unique devices
        if (ConnectInitializeDevices(context) != OpStatus::Success)
            return -1;

        // Load configuration files for each chip if specified
        if (LoadDevicesConfigurationFile(context) != OpStatus::Success)
            return -1;

        TransferSettingsToDevicesConfig(context->rfdev);

        // load power settings
        /*    for (int p = 0; p < TRX_MAX_RF_PORT; ++p)
        {
            // absolute gain info
            for (int ch = 0; ch < TRX_MAX_CHANNELS; ++ch)
            {
                double dac = 0;
                std::snprintf(varname, sizeof(varname), "port%i_ch%i_pa_dac", p, ch);
                if (trx_get_param_double(hostState, &dac, varname) == 0)
                {
                    // TODO: this is board specific, need general API
                    int32_t paramId = 2 + ch;
                    std::string units = ""s;
                    s->device[p]->CustomParameterWrite({ { paramId, dac, units } });
                }
            }

            StreamConfig::Extras* extra = new StreamConfig::Extras();

            std::snprintf(varname, sizeof(varname), "port%i_syncPPS", p);
            if (trx_get_param_double(hostState, &val, varname) == 0)
            {
                extra->waitPPS = val != 0;
                s->streamExtras[p] = extra;
            }
            std::snprintf(varname, sizeof(varname), "port%i_rxSamplesInPacket", p);
            if (trx_get_param_double(hostState, &val, varname) == 0)
            {
                extra->rxSamplesInPacket = val;
                s->streamExtras[p] = extra;
            }
            std::snprintf(varname, sizeof(varname), "port%i_rxPacketsInBatch", p);
            if (trx_get_param_double(hostState, &val, varname) == 0)
            {
                extra->rxPacketsInBatch = val;
                s->streamExtras[p] = extra;
            }
            std::snprintf(varname, sizeof(varname), "port%i_txMaxPacketsInBatch", p);
            if (trx_get_param_double(hostState, &val, varname) == 0)
            {
                extra->txMaxPacketsInBatch = val;
                s->streamExtras[p] = extra;
            }
            std::snprintf(varname, sizeof(varname), "port%i_txSamplesInPacket", p);
            if (trx_get_param_double(hostState, &val, varname) == 0)
            {
                extra->txSamplesInPacket = val;
                s->streamExtras[p] = extra;
            }
            std::snprintf(varname, sizeof(varname), "port%i_double_freq_conversion_to_lower_side", p);
            if (trx_get_param_double(hostState, &val, varname) == 0)
            {
                extra->negateQ = val;
                s->streamExtras[p] = extra;
            }
        }

        s->samplesFormat = lime::StreamConfig::F32;
*/
    } catch (std::logic_error& e)
    {
        fprintf(stderr, "Logic error: %s", e.what());
        return -1;
    } catch (std::runtime_error& e)
    {
        fprintf(stderr, "Runtime error: %s", e.what());
        return -1;
    }
    return 0;
}

static void TransferRuntimeParametersToConfig(
    const LimeRuntimeParameters& runtimeParams, const std::vector<ChannelData>& channelMap, TRXDir dir)
{
    const bool isTx = dir == TRXDir::Tx;
    const LimeRuntimeParameters::ChannelParams& params = isTx ? runtimeParams.tx : runtimeParams.rx;
    for (size_t i = 0; i < params.freq.size(); ++i)
    {
        ChannelConfig::Direction& trxConfig = dir == TRXDir::Tx
                                                  ? channelMap[i].parent->config.channel[channelMap[i].chipChannel].tx
                                                  : channelMap[i].parent->config.channel[channelMap[i].chipChannel].rx;
        trxConfig.enabled = true;
        const int portIndex = channelMap[i].parent->portIndex;
        trxConfig.sampleRate = runtimeParams.rf_ports[portIndex].sample_rate;
        if (trxConfig.centerFrequency == 0)
            trxConfig.centerFrequency = params.freq[i];
        else
        {
            char loFreqStr[1024];
            std::snprintf(loFreqStr,
                sizeof(loFreqStr),
                "%s channel%i expectedLO: %.3f MHz [override: %.3f (diff:%+.3f) MHz]",
                (dir == TRXDir::Rx ? "Rx" : "Tx"),
                static_cast<int>(i),
                params.freq[i] / 1.0e6,
                trxConfig.centerFrequency / 1.0e6,
                (trxConfig.centerFrequency - params.freq[i]) / 1.0e6);
            Log(LogLevel::Info, "%s", loFreqStr);
        }

        if (trxConfig.gfir.bandwidth == 0) // update only if not set by settings file
            trxConfig.gfir.bandwidth = params.bandwidth[i];
        trxConfig.lpf = params.bandwidth[i];
        trxConfig.lpf *= channelMap[i].parent->configInputs.lpfBandwidthScale;

        const int chipIndex = channelMap[i].parent->chipIndex;
        const auto& desc = channelMap[i].parent->device->GetDescriptor().rfSOC[chipIndex];
        const auto& paths = desc.pathNames.at(isTx ? TRXDir::Tx : TRXDir::Rx);

        Log(LogLevel::Verbose,
            "%s channel%li: dev%i chip%i ch%i , LO: %.3f MHz SR: %.3f MHz BW: %.3f MHz | path: %i('%s')",
            isTx ? "Tx" : "Rx",
            i,
            channelMap[i].parent->devIndex,
            channelMap[i].parent->chipIndex,
            channelMap[i].chipChannel,
            trxConfig.centerFrequency / 1e6,
            trxConfig.sampleRate / 1e6,
            params.bandwidth[i] / 1e6,
            trxConfig.path,
            paths[trxConfig.path].c_str());
    }
}

OpStatus ConfigureStreaming(LimePluginContext* context, const LimeRuntimeParameters* params)
{
    for (size_t p = 0; p < context->ports.size(); ++p)
    {
        PortData& port = context->ports[p];
        if (port.nodes.empty())
            continue;

        StreamConfig stream;
        stream.channels[TRXDir::Rx].resize(params->rf_ports[p].rx_channel_count);
        stream.channels[TRXDir::Tx].resize(params->rf_ports[p].tx_channel_count);
        stream.linkFormat = port.configInputs.linkFormat;
        stream.format = context->samplesFormat;
        stream.extraConfig.negateQ = port.configInputs.double_freq_conversion_to_lower_side;
        stream.extraConfig.waitPPS = port.configInputs.syncPPS;
        stream.extraConfig.rx.samplesInPacket = 256;
        stream.extraConfig.rx.packetsInBatch = 4;
        stream.extraConfig.tx.packetsInBatch = 8;
        stream.extraConfig.tx.samplesInPacket = 256;

        // Initialize streams and map channels
        for (size_t ch = 0; ch < stream.channels[TRXDir::Rx].size(); ++ch)
            stream.channels[TRXDir::Rx][ch] = ch;
        for (size_t ch = 0; ch < stream.channels[TRXDir::Tx].size(); ++ch)
            stream.channels[TRXDir::Tx][ch] = ch;

        stream.statusCallback = OnStreamStatusChange;
        stream.userData = static_cast<void*>(&portStreamStates[p]);
        stream.hintSampleRate = params->rf_ports[p].sample_rate;

        std::vector<StreamAggregate> aggregates;
        for (auto& dev : port.nodes)
        {
            if (!dev->assignedToPort)
                continue;
            std::vector<int> channels;
            for (int i = 0; i < dev->configInputs.maxChannelsToUse; ++i)
                channels.push_back(i);
            aggregates.push_back({ dev->device, channels, dev->chipIndex });
        }
        if (aggregates.empty())
            continue;

        Log(LogLevel::Debug,
            "Port[%li] Stream samples format: %s , link: %s %s",
            p,
            stream.format == DataFormat::F32 ? "F32" : "I16",
            stream.linkFormat == DataFormat::I12 ? "I12" : "I16",
            (stream.extraConfig.negateQ ? ", Negating Q samples" : ""));

        port.composite = new StreamComposite(aggregates);
        if (port.composite->StreamSetup(stream) != OpStatus::Success)
        {
            Log(LogLevel::Error, "Port%li stream setup failed.", p);
            return OpStatus::Error;
        }
    }
    return OpStatus::Success;
}

int LimePlugin_Setup(LimePluginContext* context, const LimeRuntimeParameters* params)
{
    OpStatus status = MapChannelsToDevices(context->rxChannels, context->ports, *params, TRXDir::Rx);
    if (status != OpStatus::Success)
        return -1;
    status = MapChannelsToDevices(context->txChannels, context->ports, *params, TRXDir::Tx);
    if (status != OpStatus::Success)
        return -1;

    TransferRuntimeParametersToConfig(*params, context->rxChannels, TRXDir::Rx);
    TransferRuntimeParametersToConfig(*params, context->txChannels, TRXDir::Tx);

    try
    {
        // configure all devices
        for (size_t i = 0; i < context->rfdev.size(); ++i)
        {
            DevNode& node = context->rfdev[i];
            if (node.device == nullptr)
                continue;
            else if (node.device != nullptr && !node.assignedToPort)
            {
                Log(LogLevel::Warning, "dev%li is not assigned to any port.", i);
                continue;
            }
            try
            {
                Log(LogLevel::Debug, "dev%li configure.", i);
                OpStatus status = node.device->Configure(node.config, node.chipIndex);
                if (status != OpStatus::Success)
                    return -1;
            } catch (...)
            {
                return -1;
            }

            if (node.fpgaRegisterWrites.size() > 0)
            {
                const auto slaves = node.device->GetDescriptor().spiSlaveIds;
                node.device->SPI(slaves.at("FPGA"s), node.fpgaRegisterWrites.data(), nullptr, node.fpgaRegisterWrites.size());
            }
        }

        // override gains after device Configure
        ConfigGains(context, params, context->rxChannels, false);
        ConfigGains(context, params, context->txChannels, true);
        if (ConfigureStreaming(context, params) != OpStatus::Success)
            return -1;
    } // try
    catch (std::logic_error& e)
    {
        Log(LogLevel::Error, "logic_error: %s", e.what());
        return -1;
    } catch (std::runtime_error& e)
    {
        Log(LogLevel::Error, "runtime_error: %s", e.what());
        return -1;
    }
    return 0;
}

int LimePlugin_Start(LimePluginContext* context)
{
    for (auto& port : context->ports)
    {
        if (!port.composite)
            continue;
        port.composite->StreamStart();
    }
    return 0;
}

template<class T>
static int LimePlugin_Write(LimePluginContext* context, const T* const* samples, int count, int port, StreamMeta& meta)
{
    if (!samples) // Nothing to transmit
        return 0;

    int samplesConsumed = context->ports[port].composite->StreamTx(samples, count, &meta);
    if (logVerbosity == LogLevel::Debug && samplesConsumed != count)
    {
        if (samplesConsumed < 0) // hardware timestamp is already ahead of meta.timestamp by (-samplesConsumed)
        {
            ++portStreamStates[port].tx.underrun;
            Log(LogLevel::Debug, "StreamTx: discarded samples write that is already late by (%i samples)", -samplesConsumed);
        }
        else
            Log(LogLevel::Warning, "Tx not full consumed %i/%i", samplesConsumed, count);
    }
    return samplesConsumed;
}

int LimePlugin_Write_complex32f(
    LimePluginContext* context, const lime::complex32f_t* const* samples, int count, int port, StreamMeta& meta)
{
    return LimePlugin_Write(context, samples, count, port, meta);
}

int LimePlugin_Write_complex16(
    LimePluginContext* context, const lime::complex16_t* const* samples, int count, int port, StreamMeta& meta)
{
    return LimePlugin_Write(context, samples, count, port, meta);
}

template<class T> static int LimePlugin_Read(LimePluginContext* context, T* const* samples, int count, int port, StreamMeta& meta)
{
    meta.waitForTimestamp = false;
    meta.flushPartialPacket = false;
    int samplesGot = context->ports[port].composite->StreamRx(samples, count, &meta);

    if (samplesGot == 0)
    {
        // If the device is misconfigured it might not produce data for Rx
        // in that case the host will be stuck in loop attempting to read.
        // terminate the process to avoid freeze, since samples will never arrive.
        ++portStreamStates[port].tx.underrun;
        Log(LogLevel::Error, "Read timeout for Port[%i], device might be configured incorrectly.", port);
    }
    else if (samplesGot < 0)
    {
        Log(LogLevel::Error, "Error reading samples for Port[%i]: error code %i", port, samplesGot);
    }
    return samplesGot;
}

int LimePlugin_Read_complex32f(
    LimePluginContext* context, lime::complex32f_t* const* samples, int count, int port, StreamMeta& meta)
{
    return LimePlugin_Read(context, samples, count, port, meta);
}

int LimePlugin_Read_complex16(LimePluginContext* context, lime::complex16_t* const* samples, int count, int port, StreamMeta& meta)
{
    return LimePlugin_Read(context, samples, count, port, meta);
}
