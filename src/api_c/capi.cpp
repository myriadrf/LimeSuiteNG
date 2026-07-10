/*
 * LimeSuiteNG public C API -- transitional implementation.
 *
 * A thin extern "C" wrapper over the C++ core. During the transition the
 * generic device tree collapses onto SDRDevice (the only device class so
 * far): lime_device and lime_SDRDevice are the same underlying pointer.
 * Later phases introduce the real generic device / RFE / Utility split.
 */
#include "limesuiteng/c/limesuiteng.h"

#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/RFStream.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/StreamMeta.h"
#include "limesuiteng/types.h"
#include "limesuiteng/VersionInfo.h"

#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using lime::SDRDevice;

static inline SDRDevice* sdr(lime_SDRDevice* d) { return reinterpret_cast<SDRDevice*>(d); }
static inline lime::TRXDir dir(lime_TRXDir d) { return static_cast<lime::TRXDir>(d != lime_TRXDir_Rx); }
static inline const lime::SDRDescriptor* desc(const lime_SDRDescriptor* d)
{
    return reinterpret_cast<const lime::SDRDescriptor*>(d);
}

/* The stream handle is a real object: it owns the RFStream and remembers the
 * application sample format so _recv / _send can dispatch the typed overload. */
struct lime_Stream {
    std::unique_ptr<lime::RFStream> impl;
    lime::DataFormat format;
};

extern "C" {

int lime_enumerate(lime_DeviceHandle* out, size_t max)
{
    const std::vector<lime::DeviceHandle> handles = lime::DeviceRegistry::enumerate();
    if (out != nullptr)
    {
        const size_t n = handles.size() < max ? handles.size() : max;
        for (size_t i = 0; i < n; ++i)
        {
            const std::string s = handles[i].Serialize();
            std::strncpy(out[i].str, s.c_str(), sizeof(out[i].str) - 1);
            out[i].str[sizeof(out[i].str) - 1] = '\0';
        }
    }
    return static_cast<int>(handles.size());
}

lime_device* lime_device_open(const lime_DeviceHandle* handle)
{
    if (handle == nullptr)
        return nullptr;
    const lime::DeviceHandle h{ std::string(handle->str) };
    return reinterpret_cast<lime_device*>(lime::DeviceRegistry::makeDevice(h));
}

void lime_device_close(lime_device* dev)
{
    if (dev != nullptr)
        lime::DeviceRegistry::freeDevice(reinterpret_cast<SDRDevice*>(dev));
}

/* Transitional: the only device kind is an SDR, and it is the same pointer. */
lime_Kind lime_device_kind(lime_device*) { return lime_Kind_SDR; }
size_t lime_device_child_count(lime_device*) { return 0; }
lime_device* lime_device_child(lime_device*, size_t) { return nullptr; }
lime_SDRDevice* lime_device_as_sdr(lime_device* dev) { return reinterpret_cast<lime_SDRDevice*>(dev); }

/* The public C surface takes 32-bit indices so the ABI never has to widen;
 * the internal C++ addressing is currently 8-bit, so out-of-range values are
 * rejected here rather than silently truncated. */
static inline bool narrows(uint32_t module, uint32_t channel)
{
    return module > std::numeric_limits<uint8_t>::max() || channel > std::numeric_limits<uint8_t>::max();
}

lime_OpStatus lime_sdrdevice_set_frequency(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel, double hz)
{
    if (dev == nullptr || narrows(module, channel))
        return lime_OpStatus_InvalidValue;
    return static_cast<lime_OpStatus>(
        sdr(dev)->SetFrequency(static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel), hz));
}

lime_OpStatus lime_sdrdevice_enable_channel(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel, bool enable)
{
    if (dev == nullptr || narrows(module, channel))
        return lime_OpStatus_InvalidValue;
    return static_cast<lime_OpStatus>(
        sdr(dev)->EnableChannel(static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel), enable));
}

lime_OpStatus lime_sdrdevice_set_antenna(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel, const char* name)
{
    if (dev == nullptr || name == nullptr || narrows(module, channel))
        return lime_OpStatus_InvalidValue;
    const lime::ChannelId c{ static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel) };
    return static_cast<lime_OpStatus>(sdr(dev)->SetAntenna(c, std::string(name)));
}

const lime_SDRDescriptor* lime_sdrdevice_get_descriptor(lime_SDRDevice* dev)
{
    if (dev == nullptr)
        return nullptr;
    return reinterpret_cast<const lime_SDRDescriptor*>(&sdr(dev)->GetDescriptor());
}

/* ------------------------------------------------------------------ */
/* Descriptor accessors                                                */
/* ------------------------------------------------------------------ */

const char* lime_descriptor_name(const lime_SDRDescriptor* d)
{
    return d != nullptr ? desc(d)->name.c_str() : nullptr;
}

uint64_t lime_descriptor_serial(const lime_SDRDescriptor* d)
{
    return d != nullptr ? desc(d)->serialNumber : 0;
}

size_t lime_descriptor_rfsoc_count(const lime_SDRDescriptor* d)
{
    return d != nullptr ? desc(d)->rfSOC.size() : 0;
}

const char* lime_descriptor_rfsoc_name(const lime_SDRDescriptor* d, size_t soc)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return nullptr;
    return desc(d)->rfSOC[soc].name.c_str();
}

uint32_t lime_descriptor_channel_count(const lime_SDRDescriptor* d, size_t soc)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return 0;
    return desc(d)->rfSOC[soc].channelCount;
}

size_t lime_descriptor_antenna_count(const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dr)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return 0;
    const auto& paths = desc(d)->rfSOC[soc].pathNames;
    const auto it = paths.find(dir(dr));
    return it != paths.end() ? it->second.size() : 0;
}

const char* lime_descriptor_antenna_name(const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dr, size_t index)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return nullptr;
    const auto& paths = desc(d)->rfSOC[soc].pathNames;
    const auto it = paths.find(dir(dr));
    if (it == paths.end() || index >= it->second.size())
        return nullptr;
    return it->second[index].c_str();
}

/* ------------------------------------------------------------------ */
/* Streaming                                                           */
/* ------------------------------------------------------------------ */

lime_Stream* lime_stream_create(lime_SDRDevice* dev, const lime_StreamConfig* cfg)
{
    if (dev == nullptr || cfg == nullptr || cfg->struct_size < sizeof(lime_StreamConfig))
        return nullptr;
    if ((cfg->rx_count > 0 && cfg->rx_channels == nullptr) || (cfg->tx_count > 0 && cfg->tx_channels == nullptr))
        return nullptr;

    lime::StreamConfig sc;
    for (size_t i = 0; i < cfg->rx_count; ++i)
    {
        if (cfg->rx_channels[i] > std::numeric_limits<uint8_t>::max())
            return nullptr;
        sc.channels[lime::TRXDir::Rx].push_back(static_cast<uint8_t>(cfg->rx_channels[i]));
    }
    for (size_t i = 0; i < cfg->tx_count; ++i)
    {
        if (cfg->tx_channels[i] > std::numeric_limits<uint8_t>::max())
            return nullptr;
        sc.channels[lime::TRXDir::Tx].push_back(static_cast<uint8_t>(cfg->tx_channels[i]));
    }
    sc.format = static_cast<lime::DataFormat>(cfg->format);
    sc.linkFormat = static_cast<lime::DataFormat>(cfg->link_format);
    sc.hintSampleRate = cfg->hint_sample_rate_hz;

    std::unique_ptr<lime::RFStream> stream = sdr(dev)->StreamCreate(sc, cfg->module);
    if (!stream)
        return nullptr;
    return new lime_Stream{ std::move(stream), sc.format };
}

lime_OpStatus lime_stream_start(lime_Stream* s)
{
    if (s == nullptr)
        return lime_OpStatus_InvalidValue;
    return static_cast<lime_OpStatus>(s->impl->Start());
}

void lime_stream_stop(lime_Stream* s)
{
    if (s != nullptr)
        s->impl->Stop();
}

int lime_stream_recv(lime_Stream* s, void* const* dst, size_t count, lime_StreamRxMeta* meta, uint32_t timeout_ms)
{
    if (s == nullptr || dst == nullptr || count > std::numeric_limits<uint32_t>::max())
        return lime_OpStatus_InvalidValue;

    lime::StreamRxMeta rxmeta;
    const std::chrono::microseconds timeout{ std::chrono::milliseconds(timeout_ms) };
    const uint32_t n = static_cast<uint32_t>(count);
    uint32_t transferred = 0;
    switch (s->format)
    {
    case lime::DataFormat::F32:
        transferred = s->impl->Receive(reinterpret_cast<lime::complex32f_t* const*>(dst), n, &rxmeta, timeout);
        break;
    case lime::DataFormat::I16:
        transferred = s->impl->Receive(reinterpret_cast<lime::complex16_t* const*>(dst), n, &rxmeta, timeout);
        break;
    case lime::DataFormat::I12:
        transferred = s->impl->Receive(reinterpret_cast<lime::complex12_t* const*>(dst), n, &rxmeta, timeout);
        break;
    default:
        return lime_OpStatus_InvalidValue;
    }
    if (meta != nullptr)
    {
        meta->timestamp = static_cast<uint64_t>(rxmeta.timestamp.GetTicks());
        meta->has_timestamp = rxmeta.hasTimestamp;
    }
    return static_cast<int>(transferred);
}

int lime_stream_send(lime_Stream* s, const void* const* src, size_t count, const lime_StreamTxMeta* meta, uint32_t timeout_ms)
{
    if (s == nullptr || src == nullptr || count > std::numeric_limits<uint32_t>::max())
        return lime_OpStatus_InvalidValue;

    lime::StreamTxMeta txmeta;
    txmeta.hasTimestamp = meta != nullptr ? meta->has_timestamp : false;
    txmeta.flags = (meta != nullptr && meta->flush) ? lime::StreamTxMeta::EndOfBurst : 0;
    if (txmeta.hasTimestamp)
        txmeta.timestamp = lime::Timespec(static_cast<int64_t>(meta->timestamp));

    const std::chrono::microseconds timeout{ std::chrono::milliseconds(timeout_ms) };
    const uint32_t n = static_cast<uint32_t>(count);
    uint32_t transferred = 0;
    switch (s->format)
    {
    case lime::DataFormat::F32:
        transferred = s->impl->Transmit(reinterpret_cast<const lime::complex32f_t* const*>(src), n, &txmeta, timeout);
        break;
    case lime::DataFormat::I16:
        transferred = s->impl->Transmit(reinterpret_cast<const lime::complex16_t* const*>(src), n, &txmeta, timeout);
        break;
    case lime::DataFormat::I12:
        transferred = s->impl->Transmit(reinterpret_cast<const lime::complex12_t* const*>(src), n, &txmeta, timeout);
        break;
    default:
        return lime_OpStatus_InvalidValue;
    }
    return static_cast<int>(transferred);
}

void lime_stream_destroy(lime_Stream* s)
{
    delete s; /* RFStream teardown runs in its destructor */
}

/* Transitional: GPIO capability lives on SDRDevice; the handle is the device. */
lime_GPIO* lime_sdrdevice_get_gpio(lime_SDRDevice* dev) { return reinterpret_cast<lime_GPIO*>(dev); }

lime_OpStatus lime_gpio_set_value(lime_GPIO* gpio, uint32_t pin, bool value)
{
    if (gpio == nullptr || pin >= 8)
        return lime_OpStatus_InvalidValue;
    SDRDevice* dev = reinterpret_cast<SDRDevice*>(gpio);
    uint8_t reg = 0;
    if (dev->GPIORead(&reg, 1) != lime::OpStatus::Success)
        return lime_OpStatus_IOFailure;
    reg = value ? static_cast<uint8_t>(reg | (1u << pin)) : static_cast<uint8_t>(reg & ~(1u << pin));
    return static_cast<lime_OpStatus>(dev->GPIOWrite(&reg, 1));
}

lime_OpStatus lime_gpio_get_value(lime_GPIO* gpio, uint32_t pin, bool* value)
{
    if (gpio == nullptr || value == nullptr || pin >= 8)
        return lime_OpStatus_InvalidValue;
    SDRDevice* dev = reinterpret_cast<SDRDevice*>(gpio);
    uint8_t reg = 0;
    const lime::OpStatus st = dev->GPIORead(&reg, 1);
    if (st != lime::OpStatus::Success)
        return static_cast<lime_OpStatus>(st);
    *value = (reg >> pin) & 1u;
    return lime_OpStatus_Success;
}

/* Transitional: SPI capability lives on SDRDevice; the handle is the device. */
lime_SPI* lime_sdrdevice_get_spi(lime_SDRDevice* dev) { return reinterpret_cast<lime_SPI*>(dev); }

lime_OpStatus lime_spi_transact(lime_SPI* spi, uint32_t bus_address, const uint32_t* mosi, uint32_t* miso, uint32_t count)
{
    if (spi == nullptr || (mosi == nullptr && miso == nullptr) || count == 0)
        return lime_OpStatus_InvalidValue;
    SDRDevice* dev = reinterpret_cast<SDRDevice*>(spi);
    return static_cast<lime_OpStatus>(dev->SPI(bus_address, mosi, miso, count));
}

const char* lime_get_library_version(void)
{
    static const std::string v = lime::GetLibraryVersion();
    return v.c_str();
}
const char* lime_get_api_version(void)
{
    static const std::string v = lime::GetAPIVersion();
    return v.c_str();
}
const char* lime_get_abi_version(void)
{
    static const std::string v = lime::GetABIVersion();
    return v.c_str();
}

} /* extern "C" */
