/* C wrapper over lime::RFStream: stream lifecycle and sample transfer. */
#include "limesuiteng/rfstream_c.h"
#include "private.h"

#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/StreamMeta.h"

#include <chrono>

extern "C" {

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

    if (cfg->module > std::numeric_limits<uint8_t>::max())
        return nullptr;
    std::unique_ptr<lime::RFStream> stream = sdr(dev)->StreamCreate(sc, static_cast<uint8_t>(cfg->module));
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

} /* extern "C" */
