#include "BufferInterleaving.h"

#include "FPGA_common.h"
#include "samplesConversion.h"

namespace lime {

template<class SrcT, class DestT>
static int DeinterleaveMIMO(DestT* const* dest, const std::byte* buffer, uint32_t length, const DataConversion& fmt)
{
    int samplesProduced = length / sizeof(SrcT);
    const bool mimo = fmt.channelCount > 1;
    if (!mimo)
        PathSelection(dest[0], reinterpret_cast<const SrcT*>(buffer), length / sizeof(SrcT));
    else
    {
        PathSelectionUnzip(dest[0], dest[1], reinterpret_cast<const SrcT*>(buffer), length / sizeof(SrcT));
        samplesProduced /= 2;
    }
    return samplesProduced;
}

template<class DestT>
static int DeinterleaveCompressionType(DestT* const* dest, const std::byte* buffer, uint32_t length, const DataConversion& fmt)
{
    const bool compressed = fmt.srcFormat == DataFormat::I12;
    if (!compressed)
        return DeinterleaveMIMO<complex16_t>(dest, buffer, length, fmt);
    else
        return DeinterleaveMIMO<complex12packed_t>(dest, buffer, length, fmt);
}

int Deinterleave(void* const* dest, const std::byte* buffer, uint32_t length, const DataConversion& fmt)
{
    int samplesProduced;
    switch (fmt.destFormat)
    {
    default:
    case DataFormat::I16:
        samplesProduced =
            DeinterleaveCompressionType<complex16_t>(reinterpret_cast<complex16_t* const*>(dest), buffer, length, fmt);
        break;
    case DataFormat::F32:
        samplesProduced =
            DeinterleaveCompressionType<complex32f_t>(reinterpret_cast<complex32f_t* const*>(dest), buffer, length, fmt);
        break;
    case DataFormat::I12:
        samplesProduced =
            DeinterleaveCompressionType<complex12_t>(reinterpret_cast<complex12_t* const*>(dest), buffer, length, fmt);
        break;
    }
    return samplesProduced;
}

template<class DestT, class SrcT>
static int InterleaveMIMO(std::byte* buffer, const SrcT* const* input, uint32_t count, const DataConversion& fmt)
{
    DestT* dest = reinterpret_cast<DestT*>(buffer);
    int bytesProduced = count * sizeof(DestT);
    const bool mimo = fmt.channelCount > 1;
    if (!mimo)
        PathSelection(dest, input[0], count);
    else
    {
        PathSelectionZip(dest, input[0], input[1], count);
        bytesProduced *= 2;
    }
    return bytesProduced;
}

template<class SrcT>
static int InterleaveCompressionType(std::byte* dest, const SrcT* const* src, uint32_t count, const DataConversion& fmt)
{
    const bool compressed = fmt.destFormat == DataFormat::I12;
    if (!compressed)
        return InterleaveMIMO<complex16_t>(dest, src, count, fmt);
    else
        return InterleaveMIMO<complex12packed_t>(dest, src, count, fmt);
}

int Interleave(std::byte* dest, const void* const* src, uint32_t count, const DataConversion& fmt)
{
    int bytesProduced;
    switch (fmt.srcFormat)
    {
    default:
    case DataFormat::I16:
        bytesProduced = InterleaveCompressionType<complex16_t>(dest, reinterpret_cast<const complex16_t* const*>(src), count, fmt);
        break;
    case DataFormat::F32:
        bytesProduced =
            InterleaveCompressionType<complex32f_t>(dest, reinterpret_cast<const complex32f_t* const*>(src), count, fmt);
        break;
    case DataFormat::I12:
        bytesProduced = InterleaveCompressionType<complex12_t>(dest, reinterpret_cast<const complex12_t* const*>(src), count, fmt);
        break;
    }
    return bytesProduced;
}

} // namespace lime
