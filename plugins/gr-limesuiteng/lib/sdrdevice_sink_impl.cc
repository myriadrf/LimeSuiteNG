/* -*- c++ -*- */
/*
 * Copyright 2024 Lime Microsystems Lts <info@limemicro.com>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sdrdevice_sink_impl.h"
#include <gnuradio/io_signature.h>

#include "sdrdevice_manager.h"

#include <chrono>
#include <limits>
#include <sstream>
#include <thread>

#include "limesuiteng/Logger.h"
#include "limesuiteng/RFSOCDescriptor.h"
#include "limesuiteng/RFStream.h"
#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/StreamMeta.h"
#include "limesuiteng/complex.h"

using namespace lime;

namespace gr {
namespace limesuiteng {

sdrdevice_sink::sptr sdrdevice_sink::make(const std::string& alias,
                                          const std::string& deviceHandleHint,
                                          uint32_t chipIndex,
                                          const std::vector<int>& channelIndexes,
                                          const std::string& dataFormat,
                                          const std::string& linkFormat,
                                          double sampleRate,
                                          int rf_oversampling)
{
    return gnuradio::make_block_sptr<sdrdevice_sink_impl>(alias,
                                                          deviceHandleHint,
                                                          chipIndex,
                                                          channelIndexes,
                                                          dataFormat,
                                                          linkFormat,
                                                          sampleRate,
                                                          rf_oversampling);
}

sdrdevice_sink_impl::sdrdevice_sink_impl(const std::string& alias,
                                         const std::string& deviceHandleHint,
                                         uint32_t chipIndex,
                                         const std::vector<int>& channelIndexes,
                                         const std::string& dataFormat,
                                         const std::string& linkFormat,
                                         double sampleRate,
                                         int rf_oversampling)
    : gr::sync_block((alias.empty()
                          ? fmt::format("sdrdevice_sink[{:s}]", deviceHandleHint.c_str())
                          : alias),
                     gr::io_signature::make(1 /* min outputs */,
                                            channelIndexes.size() /*max outputs */,
                                            (dataFormat == "complex16_t")
                                                ? sizeof(lime::complex16_t)
                                                : sizeof(lime::complex32f_t)),
                     gr::io_signature::make(0, 0, 0)),
      sdrdevice_block_base(TRXDir::Tx,
                           alias,
                           deviceHandleHint,
                           chipIndex,
                           channelIndexes,
                           dataFormat,
                           linkFormat,
                           sampleRate,
                           rf_oversampling,
                           d_logger,
                           d_debug_logger)
{
    // GNU radio can start feeding Sink Work() with 1 sample chunks, somehow that makes
    // the gnu radio boost thread to die. Setting the output granularity seems to
    // workaround that.
}

sdrdevice_sink_impl::~sdrdevice_sink_impl() { GR_LOG_DEBUG(d_debug_logger, __func__); }

bool sdrdevice_sink_impl::start() { return sdrdevice_block_base::start(); }

bool sdrdevice_sink_impl::stop() { return sdrdevice_block_base::stop(); }

int sdrdevice_sink_impl::work(int noutput_items,
                              gr_vector_const_void_star& input_items,
                              gr_vector_void_star& output_items)
{
    if (!canWork) {
        GR_LOG_DEBUG(d_debug_logger, "WORK_DONE");
        return gr::block::work_return_t::WORK_DONE;
    }

    assert(devContext);
    assert(devContext->stream);

    // start actual data streaming only when work starts, stream is shared by Rx/Tx
    // so start should be done once from either of them.
    if (!devContext->streamIsActive.exchange(true)) {
        StartRFStreaming();
    }

    StreamTxMeta meta;
    meta.timestamp = lime::Timespec(0l);
    meta.hasTimestamp = false;
    meta.flags = 0; // StreamTxMeta::EndOfBurst;
    int samplesSent;
    const size_t chCount = devContext->streamCfg.channels.at(direction).size();

    if (devContext->streamCfg.format == lime::DataFormat::I16) {
        const lime::complex16_t* samples[8];
        for (size_t i = 0; i < chCount; ++i)
            samples[i] = static_cast<const lime::complex16_t*>(input_items[i]);
        samplesSent = devContext->stream->Transmit(&samples[0], noutput_items, &meta);
    } else {
        const lime::complex32f_t* samples[8];
        for (size_t i = 0; i < chCount; ++i)
            samples[i] = static_cast<const lime::complex32f_t*>(input_items[i]);
        samplesSent = devContext->stream->Transmit(&samples[0], noutput_items, &meta);
    }

    if (samplesSent != noutput_items)
        GR_LOG_WARN(d_logger,
                    fmt::format("StreamTx {:d}/{:d}", samplesSent, noutput_items));

    return samplesSent;
}

void sdrdevice_sink_impl::set_config_file(const std::string& file_path)
{
    sdrdevice_block_base::set_config_file(file_path);
}

double sdrdevice_sink_impl::set_lo_frequency(double frequencyHz)
{
    return sdrdevice_block_base::set_lo_frequency(frequencyHz);
}

double sdrdevice_sink_impl::set_lpf_bandwidth(double bandwidthHz)
{
    return sdrdevice_block_base::set_lpf_bandwidth(bandwidthHz);
}

double sdrdevice_sink_impl::set_gfir_bandwidth(double bandwidthHz)
{
    return sdrdevice_block_base::set_gfir_bandwidth(bandwidthHz);
}

bool sdrdevice_sink_impl::set_antenna(const std::string& antenna_name)
{
    return sdrdevice_block_base::set_antenna(antenna_name);
}

double sdrdevice_sink_impl::set_gain_generic(double gain_dB)
{
    return sdrdevice_block_base::set_gain_generic(gain_dB);
}

double sdrdevice_sink_impl::set_nco_frequency(double frequency_offset_Hz)
{
    return sdrdevice_block_base::set_nco_frequency(frequency_offset_Hz);
}

void sdrdevice_sink_impl::set_calibration_enable(int flags)
{
    sdrdevice_block_base::set_calibration_enable(flags);
}

} /* namespace limesuiteng */
} /* namespace gr */
