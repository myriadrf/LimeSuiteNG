/* -*- c++ -*- */
/*
 * Copyright 2024 Lime Microsystems Lts <info@limemicro.com>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sdrdevice_source_impl.h"
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
#include "limesuiteng/complex.h"

using namespace lime;

namespace gr {
namespace limesuiteng {

static int GetDataFormatTypeSize(const std::string& formatname)
{
    if (formatname == "complex16_t")
        return sizeof(lime::complex16_t);
    else if (formatname == "complex12_t")
        return sizeof(lime::complex12_t);
    else if (formatname == "complex32f_t")
        return sizeof(lime::complex32f_t);
    else
        return sizeof(lime::complex32f_t);
}

sdrdevice_source::sptr sdrdevice_source::make(const std::string& alias,
                                              const std::string& deviceHandleHint,
                                              uint32_t chipIndex,
                                              const std::vector<int>& channelIndexes,
                                              const std::string& dataFormat,
                                              double sampleRate,
                                              int rf_oversampling)
{
    return gnuradio::make_block_sptr<sdrdevice_source_impl>(alias,
                                                            deviceHandleHint,
                                                            chipIndex,
                                                            channelIndexes,
                                                            dataFormat,
                                                            sampleRate,
                                                            rf_oversampling);
}

sdrdevice_source_impl::sdrdevice_source_impl(const std::string& alias,
                                             const std::string& deviceHandleHint,
                                             uint32_t chipIndex,
                                             const std::vector<int>& channelIndexes,
                                             const std::string& dataFormat,
                                             double sampleRate,
                                             int rf_oversampling)
    : gr::sync_block(
          (alias.empty() ? fmt::format("sdrdevice_source[{:s}]", deviceHandleHint.c_str())
                         : alias),
          gr::io_signature::make(0, 0, 0),
          gr::io_signature::make(1 /* min outputs */,
                                 channelIndexes.size() /*max outputs */,
                                 GetDataFormatTypeSize(dataFormat))),
      sdrdevice_block_base(TRXDir::Rx,
                           alias,
                           deviceHandleHint,
                           chipIndex,
                           channelIndexes,
                           dataFormat,
                           sampleRate,
                           rf_oversampling,
                           d_logger,
                           d_logger)
{
}

sdrdevice_source_impl::~sdrdevice_source_impl() { GR_LOG_DEBUG(d_logger, __func__); }

bool sdrdevice_source_impl::start() { return sdrdevice_block_base::start(); }

bool sdrdevice_source_impl::stop() { return sdrdevice_block_base::stop(); }

int sdrdevice_source_impl::work(int noutput_items,
                                gr_vector_const_void_star& input_items,
                                gr_vector_void_star& output_items)
{
    if (!canWork) {
        GR_LOG_DEBUG(d_logger, "WORK_DONE");
        return gr::block::work_return_t::WORK_DONE;
    }

    assert(devContext);
    assert(devContext->stream);

    lime::complex32f_t* samples[8];
    for (size_t i = 0; i < devContext->streamCfg.channels.at(direction).size(); ++i)
        samples[i] = static_cast<lime::complex32f_t*>(output_items[i]);

    StreamMeta meta;
    int samplesRead = devContext->stream->StreamRx(
        &samples[0], noutput_items, &meta, std::chrono::microseconds(1000000));

    if (samplesRead != noutput_items)
        GR_LOG_WARN(d_logger,
                    fmt::format("StreamRx {:d}/{:d}", samplesRead / noutput_items));

    return samplesRead;
}

void sdrdevice_source_impl::set_config_file(const std::string& file_path)
{
    sdrdevice_block_base::set_config_file(file_path);
}

double sdrdevice_source_impl::set_lo_frequency(double frequencyHz)
{
    return sdrdevice_block_base::set_lo_frequency(frequencyHz);
}

double sdrdevice_source_impl::set_lpf_bandwidth(double bandwidthHz)
{
    return sdrdevice_block_base::set_lpf_bandwidth(bandwidthHz);
}

bool sdrdevice_source_impl::set_antenna(const std::string& antenna_name)
{
    return sdrdevice_block_base::set_antenna(antenna_name);
}

double sdrdevice_source_impl::set_gain_generic(double gain_dB)
{
    return sdrdevice_block_base::set_gain_generic(gain_dB);
}

double sdrdevice_source_impl::set_nco_frequency(double frequency_offset_Hz)
{
    return sdrdevice_block_base::set_nco_frequency(frequency_offset_Hz);
}

} /* namespace limesuiteng */
} /* namespace gr */
