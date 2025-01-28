/* -*- c++ -*- */
/*
 * Copyright 2024 Lime Microsystems Lts <info@limemicro.com>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIMESUITENG_SDRDEVICE_SOURCE_IMPL_H
#define INCLUDED_LIMESUITENG_SDRDEVICE_SOURCE_IMPL_H

#include <gnuradio/limesuiteng/sdrdevice_source.h>

#include <memory>

#include "limesuiteng/types.h"
#include "sdrdevice_block_base.h"

namespace gr {
namespace limesuiteng {

class sdrdevice_manager;
struct sdrdevice_context;

class sdrdevice_source_impl : public sdrdevice_source, public sdrdevice_block_base
{
public:
    sdrdevice_source_impl(const std::string& alias,
                          const std::string& deviceHandleHint,
                          uint32_t chipIndex,
                          const std::vector<int>& channelIndexes,
                          const std::string& dataFormat,
                          double sampleRate,
                          int rf_oversampling);
    virtual ~sdrdevice_source_impl();

    bool start() override;
    bool stop() override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

    void set_config_file(const std::string& file_path) override;
    double set_lo_frequency(double frequencyHz) override;
    double set_lpf_bandwidth(double bandwidthHz) override;
    bool set_antenna(const std::string& antenna_name) override;
    double set_gain_generic(double gain_dB) override;
    double set_nco_frequency(double frequency_offset_Hz) override;
};

} // namespace limesuiteng
} // namespace gr

#endif /* INCLUDED_LIMESUITENG_SDRDEVICE_SOURCE_IMPL_H */
