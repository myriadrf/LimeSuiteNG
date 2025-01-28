/* -*- c++ -*- */
/*
 * Copyright 2024 Lime Microsystems Lts <info@limemicro.com>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIMESUITENG_SDRDEVICE_MANAGER_H
#define INCLUDED_LIMESUITENG_SDRDEVICE_MANAGER_H

#include <gnuradio/logger.h>

#include <list>
#include <memory>

#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/RFStream.h"
#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"

namespace gr {
namespace limesuiteng {

struct sdrdevice_context {
    std::unique_ptr<lime::SDRDevice> device;
    lime::DeviceHandle handle;
    lime::SDRConfig deviceConfig;
    std::unique_ptr<lime::RFStream> stream;
    lime::StreamConfig streamCfg;
    int sourceBlockCounter{ 0 };
    int sinkBlockCounter{ 0 };
    bool sinkConfigReady{ false };
    bool sourceConfigReady{ false };
    std::string customBaseConfigFilepath;
};

/// @brief class managing SDR devices and acting as intermediary between Sink/Source
/// blocks Sink/Source blocks have parameters dependencies between them, so they cannot
/// explicitly own and configure the sdr device on their own, needs to collect all
/// settings in one place to properly configure all parameters before graph start.
class sdrdevice_manager
{
public:
    static std::shared_ptr<sdrdevice_manager> GetSingleton();
    std::shared_ptr<sdrdevice_context>
    GetDeviceContextByHandle(const std::string& deviceHandleHint);

    ~sdrdevice_manager();
    sdrdevice_manager();

private:
    bool GetDeviceFullHandle(const std::string_view hintArguments,
                             lime::DeviceHandle& handleOutput);

    std::list<std::shared_ptr<sdrdevice_context>> m_contexts;
    std::vector<lime::DeviceHandle> enumeratedHandles;
    gr::logger _logger;
};

} // namespace limesuiteng
} // namespace gr

#endif /* INCLUDED_LIMESUITENG_SDRDEVICE_MANAGER_H */
