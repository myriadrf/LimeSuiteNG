/**
@file	Registration.cpp
@brief	Soapy SDR + IConnection module registration.
@author Lime Microsystems (www.limemicro.com)
*/

#include "Soapy_limesuiteng.h"
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

#include "limesuiteng/Logger.h"
#include "limesuiteng/DeviceHandle.h"

using namespace lime;

static void limeSuiteLogHandler(const LogLevel level, const std::string& message)
{
    switch (level)
    {
    case LogLevel::Critical:
        SoapySDR::log(SOAPY_SDR_CRITICAL, message);
        return;
    case LogLevel::Error:
        SoapySDR::log(SOAPY_SDR_ERROR, message);
        return;
    case LogLevel::Warning:
        SoapySDR::log(SOAPY_SDR_WARNING, message);
        return;
    case LogLevel::Info:
    case LogLevel::Verbose:
        SoapySDR::log(SOAPY_SDR_INFO, message);
        return;
    case LogLevel::Debug:
        SoapySDR::log(SOAPY_SDR_DEBUG, message);
        return;
    }
}

static DeviceHandle argsToHandle(const SoapySDR::Kwargs& args)
{
    DeviceHandle handle;

    // Load handle with key/value provided options
    if (args.count("media") != 0)
    {
        handle.media = args.at("media");
    }

    if (args.count("name") != 0)
    {
        handle.name = args.at("name");
    }

    if (args.count("addr") != 0)
    {
        handle.addr = args.at("addr");
    }

    if (args.count("serial") != 0)
    {
        handle.serial = args.at("serial");
    }

    return handle;
}

static SoapySDR::Kwargs handleToArgs(const DeviceHandle& handle)
{
    SoapySDR::Kwargs args;

    // Convert the handle into key/value pairs
    if (not handle.media.empty())
    {
        args["media"] = handle.media;
    }

    if (not handle.name.empty())
    {
        args["name"] = handle.name;
    }

    if (not handle.addr.empty())
    {
        args["addr"] = handle.addr;
    }

    if (not handle.serial.empty())
    {
        args["serial"] = handle.serial;
    }

    // Label connection for drop-downs and lists
    args["label"] = handle.ToString();

    return args;
}

static SoapySDR::KwargsList findIConnection(const SoapySDR::Kwargs& matchArgs)
{
    registerLogHandler(&limeSuiteLogHandler);
    SoapySDR::KwargsList results;

    for (const auto& handle : DeviceRegistry::enumerate(argsToHandle(matchArgs)))
    {
        results.push_back(handleToArgs(handle));
    }

    return results;
}

static SoapySDR::Device* makeIConnection(const SoapySDR::Kwargs& args)
{
    registerLogHandler(&limeSuiteLogHandler);
    return new Soapy_limesuiteng(argsToHandle(args), args);
}

static SoapySDR::Registry registerIConnection("limesuiteng", &findIConnection, &makeIConnection, SOAPY_SDR_ABI_VERSION);
