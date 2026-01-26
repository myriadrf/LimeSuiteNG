#ifndef LIMESUITENG_HPP
#define LIMESUITENG_HPP

#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/RFSOCDescriptor.h"
#include "limesuiteng/RFStream.h"
#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/VersionInfo.h"

#endif

/**
 * @mainpage LimeSuiteNG 1.0.0 library manual
 *
 * @section intro Introduction
 *
 * LimeSuiteNG library is a component of LimeSuiteNG suite, which enables communication 
 * with LimeSDR devices from user space. This documentation focuses on LimeSuiteNG library 
 * public API, provides other important notes, explanations and is intended for application 
 * developers, who wish to communicate with LimeSDR devices using custom built applications.
 *
 * This documentation does not provide library installation steps. For more information about 
 * the library installation for different platforms and other components of LimeSuiteNG suite, 
 * visit the main <A HREF="https://limesuiteng.myriadrf.org/">  LimeSuiteNG documentation</A> page.
 * 
 * 
 * @section lib_feat_section Library features
 * 
 * @if DEVELOP
 * TODO: Update the list of supported library features.  
 * @endif
 * 
 * <ul>
 *    <li>Quick SDR device configuration (from structure/from file)</li>
 *    <li>Advanced SDR device configuration from file/using registers</li>
 * </ul>
 * 
 * 
 * 
 * @section getting_started Getting started
 * 
 * If you are completely new to the LimeSDR device software environment or looking for information about SDR device control and configuration, it is recommended to visit
 * @ref guide_book_top "library API guide book page". Library Guide book page provides code examples, in depth explanations and other information about different library API members.
 * If you only need to reference library API member list, then @ref api "check out the sorted and ordered list of public API members".
 * 
 */

 
/**
 * @page api Application Programming Interface
 * 
 * This page provides a complete list of functions, enumerations, structures that can be accessed 
 * through public API.
 * 
 * @if RST_SUPPORT
 * @embedRstVerbatim{
 * .. admonition:: Warning
 * 
 *    All public API members are part of namespace ``lime``. To access individual API members, use scope resolution operator ``lime::function()``. To access SDR device registration functions, additionally specify the scope of device registry interface class ``lime::DeviceRegistry::function()``.}
 * @else
 * @warning All public API members are part of namespace <b>lime</b>. To access individual API members, use scope resolution operator: <b>lime::function()</b>. To access SDR device registration functions,
 * additionally specify the scope of device registry interface class: <b>lime::DeviceRegistry::function()</b>.
 * @endif
 * 
 * @section funcs Functions 
 * 
 * @subsection registration SDR Device registration
 * <ul>
 *    <li> @ref lime::DeviceRegistry::enumerate() "enumerate()" </li>
 *    <li> @ref lime::DeviceRegistry::enumerate(const lime::DeviceHandle&) "enumerate(const DeviceHandle&)"</li>
 *    <li> @ref lime::DeviceRegistry::makeDevice(const lime::DeviceHandle&) "makeDevice()"</li>
 *    <li> @ref lime::DeviceRegistry::freeDevice(lime::SDRDevice*) "freeDevice()"</li>
 *    <li> @ref lime::DeviceRegistry::moduleNames(void) "moduleNames()"</li>
 * </ul>
 * 
 * @subsection control_adn_config SDR Device control and configuration
 * <ul>
 *    <li><b>Common device control and configuration options:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::Init() "Init()"</li>
 *          <li> @ref lime::SDRDevice::Reset() "Reset()"</li>
 *          <li> @ref lime::SDRDevice::EnableChannel(uint8_t, lime::TRXDir, uint8_t, bool) "EnableChannel()"</li>
 *          <li> @ref lime::SDRDevice::GetFrequency(uint8_t,lime::TRXDir,uint8_t) "GetFrequency()"</li>
 *          <li> @ref lime::SDRDevice::SetFrequency(uint8_t, lime::TRXDir, uint8_t, double) "SetFrequency()"</li>
 *          <li> @ref lime::SDRDevice::GetLowPassFilter(uint8_t, lime::TRXDir, uint8_t) "GetLowPassFilter()"</li>
 *          <li> @ref lime::SDRDevice::SetLowPassFilter(uint8_t, lime::TRXDir, uint8_t, double) "SetLowPassFilter()"</li>
 *          <li> @ref lime::SDRDevice::Calibrate(uint8_t, lime::TRXDir, uint8_t, double) "Calibrate()"</li>
 *          <li> @ref lime::SDRDevice::GetSampleRate(uint8_t, lime::TRXDir, uint8_t, uint32_t*) "GetSampleRate()"</li>
 *          <li> @ref lime::SDRDevice::SetSampleRate(uint8_t, lime::TRXDir, uint8_t, double, uint8_t) "SetSampleRate()"</li>
 *          <li> @ref lime::SDRDevice::GetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double&) "GetGain()"</li>
 *          <li> @ref lime::SDRDevice::SetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double) "SetGain()"</li>
 *          <li> @ref lime::SDRDevice::GetAntenna(uint8_t, lime::TRXDir, uint8_t) "GetAntenna()"</li>
 *          <li> @ref lime::SDRDevice::SetAntenna(uint8_t, lime::TRXDir, uint8_t, uint8_t) "SetAntenna()"</li>
 *          <li> @ref lime::SDRDevice::StreamCreate(const lime::StreamConfig&, uint8_t) "StreamCreate()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Quick configuration:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::Configure(const lime::SDRConfig&, uint8_t) "Configure()"</li>
 *          <li> @ref lime::SDRDevice::LoadConfig(uint8_t, const std::string&) "LoadConfig()"</li>
 *          <li> @ref lime::SDRDevice::SaveConfig(uint8_t, const std::string&) "SaveConfig()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Direct register and parameter access:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GetParameter(uint8_t, uint8_t, const std::string&) "GetParameter(const string& parameterKey)"</li>
 *          <li> @ref lime::SDRDevice::SetParameter(uint8_t, uint8_t, const std::string&, uint16_t) "SetParameter(const string& parameterKey, uint16_t value)"</li>
 *          <li> @ref lime::SDRDevice::GetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t) "GetParameter(uint16_t address, uint8_t msb, uint8_t lsb)"</li>
 *          <li> @ref lime::SDRDevice::SetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t, uint16_t) "SetParameter(uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)"</li>
 *          <li> @ref lime::SDRDevice::ReadRegister(uint8_t, unsigned int, bool) "ReadRegister()"</li>
 *          <li> @ref lime::SDRDevice::WriteRegister(uint8_t, unsigned int, unsigned int, bool) "WriteRegister()"</li>
 *       </ul>
 *    </li>
 *    <li> <b> FIR filter options:</b>
 *       <ul>
 *        <li> @ref lime::SDRDevice::ConfigureGFIR(uint8_t, lime::TRXDir, uint8_t, lime::ChannelConfig::Direction::GFIRFilter) "ConfigureGFIR()"</li>
 *        <li> @ref lime::SDRDevice::GetGFIRCoefficients(uint8_t, lime::TRXDir, uint8_t, uint8_t) "GetGFIRCoefficients()"</li>
 *        <li> @ref lime::SDRDevice::SetGFIRCoefficients(uint8_t, lime::TRXDir, uint8_t, uint8_t, std::vector<double>) "SetGFIRCoefficients()"</li>
 *        <li> @ref lime::SDRDevice::SetGFIR(uint8_t, lime::TRXDir, uint8_t, uint8_t, bool) "SetGFIR()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Additional frequency configuration:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GetClockFreq(uint8_t, uint8_t) "GetClockFreq()"</li>
 *          <li> @ref lime::SDRDevice::SetClockFreq(uint8_t,double,uint8_t) "SetClockFreq()"</li>
 *          <li> @ref lime::SDRDevice::GetNCOFrequency(uint8_t, lime::TRXDir, uint8_t, uint8_t, double&) "GetNCOFrequency()"</li>
 *          <li> @ref lime::SDRDevice::SetNCOFrequency(uint8_t, lime::TRXDir, uint8_t, uint8_t, double, double) "SetNCOFrequency()"</li>
 *          <li> @ref lime::SDRDevice::GetNCOOffset(uint8_t, lime::TRXDir, uint8_t) "GetNCOOffset()"</li>
 *          <li> @ref lime::SDRDevice::GetNCOIndex(uint8_t, lime::TRXDir, uint8_t) "GetNCOIndex()"</li>
 *          <li> @ref lime::SDRDevice::SetNCOIndex(uint8_t, lime::TRXDir, uint8_t, uint8_t, bool) "SetNCOIndex()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Signal processor corrector configuration:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GetDCOffsetMode(uint8_t, lime::TRXDir, uint8_t) "GetDCOffsetMode()"</li>
 *          <li> @ref lime::SDRDevice::SetDCOffsetMode(uint8_t, lime::TRXDir, uint8_t, bool) "SetDCOffsetMode()"</li>
 *          <li> @ref lime::SDRDevice::GetDCOffset(uint8_t, lime::TRXDir, uint8_t) "GetDCOffset()"</li>
 *          <li> @ref lime::SDRDevice::SetDCOffset(uint8_t, lime::TRXDir, uint8_t, const lime::complex64f_t&) "SetDCOffset()"</li>
 *          <li> @ref lime::SDRDevice::GetIQBalance(uint8_t, lime::TRXDir, uint8_t) "GetIQBalance()"</li>
 *          <li> @ref lime::SDRDevice::SetIQBalance(uint8_t, lime::TRXDir, uint8_t, const lime::complex64f_t&) "SetIQBalance()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Test signal configuration:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GetTestSignal(uint8_t, lime::TRXDir, uint8_t) "GetTestSignal()"</li>
 *          <li> @ref lime::SDRDevice::SetTestSignal(uint8_t, lime::TRXDir, uint8_t, lime::ChannelConfig::Direction::TestSignal, int16_t, int16_t) "SetTestSignal()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Low speed interfaces:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::SPI(uint32_t, const uint32_t*, uint32_t*, uint32_t) "SPI()"</li>
 *          <li> @ref lime::SDRDevice::I2CWrite(int, const uint8_t*, uint32_t) "I2CWrite()"</li>
 *          <li> @ref lime::SDRDevice::I2CRead(int, uint8_t*, uint32_t) "I2CRead()"</li>
 *       </ul>
 *    </li>
 *    <li><b>GPIO control</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GPIOWrite(const uint8_t*, const size_t) "GPIOWrite()"</li>
 *          <li> @ref lime::SDRDevice::GPIORead(uint8_t*, const size_t) "GPIORead()"</li>
 *          <li> @ref lime::SDRDevice::GPIODirWrite(const uint8_t*, const size_t) "GPIODirWrite()"</li>
 *          <li> @ref lime::SDRDevice::GPIODirRead(uint8_t*, const size_t) "GPIODirRead()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Utility:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::UploadTxWaveform(const lime::StreamConfig&, uint8_t, const void**, uint32_t) "UploadTxWaveform()"</li>
 *          <li> @ref lime::SDRDevice::EnableCache(bool) "EnableCache()"</li>
 *          <li> @ref lime::SDRDevice::Synchronize(bool) "Synchronize()"</li>
 *          <li> @ref lime::SDRDevice::GetHardwareTimestamp(uint8_t) "GetHardwareTimestamp()"</li>
 *          <li> @ref lime::SDRDevice::SetHardwareTimestamp(uint8_t, const uint64_t) "SetHardwareTimestamp()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Additional device info:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GetDescriptor() "GetDescriptor()"</li>
 *          <li> @ref lime::SDRDevice::GetGPSLock(lime::SDRDevice::GPS_Lock*) "GetGPSLock()"</li>
 *          <li> @ref lime::SDRDevice::GetCGENLocked(uint8_t) "GetCGENLocked()"</li>
 *          <li> @ref lime::SDRDevice::GetTemperature(uint8_t) "GetTemperature()"</li>
 *          <li> @ref lime::SDRDevice::GetSXLocked(uint8_t, lime::TRXDir) "GetSXLocked()"</li>
 *       </ul>
 *    </li>
 * @if SPECIAL_API
 *    <li><b>Special:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::SetMessageLogCallback(lime::SDRDevice::LogCallbackType) "SetMessageLogCallback()"</li>
 *          <li> @ref lime::SDRDevice::GetInternalChip(uint32_t) "GetInternalChip()"</li>
 *          <li> @ref lime::SDRDevice::UploadMemory(lime::eMemoryDevice, uint8_t, const char*, size_t, lime::SDRDevice::UploadMemoryCallback) "UploadMemory()"</li>
 *          <li> @ref lime::SDRDevice::MemoryWrite(std::shared_ptr<lime::DataStorage>, lime::Region, const void*) "MemoryWrite()"</li>
 *          <li> @ref lime::SDRDevice::MemoryRead(std::shared_ptr<lime::DataStorage>, lime::Region, void*) "MemoryRead()"</li>
 *          <li> @ref lime::SDRDevice::WriteSerialNumber(uint64_t) "WriteSerialNumber()"</li>
 *          <li> @ref lime::SDRDevice::GetGPIOControls() "GetGPIOControls()"</li>
 *       </ul>
 *    </li>
 * @endif
 * </ul>
 * 
 * @subsection stream_control SDR Device stream control
 * <ul>
 *    <li> @ref lime::RFStream::GetHardwareTimestamp() "GetHardwareTimestamp()"</li>
 *    <li> @ref lime::RFStream::Setup(const lime::StreamConfig&) "Setup()"</li>
 *    <li> @ref lime::RFStream::GetConfig() "GetConfig()"</li>
 *    <li> @ref lime::RFStream::Start() "Start()"</li>
 *    <li> @ref lime::RFStream::StageStart() "StageStart()"</li>
 *    <li> @ref lime::RFStream::Stop() "Stop()"</li>
 *    <li> @ref lime::RFStream::Teardown() "Teardown()"</li>
 *    <li> @ref lime::RFStream::StreamStatus(lime::StreamStats*, lime::StreamStats*) "StreamStatus()"</li>
 *    <li> @ref lime::RFStream::Receive(lime::complex32f_t* const*, uint32_t , lime::StreamRxMeta*) "Receive(complex32f_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Receive(lime::complex16_t* const*, uint32_t , lime::StreamRxMeta*) "Receive(complex16_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Receive(lime::complex12_t* const*, uint32_t , lime::StreamRxMeta*) "Receive(complex12_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Transmit(const lime::complex32f_t* const*, uint32_t, const lime::StreamTxMeta*) "Transmit(complex32f_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Transmit(const lime::complex16_t* const*, uint32_t, const lime::StreamTxMeta*) "Transmit(complex16_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Transmit(const lime::complex12_t* const*, uint32_t, const lime::StreamTxMeta*) "Transmit(complex12_t* const* samples)"</li>
 *    <li><b>Deprecated</b> 
 *       <ul>
 *          <li> @ref lime::RFStream::StreamRx(lime::complex32f_t* const*, uint32_t, lime::StreamMeta*, std::chrono::microseconds) "StreamRx(complex32f_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamRx(lime::complex16_t* const*, uint32_t, lime::StreamMeta*, std::chrono::microseconds) "StreamRx(complex16_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamRx(lime::complex12_t* const*, uint32_t, lime::StreamMeta*, std::chrono::microseconds) "StreamRx(complex12_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamTx(const lime::complex32f_t* const*, uint32_t, const lime::StreamMeta*, std::chrono::microseconds) "StreamTx(complex32f_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamTx(const lime::complex16_t* const*, uint32_t, const lime::StreamMeta*, std::chrono::microseconds) "StreamTx(complex16_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamTx(const lime::complex12_t* const*, uint32_t, const lime::StreamMeta*, std::chrono::microseconds) "StreamTx(complex12_t* const* samples)"</li>
 *       </ul>
 *    </li>
 * </ul>
 * 
 * @subsection timestamp Timestamp management
 * <ul>
 *    <li> @ref lime::Timespec::AddTicks(int64_t) "AddTicks()"</li>
 *    <li> @ref lime::Timespec::GetTicks() "GetTicks()"</li>
 *    <li> @ref lime::Timespec::GetSeconds() "GetSeconds()"</li>
 *    <li> @ref lime::Timespec::GetFracSeconds() "GetFracSeconds()"</li>
 *    <li> @ref lime::Timespec::GetRealSeconds() "GetRealSeconds()"</li>
 *    <li> @ref lime::Timespec::SetTickRate(double) "SetTickRate()"</li>
 *    <li> @ref lime::Timespec::GetTickRate() "GetTickRate()"</li>
 *    <li><b>Operators</b>
 *       <ul>
 *          <li> @ref lime::operator==(const lime::Timespec&, const lime::Timespec&) "operator==()"</li>
 *          <li> @ref lime::operator!=(const lime::Timespec&, const lime::Timespec&) "operator!=()"</li>
 *          <li> @ref lime::operator<(const lime::Timespec&, const lime::Timespec&) "operator<()"</li>
 *          <li> @ref lime::operator>(const lime::Timespec&, const lime::Timespec&) "operator>()"</li>
 *          <li> @ref lime::operator+(lime::Timespec, const lime::Timespec&) "operator+()"</li>
 *          <li> @ref lime::operator-(lime::Timespec, const lime::Timespec&) "operator-()"</li>
 *          <li> @ref lime::abs(const lime::Timespec&) "abs()"</li>
 *       </ul>
 *    </li>
 * </ul>
 * 
 * @subsection logging Logger
 * <ul>
 *    <li> @ref lime::registerLogHandler(const lime::LogHandlerCString) "registerLogHandler(const LogHandlerCString)()"</li>
 *    <li> @ref lime::registerLogHandler(const lime::LogHandler) "registerLogHandler(const LogHandler)()"</li>
 *    <li> @ref lime::GetLastErrorMessageCString(void) "GetLastErrorMessageCString()"</li>
 *    <li> @ref lime::GetLastErrorMessage(void) "GetLastErrorMessage()"</li>
 *    <li> @ref lime::critical(const char*, ...) "critical(const char*, ...)"</li>
 *    <li> @ref lime::critical(const std::string&) "critical(const string&)"</li>
 *    <li> @ref lime::error(const char*, ...) "error(const char*, ...)"</li>
 *    <li> @ref lime::error(const std::string&) "error(const string&)"</li>
 *    <li> @ref lime::warning(const char*, ...) "warning(const char*, ...)"</li>
 *    <li> @ref lime::warning(const std::string&) "warning(const string&)"</li>
 *    <li> @ref lime::info(const char*, ...) "info(const char*, ...)"</li>
 *    <li> @ref lime::info(const std::string&) "info(const string&)"</li>
 *    <li> @ref lime::debug(const char*, ...) "debug(const char*, ...)"</li>
 *    <li> @ref lime::debug(const std::string&) "debug(const string&)"</li>
 *    <li> @ref lime::log(const lime::LogLevel, const char*, ...) "log(const LogLevel, const char*, ...)"</li>
 *    <li> @ref lime::log(const lime::LogLevel, const std::string&) "log(const LogLevel, const string&)"</li>
 *    <li> @ref lime::ReportError(const lime::OpStatus) "ReportError(const OpStatus)"</li>
 *    <li> @ref lime::ReportError(const lime::OpStatus, const char*, ...) "ReportError(const OpStatus, const char*, ...)"</li>
 *    <li> @ref lime::ReportError(const lime::OpStatus, const std::string&) "ReportError(const OpStatus, const string&)"</li>
 *    <li> @ref lime::ReportError(const int, const char*, ...) "ReportError(const int, const char*, ...)"</li>
 *    <li> @ref lime::ReportError(const int, const std::string&) "ReportError(const int, const string&)"</li>
 * </ul>
 * 
 * @subsection string_manip String manipulation
 * <ul>
 *    <li> @ref lime::ToString(lime::TRXDir) "ToString(TRXDir)"</li>
 *    <li> @ref lime::ToString(lime::OpStatus) "ToString(OpStatus)"</li>
 *    <li> @ref lime::ToString(lime::eGainTypes) "ToString(eGainTypes)"</li>
 *    <li> @ref lime::ToString(lime::eMemoryDevice) "ToString(eMemoryDevice)"</li>
 * </ul>
 * 
 * 
 * @section class Classes
 * 
 * @subsection If_classes Interface classes
 * <ul>
 *    <li> @ref lime::DeviceRegistry "DeviceRegistry"</li>
 *    <li> @ref lime::SDRDevice "SDRDevice"</li>
 *    <li> @ref lime::RFStream "RFStream"</li>
 * </ul>
 * 
 * @subsection strg_classes Storage classes
 * <ul>
 *    <li><b>SDR device description</b>
 *       <ul>
 *          <li> @ref lime::DeviceHandle "DeviceHandle"</li>
 *       </ul>
 *    </li>
 *    <li><b>Stream meta data</b>
 *       <ul>
 *          <li> @ref lime::StreamTxMeta "StreamTxMeta"</li>
 *          <li> @ref lime::StreamRxMeta "StreamRxMeta"</li>
 *          <li> @ref lime::Timespec "Timespec"</li>
 *       </ul>
 *    </li>
 * </ul>
 * 
 * @section struct Structures
 * 
 * <ul>
 *    <li><b>SDR description</b>
 *       <ul>
 *          <li> @ref lime::SDRDescriptor "SDRDescriptor"</li>
 *          <li> @ref lime::DataStorage "DataStorage"</li>
 *          <li> @ref lime::CustomParameter "CustomParameter"</li>
 *          <li> @ref lime::SDRDevice::GPS_Lock "GPS_Lock"</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR device configuration</b>
 *       <ul>
 *          <li> @ref lime::SDRConfig "SDRConfig"</li>
 *          <li> @ref lime::ChannelConfig "ChannelConfig"</li>
 *          <li> @ref lime::ChannelConfig::Direction "Direction"</li>
 *          <li> @ref lime::ChannelConfig::Direction::GFIRFilter "GFIRFilter"</li>
 *          <li> @ref lime::ChannelConfig::Direction::TestSignal "TestSignal"</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR Stream</b>
 *       <ul>
 *          <li> @ref lime::StreamConfig "StreamConfig"</li>
 *          <li> @ref lime::StreamConfig::Extras "Extras"</li>
 *          <li> @ref lime::StreamConfig::Extras::PacketTransmission "PacketTransmission"</li>
 *          <li> @ref lime::StreamStats "StreamStats"</li>
 *          <li> @ref lime::StreamStats::FIFOStats "FIFOStats"</li>
 *          <li> @ref lime::StreamMeta "StreamMeta"</li>
 *       </ul>
 *    </li>
 *    <li><b>RF SoC description</b>
 *       <ul>
 *          <li> @ref lime::GainValue "GainValue"</li>
 *          <li> @ref lime::RFSOCDescriptor "RFSOCDescriptor"</li>
 *       </ul>
 *    </li>
 *    <li><b>Registers</b>
 *       <ul>
 *          <li> @ref lime::CSRegister "CSRegister"</li>
 *          <li> @ref lime::Register "Register"</li>
 *       </ul>
 *    </li>
 *    <li><b>Complex types</b>
 *       <ul>
 *          <li> @ref lime::complex64f_t "complex64f_t"</li>
 *          <li> @ref lime::complex32f_t "complex32f_t"</li>
 *          <li> @ref lime::complex16_t "complex16_t"</li>
 *          <li> @ref lime::complex12_t "complex12_t"</li>
 *       </ul>
 *    </li>
 *    <li><b>Utility</b>
 *       <ul>
 *          <li> @ref lime::Range "Range"</li>
 *          <li> @ref lime::Region "Region"</li>
 *          <li> @ref lime::CustomParameterIO "CustomParameterIO"</li>
 *       </ul>
 *    </li> 
 * </ul>
 * 
 * @section enums Enumerations 
 * 
 * <ul>
 *    <li> @ref lime::LogLevel "LogLevel"</li>
 *    <li> @ref lime::OpStatus "OpStatus"</li>
 *    <li><b>SDR device options</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GPS_Lock::LockStatus "LockStatus"</li>
 *          <li> @ref lime::TRXDir "TRXDir"</li>
 *          <li> @ref lime::DataFormat "DataFormat"</li>
 *          <li> @ref lime::eGainTypes "eGainTypes"</li>
 *          <li> @ref lime::eMemoryDevice "eMemoryDevice</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR stream options</b>
 *       <ul>
 *          <li> @ref lime::TimestampType "TimestampType"</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR test signal options:</b>
 *       <ul>
 *          <li> @ref lime::ChannelConfig::Direction::TestSignal::Divide "Divide"</li>
 *          <li> @ref lime::ChannelConfig::Direction::TestSignal::Scale "Scale"</li>
 *       </ul>
 *    </li>
 * </ul>
*/

/**
 * @page common_parameters Common public API parameters
 * 
 * @section Device_index Device indexes
 * 
 * Some of the SDR device configuration functions from the @ref api "public API" accept a parameter called <b>moduleIndex</b>. The <b>moduleIndex</b> 
 * parameter specifies an index of a device which will be configured. Typically device index points to RF chip (LMS7002M), but in the case
 * of LimeSDR MMX8 this parameter points to the onboard LimeSDR XTRXs and allows to configure the specific parameter for the
 * individual LimeSDR XTRX board RF chip. 
 *
 * Device indexes always start with index <b>0</b>. The last device on a system is indexed as <b>total device count - 1</b>.
 * 
 * Parameter <b>moduleIndex</b> should always be set to device index <b>0</b> for LimeSDR devices that use a single RF chip.
 */

/**
 * @page guide_book_top Library guide book
 * 
 * This is the main page of LimeSuiteNG library guide book. Here you will find an organized list of topics which will help you get started with LimeSuiteNG public API.
 * Guide book topics cover: library set up requirements for custom CMake project, device discovery and registration, device configuration, device stream set up and streaming.
 * Example topics are ordereded in a consecutive way such that project set up topic follows device registration topic and etc. The last example list topic provides a minimal example
 * code which can be used as template for further development.
 * 
 * List of topics:
 * <ul>
 *    <li> @subpage cmake_set_up </li>
 *    <li> @subpage dev_discovery </li>
 *    <li> @subpage dev_config </li>
 *    <li> @subpage dev_streaming </li>
 *    <li> @subpage hello_world_example </li>
 * </ul>
 * 
 */

// This is the starting point of subpages for example page
/**
 * @page cmake_set_up CMake set up
 * 
 * The following CMakeLists.txt code is the minimum required project set up code which allows to use LimeSuiteNG API in custom applications:
 * @code{.cmake}
 * cmake_minimum_required(VERSION 3.26)
 * 
 * project(LimeSuiteNG_example LANGUAGES C CXX)
 * 
 * set(PROJECT_SOURCE 
 *    main.cpp
 *    # ... If required, more source files separated by newline
 *    )
 * 
 * add_executable(${PROJECT_NAME} ${PROJECT_SOURCE})
 * target_link_libraries(${PROJECT_NAME} PRIVATE limesuiteng)
 * @endcode
 * 
 * CMake project always starts with minimum required CMake version and project command. Specify the name of project and supported languages.
 * Set your list of source files that will be used in the project build. Update the project configuration by specifying the build type as executable.
 * Specify executable command arguments: executable name (in this case project name) and pass the source file variable that contains the list of source files. Link 
 * the LimeSuiteNG library to project executable using <b>target_link_libraries()</b> command. Once the project CMake files are set up, include the following headers
 * into project source files for access to LimeSuiteNG API:
 * @code{.cpp}
 * #include <limesuiteng/limesuiteng.hpp>
 * @endcode
 * 
 * To configure and build project, run commands in command line/terminal environment:
 * @code{.bash}
 * cmake ..
 * cmake --build .
 * @endcode
 * 
 * @ref dev_discovery "Go to SDR device discovery and registration example topic"
 */

/**
 * @page dev_discovery SDR device discovery and registration
 * 
 * @ref cmake_set_up "Back to CMake project set up example topic"  
 * 
 * This example topic explains how LimeSDR devices are discovered and registered in custom applications. In total LimeSuiteNG API contains 5 
 * functions that can be used to discover and manage SDR device connectivity.  
 * 
 * @section discovery Discovery
 * 
 * First example code explores the use of @ref lime::DeviceRegistry::enumerate() "enumerate()" function that produces the list of SDR devices that 
 * are currently connected to PC:
 * @code{.cpp}
 * int main()
 * {
 *    std::cout << "This example shows how to discover and register LimeSDR device.\n";
 * 
 *    std::vector<lime::DeviceHandle> listOfDevices = lime::DeviceRegistry::enumerate();
 *    if(listOfDevices.empty())
 *    {
 *       std::cout << "No SDR devices detected!\n";
 *       return 1;
 *    }
 * 
 *    std::cout << "Found " << listOfDevices.size() << " devices:\n";
 *    
 *     for(const auto& singleDevHandle : listOfDevices)
 *        std::cout << singleDevHandle.Serialize() << std::endl;
 *
 *    return 0;
 * }
 * @endcode
 * 
 * @if RST_SUPPORT
 * @verbatim embed:rst:leading-asterisk
 * .. important::
 *    All LimeSuiteNG API functions are part of namespace ``lime``. API members can be accessed with scope resolution operator ``lime::``. Additionally, all device discovery and registration API functions are also a part of DeviceRegistry class, therefore the scope of DeviceRegistry class must also be specified using the scope resolution operator ``DeviceRegistry::``.
 * @endverbatim
 * @else
 * @important All LimeSuiteNG API functions are part of namespace <b>lime</b>. API members can be accessed with scope resolution operator <b>lime::</b>.
 * Additionally, all device discovery and registration API functions are also a part of DeviceRegistry class, therefore the scope of DeviceRegistry class
 * must also be specified using the scope resolution operator <b>DeviceRegistry::</b>.  
 * @endif
 * 
 * In this example code, API function enumerate() returns a vector with items of @ref lime::DeviceHandle "DeviceHandle" type, which is a type that provides the 
 * base information about connected SDR devices:
 * <ol>
 *    <li>Device name</li>
 *    <li>Device connection media (USB, PCIe, Ethernet and etc.)</li>
 *    <li>Device address (USB VID:PID, IP address and etc.) </li>
 *    <li>Device serial number</li>
 * </ol>
 * 
 * If no devices are detected, function returns an empty device list which can be tested with `.empty()` or `.size()` methods. If the device list is not empty,
 * a range-for loop can be used to print items of the device list to standard output. To obtain basic device information in std::string format, a method 
 * @ref lime::DeviceHandle::Serialize() "Serialize()" or @ref lime::DeviceHandle::ToString() "ToString()" must be called for each item of @ref lime::DeviceHandle "DeviceHandle" type 
 * in device list. Both methods return string of arguments that identify the device, but the Serialize() method also appends complementary text that identifies the device argument.
 * First example code for @ref lime::DeviceRegistry::enumerate() "enumerate()" API function produces output:
 * 
 * @image{inline} html dev_registration_dev_enum.png
 * 
 * @if RST_SUPPORT
 * @embedRstVerbatim{
 * .. admonition:: Note
 * 
 *    LimeSDR devices that use USB port to connect to PC are typically USB 3.0 compliant. Here LimeSDR devices are detected as USB 2.0 devices. This can occur due to faulty USB connection (device is not fully inserted into USB port) or if a SDR device is connected through USB extender or dock that is not compliant with USB 3.0 standard. Try re-connecting the SDR device to PC or bypass any USB extenders or docs for maximum connectivity speed.}
 * @else
 * @note LimeSDR devices that use USB port to connect to PC are typically USB 3.0 compliant. Here LimeSDR devices are detected as USB 2.0 devices. This can occur due to 
 * faulty USB connection (device is not fully inserted into USB port) or if a SDR device is connected through USB extender or dock that is not compliant with USB 3.0 standard.
 * Try re-connecting the SDR device to PC or bypass any USB extenders or docs for maximum connectivity speed.  
 * @endif
 * 
 * @section filter_and_register List filtering and registration
 * 
 * Discovery function @ref lime::DeviceRegistry::enumerate() "enumerate()" also has an overloaded version @ref lime::DeviceRegistry::enumerate(const lime::DeviceHandle&) "enumerate(const DeviceHandle&)"
 * function that can accept a @ref lime::DeviceHandle "DeviceHandle" type object which can act as a device filter. If all arguments from device handle matches specified device filter arguments (unspecified 
 * filter arguments are ignored), the SDR device is included in the final device list:
 * @code{.cpp}
 * int main()
 * {
 *    std::cout << "This example shows how to discover and register LimeSDR device.\n";
 * 
 *    std::vector<lime::DeviceHandle> listOfLimeSDR_Mini = lime::DeviceRegistry::enumerate(lime::DeviceHandle("LimeSDR Mini V2"));
 *    std::vector<lime::DeviceHandle> listOfLimeSDR_USB = lime::DeviceRegistry::enumerate(lime::DeviceHandle("LimeSDR-USB"));
 * 
 *    std::cout << "List of LimeSDR Mini devices:\n";
 *    for(const auto& singleDevHandle : listOfLimeSDR_Mini)
 *       std::cout << singleDevHandle.Serialize() << std::endl;
 * 
 *    std::cout << std::endl << "List of LimeSDR USB devices:\n";
 *    for(const auto& singleDevHandle : listOfLimeSDR_USB)
 *       std::cout << singleDevHandle.Serialize() << std::endl;
 * 
 *    return 0;
 * }
 * @endcode
 *
 * The output of the second example:
 * 
 * @image{inline} html dev_registration_dev_filtering.png
 * 
 * 
 * Alternatively, it is also possible to filter requested devices from enumerated device list using @ref lime::DeviceHandle::IsEqualIgnoringEmpty(const lime::DeviceHandle&) const "IsEqualIgnoringEmpty(const DeviceHandle&)" method: 
 * @code{.cpp}
 * int main()
 * {
 *    std::cout << "This example shows how to discover and register LimeSDR device.\n";
 *    std::vector<lime::DeviceHandle> listOfDevices = lime::DeviceRegistry::enumerate();
 *    if(listOfDevices.size() == 0)
 *    {
 *       std::cout << "No SDR devices detected\n";
 *       return 1;
 *    }
 * 
 *    std::cout << "Found " << listOfDevices.size() << " devices\n";
 *    for(const auto& item : listOfDevices)
 *       std::cout << item.Serialize() << std::endl;
 * 
 *    // Filtering and extracting requested device list index by device name
 *    lime::DeviceHandle reqDevice("LimeSDR Mini V2");
 *    int reqDevID = -1;
 *    std::cout << std::endl << "Searching for requested device " << reqDevice.ToString() << " in device list\n";
 *    for(int itemID = 0; itemID < listOfDevices.size(); ++itemID)
 *    {  
 *       if(listOfDevices[itemID].IsEqualIgnoringEmpty(reqDevice))
 *       {
 *          reqDevID = itemID;
 *          std::cout << "Device " << reqDevice.ToString() << " found.\n";
 *          break;
 *       }
 *    }
 * 
 *    if(reqDevID == -1)
 *    {
 *       std::cout << "Requested device not found in enumerated device list!\n";
 *       return 1;
 *    }
 *    // End of device filtering   
 * 
 *    std::cout << "Connecting to requested device: " << reqDevice.ToString() << std::endl;
 *    lime::SDRDevice * device = lime::DeviceRegistry::makeDevice(listOfDevices.at(reqDevID));
 *    if(device == nullptr)
 *    {
 *       std::cout << "Failed to connect to SDR device\n";
 *       return 1;
 *    }
 * 
 *    std::cout << "Connection established. Disconnecting!\n";
 * 
 *    lime::DeviceRegistry::freeDevice(device);
 * 
 *    return 0;
 * }
 * @endcode
 * Following the first example, device enumeration without any device filters is used. Once device list is enumerated
 * and there are devices present in the list, device filter of @ref lime::DeviceHandle "DeviceHandle" type is created. Only device name is 
 * selected as a filtering argument. Device filter object is used as an argument for @ref lime::DeviceHandle::IsEqualIgnoringEmpty(const lime::DeviceHandle&) const "IsEqualIgnoringEmpty(const DeviceHandle&)" method
 * which compares the arguments of device handles in the list with the given device filter handle. If a device name matches the filter argument, the list item index can be saved and used to connect to the required SDR 
 * device. Connection with the device can be established using @ref lime::DeviceRegistry::makeDevice(const lime::DeviceHandle&) "makeDevice(const DeviceHandle&)" API function which requires a device handle as an argument. 
 * To test if the connection to the device is successfull, compare device pointer to <b>nullptr</b>. If connection is successful, address of the device object is returned, else the pointer will be equal to nullptr.
 * Possible (but rare) connection problems:  
 * <ol>
 *    <li>Unexpected SDR device firmware ID</li>
 *    <li>Failed connection to USB port</li>
 *    <li>Unexpected SDR device USB VID:PID values</li>
 * </ol> 
 * Once the device is no longer needed, the device object must be freed using @ref lime::DeviceRegistry::freeDevice(lime::SDRDevice*) "freeDevice(SDRDevice* device)" API function in order to release dynamically 
 * allocated memory to avoid application memory leaks on application termination. Output of the example code:
 * 
 * @image{inline} html dev_registration_dev_connection.png
 * 
 * @if RST_SUPPORT
 * @embedRstVerbatim{
 * .. admonition:: Note
 * 
 *    In the last example code, if the device filtering is not required, device filtering code (between inline comments) can be omitted and connection can be established with the first device from enumerated device list ``lime::SDRDevice * device = lime::DeviceRegistry::makeDevice(listOfDevices.front());``.}
 * @else
 * @note In the last example code, if the device filtering is not required, device filtering code (between inline comments) can be omitted and connection can be established with the first device from enumerated device list:
 * @code{.cpp}
 * lime::SDRDevice * device = lime::DeviceRegistry::makeDevice(listOfDevices.front());
 * @endcode
 * @endif
 * 
 * @ref dev_config "Go to SDR device configuration example topic"
 */

/**
 * @page dev_config SDR device configuration
 * 
 * @ref dev_discovery "Back to SDR device discovery and registration example topic"  
 * 
 * Since SDR devices are highly customizible, this example topic is separated into different sub-topics, which provide specific information and examples for various SDR control and configuration options. 
 * The quick configuration sub-topic is the main sub-topic that provides the essential steps required to configure the base SDR device parameters that are enough to run a data stream. Other sub-topics 
 * provide more information about individual SDR device parameter configuration options and other advanced configuration options.  
 * <ul>
 *    <li> @subpage quick_config "Quick configuration"</li>
 *    <li> @subpage generic_config "Common control and configuration options"
 *    <li> @subpage direct_config "Direct register and parameter access"
 *    <li> @subpage fir_filter_config "FIR filter configuration"</li>
 *    <li> @subpage frequency_config "Frequency configuration"</li>
 * </ul>
 * 
 * @ref dev_streaming "Go to SDR device stream set up and data stream example topic" 
 */

// ###########################################################
// This is the starting point of sub-topic pages for SDR 
// configuration topic
// ###########################################################

/**
 * @page generic_config Common control and configuration options
 * 
 * @ref dev_config "Back to the list of SDR device configuration sub-topics"  
 * 
 * Common SDR device control and configuration options allow to set up the communication channel structure and the base streaming parameters of individual channel directions (Tx/Rx).
 * This sub-topic explains how to configure:
 * <ol>
 *    <li>channel mode: MIMO or SISO </li>
 *    <li>channel direction LO frequency</li>
 *    <li>channel direction low pass filter bandwidth</li>
 *    <li>channel direction sample rate</li>
 *    <li>channel direction antenna</li>
 *    <li>channel direction gain</li>
 * </ol>
 * 
 * @section default_init Default initialization
 * 
 * Before starting any SDR device channel configurations it is recommended to perform default SDR device initialization using @ref lime::SDRDevice::Init() "Init()" function:
 * @code{.cpp}
 *    ... // Device discovery and registration code
 *    
 *    configStatus = device->Init();
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Default device initializtion failed with error: " << lime::ToString(configStatus) << std::endl;
 *    
 *    ... // Other SDR device configurations  
 * @endcode
 * Default initialization is particulary important for new, out of the box or cold started SDR devices. Device default initialization performs device reset and loads device specific
 * default and stable/tested device configuration which can then be freely customized to support project requirements.
 * 
 * @section channel_conf Stream channels
 * 
 * SDR device stream channels can be configured by using @ref lime::SDRDevice::EnableChannel(uint8_t, lime::TRXDir, uint8_t, bool) "EnableChannel()" function:
 * @code{.cpp}
 *    ... // Device default initialization
 *    
 *    uint8_t moduleIndex = 0;
 *    bool directionEnabled = true;
 *    configStatus = device->EnableChannel(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, directionEnabled);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to toggle channel A Tx direction with error: " << lime::ToString(configStatus) << std::endl;    
 * 
 *    ... // Other SDR device configurations
 * @endcode
 * 
 * The @ref lime::SDRDevice::EnableChannel(uint8_t, lime::TRXDir, uint8_t, bool) "EnableChannel()" function performs hardware re-configuration of specified device channel direction. If device channel
 * direction is disabled, all of the internal hardware components (transceiver signal processor, analog front end, radio front end and etc.) used for that particular direction are also disabled. Enabling 
 * device channel direction has the reverse effect of the channel direction disable. By default all device channels are enabled and device is configured to work in MIMO mode. Therefore, if a single 
 * channel and SISO mode is enough for a data stream, it is recommended to disable other unused channel directions to reduce SDR device current and power consumption. When configuring device for SISO
 * mode, it is recommended to use channel A as the default streaming channel, since some of LimeSDR devices (in particular LimeSDR Mini V2 and V1) do not have a physical connection to channel B. When 
 * configuring SDR device channels it is also important to specify the correct argument for <b>moduleIndex</b> parameter. The <b>moduleIndex</b> parameter is used to index the multiple LMS7002M  modules 
 * for SDR devices or integrated systems that can have multiple LMS7002M modules or integrate multiple SDR devices. When working with standard LimeSDR devices
 * that use a single LMS7002M RF chip, parameter <b>moduleIndex</b> should always be set to <b>0</b>. If SDR device or integrated systems have more than one LMS7002M module, appropriate index should be
 * specified to target the correct LMS7002M module. More about module indexes, can be found @ref common_parameters "here".
 * 
 * @if RST_SUPPORT
 * @embedRstVerbatim{
 * .. admonition:: Note
 * 
 *    The moduleIndex parameter is present in most of SDR device configuration functions. The purpose of the parameter, as described in stream channel set up paragraph, is to target configuration of correct device LMS7002M module.}
 * @else
 * @note The <b>moduleIndex</b> parameter is present in most of SDR device configuration functions. The purpose of the parameter, as described in stream channel set up paragraph, is to target configuration
 * of correct device LMS7002M module.
 * @endif
 * 
 * @section lo_freq LO frequency
 * 
 * To set new synthesizer LO generator frequency for a specific SDR device channel direction use @ref lime::SDRDevice::SetFrequency(uint8_t, lime::TRXDir, uint8_t, double) "SetFrequency()" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0;
 *    double frequency = 95.9e6; // Hz
 *    configStatus = device->SetFrequency(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, frequency);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to toggle channel A Tx direction with error: " << lime::ToString(configStatus) << std::endl;
 * 
 *    ... // Other program code
 * @endcode
 * 
 * To retrieve current synthesizer LO generator frequency of specified SDR device channel direction, use @ref lime::SDRDevice::GetFrequency(uint8_t, lime::TRXDir, uint8_t) "GetFrequency()" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0;
 *    double frequency = 0;
 *    frequency = device->GetFrequency(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA);
 *    std::cout << "Current LO frequency on ChA Tx = " << frequency << " Hz\n";
 * 
 *    ... // Other program code
 * @endcode
 * 
 * When configuring new synthesizer LO generator frequency, it is always recommended to check for operation errors. Possible error reasons:
 * <ul>
 *    <li>Specified LO generator frequency is not supported by VCO</li>
 *    <li>LO generator VCO tuning failed to lock to synthesizer reference clock</li>
 *    <li>LO generator VCO tuning failed, required tuning bias current too high</li>
 * </ul>
 * 
 * LO generator frequency is shared between the same directions (Rx or Tx) on different channels, when SDR device is set to operate in MIMO mode. Therefore, in MIMO mode, it is 
 * enough to set LO generator frequency once per stream direction (Rx or Tx) using @ref lime::SDRDevice::SetFrequency(uint8_t, lime::TRXDir, uint8_t, double) "SetFrequency()" function.
 *
 * @section lpf_conf Low pass filters
 *
 * To set new low pass filter value for a specified SDR device channel direction, use @ref lime::SDRDevice::SetLowPassFilter(uint8_t, lime::TRXDir, uint8_t, double) "SetLowPassFilter()" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0;
 *    double bandwidth = 2e6; // Hz
 *    configStatus = device->SetLowPassFilter(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, bandwidth);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to set channel A Tx direction low pass filter bandwidth with error: " << lime::ToString(configStatus) << std::endl;
 *
 *    ... // Other program code
 * @endcode
 * 
 * To get the current low pass filter value for a specified SDR device channel direction, use @ref lime::SDRDevice::GetLowPassFilter(uint8_t, lime::TRXDir, uint8_t) "GetLowPassFilter()" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0;
 *    double bandwidth = 0; // Hz
 *    bandwidth = device->GetLowPassFilter(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA);
 *    std::cout << "Current low pass filter bandwith value for channel A, Tx direction is " << bandwidth << std::endl;
 * 
 *    ... // Other program code
 * @endcode
 * 
 * To bypass low pass filter in transceiver baseband for a specified SDR device channel direction, set the bandwidth value to zero. If Low pass filter is bypassed in Rx direction,
 * signal is directed directly to PGA (Programabale gain amplifier). If low pass filter is bypassed in Tx direction, signal is directed from current amplifier to RF front end through 
 * real pole stage. If the specified new bandwidth value is lower than minimum or higher than maximum supported low pass filter bandwith, the new value is respectively clamped to the 
 * minimum or maximum possible low pass filter bandwidth value. Therefore, when setting new low pass filter bandwidth value, it is recommended to check if the actual value was set in HW.
 * 
 * @section sample_rate Sample rate
 * 
 * To set new sample rate value for SDR device, use @ref lime::SDRDevice::SetSampleRate(uint8_t, lime::TRXDir, uint8_t, double, uint8_t) "SetSampleRate()" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0;
 *    double sampleRate = 2e6; // Hz
 *    uint8_t oversampleRatio = 0;
 *    configStatus = device->SetSampleRate(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, sampleRate, oversampleRatio);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to set SDR device sample rate with error: " << lime::ToString(configStatus) << std::endl;
 * 
 *    ... // Other program code
 * @endcode
 * 
 * To get the current sample rate of SDR device, use @ref lime::SDRDevice::GetSampleRate(uint8_t, lime::TRXDir, uint8_t, uint32_t*) "GetSampleRate()" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0;
 *    double sampleRate = 0; // Hz
 *    uint32_t rf_sampleRate = 0; // Hz
 *    sampleRate = device->GetSampleRate(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, &rf_sampleRate);
 *    std::cout << "Current SDR device sample rate is " << sampleRate << " Hz";
 *    if(rf_sampleRate > 0)
 *       std::cout << ", RF sample rate is " << rf_sampleRate << " Hz\n";
 *    else
 *       std::cout << std::endl;
 * 
 *    ... // Other program code
 * @endcode
 * 
 * SDR device sample rate configuration is not dependant on selected SDR device channel and direction. Sample rate setting is updated for all internal ADCs and DACs of all active channel directions. The
 * actual RF sample rate is much higher than the specified sample rate and depends on the selected oversampling ratio parameter. 
 * In example code above, function @ref lime::SDRDevice::SetSampleRate(uint8_t, lime::TRXDir, uint8_t, double, uint8_t) "SetSampleRate()" accepts argument <b>oversampleRatio</b> which specifies the sample
 * interpolation and decimation ratios. If oversampling ratio parameter is set to value 0 and specified sample rate is lower or equal to 61.44 MHz, ratios are auto calculated based on specified sample rate.
 * If specified sample rate is higher than 61.44 MHz and oversamplnig ratio is set to value 0, then interpolation and decimation stage is bypassed. Alternatively, oversampling ratio can be set manually.
 * Supported oversampling ratios:
 * <ul>
 *    <li>x2  (oversampleRatio == 2)</li>
 *    <li>x4  (oversampleRatio == 4)</li>
 *    <li>x8  (oversampleRatio == 8)</li>
 *    <li>x16 (oversampleRatio == 16)</li>
 *    <li>x32 (oversampleRatio == 32)</li>
 * </ul>
 * 
 * To get the actual RF sample rate value, the last parameter of <b>GetSampleRate()</b> must accept an address of a variable where the actual RF sample rate will be stored (example code above). If the argument
 * is omitted, then only the sample rate is returned.
 * 
 * @section antenna_path Antenna path
 * 
 * To set new antenna path for SDR device, use @ref lime::SDRDevice::SetAntenna(uint8_t, lime::TRXDir, uint8_t, uint8_t) "SetAntenna()" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0;
 *    uint8_t pathID = 1;  // Tx Band1
 *    configStatus = device->SetAntenna(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, pathID);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to set SDR device antenna path with error: " << lime::ToString(configStatus) << std::endl;
 * 
 *    ... // Other program code
 * @endcode
 * 
 * To get the current antenna path of SDR device, use @ref lime::SDRDevice::GetAntenna(uint8_t, lime::TRXDir, uint8_t) "GetAntenna()" function:
 * @code{.cpp} 
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0;
 *    uint8_t pathID = 0;
 *    pathID = device->GetAntenna(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA);
 *    std::cout << "Current SDR device antenna path is " << pathID << std::endl;
 * 
 *    ... // Other program code
 * @endcode
 *
 * Depending on SDR device model, different antennas can be used both for Tx and Rx directions of each channel. If a unexpected antenna path ID is specified, the path ID is ignored and 
 * antenna is not changed. Below are lists of common antenna paths for Tx and Rx directions.
 * 
 * List of antenna path IDs for Tx direction on each channel:
 * <ul>
 *    <li> Band1 path ID is 1</li>
 *    <li> Band2 path ID is 2</li>
 * </ul>
 * 
 * List of antenna path IDs for Rx direction in each channel:
 * <ul>
 *    <li>LNAL path ID is 2</li>
 *    <li>LNAW path ID is 3</li>
 *    <li>LNAH path ID is 1</li>
 *    <li>LB1 path ID is 4</li>
 *    <li>LB2 path ID is 5</li>
 * </ul>
 * 
 * Not all SDR devices can support the listed antennas. It is recommended to check antenna support of each SDR device by retrieving SDR device descriptor:
 * @code{.cpp}
 *  int main()
 *  {
 *     std::vector<lime::DeviceHandle> listOfDevices = lime::DeviceRegistry::enumerate();
 *  
 *     lime::SDRDevice * device = lime::DeviceRegistry::makeDevice(listOfDevices.front());
 *     if(device == nullptr)
 *     {
 *        std::cout << "Failed to connect to SDR device\n";
 *        return 1;
 *     }
 *     
 *     lime::OpStatus configStatus = lime::OpStatus::Success;
 *     uint8_t moduleIndex = 0;
 *     auto SDRDescriptor = device->GetDescriptor();
 *     auto& paths = SDRDescriptor.rfSOC.at(0).pathNames;
 *  
 *     std::cout << SDRDescriptor.name << " supported antennas\n";
 *     std::cout << "Tx antennas:\n";
 *     for(int i = 0; i < paths.at(lime::TRXDir::Tx).size(); ++i)
 *        std::cout << "\t" << paths.at(lime::TRXDir::Tx)[i] << std::endl;
 *     std::cout << std::endl;
 *  
 *     std::cout << "Rx antennas:\n";
 *     for(int i = 0; i < paths.at(lime::TRXDir::Rx).size(); ++i)
 *        std::cout << "\t" << paths.at(lime::TRXDir::Rx)[i] << std::endl;
 *     std::cout << std::endl;
 *  
 *     lime::DeviceRegistry::freeDevice(device);
 *  
 *     return 0;
 *  }
 * @endcode
 * 
 * Output of the example code above for LimeSDR Mini V2:
 * 
 * @image{inline} html dev_config_antenna_support.png
 * 
 * Some antenna paths can have a <b>NC</b> postfix which indicates that the internal RF chip antenna path is not physically connected to antenna.
 * 
 * @section dev_gain Gain control
 * 
 * To set new gain value for SDR device amplifiers, use @ref lime::SDRDevice::SetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double) "SetGain()" function with amplifier type set as GENERIC:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0;
 *    double gain = 9;  // dB
 *    configStatus = device->SetGain(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, lime::eGainTypes::GENERIC, gain);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to set SDR device LNA gain with error: " << lime::ToString(configStatus) << std::endl;
 * 
 *    ... // Other program code 
 * @endcode
 * 
 * This will auto set the gain of generic amplifiers of specified channel and direction based on specified gain value. For Tx direction, PAD amplifier is considered as generic amplifier. For Rx direction, LNA,
 * PGA and TIA are considered as generic amplifiers. When setting the gain of Rx direction generic amplifiers, the nearest gain values will be auto selected based on specified gain value.
 * Specified out of range gain values are clamped to the maximum or minimum possible gain values of each amplifier. 
 * 
 * To get the current or udpated gain value of SDR device amplifiers, use @ref lime::SDRDevice::GetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double&) "GetGain()" function with amplifier type set as GENERIC:
 * @code{.cpp}
 *    ... // Other program code
 *    
 *    uint8_t moduleIndex = 0;
 *    double gain = 0; // dB
 *    configStatus = device->GetGain(moduleIndex, lime::TRXDir::Rx, lime::LMS7002M::Channel::ChA, lime::eGainTypes::GENERIC, gain);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to get SDR device LNA gain with error: " << lime::ToString(configStatus) << std::endl;
 * 
 *    ... // Other program code
 *
 * @endcode
 * 
 * For Tx direction, this will return the combined gain value of PAD and IAMP amplifiers. For Rx direction, this will return the combined gain value of LNA, TIA and PGA amplifiers. Alternatively, it is also possible to set
 * the gain of the individual SDR device amplifiers:
 * <table>
 *    <tr>
 *       <th>Amplifier type</th>
 *       <th>Gain range/values, dB</th>
 *       <th>Default value, dB</th>
 *       <th>Gain setting behaviour</th>
 *       <th>Signal strength formula</th>
 *    </tr>
 *    <tr>
 *       <td>LNA</td>
 *       <td> [0; 30] </td>
 *       <td> 30 </td>
 *       <td>Adjusts signal strength by specified amount of gain.</td> 
 *       <td><b>specified gain - 30 = signal strength dB –> 9 - 30 = -21 dB</b></td>
 *    </tr>
 *     <tr>
 *        <td>Loopback LNA</td>
 *        <td> [0; 40] </td>
 *        <td> 0 </td>
 *        <td>Adjusts signal strength by specified amount of gain.</td> 
 *        <td><b>specified gain - 40 = signal strength dB –> 23 - 40 = -17 dB</b></td>
 *     </tr>
 *     <tr>
 *        <td>PGA</td>
 *        <td> [19; -12] </td>
 *        <td> -11 </td>
 *        <td>Sets the actual signal strength from the supported gain value range.</td> 
 *        <td><b>gain value from range = signal strength –> -12 dB = -12 dB</b></td>
 *     </tr>
 *     <tr>
 *        <td>TIA</td>
 *        <td> 0, 9, 12 </td>
 *        <td> 12 </td>
 *        <td>Adjusts signal strength by specified amount of gain.</td> 
 *        <td><b>specified gain - 12 = signal strength dB –> 9 - 12 = -3 dB</b></td>
 *     </tr>
 *     <tr>
 *        <td>PAD</td>
 *        <td> [0; 52] </td>
 *        <td> 52 </td>
 *        <td>Sets the actual signal strength.</td>
 *        <td><b>gain value from range = signal strength –> 12 dB = 12 dB</b></td>
 *     </tr>
 *     <tr>
 *        <td>Loopback PAD</td>
 *        <td> [0; -4] </td>
 *        <td> -4 </td>
 *        <td>Sets the actual signal strength according to set gain intervals.</td> 
 *        <td><b>[x; y] dB = signal strength dB; [0; -0.7] = 0 dB, (-0.7; -2.35] = -1 dB, (-2.35, -3.8] = -3 dB, (-3.8, -inf) = -4 dB.</b></td>
 *     </tr>
 *     <tr>
 *        <td>IAMP</td>
 *        <td> To be updated </td>
 *        <td> To be updated </td>
 *        <td> To be updated </td>
 *        <td> To be updated </td>
 *     </tr>
 * </table>
 * 
 * If the specified gain value is not supported by specified amplifier type, the nearest gain value is applied. If a unknown amplifier type is specified when setting new gain value, gain of a
 * generic amplifier will be updated.  @ref lime::SDRDevice::GetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double&) "GetGain()" function can retrieve the gain setting values of the
 * amplifier types specified in the gain setting table above. For LNA, LoopbackLNA and TIA amplifiers, function will return the amount of gain which is used to strengthen the received signal.
 * For PGA, PAD and LoopbackPAD amplifiers, function will return the actual signal strength.
 * 
 */

/**
 * @page quick_config Quick configuration
 * @ref dev_config "Back to the list of SDR device configuration sub-topics" 
 * 
 * SDR device quick configuration can be used to quickly load or save SDR device configuration presets. SDR devices can be quick configured:
 * <ul>
 *    <li>from config structure</li>
 *    <li>from .ini file</li>
 * </ul>
 * 
 * @section quick_conf_struct From structure
 * 
 * To quick configure SDR device, use @ref lime::SDRConfig "SDRConfig" structure and @ref lime::SDRDevice::Configure(const lime::SDRConfig&, uint8_t) "Configure()" function:
 * @code{.cpp}
 * 
 *    ... // Other program code
 *    uint8_t moduleIndex = 0;          // For SDR devices with single RF chip
 *    lime::SDRConfig LimeSDRMiniConf;  // Using default configuration
 *    configStatus = device->Configure(LimeSDRMiniConf, moduleIndex);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to configure SDR device with error: " << lime::ToString(configStatus) << std::endl;
 *    ... // Other program code
 *    
 * @endcode
 * 
 * As shown in the above example code, default SDR device configuration can be loaded using empty SDRConfig structure (with no user defined structure fields). When an empty SDRConfig structure
 * is passed as an argument to @ref lime::SDRDevice::Configure(const lime::SDRConfig&, uint8_t) "Configure()" function, a @ref lime::SDRDevice::Init() "Init()" function is called to load specific SDR device default 
 * configuration with stable and tested SDR device parameters. If at any point the device becomes unstable or behaves in unexpected ways, perform default configuration to restore device. Default SDR device 
 * configuration can also be used alongside custom configuration. Simply define the SDRConfig parameter fields which you want to custom configure. When the default configuration is set up, 
 * function @ref lime::SDRDevice::Configure(const lime::SDRConfig&, uint8_t) "Configure()" continues by updating SDR device configuration with user define parameter values. User undefined parameter values 
 * (set to value 0) are not configured for SDR device. This set up allows to simultaneously reset SDR device and load SDR device custom configuration. To skip SDR device default initializtion and immediately
 * proceed with custom SDR device configuration, @ref lime::SDRConfig::skipDefaults "skipDefaults" paramter in SDRConfig structure must be set to value <b>true</b>.
 * 
 * @if RST_SUPPORT
 * @embedRstVerbatim{
 * .. admonition:: Warning
 * 
 *    It is recommended not to modify referenceClockFreq parameter in SDRConfig structure. This parameter is intended for advanced configurations and advanced users.}
 * @else
 * @warning It is recommended not to modify @ref lime::SDRConfig::referenceClockFreq "referenceClockFreq" parameter in SDRConfig structure. This parameter is intended for advanced configurations and advanced users.  
 * @endif
 * 
 * @ref lime::SDRConfig "SDRConfig" structure allows to configure up to @ref lime::SDRConfig::MAX_CHANNEL_COUNT "16" different SDR device channels. However, SDR devices based on LMS7002M RF chip only support
 * two channels at a time. If the SDR device uses more than one LMS7002M RF chip, you should instead define multiple configuration structures of @ref lime::SDRConfig "SDRConfig" type and set appropriate
 * indexes that point to the correct LMS7002M RF chips:
 * @code{.cpp}
 *    ... // Other program code
 *    uint8_t moduleIndexes[2] = {0, 1};     // SDR device with multiple RF chips
 *    lime::SDRConfig LMS7002MConf[2] = {};  // Using default configs of LMS7002M chips
 *    for(int i = 0; i < sizeof(moduleIndexes); ++i)
 *    {
 *       configStatus = device->Configure(LMS7002MConf[i], moduleIndexes[i]);
 *       if(configStatus != lime::OpStatus::Success)
 *          std::cout << "Failed to configure SDR device RF chip with " << moduleIndexes[i] << " index. Error: " << lime::ToString(configStatus) << std::endl;
 *    } 
 * 
 *    ... // Other program code
 * @endcode
 *
 * @if RST_SUPPORT
 * @else
 * @note To find out more about <b>moduleIndexes</b> parameter, @ref common_parameters "click here".  
 * @endif
 * 
 * SDR device configuration structure @ref lime::SDRConfig "SDRConfig" allows to configure each channel direction (Tx and Rx) individually. @ref lime::ChannelConfig::Direction "Click here" to view the list 
 * of individually configurable parameters for each channel direction. For gain parameter, it is recommended to set the generic gain type:
 * @code{.cpp}
 *    ... // Other program code
 *    uint8_t moduleIndex = 0;          // For SDR devices with single RF chip
 *    std::pair<lime::eGainTypes, double> TxGenericGain(lime::eGainTypes::GENERIC, 25); // in dB
 * 
 *    lime::SDRConfig LimeSDRMiniConf;
 *    LimeSDRMiniConf.channel[0].tx.gain.insert(TxGenericGain);         // Setting channel A, Tx direction generic gain alongside default device configuration
 * 
 *    configStatus = device->Configure(LimeSDRMiniConf, moduleIndex);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to configure SDR device with error: " << lime::ToString(configStatus) << std::endl;
 *    ... // Other program code
 * @endcode
 * 
 * To use GFIR filters in digital domain, enable automatic GFIR filter calculation and set up:
 * @code{.cpp}
 *    ... // Other program code
 *    uint8_t moduleIndex = 0;          // For SDR devices with single RF chip 
 *    lime::SDRConfig LimeSDRMiniConf;
 *    LimeSDRMiniConf.channel[0].tx.gfir.enabled = true;    // Enable the use of all GFIR filters for channel A, Tx direction. Alternatively, use false to disable all of them
 *    LimeSDRMiniConf.channel[0].tx.gfir.bandwidth = 2e6;   // Calculate GFIR coefficients for 2 MHz bandwidth
 * 
 *    configStatus = device->Configure(LimeSDRMiniConf, moduleIndex);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to configure SDR device with error: " << lime::ToString(configStatus) << std::endl;
 *    ... // Other program code
 * @endcode
 * 
 * For more information about the gain parameter and other device parameters, please visit @ref generic_config "Common control and configuration options" topic, which provides more information about individual parameters
 * and ways to set/get those individual parameters.  
 * For GFIR filter set up please check out this @ref fir_filter_config "section".  
 * For Test Signal parameter please check out this section. <b>TODO: Link to the actual section.</b>  
 * 
 * @subsection struct_conf_example SDR device configuration example
 * 
 * Example of minimal SDR device configuration for basic streaming using SDRConfig structure:
 * @code{.cpp}
 * int main()
 * {
 *     std::vector<lime::DeviceHandle> listOfDevices = lime::DeviceRegistry::enumerate();
 *     
 *     lime::OpStatus configStatus = lime::OpStatus::Success;
 *     lime::SDRDevice * device = lime::DeviceRegistry::makeDevice(listOfDevices.front());
 *     if(device == nullptr)
 *     {
 *         std::cout << "Failed to connect to SDR device\n";
 *         return 1;
 *     }
 *     
 *     uint8_t moduleIndex = 0;          // For SDR devices with single RF chip
 *     std::pair<lime::eGainTypes, double> TRxGenericGain(lime::eGainTypes::GENERIC, 25); // in dB
 *     lime::SDRConfig LimeSDRMiniConf;
 * 
 *     // Setting device into SISO mode, streaming channel - A
 *     LimeSDRMiniConf.channel[0].tx.enabled = true;
 *     LimeSDRMiniConf.channel[0].rx.enabled = true;
 * 
 *     // Disabling channel B. Enable for MIMO mode
 *     LimeSDRMiniConf.channel[1].tx.enabled = false;
 *     LimeSDRMiniConf.channel[1].rx.enabled = false;
 * 
 *     LimeSDRMiniConf.channel[0].tx.centerFrequency = 50e6; // in Hz
 *     LimeSDRMiniConf.channel[0].rx.centerFrequency = 50e6; // in Hz
 *     LimeSDRMiniConf.channel[0].tx.sampleRate = 5e6; // in Hz
 *     LimeSDRMiniConf.channel[0].rx.sampleRate = 5e6; // in Hz
 *     LimeSDRMiniConf.channel[0].tx.gain.insert(TRxGenericGain);  // Setting channel A, Tx direction generic gain
 *     LimeSDRMiniConf.channel[0].rx.gain.insert(TRxGenericGain);  // Setting channel A, Rx direction generic gain
 *     LimeSDRMiniConf.channel[0].tx.lpf = 10e6;   // bandwidth in Hz
 *     LimeSDRMiniConf.channel[0].rx.lpf = 10e6;   // bandwidth in Hz
 *     LimeSDRMiniConf.channel[0].tx.path = 1;  // antenna path - band1
 *     LimeSDRMiniConf.channel[0].rx.path = 2;  // antenna path - LNAL
 *     LimeSDRMiniConf.channel[0].tx.gfir.enabled = true;    // Enable the use of all GFIR filters for channel A, Tx direction. Alternatively, use false to disable all of them
 *     LimeSDRMiniConf.channel[0].tx.gfir.bandwidth = 2e6;   // Calculate GFIR coefficients for 2 MHz bandwidth
 * 
 * 
 *     configStatus = device->Configure(LimeSDRMiniConf, moduleIndex);
 *     if(configStatus != lime::OpStatus::Success)
 *        std::cout << "Failed to configure SDR device with error: " << lime::ToString(configStatus) << std::endl;
 *  
 *     ... // Other program code
 * 
 *     lime::DeviceRegistry::freeDevice(device);
 *     return 0;
 * }
 * @endcode
 * 
 * @section quick_conf_file From file
 * 
 * Quick SDR device configuration can also be performed using <b>.ini</b> files, by using @ref lime::SDRDevice::LoadConfig(uint8_t, const std::string&) "LoadConfig()" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0; // For SDR devices with single RF chip
 *    std::string configFile = "LimeSDRMini_V2_config.ini";
 *    configStatus = device->LoadConfig(moduleIndex, configFile);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to configure SDR device from file with error: " << lime::ToString(configStatus) << std::endl;
 *
 *    ... // Other program code
 * @endcode
 * 
 * @if RST_SUPPORT
 * @embedRstVerbatim{
 * .. important::
 *
 *    Make sure that the configuration file .ini is in the same directory as your executable (as is the case in the example above) or append a absolute or relative path to the configuration file name to specify the correct location of file.}
 * @else
 * @important Make sure that the configuration file <b>.ini</b> is in the same directory as your executable (as is the case in the example above) or append a absolute or relative path to the
 * configuration file name to specify the correct location of file.
 * @endif
 * 
 * To generate SDR device <b>.ini</b> file you must perform a first time SDR device configuration (check out <b>From structure</b> section above) and use the 
 * @ref lime::SDRDevice::SaveConfig(uint8_t, const std::string&) "SaveConfig()" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint8_t moduleIndex = 0; // For SDR devices with single RF chip
 *    std::string configFile = "LimeSDRMini_V2_config.ini";
 *    configStatus = device->SaveConfig(moduleIndex, configFile);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to configure SDR device from file with error: " << lime::ToString(configStatus) << std::endl;
 *
 *    ... // Other program code
 * @endcode
 * 
 * @if RST_SUPPORT
 * @embedRstVerbatim{
 * .. important::
 * 
 *    If only the name of configuration file .ini is specified, generated file is saved in the same directory as your executable (as is the case in the example above). To save file in a different directory, append a absolute or relative path to the configuration file name.}
 * @else
 * @important If only the name of configuration file <b>.ini</b> is specified, generated file is saved in the same directory as your executable (as is the case in the example above). To save file in a different
 * directory, append a absolute or relative path to the configuration file name.
 * @endif
 * 
 * Alternatively, SDR device configuration file <b>.ini</b> can also be generated using <b>limeGUI</b> application, which is bundled with the LimeSuiteNG suite. The <b>limeGUI</b> application allows to perform 
 * SDR device configuration using graphical user interface and then generate the final <b>.ini</b> configuration file, which can be loaded with 
 * @ref lime::SDRDevice::LoadConfig(uint8_t, const std::string&) "LoadConfig()" function. The <b>limeGUI</b> application allows to configure the basic SDR device parameters (center frequency, sample rate and etc.)
 * and other more advanced SDR device parameters, which are not directly exposed through public API and can only be toggled by directly writing and reading SDR device registers. This sub-topic does not provide any 
 * advice or example use of <b>limeGUI</b> application and is outside of scope of this guide book. Check out the <b>limeGUI</b> application documentation for more information about SDR device configuration options
 * and <b>.ini</b> file generation steps.
 */

/**
 * @page direct_config Direct register and parameter access
 * @ref dev_config "Back to the list of SDR device configuration sub-topics" 
 * 
 * For more advanced users, a more advanced and precise SDR device configuration is possible by directly accessing individual register parameters or entire registers of SDR device. Direct access to registers
 * also allows to configure SDR device features that are not directly exposed through API functions, but are otherwise possible to set up by using <b>limeGUI</b> graphical user interface application, which is 
 * part of LimeSuiteNG suite. Full description of register parameters is available in LMS7002M register map file. <b>TODO: Add the file name or file location for reference</b>
 * 
 * @if RST_SUPPORT
 * @embedRstVerbatim{
 * .. admonition:: Warning
 * 
 *    Direct register and parameter access requires advanced knowledge about the SDR device and LMS7002M chip. Activation of advanced features or precise configuration often requires modification of several registers in a defined order. Proceed at your own risk.}
 * @else
 * @warning Direct register and parameter access requires advanced knowledge about the SDR device and LMS7002M chip. Activation of advanced features or precise configuration often requires modification of several
 * registers in a defined order. Proceed at your own risk.
 * @endif
 * 
 * Supported register access methods:
 * <ol>
 *    <li>by register parameter key</li>
 *    <li>by addressing register parameter bit fields</li>
 *    <li>full register reads and writes</li>
 * </ol>
 * 
 * @section reg_param_conf Parameter key
 * 
 * To read register parameters by key, use @ref lime::SDRDevice::GetParameter(uint8_t, uint8_t, const std::string&) "GetParameter(const string& parameterKey)" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint16_t rxen_a = 0;
 *    uint8_t moduleIndex = 0;   // For SDR devices with a single RF chip
 *    std::string parameterKey = "RXEN_A";   // Power control for Rx A in MIMO mode parameter
 *    rxen_a = device->GetParameter(moduleIndex, lime::LMS7002M::Channel::ChA, parameterKey);
 *    std::cout << "Power control for Rx MIMO channel A is " << (rxen_a == 1 ? "enabled\n" : "disabled\n");
 * 
 *    ... // Other program code
 * @endcode
 * 
 * To set register parameters by key, use @ref lime::SDRDevice::SetParameter(uint8_t, uint8_t, const std::string&, uint16_t) "SetParameter(const string& parameterKey, uint16_t value)" function:
 * @code{.cpp}
 *    ... // Other program code
 * 
 *    uint16_t rxen_a = 0;  // Disabling power control
 *    uint8_t moduleIndex = 0;   // For SDR devices with a single RF chip
 *    std::string parameterKey = "RXEN_A";   // Power control for Rx A in MIMO mode parameter
 *    configStatus = device->SetParameter(moduleIndex, lime::LMS7002M::Channel::ChA, parameterKey, rxen_a);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to " << (rxen_a == 1 ? "enable" : "disable") << "Rx MIMO channel A power control with error: " << lime::ToString(configStatus) << std::endl;
 * 
 *    ... // Other program code
 * @endcode
 * 
 * LMS7002MCSR enumeration contains the names of parameters that can be read or modified.
 * 
 * @section reg_bitfield_conf Addressing parameter bit fields
 * 
 * Alternatively, parameters can be accessed by addressing bit fields in registers. This is especially usefull if parameter bit field is multiple bits long. 
 * For example, we can set and get the bit field value of interpolation ratio in a register. To get the parameter bit field
 * value, use @ref lime::SDRDevice::GetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t) "GetParameter(uint16_t address, uint8_t msb, uint16_t lsb)" function. Example code that gets the current
 * value of interpolation ratio bit field:
 * @code{.cpp}
 *    ... // Other program code
 *    uint16_t hbi_ovr = 0;
 *    uint16_t addr = 0x0203;
 *    uint8_t msb = 14;
 *    uint8_t = 12;
 *    uint8_t moduleIndex = 0;   // For SDR devices with single RF chip
 *    hbi_ovr = device->GetParameter(moduleIndex, lime::LMS7002M::Channel::ChA, addr, msb, lsb);
 *    std::cout << "Current interpolation stage ratio is set to ";
 *    if(hbi_ovr == 0)
 *        std::cout << "2\n";
 *    else if(hbi_ovr == 1)
 *        std::cout << "4\n";
 *    else if(hbi_ovr == 2)
 *        std::cout << "8\n";
 *    else if(hbi_ovr == 3)
 *        std::cout << "16\n";
 *    else if(hbi_ovr == 4)
 *        std::cout << "32\n";
 *    else
 *        std::cout << "bypass\n";
 *    ... // Other program code
 * @endcode
 * 
 * To set the parameter bit field value, use @ref lime::SDRDevice::SetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t, uint16_t) "SetParameter(uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)" function.
 * Example code that sets the new interpolation ratio by modifying interpolation ratio parameter bit field:
 * @code{.cpp}
 *    ... // Other program code
 *    uint16_t hbi_ovr = 3;   // New interpolation ratio is set to 16
 *    uint16_t addr = 0x0203;
 *    uint8_t msb = 14;
 *    uint8_t = 12;
 *    uint8_t moduleIndex = 0;   // For SDR devices with single RF chip
 *    device->SetParameter(moduleIndex, lime::LMS7002M::Channel::ChA, addr, msb, lsb, hbi_ovr);
 *    ... // Other program code
 * @endcode
 * 
 * In the examples above, the <b>msb</b> and <b>lsb</b> parameters address the ending and starting bit positions of a parameter bit field in a register. The bit field ending and starting positions are obtained from the 
 * register memory map documentation.
 * 
 * @section reg_full_conf Full read and write
 * 
 * For full register reads, use @ref lime::SDRDevice::ReadRegister(uint8_t, unsigned int, bool) "ReadRegister(unsigned int address)" function.
 * 
 * @if RST_SUPPORT
 * 
 * @embedRstVerbatim{
 * .. important::
 *
 *    Full read and write functions can also be used to read and write FPGA registers.}
 * 
 * @embedRstVerbatim{
 * .. important::
 * 
 *    Some SDR device configuration registers have duplicate registers that are addressed using the same address, but are physically located in different memory spaces. When directly accessing those registers, you must ensure that the correct register from the correct memory space is addressed by toggling the MAC parameter in register 0x0020.}
 * @else
 * @important Full read and write functions can also be used to read and write FPGA registers.
 * 
 * @important Some SDR device configuration registers have duplicate registers that are addressed using the same address, but are physically located in different memory spaces. When directly accessing those registers,
 * you must ensure that the correct register from the correct memory space is addressed by toggling the MAC parameter in register 0x0020.
 * @endif
 * 
 * Example code that reads the value of a register that contains the configuration of LNA, Loopback LNA and TIA amplifier gain settings for Rx direction:
 * @code{.cpp}
 *    ... // Other program code
 *    unsigned int rxgain = 0;
 *    unsigned int addr = 0x0113;
 *    uint8_t moduleIndex = 0;   // For SDR devices with single RF chip
 *    bool readFPGA = false;     // Reading SDR device LMS7002M RF chip register
 *    rxgain = device->ReadRegister(moduleIndex, addr, readFPGA);
 *    std::cout << "Gain settings for Rx direction: 0x" << std::hex << std::setfill('0') << std::setw(4) << rxgain << std::endl;
 *    
 *    ... // Other program code
 * @endcode
 * 
 * Read register default value shown in the image below matches the default value (0b0000001111000011 == 0x03C3) in SDR device register map documentation:
 * 
 * @image{inline} html dev_config_rxgain_mod.png
 * 
 * For full register writes, use @ref lime::SDRDevice::WriteRegister(uint8_t, unsigned int, unsigned int, bool) "WriteRegister(unsigned int address, unsigned int value)" function.
 * Building on the previous example, we update the Rx amplifier gain settings register value and then check if the register has been set:
 * @code{.cpp}
 *    ... // Other program code
 *    unsigned int rxgain = 0;
 *    unsigned int addr = 0x0113;
 *    uint8_t moduleIndex = 0;   // For SDR devices with single RF chip
 *    bool readFPGA = false;     // Reading SDR device LMS7002M RF chip register
 *    rxgain = device->ReadRegister(moduleIndex, addr, readFPGA);
 *    std::cout << "Default gain settings for Rx direction: 0x" << std::hex << std::setfill('0') << std::setw(4) << rxgain << std::endl;
 * 
 *    // Updating LNA gain setting, reducing from gmax-0 to gmax-5 value
 *    rxgain = rxgain & 0xFC3F;  // Clearing the LNA gain parameter bit field
 *    rxgain = rxgain | 0x0280;  // Setting the LNA gain parameter bit field with new value: 10 == gmax-5
 *    
 *    configStatus = device->WriteRegister(moduleIndex, addr, rxgain);
 *    if(configStatus != lime::OpStatus:Success)
 *       std::cout << "Failed to update LNA gain setting with error: " << lime::ToString(configStatus) << std::endl;
 *    
 *    rxgain = 0;
 *    rxgain = device->ReadRegister(moduleIndex, addr, readFPGA);
 *    std::cout << "New gain settings for Rx direction: 0x" << std::hex << std::setfill('0') << std::setw(4) << rxgain << std::endl;
 *    
 *    ... // Other program code    
 * @endcode
 * 
 * Output of the example code above:
 * 
 * @image{inline} html dev_config_rxgain_rw.png  
 * 
 */

/**
 * @page fir_filter_config FIR filter configuration
 * 
 * @ref dev_config "Back to the list of SDR device configuration sub-topics"  
 * 
 * General FIR (GFIR) filters allow to perform additional filtering or shaping of signal in digital domain and are part of LMS7002M RF chip transceiver signal processing (TSP) block. 
 * Each channel direction TSP has 3 GFIR filters whose coefficients can be auto set or individually customized. Additionally, GFIR filtering stages can be toggled individually.
 * Filtering stage control allows to enable only the required filters while preserving the auto calculated or custom loaded GFIR filter coefficients.
 * 
 * @section auto_gfir Auto setting coefficients
 * 
 * To auto calculate coefficients and enable all three stages of GFIR filter for a specified channel direction, use
 * @ref lime::SDRDevice::ConfigureGFIR(uint8_t, lime::TRXDir, uint8_t, lime::ChannelConfig::Direction::GFIRFilter) "ConfigureGFIR()" function:
 * @code{.cpp}
 *    ... // Other program code
 *    uint8_t moduleIndex = 0;   // For SDR device with single RF chip
 *    lime::ChannelConfig::Direction::GFIRFilter TxChaGfirSettings = {true, 2e6};
 *    configStatus = device->ConfigureGFIR(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, TxChaGfirSettings);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to auto set GFIR coefficients for Ch A, Tx Dir with error: " << lime::ToString(configStatus) << std::endl;
 *    ... // Other program code
 * @endcode
 * 
 * In the example code above, argument <b>TxChaGfirSettings</b> of @ref lime::ChannelConfig::Direction::GFIRFilter "GFIRFilter" type is declared, defined and is immediately initialized with values `{true, 2e6}`.
 * The values provided in the argument initialization stage enable the automatic calculation of GFIR filter coefficients for 2 MHz bandwidth. Coefficients for all three stages of GFIR filters (GFIR1, GFIR2 and GFIR3)
 * are calculated and loaded to a TSP of specified channel direction. Each GFIR1 and GFIR2 stage can store up to 40 coefficients, while the GFIR3 stage can contain up to 120 coefficients. GFIR filter coefficients are 
 * calculated using <b>"TODO: add the name of GFIR coef algoritm"</b> algorithm. The final GFIR coefficient values are dependent on specified bandwidth and current device TSP intepolation/decimation stage ratio.
 * 
 * To bypass all three GFIR filter stages at once, change the @ref lime::ChannelConfig::Direction::GFIRFilter "GFIRFilter" structure parameter @ref lime::ChannelConfig::Direction::GFIRFilter::enabled "enabled" to 
 * value <b>false</b> and re-run the ConfigureGFIR() function:
 * @code{.cpp}
 *    ... // Other program code
 *    uint8_t moduleIndex = 0;   // For SDR device with single RF chip
 *    lime::ChannelConfig::Direction::GFIRFilter TxChaGfirSettings = {false, 2e6};     // Now, bypass all of the GFIR filters in TSP
 *    configStatus = device->ConfigureGFIR(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, TxChaGfirSettings);
 *    if(configStatus != lime::OpStatus::Success)
 *       std::cout << "Failed to auto set GFIR coefficients for Ch A, Tx Dir with error: " << lime::ToString(configStatus) << std::endl;
 *    ... // Other program code
 * @endcode  
 * 
 * @section custom_gfir Custom loading coefficients
 * 
 * It is also possible to load custom GFIR filter coefficients if the automatically calculated GFIR filter coefficient values are not sufficient. To load the GFIR filter custom coefficients for a single GFIR stage,
 * use @ref lime::SDRDevice::SetGFIRCoefficients(uint8_t, lime::TRXDir, uint8_t, uint8_t, std::vector<double>) "SetGFIRCoefficients()":
 * @code{.cpp}
 *    ... // Other program code
 *    uint8_t moduleIndex = 0;            // For SDR devices with single RF chip
 *    uint8_t gfirID = 0;                 // GFIR1 filter stage
 *    std::vector<double> coeffs(40,0);   // Empty list of coefficients for GFIR1 filter
 * 
 *    ... // Calculating custom GFIR filter coefficients. Coefficients must be normalized in the range [-1; 1].
 * 
 *    configStatus = device->SetGFIRCoefficients(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, gfirID, coeffs);
 *    ... // Other program code
 * @endcode
 * 
 * GFIR filter stages (GFIR1, GFIR2 and GFIR3) for each channel direction are addressed using indexes:
 * <ol>
 *    <li>GFIR1 - 0</li>
 *    <li>GFIR2 - 1</li>
 *    <li>GFIR3 - 2</li>
 * </ol>
 * 
 * On each channel direction, both GFIR filters 1 and 2 can store a up to 40 coefficients, while GFIR filter 3 can store up to 120 coefficients. GFIR coefficients must be normalized in the range [-1; 1].
 * To get specific GFIR filter coefficients of a channel direction, use @ref lime::SDRDevice::GetGFIRCoefficients(uint8_t, lime::TRXDir, uint8_t, uint8_t) "GetGFIRCoefficients()" function:
 * @code{.cpp}
 *    ... // Other program code
 *    uint8_t moduleIndex = 0;            // For SDR devices with single RF chip
 *    uint8_t gfirID = 0;                 // GFIR1 filter stage
 *    std::vector<double> coeffs(40,0);   // Empty list of coefficients for GFIR1 filter
 * 
 *    coeffs = device->GetGFIRCoefficients(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, gfirID);
 *    ... // Other program code
 * @endcode
 * 
 * @section gfir_stage_toggle Controlling filtering stages
 * 
 * Additionally, it is also possible to enable or disable specific GFIR filter stages for each channel direction. This allows to bypass and use specific GFIR filtering stages without removing
 * the coefficients from the GFIR filters. To toggle GFIR filtering stages, use 
 * @ref lime::SDRDevice::SetGFIR(uint8_t, lime::TRXDir, uint8_t, uint8_t, bool) "SetGFIR()" function. This example code disables GFIR2 and GFIR3 stage filters for channel A, Tx direction:
 * @code{.cpp}
 *    ... // Other program code
 *    uint8_t moduleIndex = 0;   // For SDR devices with single RF chip
 *    uint8_t gfirOneID = 0;
 *    uint8_t gfirTwoID = 1;
 *    uint8_t gfirThreeID = 2;
 *    device->SetGFIR(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, gfirOneID, true);     // GFIR1 in ChA, Tx Dir TSP is enabled
 *    device->SetGFIR(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, gfirTwoID, false);    // GFIR2 in ChA, Tx Dir TSP is disabled
 *    device->SetGFIR(moduleIndex, lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA, gfirThreeID, false);  // GFIR3 in ChA, Tx Dir TSP is disabled
 *    
 *    ... // Other program code
 * @endcode
 */

/**
 * @page frequency_config Frequency configuration
 * 
 * 
*/

// ###########################################################
// This is the ending point of sub-topic pages for SDR 
// configuration topic
// ###########################################################

/**
 * @page dev_streaming SDR device stream set up and streaming
 * 
 * @ref guide_book_top "Back to the list of guide book topics"  
 * @ref dev_config "Back to SDR device configuration example topic"  
 * 
 * This example explains how to set up a stream between two LimeSDR devices and send data over RF medium.
 * Example code:
 * 
 * @code{.cpp}
 * @endcode
 * 
 * @ref guide_book_top "Back to the list of guide book topics"  
 * @ref hello_world_example "Go to minimal development template example"
 */

/**
 * @page hello_world_example Hello World example
 * 
 * @ref guide_book_top "Back to the list of guide book topics"  
 * 
 * This is a minimal example code that can be used as a starting point for further development. Public API members presented in this code have been reviewed in previous example topics.
 * Example code:
 * @code{.cpp}
 * @endcode
 *
 * @ref guide_book_top "Back to the list of guide book topics"
 */
// This is the end point of subpages for example page