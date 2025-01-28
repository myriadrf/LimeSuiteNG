#ifndef INCLUDED_LIMESUITENG_SDRDEVICE_BLOCK_BASE_H
#define INCLUDED_LIMESUITENG_SDRDEVICE_BLOCK_BASE_H

#include <gnuradio/limesuiteng/sdrdevice_source.h>

#include <memory>

#include "limesuiteng/types.h"
#include <gnuradio/logger.h>

namespace lime {
class SDRDevice;
class RFStream;
} // namespace lime

namespace gr {
namespace limesuiteng {

class sdrdevice_manager;
struct sdrdevice_context;

class sdrdevice_block_base
{
public:
    sdrdevice_block_base(lime::TRXDir dir,
                         const std::string& alias,
                         const std::string& deviceHandleHint,
                         uint32_t chipIndex,
                         const std::vector<int>& channelIndexes,
                         const std::string& dataFormat,
                         double sampleRate,
                         int rf_oversampling,
                         gr::logger_ptr logger,
                         gr::logger_ptr debug_logger);
    virtual ~sdrdevice_block_base();

    bool start();
    bool stop();

    void set_config_file(const std::string& file_path);
    double set_lo_frequency(double frequencyHz);
    double set_lpf_bandwidth(double bandwidthHz);
    bool set_antenna(const std::string& antenna_name);
    double set_gain_generic(double gain_dB);
    double set_nco_frequency(double frequency_offset_Hz);

protected:
    void ReleaseResources();
    uint32_t chipIndex;
    std::shared_ptr<sdrdevice_manager> devManager;
    std::shared_ptr<sdrdevice_context> devContext;
    const lime::TRXDir direction;
    bool autoAntenna;
    bool canWork;

private:
    gr::logger_ptr baselogger;
};

} // namespace limesuiteng
} // namespace gr

#endif /* INCLUDED_LIMESUITENG_SDRDEVICE_BLOCK_BASE_H */
