#ifndef LIME_SDR_CONFIGURATION_VIEW
#define LIME_SDR_CONFIGURATION_VIEW

#include <wx/panel.h>
#include "commonWxForwardDeclarations.h"

#include <functional>
#include <string>
#include <vector>

#include "ISOCPanel.h"
#include "limesuiteng/types.h"
#include "limesuiteng/OpStatus.h"

class wxSpinCtrlDouble;

namespace lime {
class SDRDevice;
struct SDRConfig;
} // namespace lime

/// @brief Controls of one direction (Rx or Tx) of a single channel.
struct ChannelDirectionGUI {
    wxCheckBox* enable{};
    wxChoice* antenna{};
    wxSpinCtrlDouble* gain{}; ///< Generic gain (dB), eGainTypes::GENERIC
    wxTextCtrl* lpf{};
    wxTextCtrl* nco{};
    wxCheckBox* calibrate{}; ///< Run DC/IQ calibration as part of the configuration (Aggregate mode)
    wxButton* btnCalibrate{}; ///< Run DC/IQ calibration now
};

/// @brief Simplified generic controls of a single RF SoC.
///
/// Has a section with parameters shared by all channels (reference clock, sampling rate, LOs)
/// and one section per direction (Rx, Tx), each with its own set of channels, as the number of Rx and
/// Tx channels can differ. In immediate mode every changed value is sent to the device right away
/// through the matching SDRDevice setter. Otherwise values are only collected by FillConfig().
class SOCConfig_view : public wxPanel
{
  public:
    using StatusReporter = std::function<void(const wxString& message, bool isError)>;

    SOCConfig_view(wxWindow* parent, lime::SDRDevice* device, uint8_t socIndex, StatusReporter reporter);

    void SetImmediateMode(bool immediate);
    /// @brief Fills in the configuration of this SoC from the GUI values.
    void FillConfig(lime::SDRConfig& config) const;
    /// @brief Updates the GUI values from the device's current state.
    void ReadFromDevice();
    uint8_t GetSOCIndex() const { return socIndex; }

  private:
    wxSizer* CreateSharedSection(wxWindow* parent);
    wxSizer* CreateDirectionSection(wxWindow* parent, lime::TRXDir dir);
    void CreateDirectionControls(wxWindow* parent, uint8_t channel, lime::TRXDir dir);

    /// @brief Calls onCommit whenever the user commits an edit (Enter, or focus loss after a change).
    void BindValueField(wxTextCtrl* field, std::function<void()> onCommit);
    /// @brief Executes a device call when in immediate mode and reports its outcome.
    void ApplyImmediate(const wxString& what, const std::function<lime::OpStatus()>& operation);
    void Calibrate(uint8_t channel, lime::TRXDir dir);

    void ApplyReferenceClock();
    void ApplySampleRate();
    void ApplyLO(lime::TRXDir dir);
    void OnTDDChanged();

    std::vector<ChannelDirectionGUI>& Channels(lime::TRXDir dir) { return dir == lime::TRXDir::Rx ? rxChannels : txChannels; }
    const std::vector<ChannelDirectionGUI>& Channels(lime::TRXDir dir) const
    {
        return dir == lime::TRXDir::Rx ? rxChannels : txChannels;
    }

    double GetSampleRate() const;
    uint8_t GetOversample() const;

    lime::SDRDevice* device;
    uint8_t socIndex;
    bool immediateMode{ true };
    StatusReporter report;

    wxTextCtrl* refClock{};
    wxChoice* refClockSource{};
    wxTextCtrl* sampleRate{};
    wxChoice* oversample{};
    wxTextCtrl* rxLO{};
    wxTextCtrl* txLO{};
    wxCheckBox* tdd{};
    std::vector<ChannelDirectionGUI> rxChannels;
    std::vector<ChannelDirectionGUI> txChannels;
};

/// @brief Generic SDR device configuration panel.
///
/// Works in two modes:
///  - Immediate: every changed parameter is configured to the device right away using that parameter's API.
///  - Aggregate: nothing is sent while editing. Pressing "Apply"
///    gathers all parameters into one SDRConfig per SoC and executes it with SDRDevice::Configure().
class SDRConfiguration_view : public ISOCPanel
{
  public:
    SDRConfiguration_view(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);
    bool Initialize(void* device) override;
    /// @brief Reads the current device state into the GUI.
    void UpdateGUI() override;

  protected:
    SDRConfiguration_view() = delete;
    void OnModeChanged();
    void ApplyConfiguration();
    void ReportStatus(const wxString& message, bool isError);

    std::vector<SOCConfig_view*> socGUI;
    lime::SDRDevice* sdrDevice;
    wxBoxSizer* mainSizer;
    wxBoxSizer* socSizer;
    wxRadioBox* modeSelect;
    wxButton* btnApply;
    wxButton* btnReadBack;
    wxStaticText* txtStatus;
};

#endif // LIME_SDR_CONFIGURATION_VIEW
