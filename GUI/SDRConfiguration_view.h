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

class wxGrid;
class wxGridEvent;

namespace lime {
class IDCCorrector;
class IQuadratureErrorCorrector;
class IRFChannel;
class SDRDevice;
struct SDRConfig;
} // namespace lime

/// @brief Correction blocks of one direction (Rx or Tx) of a single channel.
///
/// Their values are not part of SDRConfig, they are always written to the device directly.
/// The pointers are null if the device does not provide the block.
struct ChannelDirectionGUI {
    lime::IDCCorrector* dc{};
    lime::IQuadratureErrorCorrector* qec{};
    lime::Range<float> dcRange;
    lime::Range<float> qecGainRange;
    lime::Range<float> qecPhaseRange;
};

/// @brief Controls of one direction (Rx or Tx): a table with one row per channel.
struct DirectionGUI {
    /// Columns of the channel table
    enum Column {
        COL_ENABLE,
        COL_ANTENNA,
        COL_GAIN,
        COL_LPF,
        COL_NCO,
        COL_CALIBRATE,
        COL_DC_ENABLE,
        COL_DC_I,
        COL_DC_Q,
        COL_QEC_ENABLE,
        COL_QEC_GAIN,
        COL_QEC_PHASE,
        COL_COUNT
    };

    wxGrid* table{};
    wxArrayString antennaNames;
    lime::Range<double> gainRange{ 0, 100 };
    wxString columnTooltips[COL_COUNT];
    std::vector<ChannelDirectionGUI> channels;
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
    void CreateChannelTable(wxWindow* parent, lime::TRXDir dir);
    /// @brief Finds the channel's correction blocks and sets up their cells.
    void InitChannelBlocks(uint8_t channel, lime::TRXDir dir);
    /// @return The tooltip of a channel table cell, row < 0 for the column label.
    wxString GetCellTooltip(lime::TRXDir dir, int row, int col) const;
    /// @brief Makes a table cell read-only and grayed out, or editable again.
    void SetCellEditable(lime::TRXDir dir, int row, int col, bool editable);
    /// @brief Validates the edited cell and, in immediate mode, sends it to the device.
    void OnChannelCellChanged(lime::TRXDir dir, wxGridEvent& event);

    // Channel table values
    bool IsChannelEnabled(lime::TRXDir dir, uint8_t channel) const;
    int GetAntenna(lime::TRXDir dir, uint8_t channel) const;
    double GetGain(lime::TRXDir dir, uint8_t channel) const;
    double GetLPF(lime::TRXDir dir, uint8_t channel) const;
    double GetNCO(lime::TRXDir dir, uint8_t channel) const;
    bool IsCalibrateOnApply(lime::TRXDir dir, uint8_t channel) const;

    /// @brief Calls onCommit whenever the user commits an edit (Enter, or focus loss after a change).
    void BindValueField(wxTextCtrl* field, std::function<void()> onCommit);
    /// @brief Executes a device call when in immediate mode and reports its outcome.
    void ApplyImmediate(const wxString& what, const std::function<lime::OpStatus()>& operation);
    /// @brief Executes a device call in any mode and reports its outcome.
    void Execute(const wxString& what, const std::function<lime::OpStatus()>& operation);
    void Calibrate(uint8_t channel, lime::TRXDir dir);
    /// @return The generic RF SoC channel, or nullptr if the device does not provide it.
    lime::IRFChannel* GetRFChannel(uint8_t channel, lime::TRXDir dir) const;
    /// @brief Reads back the correction block cells, after a change that can affect them (e.g. calibration).
    void UpdateChannelBlocks(uint8_t channel, lime::TRXDir dir);

    void ApplyReferenceClock();
    void ApplySampleRate();
    void ApplyLO(lime::TRXDir dir);
    void OnTDDChanged();

    DirectionGUI& Direction(lime::TRXDir dir) { return dir == lime::TRXDir::Rx ? rxGUI : txGUI; }
    const DirectionGUI& Direction(lime::TRXDir dir) const { return dir == lime::TRXDir::Rx ? rxGUI : txGUI; }
    std::vector<ChannelDirectionGUI>& Channels(lime::TRXDir dir) { return Direction(dir).channels; }
    const std::vector<ChannelDirectionGUI>& Channels(lime::TRXDir dir) const { return Direction(dir).channels; }

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
    DirectionGUI rxGUI;
    DirectionGUI txGUI;
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
