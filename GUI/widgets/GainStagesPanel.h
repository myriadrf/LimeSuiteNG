#ifndef WIDGET_GAIN_STAGES_H
#define WIDGET_GAIN_STAGES_H

#include <vector>

#include <wx/panel.h>

class wxCommandEvent;
class wxFlexGridSizer;
class wxSpinCtrlDouble;
class wxSpinDoubleEvent;
class wxStaticText;
class wxWindow;

namespace lime {
class IGainStage;
class IRFChannel;
} // namespace lime

/// @brief Controls for a set of lime::IGainStage, one row per stage.
///
/// A value is written as soon as it is changed, then all stages are read back,
/// as setting one stage (e.g. "Generic") can change the others.
class GainStagesPanel : public wxPanel
{
  public:
    GainStagesPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = wxT("Gain"),
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~GainStagesPanel();

    /// @brief Shows all gain stages of the channel.
    /// @param channel Not owned, must outlive the panel or be replaced by another Initialize() call.
    void Initialize(lime::IRFChannel* channel);
    /// @brief Shows the given gain stages.
    /// @param stages Not owned, must outlive the panel or be replaced by another Initialize() call.
    void Initialize(const std::vector<lime::IGainStage*>& stages);
    /// @brief Reads all values back from the device.
    void UpdateGUI();

  private:
    void OnValueChanged(size_t index);
    void OnRefresh(wxCommandEvent& event);

    wxWindow* box;
    wxFlexGridSizer* grid;
    std::vector<lime::IGainStage*> gainStages;
    std::vector<wxSpinCtrlDouble*> values;
};

#endif
