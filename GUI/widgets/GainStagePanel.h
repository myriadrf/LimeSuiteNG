#ifndef WIDGET_GAIN_STAGE_H
#define WIDGET_GAIN_STAGE_H

#include <functional>

#include <wx/panel.h>

class wxCommandEvent;
class wxSpinCtrlDouble;
class wxStaticBox;
class wxStaticText;

namespace lime {
class IGainStage;
}

/// @brief Controls for a single lime::IGainStage. The value is written as soon as it is changed, then read back.
class GainStagePanel : public wxPanel
{
  public:
    /// @brief The function called after the value has been written, e.g. to refresh other controls it affects.
    using ValueChangedCallback = std::function<void(lime::IGainStage* stage)>;

    /// @param title The panel title. If empty, the name of the gain stage is used.
    GainStagePanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = wxEmptyString,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~GainStagePanel();

    /// @param dev Not owned, must outlive the panel or be replaced by another Initialize() call.
    void Initialize(lime::IGainStage* dev);
    /// @brief Reads the value back from the device.
    void UpdateGUI();
    /// @brief Sets the function called after each value write (successful or not).
    void SetOnValueChanged(ValueChangedCallback callback);

  private:
    void OnValueChanged(wxCommandEvent& event);
    void OnRefresh(wxCommandEvent& event);

    wxString title;
    wxStaticBox* box;
    wxSpinCtrlDouble* value;
    wxStaticText* lblUnits;
    wxStaticText* lblRange;
    lime::IGainStage* device;
    ValueChangedCallback onValueChanged;
};

#endif
