#ifndef WIDGET_DCCORRECTOR_H
#define WIDGET_DCCORRECTOR_H

#include <memory>

#include <wx/panel.h>

class wxCheckBox;
class wxCommandEvent;
class wxSpinCtrl;
class wxStaticText;
class wxWindow;

namespace lime {
class IDCCorrector;
}

/// @brief Controls for a lime::IDCCorrector. The offset is written as soon as it is changed, then read back.
class DCCorrectorsPanel : public wxPanel
{
  public:
    DCCorrectorsPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = wxT("DC"),
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~DCCorrectorsPanel();
    /// @param dev Shared ownership is kept by the panel.
    void Initialize(std::shared_ptr<lime::IDCCorrector> dev);
    /// @param dev Not owned, must outlive the panel or be replaced by another Initialize() call.
    void Initialize(lime::IDCCorrector* dev);
    /// @brief Reads the offset back from the device.
    void UpdateGUI();

  private:
    void WriteValues();
    void WriteEnabled();
    void UpdateEnabledControls();

    wxWindow* box;
    wxCheckBox* chkEnable;
    wxSpinCtrl* Icontrol;
    wxSpinCtrl* Qcontrol;
    wxStaticText* lblRange;
    std::shared_ptr<lime::IDCCorrector> owner;
    lime::IDCCorrector* device{};
};

#endif
