#ifndef WIDGET_LPF_H
#define WIDGET_LPF_H

#include <wx/panel.h>

class wxSpinCtrlDouble;
class wxStaticText;
class wxCommandEvent;

namespace lime {
class ILPF;
}

/// @brief Controls for a lime::ILPF. Tuning can be slow, so it is done on button press.
class LPFPanel : public wxPanel
{
  public:
    LPFPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = wxT("LPF"),
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~LPFPanel();

    /// @param dev Not owned, must outlive the panel or be replaced by another Initialize() call.
    void Initialize(lime::ILPF* dev);

  private:
    void OnTune(wxCommandEvent& event);

    wxSpinCtrlDouble* bandwidth;
    wxStaticText* lblRange;
    lime::ILPF* device;
};

#endif
