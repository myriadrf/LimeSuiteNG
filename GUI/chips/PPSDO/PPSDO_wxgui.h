#pragma once

#include "ISOCPanel.h"

#include <wx/timer.h>

namespace lime {
class PPSDO;
} // namespace lime

class wxCheckBox;
class wxStaticText;
class wxTextCtrl;

class PPSDO_wxgui : public ISOCPanel
{
  public:
    static ISOCPanel* Create(wxWindow* parent, wxWindowID id);
    PPSDO_wxgui(wxWindow* parent,
        wxWindowID id,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~PPSDO_wxgui();
    bool Initialize(lime::PPSDO* soc);
    bool Initialize(void* soc) override;
    void UpdateGUI() override;

  private:
    void OnApplyConfig(wxCommandEvent& event);
    void OnEnable(wxCommandEvent& event);
    void OnReset(wxCommandEvent& event);
    void OnRefresh(wxCommandEvent& event);
    void OnAutoRefresh(wxCommandEvent& event);
    void OnTimer(wxTimerEvent& event);

    lime::PPSDO* ppsdo;

    wxTextCtrl* txtClockFrequency;
    wxTextCtrl* txtTolerance;
    wxCheckBox* chkEnable;
    wxCheckBox* chkAutoRefresh;

    wxStaticText* lblEnabled;
    wxStaticText* lblState;
    wxStaticText* lblAccuracy;
    wxStaticText* lblPPSActive;
    wxStaticText* lblError1s;
    wxStaticText* lblError10s;
    wxStaticText* lblError100s;
    wxStaticText* lblDAC;

    wxTimer refreshTimer;
};
