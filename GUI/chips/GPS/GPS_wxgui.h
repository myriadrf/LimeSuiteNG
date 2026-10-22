#pragma once

#include "ISOCPanel.h"

#include <wx/timer.h>

namespace lime {
class GPS;
} // namespace lime

class wxCheckBox;
class wxStaticText;

class GPS_wxgui : public ISOCPanel
{
  public:
    static ISOCPanel* Create(wxWindow* parent, wxWindowID id);
    GPS_wxgui(wxWindow* parent,
        wxWindowID id,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~GPS_wxgui();
    bool Initialize(lime::GPS* soc);
    bool Initialize(void* soc) override;
    void UpdateGUI() override;

  private:
    void OnPPSChange(wxCommandEvent& event);
    void OnStandByMode(wxCommandEvent& event);
    void OnRefresh(wxCommandEvent& event);
    void OnAutoRefresh(wxCommandEvent& event);
    void OnTimer(wxTimerEvent& event);

    lime::GPS* gps;

    wxChoice* ppsAvailability;
    wxSpinCtrl* ppsPulseWidth;
    wxButton* btnStandByMode;

    wxStaticText* lblStatus;
    wxCheckBox* chkAutoRefresh;
    wxTimer refreshTimer;
};
