#ifndef __LMS8dlgTempCalibrate__
#define __LMS8dlgTempCalibrate__

namespace lime {
class LMS8001;
}

class lms8_dlgTempCalibrate : public wxDialog
{
  public:
    lms8_dlgTempCalibrate(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = wxEmptyString,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long styles = 0);
    virtual ~lms8_dlgTempCalibrate();

    void Initialize(lime::LMS8001* pControl);

    void OnClick_btnCalibrate(wxCommandEvent& event);
    void OnClick_btnReset(wxCommandEvent& event);

  protected:
    wxStaticText* m_staticText6;
    wxTextCtrl* txtValue;
    wxStaticText* m_staticText8;
    wxTextCtrl* txtTemperature;
    wxStaticText* m_staticText10;
    wxStaticText* sttxtT0;
    wxButton* btnCalibrate;
    wxButton* btnReset;

    lime::LMS8001* lmsControl;
};

#endif // __dlgTempCalibrate__
