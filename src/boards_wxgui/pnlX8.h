#pragma once

#include "limesuiteng/SDRDevice.h"
#include <map>
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/spinctrl.h>

class wxStaticText;
class wxFlexGridSizer;
class wxCheckBox;
class wxRadioButton;
class wxButton;

class pnlXTRX;

class pnlX8 : public wxPanel
{
  public:
    pnlX8(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        int style = 0,
        wxString name = wxEmptyString);
    void Initialize(lime::SDRDevice* pControl);
    virtual ~pnlX8();
    virtual void UpdatePanel();

    void OnReadAll(wxCommandEvent& event);
    void OnWriteAll(wxCommandEvent& event);

  protected:
    std::vector<pnlXTRX*> subPanels;

    DECLARE_EVENT_TABLE()
};
