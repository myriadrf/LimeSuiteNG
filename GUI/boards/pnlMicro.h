#pragma once

#include "limesuiteng/SDRDevice.h"
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/spinctrl.h>
class wxStaticText;
class wxFlexGridSizer;
class wxCheckBox;

class wxRadioButton;
class wxButton;

class pnlMicro : public wxPanel
{
  public:
    pnlMicro(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        int style = 0,
        wxString name = wxEmptyString);
    void Initialize(lime::SDRDevice* device, const std::string& spiSlaveName = std::string("FPGA"));
    virtual ~pnlMicro();
    virtual void UpdatePanel();

    void OnReadAll(wxCommandEvent& event);
    void OnWriteAll(wxCommandEvent& event);

  protected:
    void OnInputChange(wxCommandEvent& event);
    void OnClockSourceChanged(wxCommandEvent& event);

    wxCheckBox* TDDCntrl;
    wxChoice* cmbTxPath;
    wxChoice* cmbRxPath;
    wxRadioBox* rgrEXT_CLK_CTRL;

    int chipSelect;
    lime::SDRDevice* device;
    DECLARE_EVENT_TABLE()
};
