#include "lms7002_pnlXBUF_view.h"
#include "limesuiteng/LMS7002MCSR.h"
#include "commonWxHeaders.h"
#include <map>
#include "lms7002_gui_utilities.h"

using namespace lime;

lms7002_pnlXBUF_view::lms7002_pnlXBUF_view(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ILMS7002MTab(parent, id, pos, size, style)
{
    const int flags = 0;
    wxFlexGridSizer* fgSizer76;
    fgSizer76 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer76->SetFlexibleDirection(wxBOTH);
    fgSizer76->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizerPowerDowns;
    sbSizerPowerDowns = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Power down controls")), wxVERTICAL);

    chkPD_XBUF_RX =
        new wxCheckBox(sbSizerPowerDowns->GetStaticBox(), ID_PD_XBUF_RX, wxT("Power down Rx"), wxDefaultPosition, wxDefaultSize, 0);
    chkPD_XBUF_RX->SetToolTip(wxT("Power down signal PD_XBUF_RX"));

    sbSizerPowerDowns->Add(chkPD_XBUF_RX, 0, flags, 0);

    chkPD_XBUF_TX =
        new wxCheckBox(sbSizerPowerDowns->GetStaticBox(), ID_PD_XBUF_TX, wxT("Power down Tx"), wxDefaultPosition, wxDefaultSize, 0);
    chkPD_XBUF_TX->SetToolTip(wxT("Power down signal PD_XBUF_TX"));

    sbSizerPowerDowns->Add(chkPD_XBUF_TX, 0, flags, 0);

    chkEN_G_XBUF = new wxCheckBox(
        sbSizerPowerDowns->GetStaticBox(), ID_EN_G_XBUF, wxT("Enable XBUF module"), wxDefaultPosition, wxDefaultSize, 0);
    chkEN_G_XBUF->SetToolTip(wxT("Enable control for all the XBUF power downs"));

    sbSizerPowerDowns->Add(chkEN_G_XBUF, 0, flags, 0);

    fgSizer76->Add(sbSizerPowerDowns, 0, wxALL | wxALIGN_LEFT, 0);

    wxFlexGridSizer* fgSizer77;
    fgSizer77 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer77->SetFlexibleDirection(wxBOTH);
    fgSizer77->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkSLFB_XBUF_RX = new wxCheckBox(
        this, ID_SLFB_XBUF_RX, wxT("Rx Enable biasing the input's DC voltage "), wxDefaultPosition, wxDefaultSize, 0);
    chkSLFB_XBUF_RX->SetToolTip(wxT("Self biasing digital control SLFB_XBUF_RX"));

    fgSizer77->Add(chkSLFB_XBUF_RX, 0, flags, 0);

    chkSLFB_XBUF_TX = new wxCheckBox(
        this, ID_SLFB_XBUF_TX, wxT("Tx Enable biasing the input's DC voltage "), wxDefaultPosition, wxDefaultSize, 0);
    chkSLFB_XBUF_TX->SetToolTip(wxT("Self biasing digital control SLFB_XBUF_TX"));

    fgSizer77->Add(chkSLFB_XBUF_TX, 0, flags, 0);

    chkBYP_XBUF_RX =
        new wxCheckBox(this, ID_BYP_XBUF_RX, wxT("Shorts the Input 3.3 V buffer in XBUF RX"), wxDefaultPosition, wxDefaultSize, 0);
    chkBYP_XBUF_RX->SetToolTip(wxT("Shorts the Input 3.3V buffer in XBUF"));

    fgSizer77->Add(chkBYP_XBUF_RX, 0, flags, 0);

    chkBYP_XBUF_TX =
        new wxCheckBox(this, ID_BYP_XBUF_TX, wxT("Shorts the Input 3.3 V buffer in XBUF TX"), wxDefaultPosition, wxDefaultSize, 0);
    chkBYP_XBUF_TX->SetToolTip(wxT("Shorts the Input 3.3V buffer in XBUF"));

    fgSizer77->Add(chkBYP_XBUF_TX, 0, flags, 0);

    chkEN_OUT2_XBUF_TX = new wxCheckBox(this, ID_EN_OUT2_XBUF_TX, wxT("EN_OUT2_XBUF_TX"), wxDefaultPosition, wxDefaultSize, 0);
    chkEN_OUT2_XBUF_TX->SetToolTip(wxT("Enables the 2nd output of TX XBUF. This 2nd buffer goes to XBUF_RX. This should be active "
                                       "when only 1 XBUF is to be used"));

    fgSizer77->Add(chkEN_OUT2_XBUF_TX, 0, flags, 0);

    chkEN_TBUFIN_XBUF_RX =
        new wxCheckBox(this, ID_EN_TBUFIN_XBUF_RX, wxT("EN_TBUFIN_XBUF_RX"), wxDefaultPosition, wxDefaultSize, 0);
    chkEN_TBUFIN_XBUF_RX->SetToolTip(wxT("Disables the input from the external XOSC and buffers the 2nd input signal (from TX XBUF "
                                         "2nd output) to the RX. This should be active when only 1 XBUF is to be used"));

    fgSizer77->Add(chkEN_TBUFIN_XBUF_RX, 0, flags, 0);

    fgSizer76->Add(fgSizer77, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    SetSizer(fgSizer76);
    Layout();
    fgSizer76->Fit(this);

    // Connect Events
    chkPD_XBUF_RX->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlXBUF_view::ParameterChangeHandler), nullptr, this);
    chkPD_XBUF_TX->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlXBUF_view::ParameterChangeHandler), nullptr, this);
    chkEN_G_XBUF->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlXBUF_view::ParameterChangeHandler), nullptr, this);
    chkSLFB_XBUF_RX->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlXBUF_view::ParameterChangeHandler), nullptr, this);
    chkSLFB_XBUF_TX->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlXBUF_view::ParameterChangeHandler), nullptr, this);
    chkBYP_XBUF_RX->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlXBUF_view::ParameterChangeHandler), nullptr, this);
    chkBYP_XBUF_TX->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlXBUF_view::ParameterChangeHandler), nullptr, this);
    chkEN_OUT2_XBUF_TX->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlXBUF_view::ParameterChangeHandler), nullptr, this);
    chkEN_TBUFIN_XBUF_RX->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlXBUF_view::ParameterChangeHandler), nullptr, this);

    wndId2Enum[chkBYP_XBUF_RX] = LMS7002MCSR::BYP_XBUF_RX;
    wndId2Enum[chkBYP_XBUF_TX] = LMS7002MCSR::BYP_XBUF_TX;
    wndId2Enum[chkEN_G_XBUF] = LMS7002MCSR::EN_G_XBUF;
    wndId2Enum[chkEN_OUT2_XBUF_TX] = LMS7002MCSR::EN_OUT2_XBUF_TX;
    wndId2Enum[chkEN_TBUFIN_XBUF_RX] = LMS7002MCSR::EN_TBUFIN_XBUF_RX;
    wndId2Enum[chkPD_XBUF_RX] = LMS7002MCSR::PD_XBUF_RX;
    wndId2Enum[chkPD_XBUF_TX] = LMS7002MCSR::PD_XBUF_TX;
    wndId2Enum[chkSLFB_XBUF_RX] = LMS7002MCSR::SLFB_XBUF_RX;
    wndId2Enum[chkSLFB_XBUF_TX] = LMS7002MCSR::SLFB_XBUF_TX;

    LMS7002_WXGUI::UpdateTooltips(wndId2Enum, true);
}
