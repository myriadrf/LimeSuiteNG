#include "lms8001_pnlPLLConfig_view.h"
#include <map>
#include "lms8001_gui_utilities.h"

#include "chips/LMS8001/LMS8001.h"

using namespace lime;

lms8001_pnlPLLConfig_view::lms8001_pnlPLLConfig_view(
    wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ILMS8001Tab(parent, id, pos, size, style)
{
    wxFlexGridSizer* fgSizer68;
    fgSizer68 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer68->SetFlexibleDirection(wxBOTH);
    fgSizer68->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_NOTEBOOK_PLLCONFIG = new wxNotebook(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_PLLCONFIG = new wxPanel(ID_NOTEBOOK_PLLCONFIG, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer69;
    fgSizer69 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer69->SetFlexibleDirection(wxBOTH);
    fgSizer69->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer126;
    fgSizer126 = new wxFlexGridSizer(1, 0, 0, 0);
    fgSizer126->SetFlexibleDirection(wxBOTH);
    fgSizer126->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer642;
    fgSizer642 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer642->SetFlexibleDirection(wxBOTH);
    fgSizer642->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer454;
    sbSizer454 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("VCO LDO")), wxVERTICAL);

    wxFlexGridSizer* fgSizer564;
    fgSizer564 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer564->SetFlexibleDirection(wxBOTH);
    fgSizer564->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkEN_VCOBIAS = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("EN_VCOBIAS"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer564->Add(chkEN_VCOBIAS, 0, wxALL, 0);

    chkBYP_VCOREG = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("BYP_VCOREG"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer564->Add(chkBYP_VCOREG, 0, wxALL, 0);

    chkCURLIM_VCOREG =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("CURLIM_VCOREG"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer564->Add(chkCURLIM_VCOREG, 0, wxALL, 0);

    chkSPDUP_VCOREG =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("SPDUP_VCOREG"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer564->Add(chkSPDUP_VCOREG, 0, wxALL, 0);

    m_staticText991 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("VCO LDO Voltage"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText991->Wrap(-1);
    fgSizer564->Add(m_staticText991, 0, wxALL, 0);

    cmbVDIV_VCOREG =
        new wxComboBox(ID_PANEL_PLLCONFIG, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer564->Add(cmbVDIV_VCOREG, 0, wxALL, 0);

    sbSizer454->Add(fgSizer564, 1, wxEXPAND, 5);

    fgSizer642->Add(sbSizer454, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer455;
    sbSizer455 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Clock Buffer")), wxVERTICAL);

    chkPLL_XBUF_SLFBEN =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("PLL_XBUF_SLFBEN"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer455->Add(chkPLL_XBUF_SLFBEN, 0, wxALL, 0);

    chkPLL_XBUF_BYPEN =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("PLL_XBUF_BYPEN"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer455->Add(chkPLL_XBUF_BYPEN, 0, wxALL, 0);

    chkPLL_XBUF_EN = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("PLL_XBUF_EN"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer455->Add(chkPLL_XBUF_EN, 0, wxALL, 0);

    fgSizer642->Add(sbSizer455, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer457;
    sbSizer457 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Manual Calibration")), wxVERTICAL);

    m_staticText99211 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Cap. bank value"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText99211->Wrap(-1);
    sbSizer457->Add(m_staticText99211, 0, wxALL, 0);

    cmbVCO_FREQ_MAN = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 128);
    sbSizer457->Add(cmbVCO_FREQ_MAN, 0, wxALL, 0);

    m_staticText9923 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("VCO Core"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText9923->Wrap(-1);
    sbSizer457->Add(m_staticText9923, 0, wxALL, 0);

    cmbVCO_SEL_MAN = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 3, 0);
    sbSizer457->Add(cmbVCO_SEL_MAN, 0, wxALL, 0);

    wxFlexGridSizer* fgSizer565;
    fgSizer565 = new wxFlexGridSizer(2, 3, 0, 0);
    fgSizer565->SetFlexibleDirection(wxBOTH);
    fgSizer565->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText99231 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("LOW"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText99231->Wrap(-1);
    fgSizer565->Add(m_staticText99231, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 5);

    m_staticText99232 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("EQ"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText99232->Wrap(-1);
    fgSizer565->Add(m_staticText99232, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 5);

    m_staticText99233 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("HIGH"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText99233->Wrap(-1);
    fgSizer565->Add(m_staticText99233, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 5);

    chkFREQ_LOW = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    chkFREQ_LOW->Enable(false);

    fgSizer565->Add(chkFREQ_LOW, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    chkFREQ_EQUAL = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    chkFREQ_EQUAL->Enable(false);

    fgSizer565->Add(chkFREQ_EQUAL, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    chkFREQ_HIGH = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    chkFREQ_HIGH->Enable(false);

    fgSizer565->Add(chkFREQ_HIGH, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    sbSizer457->Add(fgSizer565, 0, wxEXPAND, 5);

    chkCTUNE_STEP_DONE =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("CTUNE_STEP_DONE"), wxDefaultPosition, wxDefaultSize, 0);
    chkCTUNE_STEP_DONE->Enable(false);

    sbSizer457->Add(chkCTUNE_STEP_DONE, 0, wxALL, 0);

    chkCTUNE_START = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("CTUNE_START"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer457->Add(chkCTUNE_START, 0, wxALL, 0);

    chkCTUNE_EN = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("CTUNE_EN"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer457->Add(chkCTUNE_EN, 0, wxALL, 0);

    fgSizer642->Add(sbSizer457, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer505;
    sbSizer505 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Status")), wxVERTICAL);

    m_staticText1184 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("VCO voltage"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1184->Wrap(-1);
    sbSizer505->Add(m_staticText1184, 0, wxALL, 0);

    wxFlexGridSizer* fgSizer5651;
    fgSizer5651 = new wxFlexGridSizer(2, 3, 0, 0);
    fgSizer5651->SetFlexibleDirection(wxBOTH);
    fgSizer5651->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkVTUNE_HIGH = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_VTUNE_HIGH, wxT("High"), wxDefaultPosition, wxDefaultSize, 0);
    chkVTUNE_HIGH->Enable(false);

    fgSizer5651->Add(chkVTUNE_HIGH, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    chkVTUNE_LOW = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_VTUNE_LOW, wxT("Low"), wxDefaultPosition, wxDefaultSize, 0);
    chkVTUNE_LOW->Enable(false);

    fgSizer5651->Add(chkVTUNE_LOW, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    chkPLL_LOCK = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_PLL_LOCK, wxT("PLL Lock"), wxDefaultPosition, wxDefaultSize, 0);
    chkPLL_LOCK->Enable(false);

    fgSizer5651->Add(chkPLL_LOCK, 0, wxALL, 0);

    sbSizer505->Add(fgSizer5651, 1, wxEXPAND, 5);

    fgSizer642->Add(sbSizer505, 1, wxEXPAND, 5);

    fgSizer126->Add(fgSizer642, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer456;
    sbSizer456 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Auto Calibration")), wxVERTICAL);

    chkFCAL_START = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("FCAL_START"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer456->Add(chkFCAL_START, 0, wxALL, 0);

    chkVCO_SEL_FINAL_VAL =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("VCO_SEL_FINAL_VAL"), wxDefaultPosition, wxDefaultSize, 0);
    chkVCO_SEL_FINAL_VAL->Enable(false);

    sbSizer456->Add(chkVCO_SEL_FINAL_VAL, 0, wxALL, 0);

    m_staticText992 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("VCO Core"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText992->Wrap(-1);
    sbSizer456->Add(m_staticText992, 0, wxALL, 0);

    cmbVCO_SEL_FINAL = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 3, 0);
    cmbVCO_SEL_FINAL->Enable(false);

    sbSizer456->Add(cmbVCO_SEL_FINAL, 0, wxALL, 0);

    chkFREQ_FINAL_VAL =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("FREQ_FINAL_VAL"), wxDefaultPosition, wxDefaultSize, 0);
    chkFREQ_FINAL_VAL->Enable(false);

    sbSizer456->Add(chkFREQ_FINAL_VAL, 0, wxALL, 0);

    m_staticText9921 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Cap. bank value"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText9921->Wrap(-1);
    sbSizer456->Add(m_staticText9921, 0, wxALL, 0);

    cmbFREQ_FINAL = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 0);
    cmbFREQ_FINAL->Enable(false);

    sbSizer456->Add(cmbFREQ_FINAL, 0, wxALL, 0);

    chkVCO_SEL_FORCE =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("VCO_SEL_FORCE"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer456->Add(chkVCO_SEL_FORCE, 0, wxALL, 0);

    m_staticText9922 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("VCO Core Initial"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText9922->Wrap(-1);
    sbSizer456->Add(m_staticText9922, 0, wxALL, 0);

    cmbVCO_SEL_INIT = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 3, 2);
    sbSizer456->Add(cmbVCO_SEL_INIT, 0, wxALL, 0);

    m_staticText99221 =
        new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Cap. bank search start bit"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText99221->Wrap(-1);
    sbSizer456->Add(m_staticText99221, 0, wxALL, 0);

    cmbFREQ_INIT_POS = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 7, 7);
    sbSizer456->Add(cmbFREQ_INIT_POS, 0, wxALL, 0);

    m_staticText992211 =
        new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Cap. bank initial value"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText992211->Wrap(-1);
    sbSizer456->Add(m_staticText992211, 0, wxALL, 0);

    cmbFREQ_INIT = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 0);
    sbSizer456->Add(cmbFREQ_INIT, 0, wxALL, 0);

    m_staticText9922111 =
        new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("VCO settling time"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText9922111->Wrap(-1);
    sbSizer456->Add(m_staticText9922111, 0, wxALL, 0);

    cmbFREQ_SETTLING_N = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 0);
    sbSizer456->Add(cmbFREQ_SETTLING_N, 0, wxALL, 0);

    m_staticText99221111 =
        new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("VCO intial settling time"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText99221111->Wrap(-1);
    sbSizer456->Add(m_staticText99221111, 0, wxALL, 0);

    cmbVTUNE_WAIT_N = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 64);
    sbSizer456->Add(cmbVTUNE_WAIT_N, 0, wxALL, 0);

    m_staticText992211111 =
        new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Max. cap. bank in VCO sel."), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText992211111->Wrap(-1);
    sbSizer456->Add(m_staticText992211111, 0, wxALL, 0);

    cmbVCO_SEL_FREQ_MAX = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 250);
    sbSizer456->Add(cmbVCO_SEL_FREQ_MAX, 0, wxALL, 0);

    m_staticText9922111111 =
        new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Min. cap. bank in VCO sel."), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText9922111111->Wrap(-1);
    sbSizer456->Add(m_staticText9922111111, 0, wxALL, 0);

    cmbVCO_SEL_FREQ_MIN = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 5);
    sbSizer456->Add(cmbVCO_SEL_FREQ_MIN, 0, wxALL, 0);

    fgSizer126->Add(sbSizer456, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer643;
    fgSizer643 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer643->SetFlexibleDirection(wxBOTH);
    fgSizer643->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer369;
    fgSizer369 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer369->SetFlexibleDirection(wxBOTH);
    fgSizer369->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer3941;
    sbSizer3941 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("PLL Profile Select")), wxVERTICAL);

    wxStaticBoxSizer* sbSizer118;
    sbSizer118 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Simple")), wxVERTICAL);

    wxFlexGridSizer* fgSizer152;
    fgSizer152 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer152->SetFlexibleDirection(wxBOTH);
    fgSizer152->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkSelectProfile = new wxCheckBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Select Profile"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer152->Add(chkSelectProfile, 0, wxALL, 3);

    chkPLL_CFG_INT_SEL =
        new wxComboBox(ID_PANEL_PLLCONFIG, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    chkPLL_CFG_INT_SEL->Append(wxT("0"));
    chkPLL_CFG_INT_SEL->Append(wxT("1"));
    chkPLL_CFG_INT_SEL->Append(wxT("2"));
    chkPLL_CFG_INT_SEL->Append(wxT("3"));
    chkPLL_CFG_INT_SEL->Append(wxT("4"));
    chkPLL_CFG_INT_SEL->Append(wxT("5"));
    chkPLL_CFG_INT_SEL->Append(wxT("6"));
    chkPLL_CFG_INT_SEL->Append(wxT("7"));
    chkPLL_CFG_INT_SEL->SetSelection(0);
    chkPLL_CFG_INT_SEL->Enable(false);

    fgSizer152->Add(chkPLL_CFG_INT_SEL, 0, wxALL, 0);

    sbSizer118->Add(fgSizer152, 1, wxEXPAND, 5);

    sbSizer3941->Add(sbSizer118, 0, wxEXPAND, 5);

    ID_NOTEBOOK_MUX_CONTROL_PLL = new wxNotebook(ID_PANEL_PLLCONFIG, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0 =
        new wxPanel(ID_NOTEBOOK_MUX_CONTROL_PLL, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer8822;
    fgSizer8822 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer8822->SetFlexibleDirection(wxBOTH);
    fgSizer8822->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer1602;
    fgSizer1602 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1602->SetFlexibleDirection(wxBOTH);
    fgSizer1602->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer1682;
    sbSizer1682 = new wxStaticBoxSizer(new wxStaticBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer1592;
    fgSizer1592 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer1592->SetFlexibleDirection(wxBOTH);
    fgSizer1592->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkPLL_CFG_SEL0_INTERNAL =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    chkPLL_CFG_SEL0_INTERNAL->SetValue(true);
    fgSizer1592->Add(chkPLL_CFG_SEL0_INTERNAL, 0, wxALL, 0);

    chkPLL_CFG_SEL0_INVERT =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1592->Add(chkPLL_CFG_SEL0_INVERT, 0, wxALL, 0);

    sbSizer1682->Add(fgSizer1592, 1, wxEXPAND, 5);

    fgSizer1602->Add(sbSizer1682, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer11522;
    sbSizer11522 = new wxStaticBoxSizer(new wxStaticBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8532;
    fgSizer8532 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer8532->SetFlexibleDirection(wxBOTH);
    fgSizer8532->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText26510 = new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26510->Wrap(-1);
    fgSizer8532->Add(m_staticText26510, 0, wxALL, 5);

    m_staticText26512 = new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26512->Wrap(-1);
    fgSizer8532->Add(m_staticText26512, 0, wxALL, 5);

    m_staticText26522 = new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26522->Wrap(-1);
    fgSizer8532->Add(m_staticText26522, 0, wxALL, 5);

    m_staticText26532 = new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26532->Wrap(-1);
    fgSizer8532->Add(m_staticText26532, 0, wxALL, 5);

    m_staticText26542 = new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26542->Wrap(-1);
    fgSizer8532->Add(m_staticText26542, 0, wxALL, 5);

    m_staticText26552 = new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26552->Wrap(-1);
    fgSizer8532->Add(m_staticText26552, 0, wxALL, 5);

    m_staticText26562 = new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26562->Wrap(-1);
    fgSizer8532->Add(m_staticText26562, 0, wxALL, 5);

    m_staticText26572 = new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26572->Wrap(-1);
    fgSizer8532->Add(m_staticText26572, 0, wxALL, 5);

    m_staticText26582 = new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26582->Wrap(-1);
    fgSizer8532->Add(m_staticText26582, 0, wxALL, 5);

    chkPLL_CFG_SEL0_MASK0 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_PLL_CFG_SEL0_MASK0, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkPLL_CFG_SEL0_MASK0, 0, wxALL, 0);

    chkPLL_CFG_SEL0_MASK1 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_PLL_CFG_SEL0_MASK1, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkPLL_CFG_SEL0_MASK1, 0, wxALL, 0);

    chkPLL_CFG_SEL0_MASK2 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_PLL_CFG_SEL0_MASK2, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkPLL_CFG_SEL0_MASK2, 0, wxALL, 0);

    chkPLL_CFG_SEL0_MASK3 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_PLL_CFG_SEL0_MASK3, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkPLL_CFG_SEL0_MASK3, 0, wxALL, 0);

    chkPLL_CFG_SEL0_MASK4 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_PLL_CFG_SEL0_MASK4, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkPLL_CFG_SEL0_MASK4, 0, wxALL, 0);

    chkPLL_CFG_SEL0_MASK5 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_PLL_CFG_SEL0_MASK5, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkPLL_CFG_SEL0_MASK5, 0, wxALL, 0);

    chkPLL_CFG_SEL0_MASK6 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_PLL_CFG_SEL0_MASK6, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkPLL_CFG_SEL0_MASK6, 0, wxALL, 0);

    chkPLL_CFG_SEL0_MASK7 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_PLL_CFG_SEL0_MASK7, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkPLL_CFG_SEL0_MASK7, 0, wxALL, 0);

    chkPLL_CFG_SEL0_MASK8 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, ID_PLL_CFG_SEL0_MASK8, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkPLL_CFG_SEL0_MASK8, 0, wxALL, 0);

    sbSizer11522->Add(fgSizer8532, 1, wxEXPAND, 5);

    fgSizer1602->Add(sbSizer11522, 1, wxEXPAND, 5);

    fgSizer8822->Add(fgSizer1602, 1, wxEXPAND, 5);

    ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0->SetSizer(fgSizer8822);
    ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0->Layout();
    fgSizer8822->Fit(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0);
    ID_NOTEBOOK_MUX_CONTROL_PLL->AddPage(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0, wxT("Bit 0"), true);
    ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1 =
        new wxPanel(ID_NOTEBOOK_MUX_CONTROL_PLL, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer88221;
    fgSizer88221 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer88221->SetFlexibleDirection(wxBOTH);
    fgSizer88221->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer16021;
    fgSizer16021 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer16021->SetFlexibleDirection(wxBOTH);
    fgSizer16021->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer16821;
    sbSizer16821 = new wxStaticBoxSizer(new wxStaticBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer15921;
    fgSizer15921 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer15921->SetFlexibleDirection(wxBOTH);
    fgSizer15921->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkPLL_CFG_SEL1_INTERNAL =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    chkPLL_CFG_SEL1_INTERNAL->SetValue(true);
    fgSizer15921->Add(chkPLL_CFG_SEL1_INTERNAL, 0, wxALL, 0);

    chkPLL_CFG_SEL1_INVERT =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15921->Add(chkPLL_CFG_SEL1_INVERT, 0, wxALL, 0);

    sbSizer16821->Add(fgSizer15921, 1, wxEXPAND, 5);

    fgSizer16021->Add(sbSizer16821, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer115221;
    sbSizer115221 = new wxStaticBoxSizer(new wxStaticBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer85321;
    fgSizer85321 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer85321->SetFlexibleDirection(wxBOTH);
    fgSizer85321->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText265101 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265101->Wrap(-1);
    fgSizer85321->Add(m_staticText265101, 0, wxALL, 5);

    m_staticText265121 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265121->Wrap(-1);
    fgSizer85321->Add(m_staticText265121, 0, wxALL, 5);

    m_staticText265221 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265221->Wrap(-1);
    fgSizer85321->Add(m_staticText265221, 0, wxALL, 5);

    m_staticText265321 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265321->Wrap(-1);
    fgSizer85321->Add(m_staticText265321, 0, wxALL, 5);

    m_staticText265421 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265421->Wrap(-1);
    fgSizer85321->Add(m_staticText265421, 0, wxALL, 5);

    m_staticText265521 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265521->Wrap(-1);
    fgSizer85321->Add(m_staticText265521, 0, wxALL, 5);

    m_staticText265621 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265621->Wrap(-1);
    fgSizer85321->Add(m_staticText265621, 0, wxALL, 5);

    m_staticText265721 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265721->Wrap(-1);
    fgSizer85321->Add(m_staticText265721, 0, wxALL, 5);

    m_staticText265821 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265821->Wrap(-1);
    fgSizer85321->Add(m_staticText265821, 0, wxALL, 5);

    chkPLL_CFG_SEL1_MASK0 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_PLL_CFG_SEL1_MASK0, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85321->Add(chkPLL_CFG_SEL1_MASK0, 0, wxALL, 0);

    chkPLL_CFG_SEL1_MASK1 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_PLL_CFG_SEL1_MASK1, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85321->Add(chkPLL_CFG_SEL1_MASK1, 0, wxALL, 0);

    chkPLL_CFG_SEL1_MASK2 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_PLL_CFG_SEL1_MASK2, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85321->Add(chkPLL_CFG_SEL1_MASK2, 0, wxALL, 0);

    chkPLL_CFG_SEL1_MASK3 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_PLL_CFG_SEL1_MASK3, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85321->Add(chkPLL_CFG_SEL1_MASK3, 0, wxALL, 0);

    chkPLL_CFG_SEL1_MASK4 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_PLL_CFG_SEL1_MASK4, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85321->Add(chkPLL_CFG_SEL1_MASK4, 0, wxALL, 0);

    chkPLL_CFG_SEL1_MASK5 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_PLL_CFG_SEL1_MASK5, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85321->Add(chkPLL_CFG_SEL1_MASK5, 0, wxALL, 0);

    chkPLL_CFG_SEL1_MASK6 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_PLL_CFG_SEL1_MASK6, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85321->Add(chkPLL_CFG_SEL1_MASK6, 0, wxALL, 0);

    chkPLL_CFG_SEL1_MASK7 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_PLL_CFG_SEL1_MASK7, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85321->Add(chkPLL_CFG_SEL1_MASK7, 0, wxALL, 0);

    chkPLL_CFG_SEL1_MASK8 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, ID_PLL_CFG_SEL1_MASK8, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85321->Add(chkPLL_CFG_SEL1_MASK8, 0, wxALL, 0);

    sbSizer115221->Add(fgSizer85321, 1, wxEXPAND, 5);

    fgSizer16021->Add(sbSizer115221, 1, wxEXPAND, 5);

    fgSizer88221->Add(fgSizer16021, 1, wxEXPAND, 5);

    ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1->SetSizer(fgSizer88221);
    ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1->Layout();
    fgSizer88221->Fit(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1);
    ID_NOTEBOOK_MUX_CONTROL_PLL->AddPage(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1, wxT("Bit 1"), false);
    ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2 =
        new wxPanel(ID_NOTEBOOK_MUX_CONTROL_PLL, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer882211;
    fgSizer882211 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer882211->SetFlexibleDirection(wxBOTH);
    fgSizer882211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer160211;
    fgSizer160211 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer160211->SetFlexibleDirection(wxBOTH);
    fgSizer160211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer168211;
    sbSizer168211 = new wxStaticBoxSizer(new wxStaticBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer159211;
    fgSizer159211 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer159211->SetFlexibleDirection(wxBOTH);
    fgSizer159211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkPLL_CFG_SEL2_INTERNAL = new wxCheckBox(
        ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_INTERNAL, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    chkPLL_CFG_SEL2_INTERNAL->SetValue(true);
    fgSizer159211->Add(chkPLL_CFG_SEL2_INTERNAL, 0, wxALL, 0);

    chkPLL_CFG_SEL2_INVERT = new wxCheckBox(
        ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_INVERT, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer159211->Add(chkPLL_CFG_SEL2_INVERT, 0, wxALL, 0);

    sbSizer168211->Add(fgSizer159211, 1, wxEXPAND, 5);

    fgSizer160211->Add(sbSizer168211, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer1152211;
    sbSizer1152211 =
        new wxStaticBoxSizer(new wxStaticBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer853211;
    fgSizer853211 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer853211->SetFlexibleDirection(wxBOTH);
    fgSizer853211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText2651011 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2651011->Wrap(-1);
    fgSizer853211->Add(m_staticText2651011, 0, wxALL, 5);

    m_staticText2651211 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2651211->Wrap(-1);
    fgSizer853211->Add(m_staticText2651211, 0, wxALL, 5);

    m_staticText2652211 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2652211->Wrap(-1);
    fgSizer853211->Add(m_staticText2652211, 0, wxALL, 5);

    m_staticText2653211 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2653211->Wrap(-1);
    fgSizer853211->Add(m_staticText2653211, 0, wxALL, 5);

    m_staticText2654211 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2654211->Wrap(-1);
    fgSizer853211->Add(m_staticText2654211, 0, wxALL, 5);

    m_staticText2655211 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2655211->Wrap(-1);
    fgSizer853211->Add(m_staticText2655211, 0, wxALL, 5);

    m_staticText2656211 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2656211->Wrap(-1);
    fgSizer853211->Add(m_staticText2656211, 0, wxALL, 5);

    m_staticText2657211 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2657211->Wrap(-1);
    fgSizer853211->Add(m_staticText2657211, 0, wxALL, 5);

    m_staticText2658211 =
        new wxStaticText(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2658211->Wrap(-1);
    fgSizer853211->Add(m_staticText2658211, 0, wxALL, 5);

    chkPLL_CFG_SEL2_MASK0 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_MASK0, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853211->Add(chkPLL_CFG_SEL2_MASK0, 0, wxALL, 0);

    chkPLL_CFG_SEL2_MASK1 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_MASK1, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853211->Add(chkPLL_CFG_SEL2_MASK1, 0, wxALL, 0);

    chkPLL_CFG_SEL2_MASK2 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_MASK2, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853211->Add(chkPLL_CFG_SEL2_MASK2, 0, wxALL, 0);

    chkPLL_CFG_SEL2_MASK3 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_MASK3, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853211->Add(chkPLL_CFG_SEL2_MASK3, 0, wxALL, 0);

    chkPLL_CFG_SEL2_MASK4 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_MASK4, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853211->Add(chkPLL_CFG_SEL2_MASK4, 0, wxALL, 0);

    chkPLL_CFG_SEL2_MASK5 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_MASK5, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853211->Add(chkPLL_CFG_SEL2_MASK5, 0, wxALL, 0);

    chkPLL_CFG_SEL2_MASK6 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_MASK6, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853211->Add(chkPLL_CFG_SEL2_MASK6, 0, wxALL, 0);

    chkPLL_CFG_SEL2_MASK7 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_MASK7, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853211->Add(chkPLL_CFG_SEL2_MASK7, 0, wxALL, 0);

    chkPLL_CFG_SEL2_MASK8 =
        new wxCheckBox(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, ID_PLL_CFG_SEL2_MASK8, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853211->Add(chkPLL_CFG_SEL2_MASK8, 0, wxALL, 0);

    sbSizer1152211->Add(fgSizer853211, 1, wxEXPAND, 5);

    fgSizer160211->Add(sbSizer1152211, 1, wxEXPAND, 5);

    fgSizer882211->Add(fgSizer160211, 1, wxEXPAND, 5);

    ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2->SetSizer(fgSizer882211);
    ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2->Layout();
    fgSizer882211->Fit(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2);
    ID_NOTEBOOK_MUX_CONTROL_PLL->AddPage(ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2, wxT("Bit 2"), false);

    sbSizer3941->Add(ID_NOTEBOOK_MUX_CONTROL_PLL, 1, wxEXPAND | wxALL, 5);

    wxStaticBoxSizer* sbSizer3041;
    sbSizer3041 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Internal")), wxVERTICAL);

    wxFlexGridSizer* fgSizer3701;
    fgSizer3701 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer3701->SetFlexibleDirection(wxBOTH);
    fgSizer3701->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkPLL_CFG_INT_SEL0 =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_PLL_CFG_INT_SEL0, wxT("Bit 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer3701->Add(chkPLL_CFG_INT_SEL0, 0, wxALL, 3);

    chkPLL_CFG_INT_SEL1 =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_PLL_CFG_INT_SEL1, wxT("Bit 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer3701->Add(chkPLL_CFG_INT_SEL1, 0, wxALL, 3);

    chkPLL_CFG_INT_SEL2 =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_PLL_CFG_INT_SEL2, wxT("Bit 2"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer3701->Add(chkPLL_CFG_INT_SEL2, 0, wxALL, 3);

    sbSizer3041->Add(fgSizer3701, 1, wxEXPAND, 5);

    sbSizer3941->Add(sbSizer3041, 0, wxEXPAND, 5);

    fgSizer369->Add(sbSizer3941, 1, wxEXPAND, 5);

    fgSizer643->Add(fgSizer369, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer4571;
    sbSizer4571 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Configuration")), wxVERTICAL);

    wxString rgrPLL_RSTNChoices[] = { wxT("On"), wxT("Off") };
    int rgrPLL_RSTNNChoices = sizeof(rgrPLL_RSTNChoices) / sizeof(wxString);
    rgrPLL_RSTN = new wxRadioBox(ID_PANEL_PLLCONFIG,
        wxID_ANY,
        wxT("PLL Reset"),
        wxDefaultPosition,
        wxDefaultSize,
        rgrPLL_RSTNNChoices,
        rgrPLL_RSTNChoices,
        2,
        wxRA_SPECIFY_COLS);
    rgrPLL_RSTN->SetSelection(0);
    sbSizer4571->Add(rgrPLL_RSTN, 0, wxEXPAND, 0);

    m_staticText992111 =
        new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Automatic freq. cal. res."), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText992111->Wrap(-1);
    sbSizer4571->Add(m_staticText992111, 0, wxALL, 0);

    cmbCTUNE_RES = new wxComboBox(ID_PANEL_PLLCONFIG, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbCTUNE_RES->SetSelection(0);
    sbSizer4571->Add(cmbCTUNE_RES, 0, wxALL, 0);

    chkPLL_CALIBRATION_MODE =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("PLL_CALIBRATION_MODE"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer4571->Add(chkPLL_CALIBRATION_MODE, 0, wxALL, 0);

    chkPLL_CALIBRATION_EN =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("PLL_CALIBRATION_EN"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer4571->Add(chkPLL_CALIBRATION_EN, 0, wxALL, 0);

    chkPLL_FLOCK_INTERNAL =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("PLL_FLOCK_INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer4571->Add(chkPLL_FLOCK_INTERNAL, 0, wxALL, 0);

    chkPLL_FLOCK_INTVAL =
        new wxCheckBox(ID_PANEL_PLLCONFIG, ID_CHx_PA_ILIN2X, wxT("PLL_FLOCK_INTVAL"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer4571->Add(chkPLL_FLOCK_INTVAL, 0, wxALL, 0);

    fgSizer643->Add(sbSizer4571, 1, wxEXPAND, 5);

    fgSizer126->Add(fgSizer643, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer647;
    fgSizer647 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer647->SetFlexibleDirection(wxBOTH);
    fgSizer647->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer506;
    sbSizer506 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("LO Distribution Config.")), wxVERTICAL);

    chkSEL_BIAS_CORE = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_PLL_LOCK, wxT("SEL_BIAS_CORE"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer506->Add(chkSEL_BIAS_CORE, 0, wxALL, 0);

    m_staticText1191 =
        new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Core bias current control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1191->Wrap(-1);
    sbSizer506->Add(m_staticText1191, 0, wxALL, 0);

    cmbPLL_LODIST_ICT_CORE = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_PLL_LODIST_ICT_CORE, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    sbSizer506->Add(cmbPLL_LODIST_ICT_CORE, 0, wxALL, 0);

    m_staticText11911 =
        new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Buffer bias current control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText11911->Wrap(-1);
    sbSizer506->Add(m_staticText11911, 0, wxALL, 0);

    cmbPLL_LODIST_ICT_BUF = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_PLL_LODIST_ICT_BUF, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    sbSizer506->Add(cmbPLL_LODIST_ICT_BUF, 0, wxALL, 0);

    wxStaticBoxSizer* sbSizer507;
    sbSizer507 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Drive-strength")), wxVERTICAL);

    wxFlexGridSizer* fgSizer648;
    fgSizer648 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer648->SetFlexibleDirection(wxBOTH);
    fgSizer648->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxString choPLL_ICT_OUT0Choices[] = { wxT("0"), wxT("1"), wxT("2"), wxT("3") };
    int choPLL_ICT_OUT0NChoices = sizeof(choPLL_ICT_OUT0Choices) / sizeof(wxString);
    choPLL_ICT_OUT0 = new wxChoice(
        ID_PANEL_PLLCONFIG, wxID_ANY, wxDefaultPosition, wxDefaultSize, choPLL_ICT_OUT0NChoices, choPLL_ICT_OUT0Choices, 0);
    choPLL_ICT_OUT0->SetSelection(0);
    fgSizer648->Add(choPLL_ICT_OUT0, 0, wxALL, 0);

    m_staticText1195 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Channel A"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1195->Wrap(-1);
    fgSizer648->Add(m_staticText1195, 0, wxALL, 4);

    wxString choPLL_ICT_OUT1Choices[] = { wxT("0"), wxT("1"), wxT("2"), wxT("3") };
    int choPLL_ICT_OUT1NChoices = sizeof(choPLL_ICT_OUT1Choices) / sizeof(wxString);
    choPLL_ICT_OUT1 = new wxChoice(
        ID_PANEL_PLLCONFIG, wxID_ANY, wxDefaultPosition, wxDefaultSize, choPLL_ICT_OUT1NChoices, choPLL_ICT_OUT1Choices, 0);
    choPLL_ICT_OUT1->SetSelection(0);
    fgSizer648->Add(choPLL_ICT_OUT1, 0, wxALL, 0);

    m_staticText11951 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Channel B"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText11951->Wrap(-1);
    fgSizer648->Add(m_staticText11951, 0, wxALL, 4);

    wxString choPLL_ICT_OUT2Choices[] = { wxT("0"), wxT("1"), wxT("2"), wxT("3") };
    int choPLL_ICT_OUT2NChoices = sizeof(choPLL_ICT_OUT2Choices) / sizeof(wxString);
    choPLL_ICT_OUT2 = new wxChoice(
        ID_PANEL_PLLCONFIG, wxID_ANY, wxDefaultPosition, wxDefaultSize, choPLL_ICT_OUT2NChoices, choPLL_ICT_OUT2Choices, 0);
    choPLL_ICT_OUT2->SetSelection(0);
    fgSizer648->Add(choPLL_ICT_OUT2, 0, wxALL, 0);

    m_staticText119511 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Channel C"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText119511->Wrap(-1);
    fgSizer648->Add(m_staticText119511, 0, wxALL, 4);

    wxString choPLL_ICT_OUT3Choices[] = { wxT("0"), wxT("1"), wxT("2"), wxT("3") };
    int choPLL_ICT_OUT3NChoices = sizeof(choPLL_ICT_OUT3Choices) / sizeof(wxString);
    choPLL_ICT_OUT3 = new wxChoice(
        ID_PANEL_PLLCONFIG, wxID_ANY, wxDefaultPosition, wxDefaultSize, choPLL_ICT_OUT3NChoices, choPLL_ICT_OUT3Choices, 0);
    choPLL_ICT_OUT3->SetSelection(0);
    fgSizer648->Add(choPLL_ICT_OUT3, 0, wxALL, 0);

    m_staticText1195111 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Channel D"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1195111->Wrap(-1);
    fgSizer648->Add(m_staticText1195111, 0, wxALL, 4);

    sbSizer507->Add(fgSizer648, 1, wxEXPAND, 5);

    sbSizer506->Add(sbSizer507, 1, wxEXPAND, 5);

    fgSizer647->Add(sbSizer506, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer508;
    sbSizer508 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("BIST")), wxVERTICAL);

    m_staticText1205 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Signature"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1205->Wrap(-1);
    sbSizer508->Add(m_staticText1205, 0, wxALL, 0);

    cmbBSIGL = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_PLL_LODIST_ICT_BUF, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 127, 0);
    cmbBSIGL->Enable(false);

    sbSizer508->Add(cmbBSIGL, 0, wxALL, 0);

    chkBSTATE = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_BSTATE, wxT("BSTATE"), wxDefaultPosition, wxDefaultSize, 0);
    chkBSTATE->Enable(false);

    sbSizer508->Add(chkBSTATE, 0, wxALL, 0);

    chkEN_SDM_TSTO = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_EN_SDM_TSTO, wxT("EN_SDM_TSTO"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer508->Add(chkEN_SDM_TSTO, 0, wxALL, 0);

    chkBEN = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_EN_SDM_TSTO, wxT("BEN"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer508->Add(chkBEN, 0, wxALL, 0);

    chkBSTART = new wxCheckBox(ID_PANEL_PLLCONFIG, ID_EN_SDM_TSTO, wxT("BSTART"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer508->Add(chkBSTART, 0, wxALL, 0);

    m_staticText1206 = new wxStaticText(ID_PANEL_PLLCONFIG, wxID_ANY, wxT("Output"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1206->Wrap(-1);
    sbSizer508->Add(m_staticText1206, 0, wxALL, 0);

    cmbBSIGH = new wxSpinCtrl(
        ID_PANEL_PLLCONFIG, ID_PLL_LODIST_ICT_BUF, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 65535, 0);
    cmbBSIGH->Enable(false);

    sbSizer508->Add(cmbBSIGH, 0, wxALL, 0);

    fgSizer647->Add(sbSizer508, 1, wxEXPAND, 5);

    fgSizer126->Add(fgSizer647, 1, wxEXPAND, 5);

    fgSizer69->Add(fgSizer126, 1, wxEXPAND, 5);

    ID_PANEL_PLLCONFIG->SetSizer(fgSizer69);
    ID_PANEL_PLLCONFIG->Layout();
    fgSizer69->Fit(ID_PANEL_PLLCONFIG);
    ID_NOTEBOOK_PLLCONFIG->AddPage(ID_PANEL_PLLCONFIG, wxT("PLL Configuration"), false);

    fgSizer68->Add(ID_NOTEBOOK_PLLCONFIG, 1, wxEXPAND | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    this->SetSizer(fgSizer68);
    this->Layout();
    fgSizer68->Fit(this);

    // Connect Events
    chkEN_VCOBIAS->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkBYP_VCOREG->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkCURLIM_VCOREG->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_VCOREG->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVDIV_VCOREG->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_XBUF_SLFBEN->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_XBUF_BYPEN->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_XBUF_EN->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_FREQ_MAN->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_FREQ_MAN->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_FREQ_MAN->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_MAN->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_MAN->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_MAN->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkFREQ_LOW->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkFREQ_EQUAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkFREQ_HIGH->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkCTUNE_STEP_DONE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkCTUNE_START->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkCTUNE_EN->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkVTUNE_HIGH->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkVTUNE_LOW->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_LOCK->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkFCAL_START->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkVCO_SEL_FINAL_VAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_FINAL->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_FINAL->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_FINAL->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkFREQ_FINAL_VAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_FINAL->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_FINAL->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_FINAL->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkVCO_SEL_FORCE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_INIT->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_INIT->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_INIT->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_INIT_POS->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_INIT_POS->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_INIT_POS->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_INIT->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_INIT->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_INIT->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_SETTLING_N->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_SETTLING_N->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbFREQ_SETTLING_N->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVTUNE_WAIT_N->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVTUNE_WAIT_N->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVTUNE_WAIT_N->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_FREQ_MAX->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_FREQ_MAX->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_FREQ_MAX->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_FREQ_MIN->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_FREQ_MIN->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_FREQ_MIN->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkSelectProfile->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::OnSelectProfileClick), NULL, this);
    chkPLL_CFG_INT_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL0_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL1_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_SEL2_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_INT_SEL0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_INT_SEL1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CFG_INT_SEL2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    rgrPLL_RSTN->Connect(
        wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbCTUNE_RES->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    // TODO: cmbCTUNE_RES->Connect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( lms8001_pnlPLLConfig_view::OnUpdateUI_cmbPLL_LODIST_FSP_OUTCH10_n ), NULL, this );
    chkPLL_CALIBRATION_MODE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_CALIBRATION_EN->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_FLOCK_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkPLL_FLOCK_INTVAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkSEL_BIAS_CORE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_ICT_CORE->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_ICT_CORE->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_ICT_CORE->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_ICT_BUF->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_ICT_BUF->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_ICT_BUF->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    choPLL_ICT_OUT0->Connect(
        wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    choPLL_ICT_OUT1->Connect(
        wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    choPLL_ICT_OUT2->Connect(
        wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    choPLL_ICT_OUT3->Connect(
        wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbBSIGL->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbBSIGL->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbBSIGL->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkBSTATE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkEN_SDM_TSTO->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkBEN->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    chkBSTART->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbBSIGH->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbBSIGH->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);
    cmbBSIGH->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLConfig_view::ParameterChangeHandler), NULL, this);

    wndId2Enum[chkEN_VCOBIAS] = EN_VCOBIAS;
    wndId2Enum[chkBYP_VCOREG] = BYP_VCOREG;
    wndId2Enum[chkCURLIM_VCOREG] = CURLIM_VCOREG;
    wndId2Enum[chkSPDUP_VCOREG] = SPDUP_VCOREG;
    wndId2Enum[cmbVDIV_VCOREG] = VDIV_VCOREG;

    wndId2Enum[chkPLL_XBUF_SLFBEN] = PLL_XBUF_SLFBEN;
    wndId2Enum[chkPLL_XBUF_BYPEN] = PLL_XBUF_BYPEN;
    wndId2Enum[chkPLL_XBUF_EN] = PLL_XBUF_EN;

    wndId2Enum[chkFCAL_START] = FCAL_START;
    wndId2Enum[chkVCO_SEL_FINAL_VAL] = VCO_SEL_FINAL_VAL;
    wndId2Enum[cmbVCO_SEL_FINAL] = VCO_SEL_FINAL;
    wndId2Enum[chkFREQ_FINAL_VAL] = FREQ_FINAL_VAL;
    wndId2Enum[cmbFREQ_FINAL] = FREQ_FINAL;
    wndId2Enum[chkVCO_SEL_FORCE] = VCO_SEL_FORCE;
    wndId2Enum[cmbVCO_SEL_INIT] = VCO_SEL_INIT;
    wndId2Enum[cmbFREQ_INIT_POS] = FREQ_INIT_POS;
    wndId2Enum[cmbFREQ_INIT] = FREQ_INIT;
    wndId2Enum[cmbFREQ_SETTLING_N] = FREQ_SETTLING_N;
    wndId2Enum[cmbVTUNE_WAIT_N] = VTUNE_WAIT_N;
    wndId2Enum[cmbVCO_SEL_FREQ_MAX] = VCO_SEL_FREQ_MAX;
    wndId2Enum[cmbVCO_SEL_FREQ_MIN] = VCO_SEL_FREQ_MIN;

    wndId2Enum[cmbVCO_FREQ_MAN] = VCO_FREQ_MAN;
    wndId2Enum[cmbVCO_SEL_MAN] = VCO_SEL_MAN;
    wndId2Enum[chkFREQ_LOW] = FREQ_LOW;
    wndId2Enum[chkFREQ_EQUAL] = FREQ_EQUAL;
    wndId2Enum[chkFREQ_HIGH] = FREQ_HIGH;
    wndId2Enum[chkCTUNE_STEP_DONE] = CTUNE_STEP_DONE;
    wndId2Enum[chkCTUNE_START] = CTUNE_START;
    wndId2Enum[chkCTUNE_EN] = CTUNE_EN;

    wndId2Enum[chkPLL_CFG_SEL0_INTERNAL] = PLL_CFG_SEL0_INTERNAL;
    wndId2Enum[chkPLL_CFG_SEL0_INVERT] = PLL_CFG_SEL0_INVERT;
    wndId2Enum[chkPLL_CFG_SEL0_MASK0] = PLL_CFG_SEL0_MASK0;
    wndId2Enum[chkPLL_CFG_SEL0_MASK1] = PLL_CFG_SEL0_MASK1;
    wndId2Enum[chkPLL_CFG_SEL0_MASK2] = PLL_CFG_SEL0_MASK2;
    wndId2Enum[chkPLL_CFG_SEL0_MASK3] = PLL_CFG_SEL0_MASK3;
    wndId2Enum[chkPLL_CFG_SEL0_MASK4] = PLL_CFG_SEL0_MASK4;
    wndId2Enum[chkPLL_CFG_SEL0_MASK5] = PLL_CFG_SEL0_MASK5;
    wndId2Enum[chkPLL_CFG_SEL0_MASK6] = PLL_CFG_SEL0_MASK6;
    wndId2Enum[chkPLL_CFG_SEL0_MASK7] = PLL_CFG_SEL0_MASK7;
    wndId2Enum[chkPLL_CFG_SEL0_MASK8] = PLL_CFG_SEL0_MASK8;

    wndId2Enum[chkPLL_CFG_SEL1_INTERNAL] = PLL_CFG_SEL1_INTERNAL;
    wndId2Enum[chkPLL_CFG_SEL1_INVERT] = PLL_CFG_SEL1_INVERT;
    wndId2Enum[chkPLL_CFG_SEL1_MASK0] = PLL_CFG_SEL1_MASK0;
    wndId2Enum[chkPLL_CFG_SEL1_MASK1] = PLL_CFG_SEL1_MASK1;
    wndId2Enum[chkPLL_CFG_SEL1_MASK2] = PLL_CFG_SEL1_MASK2;
    wndId2Enum[chkPLL_CFG_SEL1_MASK3] = PLL_CFG_SEL1_MASK3;
    wndId2Enum[chkPLL_CFG_SEL1_MASK4] = PLL_CFG_SEL1_MASK4;
    wndId2Enum[chkPLL_CFG_SEL1_MASK5] = PLL_CFG_SEL1_MASK5;
    wndId2Enum[chkPLL_CFG_SEL1_MASK6] = PLL_CFG_SEL1_MASK6;
    wndId2Enum[chkPLL_CFG_SEL1_MASK7] = PLL_CFG_SEL1_MASK7;
    wndId2Enum[chkPLL_CFG_SEL1_MASK8] = PLL_CFG_SEL1_MASK8;

    wndId2Enum[chkPLL_CFG_SEL2_INTERNAL] = PLL_CFG_SEL2_INTERNAL;
    wndId2Enum[chkPLL_CFG_SEL2_INVERT] = PLL_CFG_SEL2_INVERT;
    wndId2Enum[chkPLL_CFG_SEL2_MASK0] = PLL_CFG_SEL2_MASK0;
    wndId2Enum[chkPLL_CFG_SEL2_MASK1] = PLL_CFG_SEL2_MASK1;
    wndId2Enum[chkPLL_CFG_SEL2_MASK2] = PLL_CFG_SEL2_MASK2;
    wndId2Enum[chkPLL_CFG_SEL2_MASK3] = PLL_CFG_SEL2_MASK3;
    wndId2Enum[chkPLL_CFG_SEL2_MASK4] = PLL_CFG_SEL2_MASK4;
    wndId2Enum[chkPLL_CFG_SEL2_MASK5] = PLL_CFG_SEL2_MASK5;
    wndId2Enum[chkPLL_CFG_SEL2_MASK6] = PLL_CFG_SEL2_MASK6;
    wndId2Enum[chkPLL_CFG_SEL2_MASK7] = PLL_CFG_SEL2_MASK7;
    wndId2Enum[chkPLL_CFG_SEL2_MASK8] = PLL_CFG_SEL2_MASK8;

    wndId2Enum[chkPLL_CFG_INT_SEL0] = PLL_CFG_INT_SEL0;
    wndId2Enum[chkPLL_CFG_INT_SEL1] = PLL_CFG_INT_SEL1;
    wndId2Enum[chkPLL_CFG_INT_SEL2] = PLL_CFG_INT_SEL2;

    wndId2Enum[chkPLL_CFG_INT_SEL] = PLL_CFG_INT_SEL;

    wndId2Enum[rgrPLL_RSTN] = PLL_RSTN;

    //	wndId2Enum[chkPLL_RSTN] = PLL_RSTN;
    wndId2Enum[cmbCTUNE_RES] = CTUNE_RES;
    wndId2Enum[chkPLL_CALIBRATION_MODE] = PLL_CALIBRATION_MODE;
    wndId2Enum[chkPLL_CALIBRATION_EN] = PLL_CALIBRATION_EN;
    wndId2Enum[chkPLL_FLOCK_INTERNAL] = PLL_FLOCK_INTERNAL;
    wndId2Enum[chkPLL_FLOCK_INTVAL] = PLL_FLOCK_INTVAL;

    wndId2Enum[chkVTUNE_HIGH] = VTUNE_HIGH;
    wndId2Enum[chkVTUNE_LOW] = VTUNE_LOW;
    wndId2Enum[chkPLL_LOCK] = PLL_LOCK;

    wndId2Enum[chkSEL_BIAS_CORE] = SEL_BIAS_CORE;
    wndId2Enum[cmbPLL_LODIST_ICT_CORE] = PLL_LODIST_ICT_CORE;
    wndId2Enum[cmbPLL_LODIST_ICT_BUF] = PLL_LODIST_ICT_BUF;
    wndId2Enum[choPLL_ICT_OUT0] = PLL_ICT_OUT0;
    wndId2Enum[choPLL_ICT_OUT1] = PLL_ICT_OUT1;
    wndId2Enum[choPLL_ICT_OUT2] = PLL_ICT_OUT2;
    wndId2Enum[choPLL_ICT_OUT3] = PLL_ICT_OUT3;

    wndId2Enum[cmbBSIGL] = BSIGL;
    wndId2Enum[chkBSTATE] = BSTATE;
    wndId2Enum[chkEN_SDM_TSTO] = EN_SDM_TSTO;
    wndId2Enum[chkBEN] = BEN;
    wndId2Enum[chkBSTART] = BSTART;

    wndId2Enum[cmbBSIGH] = BSIGH;

    wxArrayString temp;
    temp.clear();
    for (int i = 0; i < 256; ++i)
    {
        //		temp.push_back(wxString::Format(_("%.3f V"), (860.0 + 3.92*i) / 1000.0));
        temp.push_back(wxString::Format(_("%.3f V"), 1.8 * (257 - i) / (265 - i)));
    }
    cmbVDIV_VCOREG->Append(temp);

    temp.clear();
    temp.push_back("10 MHz");
    temp.push_back("5 MHz");
    temp.push_back("2.5 MHz");
    temp.push_back("1.25 MHz");

    cmbCTUNE_RES->Append(temp);

    LMS8001_WXGUI::UpdateTooltips(wndId2Enum, true);
}

void lms8001_pnlPLLConfig_view::OnSelectProfileClick(wxCommandEvent& event)
{
    bool simple = chkSelectProfile->GetValue();
    chkPLL_CFG_INT_SEL->Enable(simple);
    ID_NOTEBOOK_MUX_CONTROL_PLL->Enable(!simple);
    chkPLL_CFG_INT_SEL0->Enable(!simple);
    chkPLL_CFG_INT_SEL1->Enable(!simple);
    chkPLL_CFG_INT_SEL2->Enable(!simple);

    if (simple)
    {
        chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, PLL_CFG_INT_SEL.defaultValue, true, chip->channel);
        chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, PLL_CFG_SEL0.defaultValue, true, chip->channel);
        chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, PLL_CFG_SEL1.defaultValue, true, chip->channel);
        chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, PLL_CFG_SEL2.defaultValue, true, chip->channel);
    }

    UpdateGUI();
}