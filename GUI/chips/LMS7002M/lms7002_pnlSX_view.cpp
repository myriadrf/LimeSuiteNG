#include "lms7002_pnlSX_view.h"
#include "commonWxHeaders.h"

#include <wx/textdlg.h>
#include <wx/valnum.h>
#include <wx/msgdlg.h>
#include <wx/spinctrl.h>
#include <map>
#include <assert.h>
#include "numericSlider.h"
#include "lms7002_gui_utilities.h"
#include "events.h"
#include "lms7002_dlgVCOfrequencies.h"
#include <string>
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"
using namespace std;
using namespace lime;
using namespace lime::LMS7002MCSR_Data;

static bool showRefClkSpurCancellation = true;

lms7002_pnlSX_view::lms7002_pnlSX_view(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ILMS7002MTab(parent, id, pos, size, style)
    , direction(TRXDir::Rx)
    , sizerR3(new wxFlexGridSizer(0, 2, 0, 0))
{
    const int flags = 0;
    wxFlexGridSizer* fgSizer92;
    fgSizer92 = new wxFlexGridSizer(0, 3, 5, 5);
    fgSizer92->SetFlexibleDirection(wxBOTH);
    fgSizer92->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer93;
    fgSizer93 = new wxFlexGridSizer(0, 1, 5, 5);
    fgSizer93->SetFlexibleDirection(wxBOTH);
    fgSizer93->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizerPowerDowns;
    sbSizerPowerDowns = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Power down controls")), wxVERTICAL);

    wxFlexGridSizer* fgSizer95;
    fgSizer95 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer95->SetFlexibleDirection(wxBOTH);
    fgSizer95->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkPD_FBDIV = new wxCheckBox(
        sbSizerPowerDowns->GetStaticBox(), ID_PD_FBDIV, wxT("Feedback divider block"), wxDefaultPosition, wxDefaultSize, 0);
    chkPD_FBDIV->SetToolTip(wxT("Power down the feedback divider block"));

    fgSizer95->Add(chkPD_FBDIV, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    chkPD_LOCH_T2RBUF = new wxCheckBox(sbSizerPowerDowns->GetStaticBox(),
        ID_PD_LOCH_T2RBUF,
        wxT("LO buffer from SXT to SXR"),
        wxDefaultPosition,
        wxDefaultSize,
        0);
    chkPD_LOCH_T2RBUF->SetToolTip(wxT("Power down for LO buffer from SXT to SXR. To be active only in the TDD mode"));

    fgSizer95->Add(chkPD_LOCH_T2RBUF, 0, flags, 0);

    chkPD_CP = new wxCheckBox(sbSizerPowerDowns->GetStaticBox(), ID_PD_CP, wxT("Charge pump"), wxDefaultPosition, wxDefaultSize, 0);
    chkPD_CP->SetToolTip(wxT("Power down for Charge Pump"));

    fgSizer95->Add(chkPD_CP, 0, flags, 0);

    chkPD_FDIV = new wxCheckBox(
        sbSizerPowerDowns->GetStaticBox(), ID_PD_FDIV, wxT("Forward frequency divider"), wxDefaultPosition, wxDefaultSize, 0);
    chkPD_FDIV->SetToolTip(wxT("Power down for feedback frequency and forward dividers"));

    fgSizer95->Add(chkPD_FDIV, 0, flags, 0);

    chkPD_SDM = new wxCheckBox(sbSizerPowerDowns->GetStaticBox(), ID_PD_SDM, wxT("SDM"), wxDefaultPosition, wxDefaultSize, 0);
    chkPD_SDM->SetToolTip(wxT("Power down for SDM"));

    fgSizer95->Add(chkPD_SDM, 0, flags, 0);

    chkPD_VCO_COMP = new wxCheckBox(
        sbSizerPowerDowns->GetStaticBox(), ID_PD_VCO_COMP, wxT("VCO comparator"), wxDefaultPosition, wxDefaultSize, 0);
    chkPD_VCO_COMP->SetToolTip(wxT("Power down for VCO comparator"));

    fgSizer95->Add(chkPD_VCO_COMP, 0, flags, 0);

    chkPD_VCO = new wxCheckBox(sbSizerPowerDowns->GetStaticBox(), ID_PD_VCO, wxT("VCO"), wxDefaultPosition, wxDefaultSize, 0);
    chkPD_VCO->SetToolTip(wxT("Power down for VCO"));

    fgSizer95->Add(chkPD_VCO, 0, flags, 0);

    chkEN_G = new wxCheckBox(
        sbSizerPowerDowns->GetStaticBox(), ID_EN_G, wxT("Enable SXR/SXT module"), wxDefaultPosition, wxDefaultSize, 0);
    chkEN_G->SetToolTip(wxT("Enable control for all the SXT power downs"));

    fgSizer95->Add(chkEN_G, 0, flags, 0);

    wxStaticBoxSizer* sbSizer74;
    sbSizer74 =
        new wxStaticBoxSizer(new wxStaticBox(sbSizerPowerDowns->GetStaticBox(), wxID_ANY, wxT("Direct control")), wxHORIZONTAL);

    chkEN_DIR_SXRSXT = new wxCheckBox(
        sbSizer74->GetStaticBox(), ID_EN_DIR_SXRSXT, wxT("Direct control of PDs and ENs"), wxDefaultPosition, wxDefaultSize, 0);
    chkEN_DIR_SXRSXT->SetToolTip(wxT("Enables direct control of PDs and ENs for SXR/SXT module"));

    sbSizer74->Add(chkEN_DIR_SXRSXT, 1, wxALIGN_LEFT, 5);

    fgSizer95->Add(sbSizer74, 1, wxALIGN_LEFT | wxALIGN_TOP | wxALL, 5);

    sbSizerPowerDowns->Add(fgSizer95, 0, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);

    fgSizer93->Add(sbSizerPowerDowns, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer77;
    sbSizer77 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("SXT/SXR controls")), wxVERTICAL);

    wxFlexGridSizer* fgSizer101;
    fgSizer101 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer101->SetFlexibleDirection(wxBOTH);
    fgSizer101->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkRESET_N = new wxCheckBox(sbSizer77->GetStaticBox(), ID_RESET_N, wxT("Reset SX"), wxDefaultPosition, wxDefaultSize, 0);
    chkRESET_N->SetToolTip(wxT("Resets SX. A pulse should be used in the start-up to reset"));

    fgSizer101->Add(chkRESET_N, 0, flags, 0);

    chkSPDUP_VCO = new wxCheckBox(
        sbSizer77->GetStaticBox(), ID_SPDUP_VCO, wxT("Bypass noise filter resistor"), wxDefaultPosition, wxDefaultSize, 0);
    chkSPDUP_VCO->SetToolTip(
        wxT("Bypasses the noise filter resistor for fast settling time. It should be connected to a 1uS pulse"));

    fgSizer101->Add(chkSPDUP_VCO, 0, flags, 0);

    chkBYPLDO_VCO =
        new wxCheckBox(sbSizer77->GetStaticBox(), ID_BYPLDO_VCO, wxT("Bypass SX LDO"), wxDefaultPosition, wxDefaultSize, 0);
    chkBYPLDO_VCO->SetToolTip(wxT("Controls the bypass signal for the SX LDO"));

    fgSizer101->Add(chkBYPLDO_VCO, 0, flags, 0);

    chkEN_COARSEPLL = new wxCheckBox(
        sbSizer77->GetStaticBox(), ID_EN_COARSEPLL, wxT("Enable coarse tuning"), wxDefaultPosition, wxDefaultSize, 0);
    chkEN_COARSEPLL->SetToolTip(wxT("Enable signal for coarse tuning block"));

    fgSizer101->Add(chkEN_COARSEPLL, 0, flags, 0);

    chkCURLIM_VCO =
        new wxCheckBox(sbSizer77->GetStaticBox(), ID_CURLIM_VCO, wxT("Enable current limit"), wxDefaultPosition, wxDefaultSize, 0);
    chkCURLIM_VCO->SetToolTip(wxT("Enables the output current limitation in the VCO regulator"));

    fgSizer101->Add(chkCURLIM_VCO, 0, flags, 0);

    chkREVPH_PFD =
        new wxCheckBox(sbSizer77->GetStaticBox(), ID_REVPH_PFD, wxT("Reverse pulses of PFD"), wxDefaultPosition, wxDefaultSize, 0);
    chkREVPH_PFD->SetToolTip(wxT("Reverse the pulses of PFD. It can be used to reverse the polarity of the PLL loop (positive "
                                 "feedback to negative feedback)"));

    fgSizer101->Add(chkREVPH_PFD, 0, flags, 0);

    chkEN_INTONLY_SDM = new wxCheckBox(
        sbSizer77->GetStaticBox(), ID_EN_INTONLY_SDM, wxT("Enable INTEGER_N mode"), wxDefaultPosition, wxDefaultSize, 0);
    chkEN_INTONLY_SDM->SetToolTip(wxT("Enables INTEGER-N mode of the SX "));

    fgSizer101->Add(chkEN_INTONLY_SDM, 0, flags, 0);

    chkEN_SDM_CLK =
        new wxCheckBox(sbSizer77->GetStaticBox(), ID_EN_SDM_CLK, wxT("Enable SDM clock"), wxDefaultPosition, wxDefaultSize, 0);
    chkEN_SDM_CLK->SetToolTip(wxT("Enables/Disables SDM clock. In INT-N mode or for noise testing, SDM clock can be disabled"));

    fgSizer101->Add(chkEN_SDM_CLK, 0, flags, 0);

    chkREV_SDMCLK =
        new wxCheckBox(sbSizer77->GetStaticBox(), ID_REV_SDMCLK, wxT("Reverse SDM clock"), wxDefaultPosition, wxDefaultSize, 0);
    chkREV_SDMCLK->SetToolTip(wxT("Reverses the SDM clock"));

    fgSizer101->Add(chkREV_SDMCLK, 0, flags, 0);

    chkSX_DITHER_EN = new wxCheckBox(
        sbSizer77->GetStaticBox(), ID_SX_DITHER_EN, wxT("Enable dithering in SDM"), wxDefaultPosition, wxDefaultSize, 0);
    chkSX_DITHER_EN->SetToolTip(wxT("Enabled dithering in SDM"));

    fgSizer101->Add(chkSX_DITHER_EN, 0, flags, 0);

    chkEN_DIV2_DIVPROGenabled = new wxCheckBox(sbSizer77->GetStaticBox(),
        ID_EN_DIV2_DIVPROGenabled,
        wxT("Enable additional DIV2 prescaler"),
        wxDefaultPosition,
        wxDefaultSize,
        0);
    chkEN_DIV2_DIVPROGenabled->Hide();

    fgSizer101->Add(chkEN_DIV2_DIVPROGenabled, 0, 0, 5);

    sbSizer77->Add(fgSizer101, 0, wxEXPAND, 0);

    fgSizer93->Add(sbSizer77, 0, wxEXPAND, 5);

    fgSizer92->Add(fgSizer93, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);

    wxFlexGridSizer* fgSizer103;
    fgSizer103 = new wxFlexGridSizer(0, 1, 5, 5);
    fgSizer103->AddGrowableCol(0);
    fgSizer103->SetFlexibleDirection(wxBOTH);
    fgSizer103->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer203;
    fgSizer203 = new wxFlexGridSizer(0, 2, 5, 5);
    fgSizer203->AddGrowableCol(1);
    fgSizer203->SetFlexibleDirection(wxBOTH);
    fgSizer203->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_STATICTEXT7 = new wxStaticText(this, wxID_ANY, wxT("Test mode of SX"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT7->Wrap(-1);
    fgSizer203->Add(ID_STATICTEXT7, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    cmbTST_SX = new wxComboBox(this, ID_TST_SX, wxEmptyString, wxDefaultPosition, wxSize(-1, -1), 0, nullptr, 0);
    fgSizer203->Add(cmbTST_SX, 0, 0, 0);

    fgSizer103->Add(fgSizer203, 0, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer198;
    fgSizer198 = new wxFlexGridSizer(0, 5, 5, 5);
    fgSizer198->SetFlexibleDirection(wxBOTH);
    fgSizer198->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizerDivisionRatio;
    sbSizerDivisionRatio = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Division ratio")), wxHORIZONTAL);

    wxFlexGridSizer* fgDivisionRatio;
    fgDivisionRatio = new wxFlexGridSizer(0, 2, 0, 5);
    fgDivisionRatio->AddGrowableCol(1);
    fgDivisionRatio->AddGrowableRow(0);
    fgDivisionRatio->AddGrowableRow(1);
    fgDivisionRatio->AddGrowableRow(2);
    fgDivisionRatio->SetFlexibleDirection(wxBOTH);
    fgDivisionRatio->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_STATICTEXT4 = new wxStaticText(
        sbSizerDivisionRatio->GetStaticBox(), wxID_ANY, wxT("Trim duty cycle of DIV2 LOCH"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT4->Wrap(-1);
    fgDivisionRatio->Add(ID_STATICTEXT4, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    ctrPW_DIV2_LOCH = new wxSpinCtrl(sbSizerDivisionRatio->GetStaticBox(),
        ID_PW_DIV2_LOCH,
        wxEmptyString,
        wxDefaultPosition,
        wxDefaultSize,
        wxSP_ARROW_KEYS,
        0,
        7,
        1);
    //ctrPW_DIV2_LOCH->SetMinSize( wxSize( 48,-1 ) );

    fgDivisionRatio->Add(ctrPW_DIV2_LOCH, 0, wxEXPAND, 5);

    ID_STATICTEXT5 = new wxStaticText(
        sbSizerDivisionRatio->GetStaticBox(), wxID_ANY, wxT("Trim duty cycle of DIV4 LOCH"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT5->Wrap(-1);
    fgDivisionRatio->Add(ID_STATICTEXT5, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    ctrPW_DIV4_LOCH = new wxSpinCtrl(sbSizerDivisionRatio->GetStaticBox(),
        ID_PW_DIV4_LOCH,
        wxEmptyString,
        wxDefaultPosition,
        wxDefaultSize,
        wxSP_ARROW_KEYS,
        0,
        7,
        2);
    //ctrPW_DIV4_LOCH->SetMinSize( wxSize( 48,-1 ) );

    fgDivisionRatio->Add(ctrPW_DIV4_LOCH, 0, wxEXPAND, 5);

    ID_STATICTEXT6 = new wxStaticText(
        sbSizerDivisionRatio->GetStaticBox(), wxID_ANY, wxT("LOCH_DIV division ratio"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT6->Wrap(-1);
    fgDivisionRatio->Add(ID_STATICTEXT6, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    ctrDIV_LOCH = new wxSpinCtrl(sbSizerDivisionRatio->GetStaticBox(),
        ID_DIV_LOCH,
        wxEmptyString,
        wxDefaultPosition,
        wxDefaultSize,
        wxSP_ARROW_KEYS,
        0,
        7,
        2);
    //ctrDIV_LOCH->SetMinSize( wxSize( 48,-1 ) );

    fgDivisionRatio->Add(ctrDIV_LOCH, 0, wxEXPAND, 5);

    sbSizerDivisionRatio->Add(fgDivisionRatio, 1, wxALL | wxEXPAND | wxALIGN_LEFT, 0);

    fgSizer198->Add(sbSizerDivisionRatio, 1, wxALIGN_LEFT | wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer78;
    sbSizer78 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("PLL loop filter")), wxHORIZONTAL);

    wxFlexGridSizer* fgSizer102;
    fgSizer102 = new wxFlexGridSizer(0, 2, 0, 5);
    fgSizer102->SetFlexibleDirection(wxBOTH);
    fgSizer102->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_STATICTEXT18 = new wxStaticText(sbSizer78->GetStaticBox(), wxID_ANY, wxT("CP2"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT18->Wrap(-1);
    fgSizer102->Add(ID_STATICTEXT18, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    cmbCP2_PLL =
        new wxComboBox(sbSizer78->GetStaticBox(), ID_CP2_PLL, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0);
    cmbCP2_PLL->SetToolTip(wxT("Controls the value of CP2 (cap from CP output to GND) in the PLL filter"));

    fgSizer102->Add(cmbCP2_PLL, 0, wxEXPAND, 0);

    ID_STATICTEXT19 = new wxStaticText(sbSizer78->GetStaticBox(), wxID_ANY, wxT("CP3"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT19->Wrap(-1);
    fgSizer102->Add(ID_STATICTEXT19, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    cmbCP3_PLL =
        new wxComboBox(sbSizer78->GetStaticBox(), ID_CP3_PLL, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0);
    cmbCP3_PLL->SetToolTip(wxT("Controls the value of CP3 (cap from VCO Vtune input to GND) in the PLL filter"));

    fgSizer102->Add(cmbCP3_PLL, 0, wxEXPAND, 0);

    ID_STATICTEXT20 = new wxStaticText(sbSizer78->GetStaticBox(), wxID_ANY, wxT("CZ"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT20->Wrap(-1);
    fgSizer102->Add(ID_STATICTEXT20, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    cmbCZ = new wxComboBox(sbSizer78->GetStaticBox(), ID_CZ, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0);
    cmbCZ->SetToolTip(wxT("Controls the value of CZ (Zero capacitor) in the PLL filter"));

    fgSizer102->Add(cmbCZ, 0, wxEXPAND, 0);

    sbSizer78->Add(fgSizer102, 1, wxEXPAND, 5);

    fgSizer198->Add(sbSizer78, 1, wxEXPAND, 5);

    wxString rgrSEL_VCOChoices[] = { wxT("VCOL"), wxT("VCOM"), wxT("VCOH") };
    int rgrSEL_VCONChoices = sizeof(rgrSEL_VCOChoices) / sizeof(wxString);
    rgrSEL_VCO = new wxRadioBox(this,
        ID_SEL_VCO,
        wxT("Active VCO"),
        wxDefaultPosition,
        wxDefaultSize,
        rgrSEL_VCONChoices,
        rgrSEL_VCOChoices,
        1,
        wxRA_SPECIFY_COLS);
    rgrSEL_VCO->SetSelection(0);
    rgrSEL_VCO->SetToolTip(wxT("Selects the active VCO. It is set by SX_SWC_calibration"));

    fgSizer198->Add(rgrSEL_VCO, 0, wxEXPAND, 0);

    wxStaticBoxSizer* sbSizer79;
    sbSizer79 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Frequency, MHz")), wxVERTICAL);

    txtFrequency = new wxTextCtrl(sbSizer79->GetStaticBox(), wxID_ANY, wxT("2140"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer79->Add(txtFrequency, 0, wxEXPAND, 5);

    btnCalculate = new wxButton(sbSizer79->GetStaticBox(), wxID_ANY, wxT("Calculate"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer79->Add(btnCalculate, 0, wxALIGN_CENTER_HORIZONTAL, 5);

    btnTune = new wxButton(sbSizer79->GetStaticBox(), wxID_ANY, wxT("Tune"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer79->Add(btnTune, 0, wxALIGN_CENTER_HORIZONTAL, 5);

    ID_BUTTON5 = new wxButton(sbSizer79->GetStaticBox(), wxID_ANY, wxT("Coarse Tune"), wxDefaultPosition, wxDefaultSize, 0);
    ID_BUTTON5->Hide();

    sbSizer79->Add(ID_BUTTON5, 1, wxALIGN_CENTER_HORIZONTAL, 5);

    fgSizer198->Add(sbSizer79, 0, wxEXPAND, 5);

    pnlRefClkSpur = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    pnlRefClkSpur->Hide();

    RefClkSpurSizer =
        new wxStaticBoxSizer(new wxStaticBox(pnlRefClkSpur, wxID_ANY, wxT("Receiver Ref Clk \nSpur Cancellation")), wxVERTICAL);

    chkEnableRefSpurCancellation =
        new wxCheckBox(RefClkSpurSizer->GetStaticBox(), wxID_ANY, wxT("Enable"), wxDefaultPosition, wxDefaultSize, 0);
    RefClkSpurSizer->Add(chkEnableRefSpurCancellation, 0, 0, 5);

    m_staticText359 = new wxStaticText(
        RefClkSpurSizer->GetStaticBox(), wxID_ANY, wxT("RF Bandwidth (MHz):"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText359->Wrap(-1);
    RefClkSpurSizer->Add(m_staticText359, 0, wxALIGN_CENTER_HORIZONTAL, 5);

    txtRefSpurBW = new wxTextCtrl(RefClkSpurSizer->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    txtRefSpurBW->Enable(false);

    RefClkSpurSizer->Add(txtRefSpurBW, 0, wxEXPAND | wxALL, 5);

    pnlRefClkSpur->SetSizer(RefClkSpurSizer);
    pnlRefClkSpur->Layout();
    RefClkSpurSizer->Fit(pnlRefClkSpur);

    fgSizer103->Add(fgSizer198, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizerFrequencyControls;
    sbSizerFrequencyControls = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Frequency Controls")), wxVERTICAL);

    wxFlexGridSizer* fgSizer194;
    fgSizer194 = new wxFlexGridSizer(0, 2, 5, 5);
    fgSizer194->AddGrowableCol(0);
    fgSizer194->SetFlexibleDirection(wxBOTH);
    fgSizer194->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer80;
    sbSizer80 = new wxStaticBoxSizer(
        new wxStaticBox(sbSizerFrequencyControls->GetStaticBox(), wxID_ANY, wxT("Calculated Values for Fractional Mode")),
        wxVERTICAL);

    wxFlexGridSizer* fgSizer104;
    fgSizer104 = new wxFlexGridSizer(0, 4, 5, 5);
    fgSizer104->AddGrowableCol(1);
    fgSizer104->AddGrowableCol(3);
    fgSizer104->SetFlexibleDirection(wxBOTH);
    fgSizer104->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_STATICTEXT2 = new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("N Integer:"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT2->Wrap(-1);
    fgSizer104->Add(ID_STATICTEXT2, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    lblINT_SDM = new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("???"), wxDefaultPosition, wxSize(-1, -1), 0);
    lblINT_SDM->Wrap(0);
    fgSizer104->Add(lblINT_SDM, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    ID_STATICTEXT21 =
        new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("N Fractional:"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT21->Wrap(-1);
    fgSizer104->Add(ID_STATICTEXT21, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    lblFRAC_SDM = new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("???"), wxDefaultPosition, wxDefaultSize, 0);
    lblFRAC_SDM->Wrap(0);
    fgSizer104->Add(lblFRAC_SDM, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    ID_STATICTEXT25 =
        new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("Output Freq, MHz:"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT25->Wrap(-1);
    fgSizer104->Add(ID_STATICTEXT25, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    lblRealOutFrequency = new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("???"), wxDefaultPosition, wxDefaultSize, 0);
    lblRealOutFrequency->Wrap(0);
    fgSizer104->Add(lblRealOutFrequency, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    ID_STATICTEXT23 = new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("Divider:"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT23->Wrap(-1);
    fgSizer104->Add(ID_STATICTEXT23, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    lblDivider = new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("???"), wxDefaultPosition, wxDefaultSize, 0);
    lblDivider->Wrap(0);
    fgSizer104->Add(lblDivider, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    ID_STATICTEXT30 =
        new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("DIV2 prescaler:"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT30->Wrap(-1);
    fgSizer104->Add(ID_STATICTEXT30, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    lblEN_DIV2_DIVPROG =
        new wxStaticText(sbSizer80->GetStaticBox(), ID_EN_DIV2_DIVPROG, wxT("?"), wxDefaultPosition, wxDefaultSize, 0);
    lblEN_DIV2_DIVPROG->Wrap(0);
    fgSizer104->Add(lblEN_DIV2_DIVPROG, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    btnChangeRefClk =
        new wxButton(sbSizer80->GetStaticBox(), wxID_ANY, wxT("Reference clock (MHz):"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer104->Add(btnChangeRefClk, 1, wxEXPAND, 5);

    lblRefClk_MHz = new wxStaticText(sbSizer80->GetStaticBox(), wxID_ANY, wxT("???"), wxDefaultPosition, wxSize(-1, -1), 0);
    lblRefClk_MHz->Wrap(0);
    fgSizer104->Add(lblRefClk_MHz, 1, wxALIGN_CENTER_VERTICAL, 5);

    sbSizer80->Add(fgSizer104, 0, wxEXPAND | wxALL, 5);

    fgSizer194->Add(sbSizer80, 0, wxLEFT | wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer76;
    sbSizer76 = new wxStaticBoxSizer(
        new wxStaticBox(sbSizerFrequencyControls->GetStaticBox(), wxID_ANY, wxT("VCO Comparators")), wxVERTICAL);

    wxFlexGridSizer* fgSizer97;
    fgSizer97 = new wxFlexGridSizer(0, 4, 0, 5);
    fgSizer97->AddGrowableRow(0);
    fgSizer97->SetFlexibleDirection(wxBOTH);
    fgSizer97->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_STATICTEXT16 = new wxStaticText(sbSizer76->GetStaticBox(), wxID_ANY, wxT("High:"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT16->Wrap(-1);
    fgSizer97->Add(ID_STATICTEXT16, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    lblVCO_CMPHO = new wxStaticText(sbSizer76->GetStaticBox(), ID_VCO_CMPHO, wxT("???"), wxDefaultPosition, wxDefaultSize, 0);
    lblVCO_CMPHO->Wrap(-1);
    fgSizer97->Add(lblVCO_CMPHO, 0, wxLEFT | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    ID_STATICTEXT17 = new wxStaticText(sbSizer76->GetStaticBox(), wxID_ANY, wxT("Low:"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT17->Wrap(-1);
    fgSizer97->Add(ID_STATICTEXT17, 0, wxLEFT | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 10);

    lblVCO_CMPLO = new wxStaticText(sbSizer76->GetStaticBox(), ID_VCO_CMPLO, wxT("???"), wxDefaultPosition, wxDefaultSize, 0);
    lblVCO_CMPLO->Wrap(-1);
    fgSizer97->Add(lblVCO_CMPLO, 0, wxLEFT | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    sbSizer76->Add(fgSizer97, 1, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    btnUpdateValues = new wxButton(sbSizer76->GetStaticBox(), wxID_ANY, wxT("Read"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer76->Add(btnUpdateValues, 1, wxALIGN_CENTER_HORIZONTAL, 5);

    fgSizer194->Add(sbSizer76, 1, wxEXPAND, 5);

    sbSizerFrequencyControls->Add(fgSizer194, 0, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer1961;
    fgSizer1961 = new wxFlexGridSizer(0, 2, 5, 5);
    fgSizer1961->AddGrowableCol(1);
    fgSizer1961->SetFlexibleDirection(wxBOTH);
    fgSizer1961->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_STATICTEXT1 =
        new wxStaticText(sbSizerFrequencyControls->GetStaticBox(), wxID_ANY, wxT("CSW_VCO"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT1->Wrap(-1);
    fgSizer1961->Add(ID_STATICTEXT1, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    ctrCSW_VCO = new NumericSlider(sbSizerFrequencyControls->GetStaticBox(),
        ID_CSW_VCO,
        wxEmptyString,
        wxDefaultPosition,
        wxDefaultSize,
        wxSP_ARROW_KEYS,
        0,
        255,
        128);
    fgSizer1961->Add(ctrCSW_VCO, 0, wxEXPAND, 5);

    ID_STATICTEXT10 = new wxStaticText(
        sbSizerFrequencyControls->GetStaticBox(), wxID_ANY, wxT("Scales VCO bias current"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT10->Wrap(-1);
    fgSizer1961->Add(ID_STATICTEXT10, 0, wxTOP | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    cmbICT_VCO = new NumericSlider(sbSizerFrequencyControls->GetStaticBox(),
        ID_ICT_VCO,
        wxEmptyString,
        wxDefaultPosition,
        wxDefaultSize,
        wxSP_ARROW_KEYS,
        0,
        255,
        0);
    fgSizer1961->Add(cmbICT_VCO, 0, wxEXPAND, 5);

    sbSizerFrequencyControls->Add(fgSizer1961, 0, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer196;
    fgSizer196 = new wxFlexGridSizer(0, 3, 5, 5);
    fgSizer196->SetFlexibleDirection(wxBOTH);
    fgSizer196->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    btnShowVCOparams = new wxButton(
        sbSizerFrequencyControls->GetStaticBox(), ID_BTN_SHOW_VCO, wxT("VCO params"), wxDefaultPosition, wxDefaultSize, 0);

    fgSizer196->Add(btnShowVCOparams, 0, wxALL | wxALIGN_CENTER_VERTICAL, 0);

    wxFlexGridSizer* fgSizer98;
    fgSizer98 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer98->SetFlexibleDirection(wxBOTH);
    fgSizer98->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_STATICTEXT9 = new wxStaticText(
        sbSizerFrequencyControls->GetStaticBox(), wxID_ANY, wxT("LDO output voltage"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT9->Wrap(-1);
    fgSizer98->Add(ID_STATICTEXT9, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    cmbVDIV_VCO = new wxComboBox(
        sbSizerFrequencyControls->GetStaticBox(), ID_VDIV_VCO, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0);
    cmbVDIV_VCO->SetToolTip(wxT("Controls VCO LDO output voltage"));

    fgSizer98->Add(cmbVDIV_VCO, 0, wxALIGN_CENTER_VERTICAL, 0);

    fgSizer196->Add(fgSizer98, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);

    sbSizerFrequencyControls->Add(fgSizer196, 0, wxEXPAND, 5);

    fgSizer103->Add(sbSizerFrequencyControls, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer193;
    fgSizer193 = new wxFlexGridSizer(0, 4, 0, 5);
    fgSizer193->AddGrowableCol(1);
    fgSizer193->SetFlexibleDirection(wxBOTH);
    fgSizer193->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_STATICTEXT13 =
        new wxStaticText(this, wxID_ANY, wxT("Scales pulse current of charge pump"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT13->Wrap(-1);
    fgSizer193->Add(ID_STATICTEXT13, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    cmbIPULSE_CP = new wxComboBox(this, ID_IPULSE_CP, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0);
    cmbIPULSE_CP->SetToolTip(wxT("Scales the pulse current of the charge pump"));

    fgSizer193->Add(cmbIPULSE_CP, 0, wxEXPAND, 0);

    ID_STATICTEXT8 = new wxStaticText(this, wxID_ANY, wxT("Feedback divider for SDM"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT8->Wrap(-1);
    fgSizer193->Add(ID_STATICTEXT8, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    cmbSEL_SDMCLK = new wxComboBox(this, ID_SEL_SDMCLK, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0);
    cmbSEL_SDMCLK->SetToolTip(wxT("Selects between the feedback divider output and Fref for SDM"));

    fgSizer193->Add(cmbSEL_SDMCLK, 0, wxEXPAND, 0);

    ID_STATICTEXT11 = new wxStaticText(this, wxID_ANY, wxT("Reference voltage"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT11->Wrap(-1);
    fgSizer193->Add(ID_STATICTEXT11, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    cmbRSEL_LDO_VCO = new wxComboBox(this, ID_RSEL_LDO_VCO, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0);
    cmbRSEL_LDO_VCO->SetToolTip(wxT("Set the reference voltage that supplies bias voltage of switch-cap array and varactor"));

    fgSizer193->Add(cmbRSEL_LDO_VCO, 0, wxEXPAND, 0);

    ID_STATICTEXT12 = new wxStaticText(this, wxID_ANY, wxT("Scales offset of charge pump"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT12->Wrap(-1);
    fgSizer193->Add(ID_STATICTEXT12, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    cmbIOFFSET_CP = new wxComboBox(this, ID_IOFFSET_CP, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0);
    cmbIOFFSET_CP->SetToolTip(wxT("Scales the offset current of the charge pump, 0-->63. This current is used in Fran-N mode to "
                                  "create an offset in the CP response and avoid the non-linear section"));

    fgSizer193->Add(cmbIOFFSET_CP, 0, wxEXPAND, 0);

    fgSizer103->Add(fgSizer193, 0, wxEXPAND, 0);

    sizerR3->SetFlexibleDirection(wxBOTH);
    sizerR3->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer103->Add(sizerR3, 1, wxEXPAND, 5);

    fgSizer92->Add(fgSizer103, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);

    fgSizer92->Add(pnlRefClkSpur, 1, 0, 5);

    SetSizer(fgSizer92);
    Layout();
    fgSizer92->Fit(this);

    // Connect Events
    chkPD_FBDIV->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkPD_LOCH_T2RBUF->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkPD_CP->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkPD_FDIV->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkPD_SDM->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkPD_VCO_COMP->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkPD_VCO->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkEN_G->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkEN_DIR_SXRSXT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkRESET_N->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkSPDUP_VCO->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkBYPLDO_VCO->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkEN_COARSEPLL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkCURLIM_VCO->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkREVPH_PFD->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkEN_INTONLY_SDM->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkEN_SDM_CLK->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkREV_SDMCLK->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    chkSX_DITHER_EN->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    //chkEN_DIV2_DIVPROGenabled->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( lms7002_pnlSX_view::OnDIV2PrescalerChange ), NULL, this );
    cmbTST_SX->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    ctrPW_DIV2_LOCH->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms7002_pnlSX_view::SpinParameterChangeHandler), nullptr, this);
    ctrPW_DIV4_LOCH->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms7002_pnlSX_view::SpinParameterChangeHandler), nullptr, this);
    ctrDIV_LOCH->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms7002_pnlSX_view::SpinParameterChangeHandler), nullptr, this);
    cmbCP2_PLL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    cmbCP3_PLL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    cmbCZ->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    rgrSEL_VCO->Connect(
        wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    btnCalculate->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::OnbtnCalculateClick), nullptr, this);
    btnTune->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::OnbtnTuneClick), nullptr, this);
    chkEnableRefSpurCancellation->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::OnEnableRefSpurCancellation), nullptr, this);
    btnChangeRefClk->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::OnbtnChangeRefClkClick), nullptr, this);
    btnUpdateValues->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::OnbtnReadComparators), nullptr, this);
    ctrCSW_VCO->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms7002_pnlSX_view::SpinParameterChangeHandler), nullptr, this);
    cmbICT_VCO->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms7002_pnlSX_view::SpinParameterChangeHandler), nullptr, this);
    btnShowVCOparams->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlSX_view::OnShowVCOclicked), nullptr, this);
    cmbVDIV_VCO->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    cmbIPULSE_CP->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    cmbSEL_SDMCLK->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    cmbRSEL_LDO_VCO->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    cmbIOFFSET_CP->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);

    sizerR3->Add(new wxStaticText(this, wxID_ANY, _("PLL LPF zero resistor:")), 1, wxALIGN_CENTER_VERTICAL, 0);
    cmbRZ_CTRL = new wxComboBox(this, wxID_ANY);
    cmbRZ_CTRL->Append(_("Rzero = 20 kOhm"));
    cmbRZ_CTRL->Append(_("Rzero = 8 kOhm"));
    cmbRZ_CTRL->Append(_("Rzero = 4 kOhm"));
    cmbRZ_CTRL->Append(_("LPF resistors are in bypass mode (<100 Ohm)"));
    cmbRZ_CTRL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    wndId2Enum[cmbRZ_CTRL] = LMS7002MCSR::RZ_CTRL;
    sizerR3->Add(cmbRZ_CTRL, 0, wxLEFT | wxEXPAND, 5);

    sizerR3->Add(new wxStaticText(this, wxID_ANY, _("CMPLO_CTRL:")), 1, wxALIGN_CENTER_VERTICAL, 0);
    cmbCMPLO_CTRL = new wxComboBox(this, wxID_ANY);
    cmbCMPLO_CTRL->Append(_("Low threshold is set to 0.18V"));
    cmbCMPLO_CTRL->Append(_("Low threshold is set to 0.1V"));
    cmbCMPLO_CTRL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms7002_pnlSX_view::ParameterChangeHandler), nullptr, this);
    sizerR3->Add(cmbCMPLO_CTRL, 0, wxLEFT | wxEXPAND, 5);
    wndId2Enum[cmbCMPLO_CTRL] = LMS7002MCSR::CMPLO_CTRL_SX;

    //ids for updating from chip
    wndId2Enum[chkBYPLDO_VCO] = LMS7002MCSR::BYPLDO_VCO;
    wndId2Enum[cmbCP2_PLL] = LMS7002MCSR::CP2_PLL;
    wndId2Enum[cmbCP3_PLL] = LMS7002MCSR::CP3_PLL;
    wndId2Enum[ctrCSW_VCO] = LMS7002MCSR::CSW_VCO;
    wndId2Enum[chkCURLIM_VCO] = LMS7002MCSR::CURLIM_VCO;
    wndId2Enum[cmbCZ] = LMS7002MCSR::CZ;
    wndId2Enum[ctrDIV_LOCH] = LMS7002MCSR::DIV_LOCH;
    wndId2Enum[chkEN_COARSEPLL] = LMS7002MCSR::EN_COARSEPLL;
    wndId2Enum[chkEN_DIV2_DIVPROGenabled] = LMS7002MCSR::EN_DIV2_DIVPROG;
    wndId2Enum[chkEN_G] = LMS7002MCSR::EN_G;
    wndId2Enum[chkEN_INTONLY_SDM] = LMS7002MCSR::EN_INTONLY_SDM;
    wndId2Enum[chkEN_SDM_CLK] = LMS7002MCSR::EN_SDM_CLK;
    wndId2Enum[cmbICT_VCO] = LMS7002MCSR::ICT_VCO;
    wndId2Enum[cmbIOFFSET_CP] = LMS7002MCSR::IOFFSET_CP;
    wndId2Enum[cmbIPULSE_CP] = LMS7002MCSR::IPULSE_CP;
    wndId2Enum[chkPD_CP] = LMS7002MCSR::PD_CP;
    wndId2Enum[chkPD_FDIV] = LMS7002MCSR::PD_FDIV;
    wndId2Enum[chkPD_LOCH_T2RBUF] = LMS7002MCSR::PD_LOCH_T2RBUF;
    wndId2Enum[chkPD_SDM] = LMS7002MCSR::PD_SDM;
    wndId2Enum[chkPD_VCO] = LMS7002MCSR::PD_VCO;
    wndId2Enum[chkPD_VCO_COMP] = LMS7002MCSR::PD_VCO_COMP;
    wndId2Enum[ctrPW_DIV2_LOCH] = LMS7002MCSR::PW_DIV2_LOCH;
    wndId2Enum[ctrPW_DIV4_LOCH] = LMS7002MCSR::PW_DIV4_LOCH;
    wndId2Enum[chkRESET_N] = LMS7002MCSR::RESET_N;
    wndId2Enum[chkREVPH_PFD] = LMS7002MCSR::REVPH_PFD;
    wndId2Enum[chkREV_SDMCLK] = LMS7002MCSR::REV_SDMCLK;
    wndId2Enum[cmbRSEL_LDO_VCO] = LMS7002MCSR::RSEL_LDO_VCO;
    wndId2Enum[cmbSEL_SDMCLK] = LMS7002MCSR::SEL_SDMCLK;
    wndId2Enum[rgrSEL_VCO] = LMS7002MCSR::SEL_VCO;
    wndId2Enum[chkSPDUP_VCO] = LMS7002MCSR::SPDUP_VCO;
    wndId2Enum[chkSX_DITHER_EN] = LMS7002MCSR::SX_DITHER_EN;
    wndId2Enum[cmbTST_SX] = LMS7002MCSR::TST_SX;
    wndId2Enum[cmbVDIV_VCO] = LMS7002MCSR::VDIV_VCO;
    wndId2Enum[chkPD_FBDIV] = LMS7002MCSR::PD_FBDIV;
    wndId2Enum[chkEN_DIR_SXRSXT] = LMS7002MCSR::EN_DIR_SXRSXT;
    wndId2Enum[lblINT_SDM] = LMS7002MCSR::INT_SDM;
    wndId2Enum[lblEN_DIV2_DIVPROG] = LMS7002MCSR::EN_DIV2_DIVPROG;

    char ctemp[80];
    wxArrayString temp;

    temp.clear();
    for (int i = 0; i < 16; ++i)
    {
        std::snprintf(ctemp, sizeof(ctemp), "%.3f pF", (i * 6 * 387.0) / 1000.0);
        temp.push_back(ctemp);
    }
    cmbCP2_PLL->Set(temp);

    temp.clear();
    for (int i = 0; i < 16; ++i)
    {
        std::snprintf(ctemp, sizeof(ctemp), "%.3f pF", (i * 6 * 980.0) / 1000.0);
        temp.push_back(ctemp);
    }
    cmbCP3_PLL->Set(temp);

    temp.clear();
    for (int i = 0; i < 16; ++i)
    {
        std::snprintf(ctemp, sizeof(ctemp), "%.3f pF", i * 8 * 5.88);
        temp.push_back(ctemp);
    }
    cmbCZ->Set(temp);

    temp.clear();
    for (int i = 0; i < 64; ++i)
    {
        std::snprintf(ctemp, sizeof(ctemp), "%.3f uA", 0.243 * i);
        temp.push_back(ctemp);
    }
    cmbIOFFSET_CP->Set(temp);

    temp.clear();
    for (int i = 0; i < 64; ++i)
    {
        std::snprintf(ctemp, sizeof(ctemp), "%.3f uA", 2.312 * i);
        temp.push_back(ctemp);
    }
    cmbIPULSE_CP->Set(temp);

    temp.clear();
    for (int i = 0; i <= 6; ++i)
    {
        std::snprintf(ctemp, sizeof(ctemp), "1.6 V");
        temp.push_back(ctemp);
    }
    for (int i = 7; i < 32; ++i)
    {
        std::snprintf(ctemp, sizeof(ctemp), "%.3f V", (0.000060 * 180000.0 / i));
        temp.push_back(ctemp);
    }
    cmbRSEL_LDO_VCO->Set(temp);

    temp.clear();
    temp.push_back("CLK_DIV");
    temp.push_back("CLK_REF");
    cmbSEL_SDMCLK->Set(temp);

    temp.clear();
    temp.push_back("TST disabled");
    temp.push_back("TST[0]=VCO/20 clk*; TST[1]=VCO/40 clk*; TSTA=Hi Z");
    temp.push_back("TST[0]=SDM clk; TST[1]=FBD output; TSTA=Vtune@60kOhm");
    temp.push_back("TST[0]=Ref clk; TST[1]=FBD output; TSTA=Vtune@10kOhm");
    temp.push_back("TST[0]=Hi Z; TST[1]=Hi Z; TSTA=Hi Z");
    temp.push_back("TST[0]=CP Down offset; TST[1]=CP Up offset; TSTA=Hi Z");
    temp.push_back("TST[0]=Hi Z; TST[1]=Hi Z; TSTA=Vtune@60kOhm");
    temp.push_back("TST[0]=Hi Z; TST[1]=Hi Z; TSTA=Vtune@10kOhm");
    cmbTST_SX->Set(temp);

    temp.clear();
    const float VDD18_VCO = 1.72;
    for (int i = 0; i < 256; ++i)
    {
        float value = VDD18_VCO * (29.1 / (29.1 + 233.0 / (i + 2)));
        temp.push_back(wxString::Format("%.4f V", value));
    }
    cmbVDIV_VCO->Set(temp);

    txtRefSpurBW->SetValue(_("5"));

    LMS7002_WXGUI::UpdateTooltips(wndId2Enum, true);
    if (showRefClkSpurCancellation)
    {
        pnlRefClkSpur->Show();
        showRefClkSpurCancellation = false;
    }
}

void lms7002_pnlSX_view::ParameterChangeHandler(wxCommandEvent& event)
{
    ILMS7002MTab::ParameterChangeHandler(event);

    if (event.GetEventObject() == ctrCSW_VCO) //for convenience refresh comparator values
    {
        wxCommandEvent evt;
        OnbtnReadComparators(evt);
    }
}

void lms7002_pnlSX_view::OnbtnReadComparators(wxCommandEvent& event)
{
    assert(lmsControl != nullptr);
    uint16_t value;
    value = ReadParam(LMS7002MCSR::VCO_CMPHO);

    lblVCO_CMPHO->SetLabel(std::to_string(value));
    if (value == 1)
        lblVCO_CMPHO->SetBackgroundColour(*wxGREEN);
    else
        lblVCO_CMPHO->SetBackgroundColour(*wxRED);

    value = ReadParam(LMS7002MCSR::VCO_CMPLO);
    lblVCO_CMPLO->SetLabel(std::to_string(value));
    if (value == 0)
        lblVCO_CMPLO->SetBackgroundColour(*wxGREEN);
    else
        lblVCO_CMPLO->SetBackgroundColour(*wxRED);
}

void lms7002_pnlSX_view::OnbtnChangeRefClkClick(wxCommandEvent& event)
{
    assert(lmsControl != nullptr);
    wxTextEntryDialog* dlg = new wxTextEntryDialog(this, _("Enter reference clock, MHz"), _("Reference clock"));
    double refClkMHz;
    dlg->SetTextValidator(wxFILTER_NUMERIC);
    double freq;

    freq = lmsControl->GetReferenceClk_SX(direction);
    dlg->SetValue(std::to_string(freq / 1e6));
    if (dlg->ShowModal() != wxID_OK)
    {
        return;
    }

    dlg->GetValue().ToDouble(&refClkMHz);
    if (refClkMHz == 0)
    {
        return;
    }

    double currentFreq_MHz;
    txtFrequency->GetValue().ToDouble(&currentFreq_MHz);
    lmsControl->SetReferenceClk_SX(direction, refClkMHz * 1e6);
    OpStatus status = lmsControl->SetFrequencySX(direction, currentFreq_MHz * 1e6);
    if (status != OpStatus::Success)
        wxMessageBox(_("Set SX frequency failed"));
    UpdateGUI();
}

void lms7002_pnlSX_view::OnbtnCalculateClick(wxCommandEvent& event)
{
    assert(lmsControl != nullptr);
    double freqMHz;
    txtFrequency->GetValue().ToDouble(&freqMHz);
    double RefClkMHz;
    lblRefClk_MHz->GetLabel().ToDouble(&RefClkMHz);
    lmsControl->SetReferenceClk_SX(direction, RefClkMHz * 1e6);

    double BWMHz;
    txtRefSpurBW->GetValue().ToDouble(&BWMHz);
    OpStatus status;

    if (chkEnableRefSpurCancellation->IsChecked())
        status = lmsControl->SetFrequencySXWithSpurCancellation(direction, freqMHz * 1e6, BWMHz * 1e6);
    else
        status = lmsControl->SetFrequencySX(direction, freqMHz * 1e6);

    if (status != OpStatus::Success)
        wxMessageBox(_("Set SX frequency failed"));
    else
    {
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        evt.SetInt(static_cast<int>(lime::LogLevel::Info));
        wxString msg = direction == TRXDir::Rx ? _("SXR") : _("SXT");
        msg += wxString::Format(_(" frequency set to %f MHz"), freqMHz);
        evt.SetString(msg);
        wxPostEvent(this, evt);
    }
    UpdateGUI();
}

void lms7002_pnlSX_view::OnbtnTuneClick(wxCommandEvent& event)
{
    assert(lmsControl != nullptr);
    OpStatus status = lmsControl->TuneVCO(direction == TRXDir::Tx ? LMS7002M::VCO_Module::VCO_SXT : LMS7002M::VCO_Module::VCO_SXR);
    if (status != OpStatus::Success)
        wxMessageBox(_("SX VCO Tune Failed"));
    UpdateGUI();
}

void lms7002_pnlSX_view::UpdateGUI()
{
    if (lmsControl == nullptr)
        return;
    LMS7002_WXGUI::UpdateControlsByMap(this, lmsControl, wndId2Enum, mChannel);
    double freq;
    freq = lmsControl->GetReferenceClk_SX(direction);
    lblRefClk_MHz->SetLabel(wxString::Format(_("%.3f"), freq / 1e6));
    freq = lmsControl->GetFrequencySX(direction);
    lblRealOutFrequency->SetLabel(wxString::Format(_("%.3f"), freq / 1e6));
    // TODO: if(chkEnableRefSpurCancellation->IsChecked())
    // {
    //     uint16_t downconvert = 0;
    //     downconvert = ReadParam(LMS7002MCSR::CMIX_SC_RXTSP);
    //     double* freqNCO = new double[16];
    //     double PHO;
    //     LMS_GetNCOFrequency(lmsControl, false, 0, freqNCO, &PHO);
    //     if(downconvert)
    //         freq += freqNCO[15];
    //     else
    //         freq -= freqNCO[15];
    //     delete[] freqNCO;
    // }
    txtFrequency->SetValue(wxString::Format(_("%.3f"), freq / 1e6));
    uint16_t div;
    div = ReadParam(LMS7002MCSR::DIV_LOCH);
    lblDivider->SetLabel("2^"s + std::to_string(div));

    uint16_t value;
    value = ReadParam(LMS7002MCSR::FRAC_SDM_MSB);
    int fracValue = value << 16;
    value = ReadParam(LMS7002MCSR::FRAC_SDM_LSB);
    fracValue |= value;
    lblFRAC_SDM->SetLabel(std::to_string(fracValue));

    if (direction == TRXDir::Rx)
        chkPD_LOCH_T2RBUF->Hide();
    else
        chkPD_LOCH_T2RBUF->Show();

    wxCommandEvent evt;
    OnbtnReadComparators(evt);
}

void lms7002_pnlSX_view::OnShowVCOclicked(wxCommandEvent& event)
{
    lms7002_dlgVCOfrequencies* dlg = new lms7002_dlgVCOfrequencies(this, lmsControl);
    dlg->ShowModal();
    dlg->Destroy();
}

void lms7002_pnlSX_view::OnEnableRefSpurCancellation(wxCommandEvent& event)
{
    // TODO:
    // txtRefSpurBW->Enable(chkEnableRefSpurCancellation->IsChecked());
    // uint16_t ch = 0;
    // ReadParam( LMS7002MCSR::MAC, &ch);
    // for(int i=0; i<2; ++i)
    // {
    //     WriteParam( LMS7002MCSR::MAC, i+1);
    //     WriteParam( LMS7002MCSR::CMIX_GAIN_RXTSP, 1);
    //     WriteParam( LMS7002MCSR::CMIX_BYP_RXTSP, 0);
    //     if(chkEnableRefSpurCancellation->IsChecked())
    //     {
    //         WriteParam( LMS7002MCSR::SEL_RX, 15);
    //     }
    //     else
    //     {
    //         WriteParam( LMS7002MCSR::SEL_RX, 14);
    //     }
    // }
    // WriteParam(LMS7002MCSR::MAC, ch);
    // UpdateGUI();
}
