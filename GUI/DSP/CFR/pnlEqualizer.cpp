#include "pnlEqualizer.h"

#include "commonWxHeaders.h"
#include <wx/filedlg.h>
#include <wx/msgdlg.h>
#include <wx/spinctrl.h>
#include "events.h"

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/LMS7002MCSR.h"

#include "chips/LMS7002M/lms7002_dlgGFIR_Coefficients.h"

using namespace lime;
using namespace std;

BEGIN_EVENT_TABLE(EqualiserTest, wxPanel)
END_EVENT_TABLE()

OpStatus EqualiserTest::FPGAWriteRegister(uint32_t addr, uint32_t val)
{
    equalizer->WriteRegister(Register(addr, 15, 0), val);
    return OpStatus::Success;
}

OpStatus EqualiserTest::FPGAReadRegister(uint32_t addr, uint16_t* value)
{
    *value = equalizer->ReadRegister(Register(addr, 15, 0));
    return OpStatus::Success;
}

EqualiserTest::EqualiserTest(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, int style)
    : ISOCPanel(parent, id, pos, size, style)
    , equalizer(nullptr)
{
#ifdef WIN32
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
#endif
    wxFlexGridSizer* mainSizer = new wxFlexGridSizer(0, 2, 5, 5);
    SetSizer(mainSizer);

    wxSize spinBoxSize = wxDefaultSize;
    long spinBoxStyle = wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER;
    wxSize freqTextfieldSize = wxDefaultSize; //(64, -1);

    wxFlexGridSizer* chSizer = new wxFlexGridSizer(0, 2, 0, 0);
    wxFlexGridSizer* LSizer = new wxFlexGridSizer(0, 1, 0, 5);
    wxFlexGridSizer* RSizer = new wxFlexGridSizer(0, 1, 0, 5);

    //wxStaticBoxSizer *chSelectGroup = new wxStaticBoxSizer(wxHORIZONTAL, this, _T(""));
    wxFlexGridSizer* chSelect = new wxFlexGridSizer(0, 2, 0, 0);
    rbChannelA = new wxRadioButton(this, wxNewId(), wxT("A CHANNEL"), wxDefaultPosition, wxDefaultSize, 0);
    rbChannelA->SetValue(true);
    rbChannelA->Connect(wxEVT_COMMAND_RADIOBUTTON_SELECTED, wxCommandEventHandler(EqualiserTest::OnSwitchToChannelA), NULL, this);
    chSelect->Add(rbChannelA, 0, wxEXPAND, 5);
    rbChannelB = new wxRadioButton(this, wxNewId(), wxT("B CHANNEL"), wxDefaultPosition, wxDefaultSize, 0);
    chSelect->Add(rbChannelB, 0, wxEXPAND, 5);
    rbChannelB->Connect(wxEVT_COMMAND_RADIOBUTTON_SELECTED, wxCommandEventHandler(EqualiserTest::OnSwitchToChannelB), NULL, this);
    //chSelectGroup->Add(chSelect, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    LSizer->Add(chSelect, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    wxStaticBoxSizer* CFRcontrolsGroup = new wxStaticBoxSizer(wxVERTICAL, this, _T("LMS#2 CFR controls"));
    wxFlexGridSizer* CFRcontrols = new wxFlexGridSizer(0, 2, 0, 0);
    chkTX_HB_BYP = new wxCheckBox(this, wxNewId(), _("Bypass HB1"));
    CFRcontrols->Add(chkTX_HB_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkTX_HB_BYP->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkTX_HB_DEL = new wxCheckBox(this, wxNewId(), _("HB1 delay"));
    CFRcontrols->Add(chkTX_HB_DEL, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkTX_HB_DEL->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);

    CFRcontrols->Add(new wxStaticText(this, wxID_ANY, _("")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    CFRcontrols->Add(new wxStaticText(this, wxID_ANY, _("")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    chkBYPASS_CFR = new wxCheckBox(this, wxNewId(), _("Bypass CFR"));
    CFRcontrols->Add(chkBYPASS_CFR, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkBYPASS_CFR->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkSLEEP_CFR = new wxCheckBox(this, wxNewId(), _("Sleep"));
    CFRcontrols->Add(chkSLEEP_CFR, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkSLEEP_CFR->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkODD_CFR = new wxCheckBox(this, wxNewId(), _("Odd"));
    CFRcontrols->Add(chkODD_CFR, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkODD_CFR->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkBYPASSGAIN_CFR = new wxCheckBox(this, wxNewId(), _("Bypass gain"));
    CFRcontrols->Add(chkBYPASSGAIN_CFR, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkBYPASSGAIN_CFR->GetId(),
        wxEVT_CHECKBOX,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    CFRcontrols->Add(new wxStaticText(this, wxID_ANY, _("CFR order")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinCFR_ORDER = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, 2, 40, 39);
    CFRcontrols->Add(spinCFR_ORDER, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(spinCFR_ORDER->GetId(), wxEVT_TEXT_ENTER, wxCommandEventHandler(EqualiserTest::OnOrderChanged), NULL, this);
    Connect(spinCFR_ORDER->GetId(), wxEVT_SPINCTRL, wxCommandEventHandler(EqualiserTest::OnOrderChanged), NULL, this);
    Connect(spinCFR_ORDER->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::OnOrderChanged), NULL, this);
    CFRcontrols->Add(new wxStaticText(this, wxID_ANY, _("Threshold")), 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
    thresholdSpin = new wxSpinCtrlDouble(this, wxNewId(), _("1.0"), wxDefaultPosition, wxDefaultSize, 0, 0.0, 1.0, 1.0, 0.01);
    CFRcontrols->Add(thresholdSpin, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(thresholdSpin->GetId(), wxEVT_SPINCTRLDOUBLE, wxCommandEventHandler(EqualiserTest::OnThresholdChanged), NULL, this);
    CFRcontrols->Add(new wxStaticText(this, wxID_ANY, _("Gain")), 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
    thresholdGain = new wxSpinCtrlDouble(this, wxNewId(), _("1.0"), wxDefaultPosition, wxDefaultSize, 0, 0.0, 4.0, 1.0, 0.01);
    CFRcontrols->Add(thresholdGain, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(thresholdGain->GetId(), wxEVT_SPINCTRLDOUBLE, wxCommandEventHandler(EqualiserTest::OnGainChanged), NULL, this);

    CFRcontrols->Add(new wxStaticText(this, wxID_ANY, _("")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    CFRcontrols->Add(new wxStaticText(this, wxID_ANY, _("")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    CFRcontrols->Add(new wxStaticText(this, wxID_ANY, _("Post-CFR FIR")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    chkSLEEP_FIR = new wxCheckBox(this, wxNewId(), _("Sleep"));
    CFRcontrols->Add(chkSLEEP_FIR, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkSLEEP_FIR->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkBYPASS_FIR = new wxCheckBox(this, wxNewId(), _("Bypass FIR"));
    CFRcontrols->Add(chkBYPASS_FIR, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkBYPASS_FIR->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkODD_FIR = new wxCheckBox(this, wxNewId(), _("Odd"));
    CFRcontrols->Add(chkODD_FIR, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkODD_FIR->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkTX_INVERTQ = new wxCheckBox(this, wxNewId(), _("Invert Q sig."));
    CFRcontrols->Add(chkTX_INVERTQ, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkTX_INVERTQ->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    setFIR1 = new wxButton(this, wxNewId(), _T("Coefficients"));
    Connect(setFIR1->GetId(), wxEVT_COMMAND_BUTTON_CLICKED, reinterpret_cast<wxObjectEventFunction>(&EqualiserTest::onbtnFIRCoef));
    CFRcontrols->Add(setFIR1, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 50);
    CFRcontrolsGroup->Add(CFRcontrols, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    LSizer->Add(CFRcontrolsGroup, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP | wxALL, 5);

    txtNcoFreq = new wxTextCtrl(this, wxNewId(), _("1.0"), wxDefaultPosition, freqTextfieldSize);
    wxArrayString insel_choices;
    insel_choices.push_back(_("Signal"));
    insel_choices.push_back(_("FPGA NCO"));
    cmbInsel = new wxChoice(this, wxNewId(), wxDefaultPosition, wxDefaultSize, insel_choices, 1);
    cmbInsel->SetSelection(0);
    Connect(cmbInsel->GetId(), wxEVT_CHOICE, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    wxStaticBoxSizer* loopbackGroup = new wxStaticBoxSizer(wxHORIZONTAL, this, _T("TxTSP NCO"));
    wxFlexGridSizer* loopbackSizer = new wxFlexGridSizer(0, 2, 0, 0);
    loopbackSizer->Add(
        new wxStaticText(this, wxID_ANY, _("Tx input source:")), 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
    loopbackSizer->Add(cmbInsel, 1, wxALIGN_LEFT | wxEXPAND, 5);
    loopbackSizer->Add(new wxStaticText(this, wxID_ANY, _("NCO (MHz):")), 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
    loopbackSizer->Add(txtNcoFreq, 1, wxALIGN_LEFT | wxEXPAND, 5);
    Connect(txtNcoFreq->GetId(), wxEVT_TEXT, wxCommandEventHandler(EqualiserTest::OnNcoFrequencyChanged), NULL, this);
    Connect(txtNcoFreq->GetId(), wxEVT_TEXT_ENTER, wxCommandEventHandler(EqualiserTest::OnNcoFrequencyChanged), NULL, this);
    loopbackGroup->Add(loopbackSizer, 1, wxALIGN_LEFT | wxALIGN_TOP | wxEXPAND, 5);
    LSizer->Add(loopbackGroup, 1, wxALIGN_LEFT | wxALIGN_TOP | wxEXPAND, 5);
    LSizer->Add(new wxStaticText(this, wxID_ANY, _("")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);

    wxStaticBoxSizer* equalizerGroup = new wxStaticBoxSizer(wxHORIZONTAL, this, _T("Read/write configuration"));
    wxFlexGridSizer* equalizerSizer = new wxFlexGridSizer(0, 3, 0, 0);
    btnResetEqualizer = new wxButton(this, wxNewId(), _T("Reset EQU"));
    equalizerSizer->Add(btnResetEqualizer, 1, wxALIGN_LEFT | wxEXPAND, 5);
    Connect(btnResetEqualizer->GetId(),
        wxEVT_COMMAND_BUTTON_CLICKED,
        reinterpret_cast<wxObjectEventFunction>(&EqualiserTest::ResetEqualizer));
    btnEqualizerSettings = new wxButton(this, wxNewId(), _T("Read"));
    equalizerSizer->Add(btnEqualizerSettings, 1, wxALIGN_LEFT | wxEXPAND, 5);
    Connect(btnEqualizerSettings->GetId(),
        wxEVT_COMMAND_BUTTON_CLICKED,
        reinterpret_cast<wxObjectEventFunction>(&EqualiserTest::LoadEqualizerSettings));
    btnSaveEqualizerSettings = new wxButton(this, wxNewId(), _T("Save"));
    equalizerSizer->Add(btnSaveEqualizerSettings, 1, wxALIGN_LEFT | wxEXPAND, 5);
    Connect(btnSaveEqualizerSettings->GetId(),
        wxEVT_COMMAND_BUTTON_CLICKED,
        reinterpret_cast<wxObjectEventFunction>(&EqualiserTest::SaveEqualizerSettings));
    equalizerGroup->Add(equalizerSizer, 1, wxALIGN_LEFT | wxALIGN_TOP | wxEXPAND, 5);
    LSizer->Add(equalizerGroup, 1, wxALIGN_LEFT | wxALIGN_TOP | wxEXPAND, 5);

    wxStaticBoxSizer* moduleEnablesGroup = new wxStaticBoxSizer(wxHORIZONTAL, this, _T("LMS#2 Rx/TxTSP controls"));
    wxFlexGridSizer* moduleEnables = new wxFlexGridSizer(0, 2, 0, 0);
    chkEN_RXTSP = new wxCheckBox(this, wxNewId(), _("En. RxTSP"));
    moduleEnables->Add(chkEN_RXTSP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkEN_RXTSP->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkEN_TXTSP = new wxCheckBox(this, wxNewId(), _("En. TxTSP"));
    moduleEnables->Add(chkEN_TXTSP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkEN_TXTSP->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    moduleEnablesGroup->Add(moduleEnables, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    RSizer->Add(moduleEnablesGroup, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP | wxALL, 5);

    wxStaticBoxSizer* bypassesGroup = new wxStaticBoxSizer(wxVERTICAL, this, _T("Bypasses"));
    wxFlexGridSizer* bypasses = new wxFlexGridSizer(0, 2, 0, 0);
    chkRX_GCORR_BYP = new wxCheckBox(this, wxNewId(), _("Rx GCORR"));
    bypasses->Add(chkRX_GCORR_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkRX_GCORR_BYP->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkTX_PHCORR_BYP = new wxCheckBox(this, wxNewId(), _("Tx PHCORR"));
    bypasses->Add(chkTX_PHCORR_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkTX_PHCORR_BYP->GetId(),
        wxEVT_CHECKBOX,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    chkRX_PHCORR_BYP = new wxCheckBox(this, wxNewId(), _("Rx PHCORR"));
    bypasses->Add(chkRX_PHCORR_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkRX_PHCORR_BYP->GetId(),
        wxEVT_CHECKBOX,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    chkTX_GCORR_BYP = new wxCheckBox(this, wxNewId(), _("Tx GCORR"));
    bypasses->Add(chkTX_GCORR_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkTX_GCORR_BYP->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkRX_DCCORR_BYP = new wxCheckBox(this, wxNewId(), _("Rx DCCORR"));
    bypasses->Add(chkRX_DCCORR_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkRX_DCCORR_BYP->GetId(),
        wxEVT_CHECKBOX,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    chkTX_DCCORR_BYP = new wxCheckBox(this, wxNewId(), _("Tx DCCORR"));
    bypasses->Add(chkTX_DCCORR_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkTX_DCCORR_BYP->GetId(),
        wxEVT_CHECKBOX,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    chkRX_DCLOOP_BYP = new wxCheckBox(this, wxNewId(), _("Rx DCLOOP"));
    bypasses->Add(chkRX_DCLOOP_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(chkRX_DCLOOP_BYP->GetId(),
        wxEVT_CHECKBOX,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    chkTX_ISINC_BYP = new wxCheckBox(this, wxNewId(), _("Tx ISINC"));
    bypasses->Add(chkTX_ISINC_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkTX_ISINC_BYP->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkRX_EQU_BYP = new wxCheckBox(this, wxNewId(), _("Rx EQU"));
    bypasses->Add(chkRX_EQU_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkRX_EQU_BYP->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    chkTX_EQU_BYP = new wxCheckBox(this, wxNewId(), _("Tx EQU"));
    bypasses->Add(chkTX_EQU_BYP, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        chkTX_EQU_BYP->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    bypassesGroup->Add(bypasses, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    RSizer->Add(bypassesGroup, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    wxStaticBoxSizer* tx_correctorsGroup = new wxStaticBoxSizer(wxVERTICAL, this, _T("TxTSP correctors"));
    wxFlexGridSizer* tx_correctorsSizer = new wxFlexGridSizer(0, 2, 0, 5);
    tx_correctorsSizer->Add(new wxStaticText(this, wxID_ANY, _("Tx PHCORR")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinTX_PHCORR = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, -2048, 2047, 0);
    tx_correctorsSizer->Add(spinTX_PHCORR, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        spinTX_PHCORR->GetId(), wxEVT_TEXT_ENTER, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(
        spinTX_PHCORR->GetId(), wxEVT_SPINCTRL, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(spinTX_PHCORR->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    tx_correctorsSizer->Add(new wxStaticText(this, wxID_ANY, _("Tx GCORRQ")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinTX_GCORRQ = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, 0, 2047, 2047);
    tx_correctorsSizer->Add(spinTX_GCORRQ, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        spinTX_GCORRQ->GetId(), wxEVT_TEXT_ENTER, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(
        spinTX_GCORRQ->GetId(), wxEVT_SPINCTRL, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(spinTX_GCORRQ->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    tx_correctorsSizer->Add(new wxStaticText(this, wxID_ANY, _("Tx GCORRI")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinTX_GCORRI = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, 0, 2047, 2047);
    tx_correctorsSizer->Add(spinTX_GCORRI, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        spinTX_GCORRI->GetId(), wxEVT_TEXT_ENTER, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(
        spinTX_GCORRI->GetId(), wxEVT_SPINCTRL, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(spinTX_GCORRI->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    tx_correctorsSizer->Add(new wxStaticText(this, wxID_ANY, _("Tx DCCORRI")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinTX_DCCORRI = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, -32768, 32767, 0);
    tx_correctorsSizer->Add(spinTX_DCCORRI, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(spinTX_DCCORRI->GetId(),
        wxEVT_TEXT_ENTER,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    Connect(
        spinTX_DCCORRI->GetId(), wxEVT_SPINCTRL, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(spinTX_DCCORRI->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    tx_correctorsSizer->Add(new wxStaticText(this, wxID_ANY, _("Tx DCCORRQ")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinTX_DCCORRQ = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, -32768, 32767, 0);
    tx_correctorsSizer->Add(spinTX_DCCORRQ, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(spinTX_DCCORRQ->GetId(),
        wxEVT_TEXT_ENTER,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    Connect(
        spinTX_DCCORRQ->GetId(), wxEVT_SPINCTRL, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(spinTX_DCCORRQ->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    tx_correctorsGroup->Add(tx_correctorsSizer, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    wxStaticBoxSizer* rx_correctorsGroup = new wxStaticBoxSizer(wxVERTICAL, this, _T("RxTSP correctors"));
    wxFlexGridSizer* rx_correctorsSizer = new wxFlexGridSizer(0, 2, 0, 5);
    rx_correctorsSizer->Add(new wxStaticText(this, wxID_ANY, _("Rx GCORRQ")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinRX_GCORRQ = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, 0, 2047, 2047);
    rx_correctorsSizer->Add(spinRX_GCORRQ, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        spinRX_GCORRQ->GetId(), wxEVT_TEXT_ENTER, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(
        spinRX_GCORRQ->GetId(), wxEVT_SPINCTRL, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(spinRX_GCORRQ->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    rx_correctorsSizer->Add(new wxStaticText(this, wxID_ANY, _("Rx GCORRI")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinRX_GCORRI = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, 0, 2047, 2047);
    rx_correctorsSizer->Add(spinRX_GCORRI, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        spinRX_GCORRI->GetId(), wxEVT_TEXT_ENTER, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(
        spinRX_GCORRI->GetId(), wxEVT_SPINCTRL, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(spinRX_GCORRI->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    rx_correctorsSizer->Add(new wxStaticText(this, wxID_ANY, _("Rx PHCORR")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinRX_PHCORR = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, -2048, 2047, 0);
    rx_correctorsSizer->Add(spinRX_PHCORR, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(
        spinRX_PHCORR->GetId(), wxEVT_TEXT_ENTER, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(
        spinRX_PHCORR->GetId(), wxEVT_SPINCTRL, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    Connect(spinRX_PHCORR->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    rx_correctorsSizer->Add(new wxStaticText(this, wxID_ANY, _("Rx DCLOOP_AVG")), 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    spinRX_DCLOOP_AVG = new wxSpinCtrl(this, wxNewId(), wxEmptyString, wxDefaultPosition, spinBoxSize, spinBoxStyle, 0, 6, 0);
    rx_correctorsSizer->Add(spinRX_DCLOOP_AVG, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);
    Connect(spinRX_DCLOOP_AVG->GetId(),
        wxEVT_TEXT_ENTER,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    Connect(spinRX_DCLOOP_AVG->GetId(),
        wxEVT_SPINCTRL,
        wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler),
        NULL,
        this);
    Connect(
        spinRX_DCLOOP_AVG->GetId(), wxEVT_SPIN, wxCommandEventHandler(EqualiserTest::RegisterParameterChangeHandler), NULL, this);
    rx_correctorsGroup->Add(rx_correctorsSizer, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    RSizer->Add(rx_correctorsGroup, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    RSizer->Add(tx_correctorsGroup, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    chSizer->Add(LSizer, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    chSizer->Add(RSizer, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    wxStaticBoxSizer* chBox = new wxStaticBoxSizer(wxVERTICAL, this, _T(""));
    chBox->Add(chSizer, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP | wxALL, 5);
    wxFlexGridSizer* FlexGridSizer1 = new wxFlexGridSizer(0, 1, 5, 5);
    FlexGridSizer1->Add(chBox, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    mainSizer->Add(FlexGridSizer1, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP | wxALL, 5);
    mainSizer->Fit(this);
    mainSizer->SetSizeHints(this);
    Layout();

    controlsPtr2Registers.clear();
    controlsPtr2Registers[chkEN_RXTSP] = CFR::EN_RXTSP;
    controlsPtr2Registers[chkEN_TXTSP] = CFR::EN_TXTSP;

    controlsPtr2Registers[chkRX_DCCORR_BYP] = CFR::RX_DCCORR_BYP;
    controlsPtr2Registers[chkRX_PHCORR_BYP] = CFR::RX_PHCORR_BYP;
    controlsPtr2Registers[chkRX_GCORR_BYP] = CFR::RX_GCORR_BYP;

    controlsPtr2Registers[chkRX_EQU_BYP] = CFR::RX_EQU_BYP;

    controlsPtr2Registers[chkRX_DCLOOP_BYP] = CFR::RX_DCLOOP_BYP;
    controlsPtr2Registers[spinRX_DCLOOP_AVG] = CFR::RX_DCLOOP_AVG;

    controlsPtr2Registers[chkTX_HB_BYP] = CFR::TX_HB_BYP;
    controlsPtr2Registers[chkTX_HB_DEL] = CFR::TX_HB_DEL;
    controlsPtr2Registers[chkSLEEP_CFR] = CFR::SLEEP_CFR;
    controlsPtr2Registers[chkBYPASS_CFR] = CFR::BYPASS_CFR;
    controlsPtr2Registers[chkODD_CFR] = CFR::ODD_CFR;
    controlsPtr2Registers[chkBYPASSGAIN_CFR] = CFR::BYPASSGAIN_CFR;
    controlsPtr2Registers[chkSLEEP_FIR] = CFR::SLEEP_FIR;
    controlsPtr2Registers[chkBYPASS_FIR] = CFR::BYPASS_FIR;

    controlsPtr2Registers[chkODD_FIR] = CFR::ODD_FIR;
    controlsPtr2Registers[chkTX_PHCORR_BYP] = CFR::TX_PHCORR_BYP;
    controlsPtr2Registers[chkTX_GCORR_BYP] = CFR::TX_GCORR_BYP;
    controlsPtr2Registers[chkTX_DCCORR_BYP] = CFR::TX_DCCORR_BYP;
    controlsPtr2Registers[chkTX_ISINC_BYP] = CFR::TX_ISINC_BYP;
    controlsPtr2Registers[chkTX_EQU_BYP] = CFR::TX_EQU_BYP;
    controlsPtr2Registers[chkTX_INVERTQ] = CFR::TX_INVERTQ;

    controlsPtr2Registers[spinTX_GCORRQ] = CFR::TX_GCORRQ;
    controlsPtr2Registers[spinTX_GCORRI] = CFR::TX_GCORRI;
    controlsPtr2Registers[spinTX_PHCORR] = CFR::TX_PHCORR;
    controlsPtr2Registers[spinTX_DCCORRI] = CFR::TX_DCCORRI;
    controlsPtr2Registers[spinTX_DCCORRQ] = CFR::TX_DCCORRQ;
    controlsPtr2Registers[thresholdSpin] = CFR::thresholdSpin;
    controlsPtr2Registers[thresholdGain] = CFR::thresholdGain;
    controlsPtr2Registers[spinCFR_ORDER] = CFR::CFR_ORDER;

    controlsPtr2Registers[spinRX_GCORRQ] = CFR::RX_GCORRQ;
    controlsPtr2Registers[spinRX_GCORRI] = CFR::RX_GCORRI;
    controlsPtr2Registers[spinRX_PHCORR] = CFR::RX_PHCORR;

    controlsPtr2Registers[cmbInsel] = CSRegister(0x0080, 2, 2, 0, 0);

    Bind(READ_ALL_VALUES, &EqualiserTest::OnReadAll, this, this->GetId());
    Bind(WRITE_ALL_VALUES, &EqualiserTest::OnWriteAll, this, this->GetId());

    //chkTX_HB_DEL->Enable(false);
}

void EqualiserTest::Initialize(lime::CrestFactorReduction* pControl)
{
    equalizer = pControl;
    FPGAWriteRegister(0xFFFF, rbChannelB->GetValue() ? 0x2 : 0x1);
    wxCommandEvent evt;
    OnSwitchToChannelA(evt);
}

void EqualiserTest::SetRegValue(const Register& reg, uint16_t newValue)
{
    unsigned short mask = (~(~0 << (reg.msb - reg.lsb + 1))) << reg.lsb; // creates bit mask

    uint16_t regValue;
    FPGAReadRegister(reg.address, &regValue); // read bit content

    regValue &= ~mask;
    regValue |= (newValue << reg.lsb) & mask;

    FPGAWriteRegister(reg.address, regValue); // register write
}

void EqualiserTest::OnOrderChanged(wxCommandEvent& event)
{
    int val = 0;

    wxSpinCtrl* p = reinterpret_cast<wxSpinCtrl*>(event.GetEventObject());
    val = p->GetValue();

    Register reg = controlsPtr2Registers[event.GetEventObject()];
    unsigned short mask = (~(~0 << (reg.msb - reg.lsb + 1))) << reg.lsb; // creates bit mask

    uint16_t regValue;
    FPGAReadRegister(reg.address, &regValue); // read bit content

    int order = val;
    if (order > 32)
        order = 32;
    if (order < 2)
        order = 2;

    regValue &= ~mask;
    regValue |= (order << reg.lsb) & mask;

    FPGAWriteRegister(reg.address, regValue); // register write

    equalizer->UpdateHannCoeff(order);
    wxCommandEvent evt;
    OnbtnUpdateAll(evt);
}

void EqualiserTest::OnThresholdChanged(wxCommandEvent& event)
{
    double threshold;

    wxSpinCtrlDouble* p = reinterpret_cast<wxSpinCtrlDouble*>(event.GetEventObject());
    threshold = p->GetValue();

    Register reg = controlsPtr2Registers[event.GetEventObject()];
    unsigned short mask = (~(~0 << (reg.msb - reg.lsb + 1))) << reg.lsb; // creates bit mask

    uint16_t regValue;
    FPGAReadRegister(reg.address, &regValue); // read bit content

    uint16_t code = std::clamp(threshold * 65535, 0.0, 65535.0);

    regValue &= ~mask;
    regValue |= (code << reg.lsb) & mask;

    FPGAWriteRegister(reg.address, regValue);
}

void EqualiserTest::OnGainChanged(wxCommandEvent& event)
{
    double threshold;
    wxSpinCtrlDouble* p = reinterpret_cast<wxSpinCtrlDouble*>(event.GetEventObject());
    threshold = p->GetValue();

    Register reg = controlsPtr2Registers[event.GetEventObject()];
    unsigned short mask = (~(~0 << (reg.msb - reg.lsb + 1))) << reg.lsb; // creates bit mask

    uint16_t regValue;
    FPGAReadRegister(reg.address, &regValue); // read bit content

    uint16_t code = std::clamp(threshold * 8192, 0.0, 32768.0); // range (-4.0, 4.0)

    regValue &= ~mask;
    regValue |= (code << reg.lsb) & mask;

    FPGAWriteRegister(reg.address, regValue);
}

void EqualiserTest::RegisterParameterChangeHandler(wxCommandEvent& event)
{
    if (controlsPtr2Registers.find(event.GetEventObject()) == controlsPtr2Registers.end())
        return; // control not found in the table

    Register reg = controlsPtr2Registers[event.GetEventObject()];
    int mac = (reg.address != 0x17) && (rbChannelB->GetValue()) ? 0x2 : 0x1;
    if (FPGAWriteRegister(0xFFFF, mac) != OpStatus::Success)
    {
        wxMessageBox(_("Write FPGA register failed"), _("Error"), wxICON_ERROR | wxOK);
        return;
    }

    unsigned short mask = (~(~0u << (reg.msb - reg.lsb + 1))) << reg.lsb; // creates bit mask

    uint16_t regValue;
    FPGAReadRegister(reg.address, &regValue);

    regValue &= ~mask;
    int evtVal = event.GetInt();
    regValue |= (evtVal << reg.lsb) & mask;

    FPGAWriteRegister(reg.address, regValue);
}

void EqualiserTest::onbtnFIRCoef()
{
    Register reg, reg2;
    lms7002_dlgGFIR_Coefficients* dlg = new lms7002_dlgGFIR_Coefficients(this, wxNewId(), wxEmptyString);
    std::vector<double> coefficients;
    const int maxCoefCount = 32;

    // FIR registers
    uint16_t maddressf0 = 0x000E; // 14x64 = 896;
    uint16_t maddressf1 = 0x000F; // 15x64 = 960;

    uint16_t regValue = 0;
    uint16_t Filt_N = 0;
    uint16_t NN = 0;
    uint16_t addr = 0;
    uint16_t data = 0;
    uint16_t msb = 0;
    uint16_t lsb = 0;
    uint16_t i = 0;

    Filt_N = 32;
    NN = 15;

    coefficients.resize(Filt_N, 0);
    reg = controlsPtr2Registers[chkSLEEP_FIR];
    reg2 = controlsPtr2Registers[chkODD_FIR];

    for (i = 0; i < Filt_N; i++) // maxCoefCount
        coefficients[i] = 0.0;

    // read coeffs
    msb = lsb = i = 0;
    while (i <= ((Filt_N)-1))
    {
        addr = (maddressf0 << 6) + (msb << 4) + lsb;
        FPGAReadRegister(addr, &regValue);
        coefficients[i] = static_cast<int16_t>(regValue);
        if (lsb >= NN) // 15
        {
            lsb = 0;
            msb++;
        }
        else
            lsb++;
        i++;
    }
    dlg->SetCoefficients(coefficients);

    if (dlg->ShowModal() == wxID_OK)
    {
        coefficients = dlg->GetCoefficients();

        SetRegValue(reg, 1); // sleep <= '1';

        for (i = 0; i < maxCoefCount; i++)
        {
            addr = (maddressf0 << 6) + i;
            FPGAWriteRegister(addr, 0);
            addr = (maddressf1 << 6) + i;
            FPGAWriteRegister(addr, 0);
        }

        msb = lsb = i = 0;
        while (i <= ((Filt_N)-1))
        {
            addr = (maddressf0 << 6) + (msb << 4) + lsb;
            data = static_cast<uint16_t>(coefficients[i]);
            FPGAWriteRegister(addr, data);

            addr = (maddressf1 << 6) + (msb << 4) + lsb;
            data = static_cast<uint16_t>(coefficients[i]);
            FPGAWriteRegister(addr, data);
            if (lsb >= NN) // 15
            {
                lsb = 0;
                msb++;
            }
            else
                lsb++;
            i++;
        }

        if ((Filt_N % 2) == 1)
            SetRegValue(reg2, 1); // odd
        else
            SetRegValue(reg2, 0); // even

        SetRegValue(reg, 0); // sleep <= '0';
    }
    dlg->Destroy();
}

EqualiserTest::~EqualiserTest()
{
    // mPanelStreamPLL->Disconnect(wxEVT_BUTTON, btnConfigurePLL->GetId(), wxCommandEventHandler(EqualiserTest::OnConfigurePLL), 0, this);
}

void EqualiserTest::OnbtnUpdateAll(wxCommandEvent& event)
{
    double tempd = 0.0;
    map<wxObject*, CSRegister>::iterator iter;
    wxClassInfo* spinctr = wxClassInfo::FindClass("wxSpinCtrl");
    wxClassInfo* checkboxctr = wxClassInfo::FindClass("wxCheckBox");
    wxClassInfo* choicectr = wxClassInfo::FindClass("wxChoice");

    if (FPGAWriteRegister(0xFFFF, rbChannelB->GetValue() ? 0x2 : 0x1) != OpStatus::Success)
    {
        wxMessageBox(_("Write FPGA register failed"), _("Error"), wxICON_ERROR | wxOK);
        return;
    }
    for (iter = controlsPtr2Registers.begin(); iter != controlsPtr2Registers.end(); ++iter)
    {

        const CSRegister& reg = iter->second;
        uint16_t value;
        unsigned short mask = (~(~0 << (reg.msb - reg.lsb + 1))) << reg.lsb; // creates bit mask

        FPGAReadRegister(reg.address, &value);
        // value <<= 15 - reg.msb;
        // value >>= reg.lsb + (15 - reg.msb);
        value = value & mask;
        value = value >> reg.lsb;

        unsigned short mask2 = (~(~0 << (reg.msb - reg.lsb + 1)));
        short value2 = 0;
        if ((reg.twoComplement == 1) && (value > (mask2 >> 1)))
            value2 = value - (mask2 + 1);
        else
            value2 = value;

        if (iter->first->IsKindOf(spinctr))
            reinterpret_cast<wxSpinCtrl*>(iter->first)->SetValue(value2);
        else if (iter->first->IsKindOf(checkboxctr))
            reinterpret_cast<wxCheckBox*>(iter->first)->SetValue(value);
        else if (iter->first->IsKindOf(choicectr))
            reinterpret_cast<wxComboBox*>(iter->first)->SetSelection(value2);
        else if (iter->first == thresholdSpin)
        {
            tempd = value / 65535.0;
            if (tempd > 1.0)
                tempd = 1.0;
            else if (tempd < 0.0)
                tempd = 0.0;
            reinterpret_cast<wxSpinCtrlDouble*>(iter->first)->SetValue(tempd);
        }
        else if (iter->first == thresholdGain)
        {
            tempd = value / (65535.0 / 8.0);
            if (tempd > 2.0)
                tempd = 2.0;
            else if (tempd < 0.0)
                tempd = 0.0;
            reinterpret_cast<wxSpinCtrlDouble*>(iter->first)->SetValue(tempd);
        }
    }

    double refClk_MHz, ncoFreq_MHz;
    refClk_MHz = 2.0 * 122.88;
    int32_t fcw = 0;
    vector<uint32_t> addrs = { 0x008E, 0x008F }; // OK

    for (size_t i = 0; i < addrs.size(); i++)
    {
        uint16_t value;
        if (FPGAReadRegister(addrs[i], &value) != OpStatus::Success)
        {
            wxMessageBox(_("Read FPGA register failed"), _("Error"), wxICON_ERROR | wxOK);
            return;
        }
        fcw <<= 16;
        fcw += value;
    }
    ncoFreq_MHz = fcw * refClk_MHz / 4294967296;
    txtNcoFreq->SetValue(wxString::Format("%1.3f", ncoFreq_MHz));
}

static uint32_t nco_freq_to_fcw(uint64_t freq, uint64_t refclk)
{
    uint64_t fp_number = static_cast<uint64_t>(freq) << 32;
    return (fp_number / refclk) & 0xFFFFFFFF;
}

void EqualiserTest::OnNcoFrequencyChanged(wxCommandEvent& event)
{
    FPGAWriteRegister(0xFFFF, rbChannelB->GetValue() ? 0x2 : 0x1);
    double refClk_MHz, ncoFreq_MHz;
    refClk_MHz = 2.0 * 122.88; // TODO: Get actual ref clk
    txtNcoFreq->GetValue().ToDouble(&ncoFreq_MHz);
    uint32_t fcw = nco_freq_to_fcw(ncoFreq_MHz * 1e6, refClk_MHz * 1e6);

    vector<uint32_t> addrs = { 0x008E, 0x008F };
    vector<uint32_t> values = { (fcw >> 16) & 0xFFFF, fcw & 0xFFFF };

    for (size_t i = 0; i < values.size(); i++)
    {
        if (FPGAWriteRegister(addrs[i], values[i]) != OpStatus::Success)
        {
            wxMessageBox(_("Write FPGA register failed"), _("Error"), wxICON_ERROR | wxOK);
            return;
        }
    }
}

void EqualiserTest::OnReadAll(wxCommandEvent& event)
{
    OnbtnUpdateAll(event);
}

void EqualiserTest::OnWriteAll(wxCommandEvent& event)
{
    OnNcoFrequencyChanged(event);
}

void EqualiserTest::OnSwitchToChannelA(wxCommandEvent& event)
{
    if (FPGAWriteRegister(0xFFFF, 0x1) != OpStatus::Success)
    {
        wxMessageBox(_("Write FPGA register failed"), _("Error"), wxICON_ERROR | wxOK);
        return;
    }
    ChA = true;
    wxCommandEvent evt;
    OnbtnUpdateAll(evt);
}

void EqualiserTest::OnSwitchToChannelB(wxCommandEvent& event)
{
    if (FPGAWriteRegister(0xFFFF, 0x2) != OpStatus::Success)
    {
        wxMessageBox(_("Write FPGA register failed"), _("Error"), wxICON_ERROR | wxOK);
        return;
    }
    ChA = false;
    wxCommandEvent evt;
    OnbtnUpdateAll(evt);
}

void EqualiserTest::SaveEqualizerSettings(wxCommandEvent& event)
{
    wxFileDialog dlg(
        this, _("Save LimeSDR-5GRadio config file"), "", "", "Project-File (*.ini2)|*.ini2", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() == wxID_CANCEL)
        return;

    wxString m_sConfigFilename = dlg.GetPath();
    const char* config_filename2 = m_sConfigFilename.mb_str();
    std::string config_filename3 = config_filename2;

    equalizer->FDQIE_SaveEqualiser(15, config_filename3);

    wxCommandEvent evt;
    OnbtnUpdateAll(evt);
}

void EqualiserTest::LoadEqualizerSettings(wxCommandEvent& event)
{
    wxFileDialog dlg(
        this, _("Open LimeSDR-5GRadio config file"), "", "", "Project-File (*.ini2)|*.ini2", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() == wxID_CANCEL)
        return;

    wxString m_sConfigFilename = dlg.GetPath();
    const char* config_filename2 = m_sConfigFilename.mb_str();
    std::string config_filename3 = config_filename2;

    equalizer->FDQIE_LoadEqualiser(config_filename3);

    wxCommandEvent evt;
    OnbtnUpdateAll(evt);
}

void EqualiserTest::ResetEqualizer(wxCommandEvent& event)
{
    int kk = 1;
    if (ChA == true)
        kk = 1;
    else
        kk = 2;

    equalizer->FDQIE_ResetEqualiser(kk);

    wxCommandEvent evt;
    OnbtnUpdateAll(evt);
}
