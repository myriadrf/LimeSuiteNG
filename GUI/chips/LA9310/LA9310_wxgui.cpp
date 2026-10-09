#include "LA9310_wxgui.h"
#include "limesuiteng/Logger.h"

#include "SOC_GUIFactory.h"

#include "chips/LA9310/LA9310.h"
#include "chips/LA9310/PHYTimer.h"
#include "chips/LA9310/firmware/IQStreamer.h"
#include "chips/LA9310/firmware/LA9310_FW_Impl.h"

#include "widgets/DCCorrectorPanel.h"
#include "widgets/QECPanel.h"
#include "widgets/ToneGeneratorPanel.h"

#include <vector>
using namespace lime;

static bool isRegistered = RegisterToFactory<SOC_GUIFactory, &LA9310_wxgui::Create>("LA9310");

ISOCPanel* LA9310_wxgui::Create(wxWindow* parent, wxWindowID id)
{
    return new LA9310_wxgui(parent, id);
}

LA9310_wxgui::LA9310_wxgui(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
{
    wxFlexGridSizer* fgSizer246;
    fgSizer246 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer246->SetFlexibleDirection(wxVERTICAL);
    fgSizer246->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    rxdcpanel = std::make_unique<DCCorrectorsPanel>(this, wxID_ANY, "Rx DC (digital)");
    fgSizer246->Add(rxdcpanel.get());

    txdcpanel = std::make_unique<DCCorrectorsPanel>(this, wxID_ANY, "Tx DC (digital)");
    fgSizer246->Add(txdcpanel.get());

    rxqecpanel = std::make_unique<QECPanel>(this, wxID_ANY, "Rx QEC");
    fgSizer246->Add(rxqecpanel.get());

    txqecpanel = std::make_unique<QECPanel>(this, wxID_ANY, "Tx QEC");
    fgSizer246->Add(txqecpanel.get());

    txtonepanel = std::make_unique<ToneGeneratorPanel>(this, wxID_ANY, "Tx Tone");
    fgSizer246->Add(txtonepanel.get());

    chkDAC_IQ = new wxCheckBox(this, wxID_ANY, "DAC_IQ (tx_dma_allowed)");
    chkDAC_IQ->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onPhytimer), nullptr, this);
    fgSizer246->Add(chkDAC_IQ);
    timer_map[chkDAC_IQ] = 11;

    chkPA_EN = new wxCheckBox(this, wxID_ANY, "PA_EN/GPIO_12");
    chkPA_EN->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onPhytimer), nullptr, this);
    fgSizer246->Add(chkPA_EN);
    timer_map[chkPA_EN] = 20;

    chkLNA1_EN = new wxCheckBox(this, wxID_ANY, "LNA1_EN/GPIO_11");
    chkLNA1_EN->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onPhytimer), nullptr, this);
    fgSizer246->Add(chkLNA1_EN);
    timer_map[chkLNA1_EN] = 19;

    chkTXRX1 = new wxCheckBox(this, wxID_ANY, "TXRX1/GPIO_08 (Tx RF switch)");
    chkTXRX1->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onPhytimer), nullptr, this);
    fgSizer246->Add(chkTXRX1);
    timer_map[chkTXRX1] = 15;

    chkTXRX0 = new wxCheckBox(this, wxID_ANY, "TXRX0/GPIO_07");
    chkTXRX0->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onPhytimer), nullptr, this);
    fgSizer246->Add(chkTXRX0);
    timer_map[chkTXRX0] = 16;

    chkAXIQLoopback = new wxCheckBox(this, wxID_ANY, "AXIQ loopback");
    chkAXIQLoopback->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onLoopback), nullptr, this);
    fgSizer246->Add(chkAXIQLoopback);

    wxFlexGridSizer* tddsizer;
    tddsizer = new wxFlexGridSizer(0, 5, 0, 0);
    tddsizer->SetFlexibleDirection(wxVERTICAL);
    tddsizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxArrayString comparator_values;
    comparator_values.Add(wxT("NoChange"));
    comparator_values.Add(wxT("0"));
    comparator_values.Add(wxT("1"));
    comparator_values.Add(wxT("Toggle"));

    tddsizer->Add(new wxStaticText(this, wxID_ANY, wxT("RFSW"), wxDefaultPosition, wxDefaultSize, 0),
        1,
        wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL,
        0);
    tddgui.spinRFON = new wxSpinCtrl(
        this, wxNewId(), _("0"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, -256, 255, 0);
    tddgui.spinRFON->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(LA9310_wxgui::onTDDchange), nullptr, this);
    tddsizer->Add(tddgui.spinRFON, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);

    tddgui.cmbRFON = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, comparator_values);
    tddgui.cmbRFON->SetSelection(0);
    tddgui.cmbRFON->Connect(wxEVT_CHOICE, wxCommandEventHandler(LA9310_wxgui::onTDDchange), nullptr, this);
    tddsizer->Add(tddgui.cmbRFON, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);

    tddgui.spinRFOFF = new wxSpinCtrl(
        this, wxNewId(), _("0"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, -256, 255, 0);
    tddsizer->Add(tddgui.spinRFOFF, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    tddgui.spinRFOFF->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(LA9310_wxgui::onTDDchange), nullptr, this);

    tddgui.cmbRFOFF = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, comparator_values);
    tddgui.cmbRFOFF->SetSelection(0);
    tddgui.cmbRFOFF->Connect(wxEVT_CHOICE, wxCommandEventHandler(LA9310_wxgui::onTDDchange), nullptr, this);
    tddsizer->Add(tddgui.cmbRFOFF, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);

    tddsizer->Add(new wxStaticText(this, wxID_ANY, wxT("PA"), wxDefaultPosition, wxDefaultSize, 0),
        1,
        wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL,
        0);
    tddgui.spinPAON = new wxSpinCtrl(
        this, wxNewId(), _("0"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, -256, 255, 0);
    tddsizer->Add(tddgui.spinPAON, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    tddgui.spinPAON->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(LA9310_wxgui::onTDDchange), nullptr, this);

    tddgui.cmbPAON = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, comparator_values);
    tddgui.cmbPAON->SetSelection(2);
    tddgui.cmbPAON->Connect(wxEVT_CHOICE, wxCommandEventHandler(LA9310_wxgui::onTDDchange), nullptr, this);
    tddsizer->Add(tddgui.cmbPAON, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);

    tddgui.spinPAOFF = new wxSpinCtrl(
        this, wxNewId(), _("0"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, -256, 255, 0);
    tddsizer->Add(tddgui.spinPAOFF, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    tddgui.spinPAOFF->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(LA9310_wxgui::onTDDchange), nullptr, this);

    tddgui.cmbPAOFF = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, comparator_values);
    tddgui.cmbPAOFF->SetSelection(1);
    tddgui.cmbPAOFF->Connect(wxEVT_CHOICE, wxCommandEventHandler(LA9310_wxgui::onTDDchange), nullptr, this);
    tddsizer->Add(tddgui.cmbPAOFF, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);

    tddgui.spinDAC = new wxSpinCtrl(
        this, wxNewId(), _("0"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, -32768, 32767, 0);
    tddsizer->Add(new wxStaticText(this, wxID_ANY, wxT("DAC"), wxDefaultPosition, wxDefaultSize, 0),
        1,
        wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL,
        0);
    tddsizer->Add(tddgui.spinDAC, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    tddgui.spinDAC->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(LA9310_wxgui::onTDDchange), nullptr, this);

    fgSizer246->Add(tddsizer);

    SetSizer(fgSizer246);
    Layout();
    fgSizer246->Fit(this);
}

LA9310_wxgui::~LA9310_wxgui()
{
}

bool LA9310_wxgui::Initialize(lime::LA9310_IQStreamer* soc)
{
    if (!soc)
        return false;

    iqstreamer = soc;
    rxdcpanel->Initialize(soc->GetRxDCCorrector(0));
    txdcpanel->Initialize(soc->GetTxDCCorrector(0));
    for (auto ctrl : timer_map)
    {
        ctrl.first->SetValue(iqstreamer->fw->phytimer.GetTimerControl(ctrl.second).GetTriggerValue());
    }

    rxqecpanel->Initialize(soc->GetRxQEC(0));
    txqecpanel->Initialize(soc->GetTxQEC(0));
    txtonepanel->Initialize(soc->GetTxToneGenerator(0));
    return true;
}

bool LA9310_wxgui::Initialize(void* soc)
{
    return Initialize(reinterpret_cast<lime::LA9310_IQStreamer*>(soc));
}

void LA9310_wxgui::UpdateGUI()
{
}

void LA9310_wxgui::onPhytimer(wxCommandEvent& event)
{
    const uint16_t timer_id = timer_map.at(reinterpret_cast<wxCheckBox*>(event.GetEventObject()));
    iqstreamer->fw->phytimer.GetTimerControl(timer_id).TriggerDirectly(
        event.IsChecked() ? PHYTimerControl::TriggerLogic::ForceOne : PHYTimerControl::TriggerLogic::ForceZero);
}

void LA9310_wxgui::onLoopback(wxCommandEvent& event)
{
    iqstreamer->fw->DigitalLoopback(event.IsChecked());
}

void LA9310_wxgui::onTDDchange(wxSpinEvent& event)
{
    if (!iqstreamer->tdd_control)
        return;

    iqstreamer->tdd_control->dac_allowed = tddgui.spinDAC->GetValue();
    iqstreamer->tdd_control->rf_sw_on = tddgui.spinRFON->GetValue();
    iqstreamer->tdd_control->rf_sw_off = tddgui.spinRFOFF->GetValue();
    iqstreamer->tdd_control->pa_on = tddgui.spinPAON->GetValue();
    iqstreamer->tdd_control->pa_off = tddgui.spinPAOFF->GetValue();
}

void LA9310_wxgui::onTDDchange(wxCommandEvent& event)
{
    if (!iqstreamer->tdd_control)
        return;

    iqstreamer->tdd_control->rf_sw_on_comparator_out = tddgui.cmbRFON->GetSelection();
    iqstreamer->tdd_control->rf_sw_off_comparator_out = tddgui.cmbRFOFF->GetSelection();
    iqstreamer->tdd_control->pa_on_comparator_out = tddgui.cmbPAON->GetSelection();
    iqstreamer->tdd_control->pa_off_comparator_out = tddgui.cmbPAOFF->GetSelection();
}