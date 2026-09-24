#include "LA9310_wxgui.h"
#include "limesuiteng/Logger.h"

#include "SOC_GUIFactory.h"

#include "chips/LA9310/LA9310.h"
#include "chips/LA9310/PHYTimer.h"
#include "chips/LA9310/firmware/IQStreamer.h"
#include "chips/LA9310/firmware/LA9310_FW_Impl.h"

#include "widgets/DCCorrectorPanel.h"
#include "widgets/QECPanel.h"

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

    chkTxToneGenerator = new wxCheckBox(this, wxID_ANY, "TxTone");
    chkTxToneGenerator->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onTxToneGeneratorClick), nullptr, this);
    spinTxToneBin = new wxSpinCtrl(this, wxID_ANY, "64", wxDefaultPosition, wxDefaultSize, 0, 0, 32767, 8192);

    fgSizer246->Add(chkTxToneGenerator);
    fgSizer246->Add(spinTxToneBin);

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

    SetSizer(fgSizer246);
    Layout();
    fgSizer246->Fit(this);
}

LA9310_wxgui::~LA9310_wxgui()
{
    // Disconnect Events
    // m_PrimaryFreq->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(LA9310_wxgui::OnChange), nullptr, this);
}

bool LA9310_wxgui::Initialize(lime::LA9310_IQStreamer* soc)
{
    if (!soc)
        return false;

    iqstreamer = soc;
    // rxdcpanel->Initialize(soc->GetRxDCCorrector(0));
    // txdcpanel->Initialize(soc->GetTxDCCorrector(0));
    // rxqecpanel->Initialize(soc->GetRxQEC(0));
    // txqecpanel->Initialize(soc->GetTxQEC(0));
    for (auto ctrl : timer_map)
    {
        ctrl.first->SetValue(iqstreamer->fw->phytimer.GetTimerControl(ctrl.second).GetTriggerValue());
    }
    return true;
}

bool LA9310_wxgui::Initialize(void* soc)
{
    return Initialize(reinterpret_cast<lime::LA9310_IQStreamer*>(soc));
}

void LA9310_wxgui::UpdateGUI()
{
}

void LA9310_wxgui::onTxToneGeneratorClick(wxCommandEvent& event)
{
    if (!iqstreamer)
        return;

    OpStatus status =
        OpStatus::NotImplemented; //iqstreamer->GenerateTxTone(chkTxToneGenerator->GetValue(), spinTxToneBin->GetValue());
    if (status != OpStatus::Success)
        printf("Failed to set Tx tone\n");
}

void LA9310_wxgui::onPhytimer(wxCommandEvent& event)
{
    const uint16_t timer_id = timer_map.at(reinterpret_cast<wxCheckBox*>(event.GetEventObject()));
    iqstreamer->fw->phytimer.GetTimerControl(timer_id).TriggerDirectly(
        event.IsChecked() ? PHYTimerControl::TriggerLogic::ForceOne : PHYTimerControl::TriggerLogic::ForceZero);
}
