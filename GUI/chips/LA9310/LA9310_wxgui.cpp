#include "LA9310_wxgui.h"
#include "limesuiteng/Logger.h"

#include "SOC_GUIFactory.h"

#include "chips/LA9310/LA9310.h"
#include "chips/LA9310/PHYTimer.h"

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

    chkDAC_IQ = new wxCheckBox(this, wxID_ANY, "DAC_IQ");
    chkDAC_IQ->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onPhytimer), nullptr, this);
    fgSizer246->Add(chkDAC_IQ);

    chkTxEn = new wxCheckBox(this, wxID_ANY, "TxEnable");
    chkTxEn->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onTxEnable), nullptr, this);
    fgSizer246->Add(chkTxEn);

    chkAXIQ = new wxCheckBox(this, wxID_ANY, "chkAXIQ");
    chkAXIQ->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::onAXIQEnable), nullptr, this);
    fgSizer246->Add(chkAXIQ);

    btnTransmit = new wxButton(this, wxID_ANY, wxT("Send data"), wxDefaultPosition, wxDefaultSize, 0);
    btnTransmit->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(LA9310_wxgui::onTransmit), nullptr, this);
    fgSizer246->Add(btnTransmit);

    btnRst = new wxButton(this, wxID_ANY, wxT("btnRst"), wxDefaultPosition, wxDefaultSize, 0);
    btnRst->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(LA9310_wxgui::onRst), nullptr, this);
    fgSizer246->Add(btnRst);

    btnClear = new wxButton(this, wxID_ANY, wxT("Clear PRod"), wxDefaultPosition, wxDefaultSize, 0);
    btnClear->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(LA9310_wxgui::onClear), nullptr, this);
    fgSizer246->Add(btnClear);

    SetSizer(fgSizer246);
    Layout();
    fgSizer246->Fit(this);
}

LA9310_wxgui::~LA9310_wxgui()
{
    // Disconnect Events
    // m_PrimaryFreq->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(LA9310_wxgui::OnChange), nullptr, this);
}

bool LA9310_wxgui::Initialize(lime::LA9310* soc)
{
    if (!soc)
        return false;

    la9310 = soc;
    rxdcpanel->Initialize(soc->vspa.GetRxDCCorrector());
    txdcpanel->Initialize(soc->vspa.GetTxDCCorrector());
    rxqecpanel->Initialize(soc->vspa.GetRxQEC());
    txqecpanel->Initialize(soc->vspa.GetTxQEC());
    return true;
}

bool LA9310_wxgui::Initialize(void* soc)
{
    return Initialize(reinterpret_cast<lime::LA9310*>(soc));
}

void LA9310_wxgui::UpdateGUI()
{
}

void LA9310_wxgui::onTxToneGeneratorClick(wxCommandEvent& event)
{
    if (!la9310)
        return;

    OpStatus status = la9310->vspa.GenerateTxTone(chkTxToneGenerator->GetValue(), spinTxToneBin->GetValue());
    if (status != OpStatus::Success)
        printf("Failed to set Tx tone\n");
}

void LA9310_wxgui::onPhytimer(wxCommandEvent& event)
{
    la9310->phytimer.GetTimerControl(11).TriggerDirectly(
        event.IsChecked() ? PHYTimerControl::TriggerLogic::ForceOne : PHYTimerControl::TriggerLogic::ForceZero);
}

void LA9310_wxgui::onTxEnable(wxCommandEvent& event)
{
    if (event.IsChecked())
    {
        printf("TxON\n");
        la9310->vspa.TxEnable(true, true);
    }
    else
    {
        printf("TxOFF\n");
        la9310->vspa.TxEnable(false);
    }
}

void LA9310_wxgui::onAXIQEnable(wxCommandEvent& event)
{
    uint64_t cmd = 0x12lu << 56;
    if (event.IsChecked())
    {
        printf("AXIQ ON\n");
        // la9310->vspa.mailbox->Message(0, 0, cmd | 1);
    }
    else
    {
        printf("AXIQ OFF\n");
        // la9310->vspa.mailbox->Message(0, 0, cmd);
    }
}

void LA9310_wxgui::onRst(wxCommandEvent& event)
{
    uint64_t cmd = 0x13lu << 56;
    printf("PTR RST\n");
    // la9310->vspa.mailbox->Message(0, 0, cmd | 1);
}

void LA9310_wxgui::onTransmit(wxCommandEvent& event)
{
    uint32_t data[512];
    printf("Send 512B\n");
    la9310->vspa.Transmit(data, sizeof(data), 0);
}

void LA9310_wxgui::onClear(wxCommandEvent& event)
{
    printf("clear\n");
    la9310->vspa.ClearStats();
}