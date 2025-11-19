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

    wxCheckBox* chkTxTone = new wxCheckBox(this, wxID_ANY, wxT("Enable Tx Tone"));
    chkTxTone->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::TxToneToggle), nullptr, this);
    fgSizer246->Add(chkTxTone);

    t11Trigger = new wxCheckBox(this, wxID_ANY, wxT("PhyTimer T11"));
    wxButton* t11button = new wxButton(this, wxID_ANY, "Trigger");
    t11button->Connect(wxEVT_BUTTON, wxCommandEventHandler(LA9310_wxgui::ToggleT11), nullptr, this);
    fgSizer246->Add(t11Trigger);
    fgSizer246->Add(t11button);

    wxFlexGridSizer* txSizer = new wxFlexGridSizer(0, 2, 0, 0);
    txEnable = new wxCheckBox(this, wxID_ANY, wxT("TxEnable"));
    txSizer->Add(txEnable);
    wxButton* txEnableButton = new wxButton(this, wxID_ANY, "Start/Stop");
    txEnableButton->Connect(wxEVT_BUTTON, wxCommandEventHandler(LA9310_wxgui::ToggleTx), nullptr, this);
    txSizer->Add(txEnableButton);

    txtTxValue = new wxTextCtrl(this, wxID_ANY, wxT("7FFF"));
    txSizer->Add(txtTxValue);
    wxButton* btnSendData = new wxButton(this, wxID_ANY, "Transmit");
    btnSendData->Connect(wxEVT_BUTTON, wxCommandEventHandler(LA9310_wxgui::SendData), nullptr, this);
    txSizer->Add(btnSendData);

    wxButton* btnClearStats = new wxButton(this, wxID_ANY, "Clear stats");
    btnClearStats->Connect(wxEVT_BUTTON, wxCommandEventHandler(LA9310_wxgui::ClearStats), nullptr, this);
    txSizer->Add(btnClearStats);

    wxButton* btnAbort = new wxButton(this, wxID_ANY, "Tx abort");
    btnAbort->Connect(wxEVT_BUTTON, wxCommandEventHandler(LA9310_wxgui::TxAbort), nullptr, this);
    txSizer->Add(btnAbort);

    wxStaticText* addrText = new wxStaticText(this, wxID_ANY, wxT("Address(Hex):"));
    txSizer->Add(addrText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    txtVSPAAddr = new wxTextCtrl(this, wxID_ANY, wxT("C0"));
    txSizer->Add(txtVSPAAddr, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);
    txtVSPAValue = new wxTextCtrl(this, wxID_ANY, wxT("0"));
    txSizer->Add(txtVSPAValue, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);
    wxButton* btnWriteVSPA = new wxButton(this, wxID_ANY, "Writereg");
    txSizer->Add(btnWriteVSPA, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);
    btnWriteVSPA->Connect(wxEVT_BUTTON, wxCommandEventHandler(LA9310_wxgui::WriteVSPAAddr), nullptr, this);

    wxButton* btnResetDMA = new wxButton(this, wxID_ANY, "btnResetTxCounter");
    btnResetDMA->Connect(wxEVT_BUTTON, wxCommandEventHandler(LA9310_wxgui::ResetTxStats), nullptr, this);
    txSizer->Add(btnResetDMA, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxButton* btnPtrRst = new wxButton(this, wxID_ANY, "btnPtrRst");
    btnPtrRst->Connect(wxEVT_BUTTON, wxCommandEventHandler(LA9310_wxgui::ResetPtr), nullptr, this);
    txSizer->Add(btnPtrRst, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxButton* btnAxiqEn = new wxButton(this, wxID_ANY, "btnAxiqEn");
    btnAxiqEn->Connect(wxEVT_BUTTON, wxCommandEventHandler(LA9310_wxgui::AxiqEn), nullptr, this);
    txSizer->Add(btnAxiqEn, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);
    chkAxiqEn = new wxCheckBox(this, wxID_ANY, wxT("Enable TxAXIQ"));
    txSizer->Add(chkAxiqEn, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    fgSizer246->Add(txSizer);

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

void LA9310_wxgui::TxToneToggle(wxCommandEvent& event)
{
    bool enable = event.GetInt();
    printf("Tx tone :%i\n", enable);
    la9310->vspa.StartTxTone(enable);
}

void LA9310_wxgui::ToggleT11(wxCommandEvent& event)
{
    bool enable = t11Trigger->GetValue();
    PHYTimerControl timer = la9310->phytimer.GetTimerControl(11);
    timer.TriggerDirectly(enable ? PHYTimerControl::TriggerLogic::ForceOne : PHYTimerControl::TriggerLogic::ForceZero);
}

void LA9310_wxgui::ToggleTx(wxCommandEvent& event)
{
    bool enable = txEnable->GetValue();
    if (enable)
        la9310->vspa.StartTx();
    else
        la9310->vspa.StopTx();
}

void LA9310_wxgui::SendData(wxCommandEvent& event)
{
    // const uint32_t txsize = 2048;
    std::vector<complex16_t> data;
    data.resize(512);

    const wxString strValue = txtTxValue->GetValue();
    long val = 0;
    strValue.ToLong(&val, 16);

    for (size_t i = 0; i < data.size(); ++i)
        data[i] = complex16_t(val, 0);
    la9310->vspa.Transmit(data.data(), data.size() * sizeof(complex16_t), 0);
}

void LA9310_wxgui::ClearStats(wxCommandEvent& event)
{
    la9310->vspa.ClearStats();
}

void LA9310_wxgui::WriteVSPAAddr(wxCommandEvent& event)
{
    const wxString strAddress = txtVSPAAddr->GetValue();
    long addr = 0;
    strAddress.ToLong(&addr, 16);

    const wxString strVal = txtVSPAValue->GetValue();
    long value = 0;
    strVal.ToLong(&value, 16);
    la9310->vspa.WriteVSPA_IPReg(addr, value);
}

void LA9310_wxgui::ResetTxStats(wxCommandEvent& event)
{
    OpStatus status = la9310->vspa.ResetTxStats();
    if (status != OpStatus::Success)
        printf("FAILED TO RESET DMA\n");
}
void LA9310_wxgui::ResetPtr(wxCommandEvent& event)
{
    OpStatus status = la9310->vspa.TxRstPtr();
    if (status != OpStatus::Success)
        printf("FAILED TO RESET DMA\n");
}
void LA9310_wxgui::AxiqEn(wxCommandEvent& event)
{
    OpStatus status = la9310->vspa.TxAxiqEnable(chkAxiqEn->GetValue());
    if (status != OpStatus::Success)
        printf("FAILED TO RESET DMA\n");
}
void LA9310_wxgui::TxAbort(wxCommandEvent& event)
{
    OpStatus status = la9310->vspa.TxAbort();
    if (status != OpStatus::Success)
        printf("FAILED TO RESET DMA\n");
}