#include "LA9310_wxgui.h"
#include "limesuiteng/Logger.h"

#include "chips/LA9310/LA9310.h"

#include "widgets/DCCorrectorPanel.h"
#include "widgets/QECPanel.h"

LA9310_wxgui::LA9310_wxgui(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
{
    wxFlexGridSizer* fgSizer246;
    fgSizer246 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer246->SetFlexibleDirection(wxVERTICAL);
    fgSizer246->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    rxdcpanel = std::make_unique<DCCorrectorsPanel>(this, wxID_ANY);
    fgSizer246->Add(rxdcpanel.get());

    txdcpanel = std::make_unique<DCCorrectorsPanel>(this, wxID_ANY);
    fgSizer246->Add(txdcpanel.get());

    rxqecpanel = std::make_unique<QECPanel>(this, wxID_ANY);
    fgSizer246->Add(rxqecpanel.get());

    txqecpanel = std::make_unique<QECPanel>(this, wxID_ANY);
    fgSizer246->Add(txqecpanel.get());

    wxCheckBox* chkTxTone = new wxCheckBox(this, wxID_ANY, wxT("Enable Tx Tone"));
    chkTxTone->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(LA9310_wxgui::TxToneToggle), nullptr, this);
    fgSizer246->Add(chkTxTone);

    SetSizer(fgSizer246);
    Layout();
    fgSizer246->Fit(this);
}

LA9310_wxgui::~LA9310_wxgui()
{
    // Disconnect Events
    // m_PrimaryFreq->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(LA9310_wxgui::OnChange), nullptr, this);
}

void LA9310_wxgui::Initialize(lime::LA9310* soc)
{
    la9310 = soc;
    rxdcpanel->Initialize(soc->vspa.GetRxDCCorrector());
    txdcpanel->Initialize(soc->vspa.GetTxDCCorrector());
    rxqecpanel->Initialize(soc->vspa.GetRxQEC());
    txqecpanel->Initialize(soc->vspa.GetTxQEC());
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