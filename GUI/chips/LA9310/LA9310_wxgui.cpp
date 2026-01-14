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
