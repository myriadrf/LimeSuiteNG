#include "QECPanel.h"

#include "interface/IQuadratureErrorCorrector.h"

#include "numericSlider.h"
#include <wx/spinctrl.h>

using namespace lime;

QECPanel::QECPanel(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
{
    constexpr int margins = 0;
    const int textFlags = wxALIGN_LEFT | wxLEFT | wxALIGN_CENTER_VERTICAL;

    Create(parent, id, pos, size, style);

    wxStaticBoxSizer* sbSizerDC = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, title), wxVERTICAL);

    wxFlexGridSizer* fgSizer45 = new wxFlexGridSizer(0, 2, 0, margins);
    fgSizer45->AddGrowableCol(1);
    fgSizer45->SetFlexibleDirection(wxBOTH);
    fgSizer45->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer45->Add(new wxStaticText(sbSizerDC->GetStaticBox(), wxID_ANY, wxT("gain:")), 0, textFlags, margins);
    gainImbalance = new NumericSlider(
        sbSizerDC->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -6000, 6000, 0);
    fgSizer45->Add(gainImbalance, 0, wxEXPAND | wxRIGHT, margins);
    gainImbalance->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(QECPanel::WriteValues), nullptr, this);

    fgSizer45->Add(new wxStaticText(sbSizerDC->GetStaticBox(), wxID_ANY, wxT("phase:")), 0, textFlags, margins);
    phaseImbalance = new NumericSlider(
        sbSizerDC->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -45000, 45000, 0);
    fgSizer45->Add(phaseImbalance, 0, wxEXPAND | wxRIGHT, margins);
    phaseImbalance->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(QECPanel::WriteValues), nullptr, this);

    sbSizerDC->Add(fgSizer45, 0, wxEXPAND, 0);

    SetSizer(sbSizerDC);
    Layout();
    sbSizerDC->Fit(this);
}

void QECPanel::Initialize(std::shared_ptr<lime::IQuadratureErrorCorrector> dev)
{
    device = dev;
}

QECPanel::~QECPanel()
{
}

void QECPanel::WriteValues(wxSpinEvent& event)
{
    wxCommandEvent evt;
    WriteValues(evt);
}

void QECPanel::WriteValues(wxCommandEvent& event)
{
    if (!device)
        return;
    device->SetImbalance(gainImbalance->GetValue() / 1000.0, phaseImbalance->GetValue() / 1000.0);
}