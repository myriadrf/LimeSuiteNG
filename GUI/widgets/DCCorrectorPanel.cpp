#include "DCCorrectorPanel.h"

#include "interface/IDCCorrector.h"

#include "numericSlider.h"
#include <wx/spinctrl.h>

using namespace lime;

DCCorrectorsPanel::DCCorrectorsPanel(
    wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
{
    constexpr int margins = 0;
    const int textFlags = wxALIGN_LEFT | wxLEFT | wxALIGN_CENTER_VERTICAL;

    Create(parent, id, pos, size, style);

    wxStaticBoxSizer* sbSizerDC = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, title), wxVERTICAL);

    wxFlexGridSizer* fgSizer45 = new wxFlexGridSizer(0, 2, 0, margins);
    fgSizer45->AddGrowableCol(1);
    fgSizer45->SetFlexibleDirection(wxBOTH);
    fgSizer45->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer45->Add(new wxStaticText(sbSizerDC->GetStaticBox(), wxID_ANY, wxT("I:")), 0, textFlags, margins);
    Icontrol = new NumericSlider(
        sbSizerDC->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -32768, 32767, 0);
    fgSizer45->Add(Icontrol, 0, wxEXPAND | wxRIGHT, margins);
    Icontrol->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(DCCorrectorsPanel::WriteValues), nullptr, this);

    fgSizer45->Add(new wxStaticText(sbSizerDC->GetStaticBox(), wxID_ANY, wxT("Q:")), 0, textFlags, margins);
    Qcontrol = new NumericSlider(
        sbSizerDC->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -32768, 32767, 0);
    fgSizer45->Add(Qcontrol, 0, wxEXPAND | wxRIGHT, margins);
    Qcontrol->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(DCCorrectorsPanel::WriteValues), nullptr, this);

    sbSizerDC->Add(fgSizer45, 0, wxEXPAND, 0);

    SetSizer(sbSizerDC);
    Layout();
    sbSizerDC->Fit(this);
}

void DCCorrectorsPanel::Initialize(std::shared_ptr<lime::IDCCorrector> dev)
{
    device = dev;
}

DCCorrectorsPanel::~DCCorrectorsPanel()
{
}

void DCCorrectorsPanel::WriteValues(wxSpinEvent& event)
{
    wxCommandEvent evt;
    WriteValues(evt);
}

void DCCorrectorsPanel::WriteValues(wxCommandEvent& event)
{
    if (!device)
        return;

    complex16_t offset(Icontrol->GetValue(), Qcontrol->GetValue());
    device->SetDCOffset(offset);
}