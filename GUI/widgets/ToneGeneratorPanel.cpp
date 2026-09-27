#include "ToneGeneratorPanel.h"

#include "interface/IToneGenerator.h"

#include "numericSliderDouble.h"
#include "numericSlider.h"
#include <wx/spinctrl.h>

using namespace lime;

ToneGeneratorPanel::ToneGeneratorPanel(
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

    chkEnable = new wxCheckBox(sbSizerDC->GetStaticBox(), wxNewId(), wxT("Enable"));
    chkEnable->Connect(wxEVT_CHECKBOX, wxCommandEventHandler(ToneGeneratorPanel::EnableChanged), nullptr, this);
    fgSizer45->Add(chkEnable, 0, wxEXPAND | wxRIGHT, margins);

    fgSizer45->Add(new wxStaticText(sbSizerDC->GetStaticBox(), wxID_ANY, wxT("amplitude:")), 0, textFlags, margins);
    amplitude = new NumericSliderDouble(
        sbSizerDC->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 1.0, 0);
    fgSizer45->Add(amplitude, 0, wxEXPAND | wxRIGHT, margins);
    amplitude->Connect(
        wxEVT_COMMAND_SPINCTRLDOUBLE_UPDATED, wxSpinDoubleEventHandler(ToneGeneratorPanel::ValuesChanged), nullptr, this);

    fgSizer45->Add(new wxStaticText(sbSizerDC->GetStaticBox(), wxID_ANY, wxT("fftbin:")), 0, textFlags, margins);
    fftBin = new NumericSlider(
        sbSizerDC->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 65535, 0);
    fgSizer45->Add(fftBin, 0, wxEXPAND | wxRIGHT, margins);
    fftBin->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinDoubleEventHandler(ToneGeneratorPanel::ValuesChanged), nullptr, this);

    sbSizerDC->Add(fgSizer45, 0, wxEXPAND, 0);

    SetSizer(sbSizerDC);
    Layout();
    sbSizerDC->Fit(this);
}

void ToneGeneratorPanel::Initialize(std::shared_ptr<lime::IToneGenerator> dev)
{
    device = dev;
    if (!dev)
        return;

    amplitude->SetRange(0.0, 1.0, 0.1);
    amplitude->SetValue(0.9);
    fftBin->SetRange(0, 65535, 1);
    fftBin->SetValue(8192);
}

ToneGeneratorPanel::~ToneGeneratorPanel()
{
}

void ToneGeneratorPanel::ValuesChanged(wxSpinDoubleEvent& event)
{
    if (!device)
        return;
    printf("amp: %f b:%u\n", amplitude->GetValue(), fftBin->GetValue());
    OpStatus status = device->SetParameters(amplitude->GetValue(), fftBin->GetValue());
    if (status != OpStatus::Success)
        wxMessageBox("ToneGenerator set parameters failed.", _("Error"));
}

void ToneGeneratorPanel::EnableChanged(wxCommandEvent& event)
{
    if (!device)
        return;
    OpStatus status = device->Enabled(chkEnable->IsChecked());
    if (status != OpStatus::Success)
        wxMessageBox("ToneGenerator enable failed.", _("Error"));
}