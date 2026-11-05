#include "OversamplerPanel.h"

#include "limesuiteng/IOversampler.h"

#include <wx/choice.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

using namespace lime;

OversamplerPanel::OversamplerPanel(
    wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
    : device(nullptr)
{
    const int textFlags = wxALIGN_LEFT | wxLEFT | wxALIGN_CENTER_VERTICAL;

    Create(parent, id, pos, size, style);

    wxStaticBoxSizer* sbSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, title), wxVERTICAL);
    wxWindow* box = sbSizer->GetStaticBox();

    wxFlexGridSizer* grid = new wxFlexGridSizer(0, 2, 0, 5);
    grid->AddGrowableCol(1);

    grid->Add(new wxStaticText(box, wxID_ANY, wxT("Oversample:")), 0, textFlags, 0);
    cmbOversample = new wxChoice(box, wxID_ANY);
    cmbOversample->Bind(wxEVT_CHOICE, &OversamplerPanel::OnSelect, this);
    grid->Add(cmbOversample, 0, wxEXPAND, 0);

    sbSizer->Add(grid, 0, wxEXPAND, 0);

    SetSizer(sbSizer);
    Layout();
    sbSizer->Fit(this);
}

OversamplerPanel::~OversamplerPanel()
{
}

void OversamplerPanel::Initialize(lime::IOversampler* dev, uint32_t maxOversamplePow2)
{
    if (!deviceOwner || deviceOwner.get() != dev)
        deviceOwner.reset();
    device = dev;
    Enable(device != nullptr);

    cmbOversample->Clear();
    for (uint32_t pow2 = 0; pow2 <= maxOversamplePow2; ++pow2)
        cmbOversample->Append(pow2 == 0 ? wxString(wxT("1x (bypass)")) : wxString::Format(wxT("%ux"), 1u << pow2));
    UpdateGUI();
}

void OversamplerPanel::Initialize(std::shared_ptr<lime::IOversampler> dev, uint32_t maxOversamplePow2)
{
    deviceOwner = dev;
    Initialize(dev.get(), maxOversamplePow2);
}

void OversamplerPanel::UpdateGUI()
{
    if (!device)
        return;
    const uint32_t pow2 = device->GetOversample();
    if (pow2 < cmbOversample->GetCount())
        cmbOversample->SetSelection(pow2);
    else
        cmbOversample->SetSelection(wxNOT_FOUND);
}

void OversamplerPanel::OnSelect(wxCommandEvent& event)
{
    if (!device || cmbOversample->GetSelection() < 0)
        return;
    if (device->SetOversample(cmbOversample->GetSelection()) != OpStatus::Success)
    {
        wxMessageBox(wxT("Set oversample failed."), _("Error"));
        UpdateGUI();
    }
}
