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

void OversamplerPanel::Initialize(lime::IOversampler* dev)
{
    if (!deviceOwner || deviceOwner.get() != dev)
        deviceOwner.reset();
    device = dev;
    Enable(device != nullptr);

    cmbOversample->Clear();
    // offer power of 2 ratios, supported by all devices; other ratios are rounded by the device
    const uint32_t maxOversample = device ? device->GetMaxOversample() : 1;
    for (uint32_t ratio = 1; ratio <= maxOversample && ratio != 0; ratio <<= 1)
        cmbOversample->Append(ratio == 1 ? wxString(wxT("1x (bypass)")) : wxString::Format(wxT("%ux"), ratio));
    UpdateGUI();
}

void OversamplerPanel::Initialize(std::shared_ptr<lime::IOversampler> dev)
{
    deviceOwner = dev;
    Initialize(dev.get());
}

void OversamplerPanel::UpdateGUI()
{
    if (!device)
        return;
    const uint32_t ratio = device->GetOversample();
    int selection = wxNOT_FOUND;
    for (unsigned i = 0; i < cmbOversample->GetCount(); ++i)
    {
        if ((1u << i) == ratio)
        {
            selection = i;
            break;
        }
    }
    cmbOversample->SetSelection(selection);
}

void OversamplerPanel::OnSelect(wxCommandEvent& event)
{
    if (!device || cmbOversample->GetSelection() < 0)
        return;
    if (device->SetOversample(1u << cmbOversample->GetSelection()) != OpStatus::Success)
    {
        wxMessageBox(wxT("Set oversample failed."), _("Error"));
        UpdateGUI();
    }
}
