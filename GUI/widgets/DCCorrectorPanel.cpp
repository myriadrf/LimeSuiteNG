#include "DCCorrectorPanel.h"

#include "limesuiteng/IDCCorrector.h"

#include <algorithm>
#include <cmath>

#include <wx/checkbox.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/wupdlock.h>

using namespace lime;

static wxCheckBoxState ToCheckBoxState(TriState state)
{
    switch (state)
    {
    case TriState::Enabled:
        return wxCHK_CHECKED;
    case TriState::Disabled:
        return wxCHK_UNCHECKED;
    default:
        return wxCHK_UNDETERMINED;
    }
}

static inline wxString RangeToString(const Range<float>& range)
{
    return wxString::Format(wxT("[%g, %g]"), range.min, range.max);
}

DCCorrectorsPanel::DCCorrectorsPanel(
    wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
{
    const int textFlags = wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL;

    Create(parent, id, pos, size, style);

    wxWindowUpdateLocker noUpdates(this);

    wxStaticBoxSizer* sbSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, title), wxVERTICAL);
    box = sbSizer->GetStaticBox();

    wxFlexGridSizer* grid = new wxFlexGridSizer(0, 2, 2, 5);
    grid->AddGrowableCol(1);

    wxBoxSizer* bSizer = new wxBoxSizer(wxHORIZONTAL);

    chkEnable = new wxCheckBox(box, wxID_ANY, wxT("Enable"), wxDefaultPosition, wxDefaultSize, wxCHK_3STATE);
    chkEnable->Set3StateValue(wxCHK_CHECKED);
    chkEnable->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent&) { WriteEnabled(); });
    bSizer->Add(chkEnable, 0, textFlags, 0);

    lblRange = new wxStaticText(box, wxID_ANY, wxT("[-, -]"));
    bSizer->Add(lblRange, 0, textFlags, 0);

    grid->AddSpacer(0);
    grid->Add(bSizer);
    grid->Add(new wxStaticText(box, wxID_ANY, "I:"), 0, textFlags, 0);
    Icontrol = new wxSpinCtrl(
        box, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, -32768, 32767, 0);
    Icontrol->Bind(wxEVT_SPINCTRL, [this](wxSpinEvent&) { WriteValues(); });
    Icontrol->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent&) { WriteValues(); });
    grid->Add(Icontrol, 0, wxEXPAND, 0);

    grid->Add(new wxStaticText(box, wxID_ANY, "Q:"), 0, textFlags, 0);
    Qcontrol = new wxSpinCtrl(
        box, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, -32768, 32767, 0);
    Qcontrol->Bind(wxEVT_SPINCTRL, [this](wxSpinEvent&) { WriteValues(); });
    Qcontrol->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent&) { WriteValues(); });
    grid->Add(Qcontrol, 0, wxEXPAND, 0);

    sbSizer->Add(grid, 0, wxEXPAND, 0);

    SetSizer(sbSizer);
    Layout();
    sbSizer->Fit(this);
}

DCCorrectorsPanel::~DCCorrectorsPanel()
{
}

void DCCorrectorsPanel::Initialize(std::shared_ptr<lime::IDCCorrector> dev)
{
    Initialize(dev.get());
    owner = std::move(dev);
}

void DCCorrectorsPanel::Initialize(lime::IDCCorrector* dev)
{
    wxWindowUpdateLocker noUpdates(this);

    owner.reset();
    device = dev;
    Enable(device != nullptr);
    if (!device)
    {
        lblRange->SetLabel(wxT("[-, -]"));
        UpdateEnabledControls();
        return;
    }

    const Range<float> range = device->GetRange();
    const int increment = std::max(1, static_cast<int>(std::lround(range.step)));
    for (wxSpinCtrl* control : { Icontrol, Qcontrol })
    {
        control->SetRange(std::lround(range.min), std::lround(range.max));
        control->SetIncrement(increment);
    }
    lblRange->SetLabel(RangeToString(range));

    UpdateGUI();
    GetSizer()->Fit(this);
    Layout();
}

void DCCorrectorsPanel::UpdateGUI()
{
    if (!device)
        return;
    wxWindowUpdateLocker noUpdates(this);

    const complex16_t offset = device->GetDCOffset();
    Icontrol->SetValue(offset.real());
    Qcontrol->SetValue(offset.imag());
    chkEnable->Set3StateValue(ToCheckBoxState(device->IsEnabled()));
    UpdateEnabledControls();
}

void DCCorrectorsPanel::WriteValues()
{
    if (!device)
        return;

    const complex16_t offset(Icontrol->GetValue(), Qcontrol->GetValue());
    if (device->SetDCOffset(offset) != OpStatus::Success)
        wxMessageBox(wxT("Failed to set the DC offset."), _("Error"));
}

void DCCorrectorsPanel::WriteEnabled()
{
    if (!device)
        return;

    const bool enable = chkEnable->Get3StateValue() == wxCHK_CHECKED;
    if (device->Enabled(enable) != OpStatus::Success)
        wxMessageBox(enable ? wxT("Failed to enable the DC corrector.") : wxT("Failed to disable the DC corrector."), _("Error"));
    chkEnable->Set3StateValue(ToCheckBoxState(device->IsEnabled()));
    UpdateEnabledControls();
}

void DCCorrectorsPanel::UpdateEnabledControls()
{
    // undetermined: the offset may still be in use, keep it editable
    const bool enabled = chkEnable->Get3StateValue() != wxCHK_UNCHECKED;
    Icontrol->Enable(enabled);
    Qcontrol->Enable(enabled);
}
