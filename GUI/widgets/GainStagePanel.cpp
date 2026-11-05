#include "GainStagePanel.h"
#include "GainStageFormat.h"

#include "limesuiteng/IGainStage.h"

#include <wx/button.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

using namespace lime;
using namespace GainStageFormat;

GainStagePanel::GainStagePanel(
    wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
    : title(title)
    , device(nullptr)
{
    const int textFlags = wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL;

    Create(parent, id, pos, size, style);

    wxStaticBoxSizer* sbSizer =
        new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, title.empty() ? wxString(wxT("Gain")) : title), wxVERTICAL);
    box = sbSizer->GetStaticBox();

    wxFlexGridSizer* grid = new wxFlexGridSizer(0, 3, 0, 5);
    grid->AddGrowableCol(0);

    value = new wxSpinCtrlDouble(
        box, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, 0, 0, 0, 1);
    value->Bind(wxEVT_SPINCTRLDOUBLE, [this](wxSpinDoubleEvent& event) { OnValueChanged(event); });
    value->Bind(wxEVT_TEXT_ENTER, &GainStagePanel::OnValueChanged, this);
    grid->Add(value, 0, wxEXPAND, 0);

    lblUnits = new wxStaticText(box, wxID_ANY, wxEmptyString);
    grid->Add(lblUnits, 0, textFlags, 0);

    wxButton* btnRefresh = new wxButton(box, wxID_ANY, wxT("Read back"));
    btnRefresh->Bind(wxEVT_BUTTON, &GainStagePanel::OnRefresh, this);
    grid->Add(btnRefresh, 0, wxALIGN_CENTER_VERTICAL, 0);

    sbSizer->Add(grid, 0, wxEXPAND, 0);
    lblRange = new wxStaticText(box, wxID_ANY, wxT("Range: -"));
    sbSizer->Add(lblRange, 0, wxALL, 2);

    SetSizer(sbSizer);
    Layout();
    sbSizer->Fit(this);
}

GainStagePanel::~GainStagePanel()
{
}

void GainStagePanel::Initialize(lime::IGainStage* dev)
{
    device = dev;
    Enable(device != nullptr);
    if (!device)
    {
        lblUnits->SetLabel(wxEmptyString);
        lblRange->SetLabel(wxT("Range: -"));
        return;
    }

    if (title.empty())
        box->SetLabel(wxString::FromUTF8(device->GetName()));

    const Range<double> range = device->GetRange();
    const double increment = GetIncrement(range);
    value->SetRange(range.min, range.max);
    value->SetIncrement(increment);
    value->SetDigits(GetDigits(increment));

    const wxString units = UnitsToString(device->GetUnits());
    lblUnits->SetLabel(units);
    lblRange->SetLabel(wxString::Format(wxT("Range: %g - %g %s"), range.min, range.max, units));

    UpdateGUI();
    GetSizer()->Fit(this);
    Layout();
}

void GainStagePanel::UpdateGUI()
{
    if (!device)
        return;

    double current = 0;
    const bool ok = device->GetValue(current) == OpStatus::Success;
    if (ok)
        value->SetValue(current);
    value->SetToolTip(ok ? wxString() : wxString(wxT("Failed to read the value")));
}

void GainStagePanel::SetOnValueChanged(ValueChangedCallback callback)
{
    onValueChanged = std::move(callback);
}

void GainStagePanel::OnValueChanged(wxCommandEvent& event)
{
    if (!device)
        return;

    if (device->SetValue(value->GetValue()) != OpStatus::Success)
        wxMessageBox(wxString::Format(wxT("Failed to set %s gain."), wxString::FromUTF8(device->GetName())), _("Error"));

    // Show the value the hardware actually uses
    UpdateGUI();
    if (onValueChanged)
        onValueChanged(device);
}

void GainStagePanel::OnRefresh(wxCommandEvent& event)
{
    UpdateGUI();
}
