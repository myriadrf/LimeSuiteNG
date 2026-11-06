#include "QECPanel.h"
#include "GainStageFormat.h"

#include "limesuiteng/IQuadratureErrorCorrector.h"

#include <algorithm>

#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

using namespace lime;

namespace {

/// Sets the control's range and increment, a step of 0 means the values are not evenly spaced
void ConfigureControl(wxSpinCtrlDouble* control, wxStaticText* lblRange, const Range<float>& range)
{
    const Range<double> r(range.min, range.max, range.step);
    const double increment = GainStageFormat::GetIncrement(r);
    control->SetRange(r.min, r.max);
    control->SetIncrement(increment);
    // the hardware steps can need many digits, show a reasonable amount
    control->SetDigits(std::min(4u, GainStageFormat::GetDigits(increment)));
    lblRange->SetLabel(wxString::Format(wxT("[%g, %g]"), r.min, r.max));
}

} // namespace

QECPanel::QECPanel(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
{
    const int textFlags = wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL;

    Create(parent, id, pos, size, style);

    wxStaticBoxSizer* sbSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, title), wxVERTICAL);
    box = sbSizer->GetStaticBox();

    wxFlexGridSizer* grid = new wxFlexGridSizer(0, 4, 2, 5);
    grid->AddGrowableCol(1);

    auto addRow = [&](const wxString& label, const wxString& units, wxSpinCtrlDouble*& control, wxStaticText*& lblRange) {
        grid->Add(new wxStaticText(box, wxID_ANY, label), 0, textFlags, 0);
        control = new wxSpinCtrlDouble(
            box, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, 0, 0, 0, 1);
        control->Bind(wxEVT_SPINCTRLDOUBLE, [this](wxSpinDoubleEvent&) { WriteValues(); });
        control->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent&) { WriteValues(); });
        grid->Add(control, 0, wxEXPAND, 0);
        grid->Add(new wxStaticText(box, wxID_ANY, units), 0, textFlags, 0);
        lblRange = new wxStaticText(box, wxID_ANY, wxT("[-, -]"));
        grid->Add(lblRange, 0, textFlags, 0);
    };
    addRow(wxT("Gain:"), wxT("dB"), gainImbalance, lblGainRange);
    addRow(wxT("Phase:"), wxT("deg"), phaseImbalance, lblPhaseRange);

    sbSizer->Add(grid, 0, wxEXPAND, 0);

    SetSizer(sbSizer);
    Layout();
    sbSizer->Fit(this);
}

QECPanel::~QECPanel()
{
}

void QECPanel::Initialize(std::shared_ptr<lime::IQuadratureErrorCorrector> dev)
{
    Initialize(dev.get());
    owner = std::move(dev);
}

void QECPanel::Initialize(lime::IQuadratureErrorCorrector* dev)
{
    owner.reset();
    device = dev;
    Enable(device != nullptr);
    if (!device)
    {
        lblGainRange->SetLabel(wxT("[-, -]"));
        lblPhaseRange->SetLabel(wxT("[-, -]"));
        return;
    }

    ConfigureControl(gainImbalance, lblGainRange, device->GetGainRange());
    ConfigureControl(phaseImbalance, lblPhaseRange, device->GetPhaseRange());

    UpdateGUI();
    GetSizer()->Fit(this);
    Layout();
}

void QECPanel::UpdateGUI()
{
    if (!device)
        return;

    float gain = 0, phase = 0;
    if (device->GetImbalance(gain, phase) != OpStatus::Success)
        return;
    gainImbalance->SetValue(gain);
    phaseImbalance->SetValue(phase);
}

void QECPanel::WriteValues()
{
    if (!device)
        return;

    if (device->SetImbalance(gainImbalance->GetValue(), phaseImbalance->GetValue()) != OpStatus::Success)
        wxMessageBox(wxT("Failed to set the QEC correction."), _("Error"));
    // Show the values the hardware actually uses, if it can be read back
    UpdateGUI();
}
