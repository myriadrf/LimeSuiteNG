#include "GainStagesPanel.h"
#include "GainStageFormat.h"

#include "limesuiteng/IGainStage.h"
#include "limesuiteng/IRFChannel.h"

#include <wx/button.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

using namespace lime;
using namespace GainStageFormat;

GainStagesPanel::GainStagesPanel(
    wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
{
    Create(parent, id, pos, size, style);

    wxStaticBoxSizer* sbSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, title), wxVERTICAL);
    box = sbSizer->GetStaticBox();

    grid = new wxFlexGridSizer(0, 4, 2, 5);
    grid->AddGrowableCol(1);
    sbSizer->Add(grid, 0, wxEXPAND, 0);

    wxButton* btnRefresh = new wxButton(box, wxID_ANY, wxT("Read back"));
    btnRefresh->Bind(wxEVT_BUTTON, &GainStagesPanel::OnRefresh, this);
    sbSizer->Add(btnRefresh, 0, wxTOP, 2);

    SetSizer(sbSizer);
    Layout();
    sbSizer->Fit(this);
}

GainStagesPanel::~GainStagesPanel()
{
}

void GainStagesPanel::Initialize(lime::IRFChannel* channel)
{
    std::vector<IGainStage*> stages;
    if (channel)
    {
        for (uint8_t i = 0; i < channel->GetGainStageCount(); ++i)
        {
            IGainStage* stage = channel->GetGainStage(i);
            if (stage)
                stages.push_back(stage);
        }
    }
    Initialize(stages);
}

void GainStagesPanel::Initialize(const std::vector<lime::IGainStage*>& stages)
{
    grid->Clear(true);
    values.clear();
    gainStages = stages;

    const int textFlags = wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL;
    for (size_t i = 0; i < gainStages.size(); ++i)
    {
        IGainStage* stage = gainStages[i];
        const Range<double> range = stage->GetRange();
        const double increment = GetIncrement(range);
        const wxString units = UnitsToString(stage->GetUnits());

        grid->Add(new wxStaticText(box, wxID_ANY, wxString::FromUTF8(stage->GetName()) + wxT(":")), 0, textFlags, 0);

        wxSpinCtrlDouble* value = new wxSpinCtrlDouble(box,
            wxID_ANY,
            wxEmptyString,
            wxDefaultPosition,
            wxDefaultSize,
            wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER,
            range.min,
            range.max,
            range.min,
            increment);
        value->SetDigits(GetDigits(increment));
        value->Bind(wxEVT_SPINCTRLDOUBLE, [this, i](wxSpinDoubleEvent&) { OnValueChanged(i); });
        value->Bind(wxEVT_TEXT_ENTER, [this, i](wxCommandEvent&) { OnValueChanged(i); });
        grid->Add(value, 0, wxEXPAND, 0);
        values.push_back(value);

        grid->Add(new wxStaticText(box, wxID_ANY, units), 0, textFlags, 0);
        grid->Add(new wxStaticText(box, wxID_ANY, wxString::Format(wxT("[%g, %g]"), range.min, range.max)), 0, textFlags, 0);
    }

    Enable(!gainStages.empty());
    UpdateGUI();
    GetSizer()->Fit(this);
    Layout();
    if (GetParent())
        GetParent()->Layout();
}

void GainStagesPanel::UpdateGUI()
{
    for (size_t i = 0; i < gainStages.size(); ++i)
    {
        double value = 0;
        const bool ok = gainStages[i]->GetValue(value) == OpStatus::Success;
        if (ok)
            values[i]->SetValue(value);
        values[i]->SetToolTip(ok ? wxString() : wxString(wxT("Failed to read the value")));
    }
}

void GainStagesPanel::OnValueChanged(size_t index)
{
    if (index >= gainStages.size())
        return;

    IGainStage* stage = gainStages[index];
    if (stage->SetValue(values[index]->GetValue()) != OpStatus::Success)
        wxMessageBox(wxString::Format(wxT("Failed to set %s gain."), wxString::FromUTF8(stage->GetName())), _("Error"));

    // Show the values the hardware actually uses, setting one stage can change the others
    UpdateGUI();
}

void GainStagesPanel::OnRefresh(wxCommandEvent& event)
{
    UpdateGUI();
}
