#include "RFTimingPanel.h"

#include "SOC_GUIFactory.h"

#include "interface/IRFTiming.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

using namespace lime;

static bool rftiming_gui_isRegistered = RegisterToFactory<SOC_GUIFactory, &RFTimingPanel::Create>(IRFTiming::DeviceTreeClass);

// Selects the trigger value in the choice, adding it if the device reported a value that is not listed
static void SelectTrigger(wxChoice* choice, const std::string& trigger)
{
    int index = choice->FindString(trigger);
    if (index == wxNOT_FOUND)
        index = choice->Append(trigger);
    choice->SetSelection(index);
}

ISOCPanel* RFTimingPanel::Create(wxWindow* parent, wxWindowID id)
{
    return new RFTimingPanel(parent, id);
}

RFTimingPanel::RFTimingPanel(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
    , timing(nullptr)
{
    wxFlexGridSizer* mainSizer = new wxFlexGridSizer(0, 1, 0, 0);
    mainSizer->SetFlexibleDirection(wxVERTICAL);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    timingBox = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, "RF timing"), wxVERTICAL);
    timingBox->Add(new wxStaticText(timingBox->GetStaticBox(),
        wxID_ANY,
        "Offsets in device timer ticks, relative to the burst start/end (negative = before)."));

    controlsGrid = new wxFlexGridSizer(0, 5, 2, 5);
    controlsGrid->SetFlexibleDirection(wxBOTH);
    controlsGrid->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);
    for (const char* header : { "Control", "Start offset", "Start action", "End offset", "End action" })
        controlsGrid->Add(new wxStaticText(timingBox->GetStaticBox(), wxID_ANY, header), 0, wxALIGN_CENTER_VERTICAL);
    timingBox->Add(controlsGrid, 0, wxTOP, 5);

    wxButton* btnRead = new wxButton(timingBox->GetStaticBox(), wxID_ANY, "Read");
    btnRead->Bind(wxEVT_BUTTON, &RFTimingPanel::OnRead, this);
    timingBox->Add(btnRead, 0, wxTOP, 5);

    mainSizer->Add(timingBox);

    SetSizer(mainSizer);
    Layout();
    mainSizer->Fit(this);
}

RFTimingPanel::~RFTimingPanel()
{
}

bool RFTimingPanel::Initialize(lime::IRFTiming* soc)
{
    if (!soc)
        return false;

    timing = soc;
    timingBox->GetStaticBox()->SetLabel(timing->GetDirection() == TRXDir::Rx ? "Rx RF timing" : "Tx RF timing");

    wxWindow* parent = timingBox->GetStaticBox();
    for (const IRFTiming::ControlDescriptor& desc : timing->GetControls())
    {
        ControlRow row{ desc.name, desc.sharedOffset, nullptr, nullptr, nullptr, nullptr };

        wxStaticText* label = new wxStaticText(parent, wxID_ANY, desc.name);
        label->SetToolTip(desc.description);
        controlsGrid->Add(label, 0, wxALIGN_CENTER_VERTICAL);

        auto makeOffset = [&]() {
            wxSpinCtrl* spin = new wxSpinCtrl(parent,
                wxID_ANY,
                wxEmptyString,
                wxDefaultPosition,
                wxDefaultSize,
                wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER,
                desc.offsetRange.min,
                desc.offsetRange.max,
                0);
            controlsGrid->Add(spin, 0, wxEXPAND);
            return spin;
        };
        wxArrayString triggers;
        for (const std::string& value : desc.triggerValues)
            triggers.Add(value);
        auto makeAction = [&]() {
            wxChoice* choice = new wxChoice(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, triggers);
            choice->Enable(desc.configurableTriggers);
            controlsGrid->Add(choice, 0, wxEXPAND);
            return choice;
        };
        row.startOffset = makeOffset();
        row.startAction = makeAction();
        row.endOffset = makeOffset();
        row.endAction = makeAction();
        // a shared offset is set with the start offset only
        row.endOffset->Enable(!desc.sharedOffset);
        if (desc.sharedOffset)
            row.endOffset->SetToolTip("Same as the start offset");

        rows.push_back(row);
    }

    // bind after all rows exist, the handlers look their row up by index
    for (size_t i = 0; i < rows.size(); ++i)
    {
        auto onChange = [this, i](wxCommandEvent&) { WriteControl(rows[i]); };
        rows[i].startOffset->Bind(wxEVT_SPINCTRL, onChange);
        rows[i].startOffset->Bind(wxEVT_TEXT_ENTER, onChange);
        rows[i].endOffset->Bind(wxEVT_SPINCTRL, onChange);
        rows[i].endOffset->Bind(wxEVT_TEXT_ENTER, onChange);
        rows[i].startAction->Bind(wxEVT_CHOICE, onChange);
        rows[i].endAction->Bind(wxEVT_CHOICE, onChange);
    }

    UpdateGUI();
    Layout();
    GetSizer()->Fit(this);
    return true;
}

bool RFTimingPanel::Initialize(void* soc)
{
    return Initialize(reinterpret_cast<lime::IRFTiming*>(soc));
}

void RFTimingPanel::UpdateGUI()
{
    if (!timing)
        return;

    for (ControlRow& row : rows)
    {
        IRFTiming::Control control;
        if (timing->GetControl(row.name, control) != OpStatus::Success)
            continue;
        row.startOffset->SetValue(control.start.offset);
        SelectTrigger(row.startAction, control.start.trigger);
        row.endOffset->SetValue(control.end.offset);
        SelectTrigger(row.endAction, control.end.trigger);
    }
}

void RFTimingPanel::WriteControl(ControlRow& row)
{
    if (!timing)
        return;

    if (row.sharedOffset)
        row.endOffset->SetValue(row.startOffset->GetValue());

    IRFTiming::Control control;
    control.start.offset = row.startOffset->GetValue();
    control.start.trigger = row.startAction->GetStringSelection().ToStdString();
    control.end.offset = row.endOffset->GetValue();
    control.end.trigger = row.endAction->GetStringSelection().ToStdString();

    OpStatus status = timing->SetControl(row.name, control);
    if (status != OpStatus::Success)
        wxMessageBox(wxString::Format("RF timing %s write failed.", row.name), "Error");
}

void RFTimingPanel::OnRead(wxCommandEvent& event)
{
    UpdateGUI();
}
