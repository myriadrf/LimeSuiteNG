#include "PPSDO_wxgui.h"
#include "limesuiteng/Logger.h"

#include "SOC_GUIFactory.h"

#include "limesuiteng/PPSDO.h"
#include "limesuiteng/ToString.h"

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <array>

using namespace lime;

static bool ppsdo_gui_isRegistered = RegisterToFactory<SOC_GUIFactory, &PPSDO_wxgui::Create>("PPSDO");

static constexpr int statusRefreshPeriod_ms = 1000;

static wxString StateToString(PPSDO::TuneState state)
{
    switch (state)
    {
    case PPSDO::TuneState::Idle:
        return "Idle";
    case PPSDO::TuneState::CoarseTune:
        return "Coarse tune";
    case PPSDO::TuneState::FineTune:
        return "Fine tune";
    default:
        return "Unknown";
    }
}

static wxString AccuracyToString(uint8_t accuracy)
{
    static const std::array<wxString, 4> levels = { "Disabled/Lowest", "1s Tune", "2s Tune", "3s Tune (Highest)" };
    if (accuracy < levels.size())
        return levels[accuracy];
    return wxString::Format("Unknown(%i)", accuracy);
}

ISOCPanel* PPSDO_wxgui::Create(wxWindow* parent, wxWindowID id)
{
    return new PPSDO_wxgui(parent, id);
}

PPSDO_wxgui::PPSDO_wxgui(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
    , ppsdo(nullptr)
    , refreshTimer(this)
{
    wxFlexGridSizer* mainSizer = new wxFlexGridSizer(0, 1, 5, 5);
    mainSizer->SetFlexibleDirection(wxBOTH);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    // Configuration
    wxStaticBoxSizer* configBox = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, "Configuration"), wxVERTICAL);
    wxFlexGridSizer* configGrid = new wxFlexGridSizer(0, 2, 5, 5);
    configGrid->Add(new wxStaticText(this, wxID_ANY, "Clock frequency (MHz):"), 0, wxALIGN_CENTER_VERTICAL);
    txtClockFrequency = new wxTextCtrl(this, wxID_ANY, "0");
    txtClockFrequency->SetToolTip("Frequency of the clock counted between PPS pulses. 0 lets the device use its own, "
                                  "devices that measure an internal clock known to them ignore it.");
    configGrid->Add(txtClockFrequency);
    configGrid->Add(new wxStaticText(this, wxID_ANY, "Tolerance (ppm):"), 0, wxALIGN_CENTER_VERTICAL);
    txtTolerance = new wxTextCtrl(this, wxID_ANY, "0");
    txtTolerance->SetToolTip("Frequency error tolerated by each measurement window. 0 selects the device default, if it has one.");
    configGrid->Add(txtTolerance);
    configBox->Add(configGrid);

    wxBoxSizer* controlSizer = new wxBoxSizer(wxHORIZONTAL);
    wxButton* btnApply = new wxButton(this, wxID_ANY, "Apply");
    btnApply->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(PPSDO_wxgui::OnApplyConfig), nullptr, this);
    controlSizer->Add(btnApply, 0, wxALL, 2);
    chkEnable = new wxCheckBox(this, wxID_ANY, "Enable");
    chkEnable->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(PPSDO_wxgui::OnEnable), nullptr, this);
    controlSizer->Add(chkEnable, 0, wxALL | wxALIGN_CENTER_VERTICAL, 2);
    wxButton* btnReset = new wxButton(this, wxID_ANY, "Reset");
    btnReset->SetToolTip("Disable and re-enable, restarts the calibration");
    btnReset->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(PPSDO_wxgui::OnReset), nullptr, this);
    controlSizer->Add(btnReset, 0, wxALL, 2);
    configBox->Add(controlSizer);
    mainSizer->Add(configBox, 0, wxEXPAND);

    // Status
    wxStaticBoxSizer* statusBox = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, "Status"), wxVERTICAL);
    wxFlexGridSizer* statusGrid = new wxFlexGridSizer(0, 2, 2, 10);
    auto addStatusRow = [this, statusGrid](const wxString& title) {
        statusGrid->Add(new wxStaticText(this, wxID_ANY, title));
        wxStaticText* value = new wxStaticText(this, wxID_ANY, "-", wxDefaultPosition, wxSize(140, -1));
        statusGrid->Add(value);
        return value;
    };
    lblEnabled = addStatusRow("Enabled:");
    lblState = addStatusRow("State:");
    lblAccuracy = addStatusRow("Accuracy:");
    lblPPSActive = addStatusRow("PPS active:");
    lblError1s = addStatusRow("1s error:");
    lblError10s = addStatusRow("10s error:");
    lblError100s = addStatusRow("100s error:");
    lblDAC = addStatusRow("DAC value:");
    statusBox->Add(statusGrid);

    wxBoxSizer* refreshSizer = new wxBoxSizer(wxHORIZONTAL);
    wxButton* btnRefresh = new wxButton(this, wxID_ANY, "Refresh");
    btnRefresh->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(PPSDO_wxgui::OnRefresh), nullptr, this);
    refreshSizer->Add(btnRefresh, 0, wxALL, 2);
    chkAutoRefresh = new wxCheckBox(this, wxID_ANY, "Auto refresh");
    chkAutoRefresh->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(PPSDO_wxgui::OnAutoRefresh), nullptr, this);
    refreshSizer->Add(chkAutoRefresh, 0, wxALL | wxALIGN_CENTER_VERTICAL, 2);
    statusBox->Add(refreshSizer);
    mainSizer->Add(statusBox, 0, wxEXPAND);

    Bind(wxEVT_TIMER, &PPSDO_wxgui::OnTimer, this, refreshTimer.GetId());

    SetSizer(mainSizer);
    Layout();
    mainSizer->Fit(this);
}

PPSDO_wxgui::~PPSDO_wxgui()
{
    refreshTimer.Stop();
}

bool PPSDO_wxgui::Initialize(lime::PPSDO* soc)
{
    if (!soc)
        return false;

    ppsdo = soc;
    UpdateGUI();
    return true;
}

bool PPSDO_wxgui::Initialize(void* soc)
{
    return Initialize(reinterpret_cast<lime::PPSDO*>(soc));
}

void PPSDO_wxgui::UpdateGUI()
{
    if (!ppsdo)
        return;

    PPSDO::Status status;
    if (ppsdo->GetStatus(status) != OpStatus::Success)
    {
        lblState->SetLabel("Read failed");
        // Don't keep spamming errors.
        chkAutoRefresh->SetValue(false);
        refreshTimer.Stop();
        return;
    }

    chkEnable->SetValue(status.enabled);
    lblEnabled->SetLabel(status.enabled ? "true" : "false");
    lblState->SetLabel(StateToString(status.state));
    lblAccuracy->SetLabel(AccuracyToString(status.accuracy));
    lblPPSActive->SetLabel(status.ppsActive ? "true" : "false");
    lblError1s->SetLabel(wxString::Format("%i", status.error1s));
    lblError10s->SetLabel(wxString::Format("%i", status.error10s));
    lblError100s->SetLabel(wxString::Format("%i", status.error100s));
    lblDAC->SetLabel(wxString::Format("0x%08X (%u)", status.dacValue, status.dacValue));
}

void PPSDO_wxgui::OnApplyConfig(wxCommandEvent& event)
{
    if (!ppsdo)
        return;

    PPSDO::Config config;
    double clockMHz = 0;
    if (!txtClockFrequency->GetValue().ToDouble(&clockMHz) || !txtTolerance->GetValue().ToDouble(&config.tolerancePPM))
    {
        wxMessageBox("Invalid number.", "Error");
        return;
    }
    config.clockFrequency = clockMHz * 1e6;

    OpStatus status = ppsdo->Configure(config);
    if (status != OpStatus::Success)
        wxMessageBox(wxString::Format("PPSDO configuration failed: %s", ToString(status).c_str()), "Error");
    UpdateGUI();
}

void PPSDO_wxgui::OnEnable(wxCommandEvent& event)
{
    if (!ppsdo)
        return;

    OpStatus status = ppsdo->Enable(event.IsChecked());
    if (status != OpStatus::Success)
    {
        chkEnable->SetValue(!event.IsChecked());
        wxMessageBox(
            wxString::Format("PPSDO %s failed: %s", event.IsChecked() ? "enable" : "disable", ToString(status).c_str()), "Error");
    }
    UpdateGUI();
}

void PPSDO_wxgui::OnReset(wxCommandEvent& event)
{
    if (!ppsdo)
        return;

    OpStatus status = ppsdo->Enable(false);
    if (status == OpStatus::Success)
        status = ppsdo->Enable(true);
    if (status != OpStatus::Success)
        wxMessageBox(wxString::Format("PPSDO reset failed: %s", ToString(status).c_str()), "Error");
    UpdateGUI();
}

void PPSDO_wxgui::OnRefresh(wxCommandEvent& event)
{
    UpdateGUI();
}

void PPSDO_wxgui::OnAutoRefresh(wxCommandEvent& event)
{
    if (event.IsChecked())
    {
        refreshTimer.Start(statusRefreshPeriod_ms);
        UpdateGUI();
    }
    else
        refreshTimer.Stop();
}

void PPSDO_wxgui::OnTimer(wxTimerEvent& event)
{
    UpdateGUI();
}
