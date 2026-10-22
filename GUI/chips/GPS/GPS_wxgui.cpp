#include "GPS_wxgui.h"
#include "limesuiteng/Logger.h"

#include "SOC_GUIFactory.h"

#include "limesuiteng/GPS.h"

#include <wx/checkbox.h>
#include <wx/font.h>
#include <wx/stattext.h>

#include <vector>

using namespace lime;

static bool gps_gui_isRegistered = RegisterToFactory<SOC_GUIFactory, &GPS_wxgui::Create>("GPS");

static constexpr int statusRefreshPeriod_ms = 1000;

ISOCPanel* GPS_wxgui::Create(wxWindow* parent, wxWindowID id)
{
    return new GPS_wxgui(parent, id);
}

GPS_wxgui::GPS_wxgui(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
    , gps(nullptr)
    , refreshTimer(this)
{
    wxFlexGridSizer* mainSizer;
    mainSizer = new wxFlexGridSizer(0, 1, 0, 0);
    mainSizer->SetFlexibleDirection(wxVERTICAL);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* ppsBox = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, "PPS"), wxHORIZONTAL);
    wxArrayString ppsChoices;
    ppsChoices.Add("Disabled");
    ppsChoices.Add("AfterFirstFix");
    ppsChoices.Add("3D Fix only");
    ppsChoices.Add("2D/3D Fix only");
    ppsChoices.Add("Always");

    ppsAvailability = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, ppsChoices);
    ppsAvailability->Connect(wxEVT_CHOICE, wxCommandEventHandler(GPS_wxgui::OnPPSChange), nullptr, this);
    ppsBox->Add(ppsAvailability);

    ppsBox->Add(new wxStaticText(this, wxID_ANY, "Pulse width(ms):"));
    ppsPulseWidth = new wxSpinCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 500, 0);
    ppsPulseWidth->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(GPS_wxgui::OnPPSChange), nullptr, this);
    ppsBox->Add(ppsPulseWidth);
    mainSizer->Add(ppsBox);

    btnStandByMode = new wxButton(this, wxID_ANY, "StandBy");
    btnStandByMode->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(GPS_wxgui::OnStandByMode), nullptr, this);
    mainSizer->Add(btnStandByMode);

    wxStaticBoxSizer* statusBox = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, "Status"), wxVERTICAL);
    lblStatus = new wxStaticText(this, wxID_ANY, "-");
    lblStatus->SetFont(wxFont(wxFontInfo().Family(wxFONTFAMILY_TELETYPE)));
    statusBox->Add(lblStatus, 0, wxALL, 2);

    wxBoxSizer* refreshSizer = new wxBoxSizer(wxHORIZONTAL);
    wxButton* btnRefresh = new wxButton(this, wxID_ANY, "Refresh");
    btnRefresh->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(GPS_wxgui::OnRefresh), nullptr, this);
    refreshSizer->Add(btnRefresh, 0, wxALL, 2);
    chkAutoRefresh = new wxCheckBox(this, wxID_ANY, "Auto refresh");
    chkAutoRefresh->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(GPS_wxgui::OnAutoRefresh), nullptr, this);
    refreshSizer->Add(chkAutoRefresh, 0, wxALL | wxALIGN_CENTER_VERTICAL, 2);
    statusBox->Add(refreshSizer);
    mainSizer->Add(statusBox, 0, wxEXPAND);

    Bind(wxEVT_TIMER, &GPS_wxgui::OnTimer, this, refreshTimer.GetId());

    SetSizer(mainSizer);
    Layout();
    mainSizer->Fit(this);
}

GPS_wxgui::~GPS_wxgui()
{
    refreshTimer.Stop();
}

bool GPS_wxgui::Initialize(lime::GPS* soc)
{
    if (!soc)
        return false;

    gps = soc;
    UpdateGUI();
    return true;
}

bool GPS_wxgui::Initialize(void* soc)
{
    return Initialize(reinterpret_cast<lime::GPS*>(soc));
}

void GPS_wxgui::UpdateGUI()
{
    if (!gps)
        return;

    GPS::Status status;
    if (gps->GetStatus(status) != OpStatus::Success)
    {
        lblStatus->SetLabel("Failed to read GPS status");
        // Don't keep spamming errors.
        chkAutoRefresh->SetValue(false);
        refreshTimer.Stop();
    }
    else
        lblStatus->SetLabel(status.ToString());

    // Line lengths vary with the data.
    Layout();
    GetParent()->Layout();
}

void GPS_wxgui::OnPPSChange(wxCommandEvent& event)
{
    OpStatus status = gps->SetPPSConfig(
        static_cast<lime::GPS::PPSmode>(ppsAvailability->GetSelection()), std::chrono::milliseconds(ppsPulseWidth->GetValue()));
    if (status != OpStatus::Success)
        wxMessageBox("GPS PPS config failed.", "Error");
}

void GPS_wxgui::OnStandByMode(wxCommandEvent& event)
{
    OpStatus status = gps->StandByMode(true);
    if (status != OpStatus::Success)
        wxMessageBox("GPS standby failed.", "Error");
}
void GPS_wxgui::OnRefresh(wxCommandEvent& event)
{
    UpdateGUI();
}

void GPS_wxgui::OnAutoRefresh(wxCommandEvent& event)
{
    if (event.IsChecked())
    {
        refreshTimer.Start(statusRefreshPeriod_ms);
        UpdateGUI();
    }
    else
        refreshTimer.Stop();
}

void GPS_wxgui::OnTimer(wxTimerEvent& event)
{
    UpdateGUI();
}
