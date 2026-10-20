#include "GPS_wxgui.h"
#include "limesuiteng/Logger.h"

#include "SOC_GUIFactory.h"

#include "limesuiteng/GPS.h"

#include <vector>

using namespace lime;

static bool gps_gui_isRegistered = RegisterToFactory<SOC_GUIFactory, &GPS_wxgui::Create>("GPS");

ISOCPanel* GPS_wxgui::Create(wxWindow* parent, wxWindowID id)
{
    return new GPS_wxgui(parent, id);
}

GPS_wxgui::GPS_wxgui(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
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

    SetSizer(mainSizer);
    Layout();
    mainSizer->Fit(this);
}

GPS_wxgui::~GPS_wxgui()
{
}

bool GPS_wxgui::Initialize(lime::GPS* soc)
{
    if (!soc)
        return false;

    gps = soc;

    return true;
}

bool GPS_wxgui::Initialize(void* soc)
{
    return Initialize(reinterpret_cast<lime::GPS*>(soc));
}

void GPS_wxgui::UpdateGUI()
{
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