#include "dlgTempCalibrate.h"

#include "chips/LMS8001/LMS8001.h"

lms8_dlgTempCalibrate::lms8_dlgTempCalibrate(
    wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
    : wxDialog(parent, id, title, pos, size, style)
{
    this->SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxFlexGridSizer* fgSizer12;
    fgSizer12 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer12->SetFlexibleDirection(wxBOTH);
    fgSizer12->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer8;
    fgSizer8 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer8->AddGrowableCol(1);
    fgSizer8->SetFlexibleDirection(wxBOTH);
    fgSizer8->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText6 = new wxStaticText(this, wxID_ANY, wxT("Value:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText6->Wrap(-1);
    fgSizer8->Add(m_staticText6, 0, wxALL, 4);

    txtValue = new wxTextCtrl(this, wxID_ANY, wxT("113"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8->Add(txtValue, 0, wxALL, 0);

    m_staticText8 = new wxStaticText(this, wxID_ANY, wxT("Temperature  [°C]:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText8->Wrap(-1);
    fgSizer8->Add(m_staticText8, 0, wxALL, 4);

    txtTemperature = new wxTextCtrl(this, wxID_ANY, wxT("25.5"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8->Add(txtTemperature, 0, wxALL, 0);

    m_staticText10 = new wxStaticText(this, wxID_ANY, wxT("Coeff. T0:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText10->Wrap(-1);
    fgSizer8->Add(m_staticText10, 0, wxALL, 5);

    sttxtT0 = new wxStaticText(this, wxID_ANY, wxT("?"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtT0->Wrap(-1);
    fgSizer8->Add(sttxtT0, 0, wxALL, 5);

    fgSizer12->Add(fgSizer8, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer38;
    fgSizer38 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer38->SetFlexibleDirection(wxBOTH);
    fgSizer38->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    btnCalibrate = new wxButton(this, wxID_ANY, wxT("Calibrate"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer38->Add(btnCalibrate, 0, wxALL, 5);

    btnReset = new wxButton(this, wxID_ANY, wxT("Reset"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer38->Add(btnReset, 0, wxALL, 5);

    fgSizer12->Add(fgSizer38, 1, wxALIGN_CENTER_HORIZONTAL, 5);

    this->SetSizer(fgSizer12);
    this->Layout();
    fgSizer12->Fit(this);

    this->Centre(wxBOTH);

    // Connect Events
    btnCalibrate->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8_dlgTempCalibrate::OnClick_btnCalibrate), NULL, this);
    btnReset->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8_dlgTempCalibrate::OnClick_btnReset), NULL, this);
}

lms8_dlgTempCalibrate::~lms8_dlgTempCalibrate()
{
    // Disconnect Events
    btnCalibrate->Disconnect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8_dlgTempCalibrate::OnClick_btnCalibrate), NULL, this);
    btnReset->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8_dlgTempCalibrate::OnClick_btnReset), NULL, this);
}

void lms8_dlgTempCalibrate::Initialize(lime::LMS8001* pControl)
{
    lmsControl = pControl;

    double tempSens_T0 = lmsControl->tempSens_T0;
    sttxtT0->SetLabel(wxString::Format(_("%.2f"), tempSens_T0));
}

void lms8_dlgTempCalibrate::OnClick_btnCalibrate(wxCommandEvent& event)
{
    double code;
    txtValue->GetValue().ToDouble(&code);
    double temperature;
    txtTemperature->GetValue().ToDouble(&temperature);

    double tempSens_T1 = TEMPSENS_T1;
    double tempSens_T2 = TEMPSENS_T2;
    double tempSens_T0;

    tempSens_T0 = temperature - tempSens_T1 * code - tempSens_T2 * pow(code, 2);

    lmsControl->tempSens_T0 = tempSens_T0;
    sttxtT0->SetLabel(wxString::Format(_("%.2f"), tempSens_T0));
}

void lms8_dlgTempCalibrate::OnClick_btnReset(wxCommandEvent& event)
{
    double tempSens_T0 = TEMPSENS_T0;
    lmsControl->tempSens_T0 = tempSens_T0;
    sttxtT0->SetLabel(wxString::Format(_("%.2f"), tempSens_T0));
}
