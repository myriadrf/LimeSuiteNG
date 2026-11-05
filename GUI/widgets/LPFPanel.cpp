#include "LPFPanel.h"

#include "limesuiteng/ILPF.h"

#include <wx/button.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

using namespace lime;

LPFPanel::LPFPanel(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
    : device(nullptr)
{
    const int textFlags = wxALIGN_LEFT | wxLEFT | wxALIGN_CENTER_VERTICAL;

    Create(parent, id, pos, size, style);

    wxStaticBoxSizer* sbSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, title), wxVERTICAL);
    wxWindow* box = sbSizer->GetStaticBox();

    wxFlexGridSizer* grid = new wxFlexGridSizer(0, 3, 0, 5);
    grid->AddGrowableCol(1);

    grid->Add(new wxStaticText(box, wxID_ANY, wxT("Bandwidth (MHz):")), 0, textFlags, 0);
    bandwidth = new wxSpinCtrlDouble(
        box, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, 0, 1000, 0, 0.1);
    bandwidth->SetDigits(3);
    bandwidth->Bind(wxEVT_TEXT_ENTER, &LPFPanel::OnTune, this);
    grid->Add(bandwidth, 0, wxEXPAND, 0);

    wxButton* btnTune = new wxButton(box, wxID_ANY, wxT("Tune"));
    btnTune->Bind(wxEVT_BUTTON, &LPFPanel::OnTune, this);
    grid->Add(btnTune, 0, wxALIGN_CENTER_VERTICAL, 0);

    sbSizer->Add(grid, 0, wxEXPAND, 0);
    lblRange = new wxStaticText(box, wxID_ANY, wxT("Range: -"));
    sbSizer->Add(lblRange, 0, wxALL, 2);

    SetSizer(sbSizer);
    Layout();
    sbSizer->Fit(this);
}

LPFPanel::~LPFPanel()
{
}

void LPFPanel::Initialize(lime::ILPF* dev)
{
    device = dev;
    Enable(device != nullptr);
    if (!device)
        return;

    const Range<double> range = device->GetBandwidthRange();
    bandwidth->SetRange(range.min / 1e6, range.max / 1e6);
    lblRange->SetLabel(wxString::Format(wxT("Range: %.3f - %.3f MHz"), range.min / 1e6, range.max / 1e6));

    const double current = device->GetBandwidth();
    bandwidth->SetValue((current > 0 ? current : range.max) / 1e6);
    Layout();
}

void LPFPanel::OnTune(wxCommandEvent& event)
{
    if (!device)
        return;

    wxBusyCursor busy;
    if (device->SetBandwidth(bandwidth->GetValue() * 1e6) != OpStatus::Success)
        wxMessageBox(wxT("LPF tuning failed."), _("Error"));
}
