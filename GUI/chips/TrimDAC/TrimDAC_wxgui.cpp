#include "TrimDAC_wxgui.h"

#include "SOC_GUIFactory.h"

#include "limesuiteng/ITrimDAC.h"
#include "limesuiteng/ToString.h"

#include <wx/button.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

#include <algorithm>
#include <climits>

using namespace lime;

static bool trimdac_gui_isRegistered = RegisterToFactory<SOC_GUIFactory, &TrimDAC_wxgui::Create>(ITrimDAC::DeviceTreeClass);

static wxString ValueToString(uint32_t value)
{
    return wxString::Format("%u (0x%04X)", value, value);
}

ISOCPanel* TrimDAC_wxgui::Create(wxWindow* parent, wxWindowID id)
{
    return new TrimDAC_wxgui(parent, id);
}

TrimDAC_wxgui::TrimDAC_wxgui(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
    , dac(nullptr)
{
    wxFlexGridSizer* mainSizer = new wxFlexGridSizer(0, 1, 5, 5);
    mainSizer->SetFlexibleDirection(wxBOTH);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* infoGrid = new wxFlexGridSizer(0, 2, 2, 10);
    infoGrid->Add(new wxStaticText(this, wxID_ANY, "Oscillator:"));
    lblName = new wxStaticText(this, wxID_ANY, "-");
    infoGrid->Add(lblName);
    infoGrid->Add(new wxStaticText(this, wxID_ANY, "Range:"));
    lblRange = new wxStaticText(this, wxID_ANY, "-");
    infoGrid->Add(lblRange);
    mainSizer->Add(infoGrid, 0, wxALL, 2);

    // Current (volatile) value
    wxStaticBoxSizer* valueBox = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, "Current value"), wxVERTICAL);
    wxBoxSizer* setSizer = new wxBoxSizer(wxHORIZONTAL);
    spinValue = new wxSpinCtrl(this, wxID_ANY, "0", wxDefaultPosition, wxSize(120, -1), wxSP_ARROW_KEYS, 0, 65535, 0);
    spinValue->SetToolTip("Value to set or store");
    setSizer->Add(spinValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 2);
    wxButton* btnSet = new wxButton(this, wxID_ANY, "Set");
    btnSet->SetToolTip("Set the DAC output. The value is lost at power off unless it is stored.");
    btnSet->Bind(wxEVT_BUTTON, &TrimDAC_wxgui::OnSet, this);
    setSizer->Add(btnSet, 0, wxALL, 2);
    wxButton* btnRead = new wxButton(this, wxID_ANY, "Read");
    btnRead->Bind(wxEVT_BUTTON, &TrimDAC_wxgui::OnRead, this);
    setSizer->Add(btnRead, 0, wxALL, 2);
    valueBox->Add(setSizer);
    wxBoxSizer* currentSizer = new wxBoxSizer(wxHORIZONTAL);
    currentSizer->Add(new wxStaticText(this, wxID_ANY, "DAC value:"), 0, wxALL, 2);
    lblValue = new wxStaticText(this, wxID_ANY, "-", wxDefaultPosition, wxSize(140, -1));
    currentSizer->Add(lblValue, 0, wxALL, 2);
    valueBox->Add(currentSizer);
    mainSizer->Add(valueBox, 0, wxEXPAND);

    // Non-volatile value
    wxStaticBoxSizer* storedBox =
        new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, "Stored value (loaded at power up)"), wxVERTICAL);
    wxBoxSizer* storedSizer = new wxBoxSizer(wxHORIZONTAL);
    storedSizer->Add(new wxStaticText(this, wxID_ANY, "Stored value:"), 0, wxALL, 2);
    lblStored = new wxStaticText(this, wxID_ANY, "-", wxDefaultPosition, wxSize(140, -1));
    storedSizer->Add(lblStored, 0, wxALL, 2);
    storedBox->Add(storedSizer);
    wxBoxSizer* storeButtons = new wxBoxSizer(wxHORIZONTAL);
    btnStore = new wxButton(this, wxID_ANY, "Store");
    btnStore->SetToolTip("Write the value from the box above to the non-volatile memory. Does not change the DAC output.");
    btnStore->Bind(wxEVT_BUTTON, &TrimDAC_wxgui::OnStore, this);
    storeButtons->Add(btnStore, 0, wxALL, 2);
    btnLoadStored = new wxButton(this, wxID_ANY, "Set stored");
    btnLoadStored->SetToolTip("Set the DAC output to the stored value");
    btnLoadStored->Bind(wxEVT_BUTTON, &TrimDAC_wxgui::OnLoadStored, this);
    storeButtons->Add(btnLoadStored, 0, wxALL, 2);
    storedBox->Add(storeButtons);
    mainSizer->Add(storedBox, 0, wxEXPAND);

    SetSizer(mainSizer);
    Layout();
    mainSizer->Fit(this);
}

bool TrimDAC_wxgui::Initialize(lime::ITrimDAC* soc)
{
    dac = soc;
    if (!dac)
        return false;

    const Range<uint32_t> range = dac->GetRange();
    const int maxValue = static_cast<int>(std::min<uint32_t>(range.max, INT_MAX));
    const int minValue = static_cast<int>(std::min<uint32_t>(range.min, static_cast<uint32_t>(maxValue)));
    spinValue->SetRange(minValue, maxValue);
    lblName->SetLabel(dac->GetName());
    lblRange->SetLabel(wxString::Format("%u to %u", range.min, range.max));

    UpdateGUI();
    uint32_t value = 0;
    if (dac->GetValue(value) == OpStatus::Success)
        spinValue->SetValue(static_cast<int>(value));
    Layout();
    return true;
}

bool TrimDAC_wxgui::Initialize(void* soc)
{
    return Initialize(reinterpret_cast<lime::ITrimDAC*>(soc));
}

void TrimDAC_wxgui::UpdateGUI()
{
    if (!dac)
        return;

    uint32_t value = 0;
    if (dac->GetValue(value) == OpStatus::Success)
        lblValue->SetLabel(ValueToString(value));
    else
        lblValue->SetLabel("Read failed");

    OpStatus status = dac->GetNonVolatileValue(value);
    const bool stored = status != OpStatus::NotSupported;
    if (status == OpStatus::Success)
        lblStored->SetLabel(ValueToString(value));
    else if (!stored)
        lblStored->SetLabel("Not stored by the device");
    else
        lblStored->SetLabel("Read failed");
    btnStore->Enable(stored);
    btnLoadStored->Enable(status == OpStatus::Success);
}

void TrimDAC_wxgui::OnSet(wxCommandEvent& event)
{
    if (!dac)
        return;

    OpStatus status = dac->SetValue(static_cast<uint32_t>(spinValue->GetValue()));
    if (status != OpStatus::Success)
        wxMessageBox(wxString::Format("Trim DAC set failed: %s", ToString(status).c_str()), "Error");
    UpdateGUI();
}

void TrimDAC_wxgui::OnRead(wxCommandEvent& event)
{
    if (!dac)
        return;

    uint32_t value = 0;
    if (dac->GetValue(value) == OpStatus::Success)
        spinValue->SetValue(static_cast<int>(value));
    UpdateGUI();
}

void TrimDAC_wxgui::OnStore(wxCommandEvent& event)
{
    if (!dac)
        return;

    const uint32_t value = static_cast<uint32_t>(spinValue->GetValue());
    if (wxMessageBox(wxString::Format("Store %u as the power up trim DAC value?", value), "Store trim DAC value", wxYES_NO, this) !=
        wxYES)
        return;

    OpStatus status = dac->SetNonVolatileValue(value);
    if (status != OpStatus::Success)
        wxMessageBox(wxString::Format("Trim DAC store failed: %s", ToString(status).c_str()), "Error");
    UpdateGUI();
}

void TrimDAC_wxgui::OnLoadStored(wxCommandEvent& event)
{
    if (!dac)
        return;

    uint32_t value = 0;
    OpStatus status = dac->GetNonVolatileValue(value);
    if (status == OpStatus::Success)
        status = dac->SetValue(value);
    if (status != OpStatus::Success)
        wxMessageBox(wxString::Format("Trim DAC set failed: %s", ToString(status).c_str()), "Error");
    else
        spinValue->SetValue(static_cast<int>(value));
    UpdateGUI();
}
