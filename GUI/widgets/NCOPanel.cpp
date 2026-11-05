#include "NCOPanel.h"

#include "limesuiteng/INCO.h"

#include <algorithm>

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

using namespace lime;

NCOPanel::NCOPanel(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
    : device(nullptr)
{
    const int textFlags = wxALIGN_LEFT | wxLEFT | wxALIGN_CENTER_VERTICAL;

    Create(parent, id, pos, size, style);

    wxStaticBoxSizer* sbSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, title), wxVERTICAL);
    box = sbSizer->GetStaticBox();

    wxFlexGridSizer* controls = new wxFlexGridSizer(0, 4, 0, 5);

    chkEnable = new wxCheckBox(box, wxID_ANY, wxT("Enable"));
    chkEnable->Bind(wxEVT_CHECKBOX, &NCOPanel::OnEnable, this);
    controls->Add(chkEnable, 0, wxALIGN_CENTER_VERTICAL, 0);

    controls->Add(new wxStaticText(box, wxID_ANY, wxT("Active preset:")), 0, textFlags, 0);
    cmbPreset = new wxChoice(box, wxID_ANY);
    cmbPreset->Bind(wxEVT_CHOICE, &NCOPanel::OnSelectPreset, this);
    controls->Add(cmbPreset, 0, wxALIGN_CENTER_VERTICAL, 0);

    wxButton* btnRefresh = new wxButton(box, wxID_ANY, wxT("Read back"));
    btnRefresh->Bind(wxEVT_BUTTON, &NCOPanel::OnRefresh, this);
    controls->Add(btnRefresh, 0, wxALIGN_CENTER_VERTICAL, 0);

    sbSizer->Add(controls, 0, wxEXPAND | wxBOTTOM, 2);

    lblRange = new wxStaticText(box, wxID_ANY, wxT("Frequency range: -"));
    sbSizer->Add(lblRange, 0, wxALL, 2);

    presetGrid = new wxFlexGridSizer(0, 3, 0, 5);
    presetGrid->AddGrowableCol(1);
    presetGrid->AddGrowableCol(2);
    sbSizer->Add(presetGrid, 0, wxEXPAND, 0);

    SetSizer(sbSizer);
    Layout();
    sbSizer->Fit(this);
}

NCOPanel::~NCOPanel()
{
}

void NCOPanel::CreatePresetRows(uint8_t count)
{
    presetGrid->Clear(true);
    frequencies.clear();
    phases.clear();
    cmbPreset->Clear();

    if (count == 0)
        return;

    presetGrid->Add(new wxStaticText(box, wxID_ANY, wxT("#")), 0, wxALIGN_CENTER_VERTICAL, 0);
    presetGrid->Add(new wxStaticText(box, wxID_ANY, wxT("Frequency (MHz)")), 0, wxALIGN_CENTER_VERTICAL, 0);
    presetGrid->Add(new wxStaticText(box, wxID_ANY, wxT("Phase (deg)")), 0, wxALIGN_CENTER_VERTICAL, 0);

    for (uint8_t i = 0; i < count; ++i)
    {
        cmbPreset->Append(wxString::Format(wxT("%u"), i));
        presetGrid->Add(new wxStaticText(box, wxID_ANY, wxString::Format(wxT("%u"), i)), 0, wxALIGN_CENTER_VERTICAL, 0);

        wxSpinCtrlDouble* freq = new wxSpinCtrlDouble(
            box, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, 0, 1000, 0, 0.1);
        freq->SetDigits(6);
        freq->Bind(wxEVT_SPINCTRLDOUBLE, &NCOPanel::OnFrequencyChanged, this);
        presetGrid->Add(freq, 0, wxEXPAND, 0);
        frequencies.push_back(freq);

        wxSpinCtrlDouble* phase = new wxSpinCtrlDouble(
            box, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, 0, 360, 0, 1);
        phase->SetDigits(3);
        phase->Bind(wxEVT_SPINCTRLDOUBLE, &NCOPanel::OnPhaseChanged, this);
        presetGrid->Add(phase, 0, wxEXPAND, 0);
        phases.push_back(phase);
    }
}

void NCOPanel::Initialize(lime::INCO* dev)
{
    device = dev;
    Enable(device != nullptr);
    CreatePresetRows(device ? device->GetPresetCount() : 0);
    UpdateGUI();
}

void NCOPanel::UpdateGUI()
{
    if (device)
    {
        // The frequency range depends on the sample rate, so it is refreshed on every read back.
        const Range<double> range = device->GetFrequencyRange();
        lblRange->SetLabel(wxString::Format(wxT("Frequency range: %.6f - %.6f MHz"), range.min / 1e6, range.max / 1e6));

        chkEnable->SetValue(device->IsEnabled());
        const uint8_t selected = device->GetSelectedPreset();
        if (selected < cmbPreset->GetCount())
            cmbPreset->SetSelection(selected);

        for (std::size_t i = 0; i < frequencies.size(); ++i)
        {
            frequencies[i]->SetRange(range.min / 1e6, range.max / 1e6);
            frequencies[i]->SetValue(device->GetFrequency(i) / 1e6);
            phases[i]->SetValue(device->GetPhaseOffset(i));
        }
    }
    Layout();
    GetSizer()->Fit(this);
    if (GetParent())
        GetParent()->Layout();
}

int NCOPanel::FindRow(const std::vector<wxSpinCtrlDouble*>& controls, const wxObject* control) const
{
    auto iter = std::find(controls.begin(), controls.end(), control);
    return iter == controls.end() ? -1 : static_cast<int>(iter - controls.begin());
}

void NCOPanel::OnEnable(wxCommandEvent& event)
{
    if (!device)
        return;
    if (device->Enable(chkEnable->IsChecked()) != OpStatus::Success)
        wxMessageBox(wxT("NCO enable failed."), _("Error"));
}

void NCOPanel::OnSelectPreset(wxCommandEvent& event)
{
    if (!device || cmbPreset->GetSelection() < 0)
        return;
    if (device->SelectPreset(cmbPreset->GetSelection()) != OpStatus::Success)
        wxMessageBox(wxT("NCO preset selection failed."), _("Error"));
}

void NCOPanel::OnFrequencyChanged(wxSpinDoubleEvent& event)
{
    const int row = FindRow(frequencies, event.GetEventObject());
    if (!device || row < 0)
        return;
    if (device->SetFrequency(row, frequencies[row]->GetValue() * 1e6) != OpStatus::Success)
        wxMessageBox(wxString::Format(wxT("NCO[%i] set frequency failed."), row), _("Error"));
}

void NCOPanel::OnPhaseChanged(wxSpinDoubleEvent& event)
{
    const int row = FindRow(phases, event.GetEventObject());
    if (!device || row < 0)
        return;
    if (device->SetPhaseOffset(row, phases[row]->GetValue()) != OpStatus::Success)
        wxMessageBox(wxString::Format(wxT("NCO[%i] set phase failed."), row), _("Error"));
}

void NCOPanel::OnRefresh(wxCommandEvent& event)
{
    UpdateGUI();
}
