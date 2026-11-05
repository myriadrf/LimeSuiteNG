#ifndef WIDGET_NCO_H
#define WIDGET_NCO_H

#include <vector>

#include <wx/panel.h>

class wxCheckBox;
class wxChoice;
class wxCommandEvent;
class wxFlexGridSizer;
class wxSpinCtrlDouble;
class wxSpinDoubleEvent;
class wxStaticText;

namespace lime {
class INCO;
}

/// @brief Controls for a lime::INCO: enable, active preset and the frequency/phase of every preset.
class NCOPanel : public wxPanel
{
  public:
    NCOPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = wxT("NCO"),
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~NCOPanel();

    /// @param dev Not owned, must outlive the panel or be replaced by another Initialize() call.
    void Initialize(lime::INCO* dev);
    /// @brief Reads all the values back from the device.
    void UpdateGUI();

  private:
    void OnEnable(wxCommandEvent& event);
    void OnSelectPreset(wxCommandEvent& event);
    void OnFrequencyChanged(wxSpinDoubleEvent& event);
    void OnPhaseChanged(wxSpinDoubleEvent& event);
    void OnRefresh(wxCommandEvent& event);

    void CreatePresetRows(uint8_t count);
    int FindRow(const std::vector<wxSpinCtrlDouble*>& controls, const wxObject* control) const;

    wxWindow* box;
    wxCheckBox* chkEnable;
    wxChoice* cmbPreset;
    wxStaticText* lblRange;
    wxFlexGridSizer* presetGrid;
    std::vector<wxSpinCtrlDouble*> frequencies;
    std::vector<wxSpinCtrlDouble*> phases;
    lime::INCO* device;
};

#endif
