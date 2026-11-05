#ifndef WIDGET_TONEGENERATOR_H
#define WIDGET_TONEGENERATOR_H

#include <memory>

#include <wx/panel.h>

class wxSpinCtrl;
class wxScrollBar;

class NumericSliderDouble;
class NumericSlider;

namespace lime {
class IToneGenerator;
}

class ToneGeneratorPanel : public wxPanel
{
  public:
    ToneGeneratorPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = wxT("Tone"),
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~ToneGeneratorPanel();
    /// @param dev Not owned, must outlive the panel or be replaced by another Initialize() call.
    void Initialize(lime::IToneGenerator* dev);
    /// @param dev Kept alive by the panel.
    void Initialize(std::shared_ptr<lime::IToneGenerator> dev);

    void ValuesChanged(wxSpinDoubleEvent& event);
    void EnableChanged(wxCommandEvent& event);

  private:
    wxCheckBox* chkEnable;
    NumericSliderDouble* amplitude;
    NumericSliderDouble* freq_weight;
    lime::IToneGenerator* device;
    std::shared_ptr<lime::IToneGenerator> deviceOwner;
};

#endif
