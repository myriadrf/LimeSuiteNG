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
    void Initialize(std::shared_ptr<lime::IToneGenerator> dev);

    void ValuesChanged(wxSpinDoubleEvent& event);
    void EnableChanged(wxCommandEvent& event);

  private:
    wxCheckBox* chkEnable;
    NumericSliderDouble* amplitude;
    NumericSlider* fftBin;
    std::shared_ptr<lime::IToneGenerator> device;
};

#endif
