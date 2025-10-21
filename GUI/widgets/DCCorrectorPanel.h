#ifndef WIDGET_DCCORRECTOR_H
#define WIDGET_DCCORRECTOR_H

#include <memory>

#include <wx/panel.h>

class wxSpinCtrl;
class wxScrollBar;

class NumericSlider;

namespace lime {
class IDCCorrector;
}

class DCCorrectorsPanel : public wxPanel
{
  public:
    DCCorrectorsPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~DCCorrectorsPanel();
    void Initialize(std::shared_ptr<lime::IDCCorrector> dev);

    void WriteValues(wxSpinEvent& event);
    void WriteValues(wxCommandEvent& event);

  private:
    NumericSlider* Icontrol;
    NumericSlider* Qcontrol;
    std::shared_ptr<lime::IDCCorrector> device;
};

#endif
