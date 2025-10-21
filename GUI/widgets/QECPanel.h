#ifndef WIDGET_QECCORRECTOR_H
#define WIDGET_QECCORRECTOR_H

#include <memory>

#include <wx/panel.h>

class wxSpinCtrl;
class wxScrollBar;

class NumericSlider;

namespace lime {
class IQuadratureErrorCorrector;
}

class QECPanel : public wxPanel
{
  public:
    QECPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~QECPanel();
    void Initialize(std::shared_ptr<lime::IQuadratureErrorCorrector> dev);

    void WriteValues(wxSpinEvent& event);
    void WriteValues(wxCommandEvent& event);

  private:
    NumericSlider* gainImbalance;
    NumericSlider* phaseImbalance;
    std::shared_ptr<lime::IQuadratureErrorCorrector> device;
};

#endif
