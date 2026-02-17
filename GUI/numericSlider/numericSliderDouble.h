/**
@file	NumericSliderDouble.h
@brief	Class for integer input using slider or text field
@author Lime Microsystems (www.limemicro.com)
*/

#ifndef NUMERIC_SLIDER_DOUBLE_H
#define NUMERIC_SLIDER_DOUBLE_H

#include <wx/panel.h>

class wxSpinCtrlDouble;
class wxScrollBar;

class NumericSliderDouble : public wxPanel
{
  public:
    NumericSliderDouble();
    NumericSliderDouble(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& value = wxEmptyString,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxSP_ARROW_KEYS,
        double min = 0,
        double max = 100,
        double initial = 0,
        const wxString& name = "NumericSliderDouble");
    ~NumericSliderDouble();

    void SetValue(double integer);
    double GetValue();

    virtual void SetToolTip(const wxString& tipString);
    void SetRange(double min, double max, double step);

  protected:
    void OnSpinnerChangeEnter(wxSpinDoubleEvent& event);
    void OnSpinnerChange(wxSpinDoubleEvent& event);
    void OnScrollChange(wxScrollEvent& event);
    wxSpinCtrlDouble* mSpinner{};
    wxScrollBar* mScroll{};

  private:
    DECLARE_DYNAMIC_CLASS(NumericSliderDouble)
    DECLARE_EVENT_TABLE()
};

#endif
