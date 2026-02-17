/**
@file	NumericSliderDouble.cpp
@brief	Slider control with text field for integer entry
@author	Lime Microsystems (www.limemicro.com)
*/

#include "numericSliderDouble.h"
#include <wx/event.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/sizer.h>
#include <wx/scrolbar.h>
#include <wx/textctrl.h>

IMPLEMENT_DYNAMIC_CLASS(NumericSliderDouble, wxPanel)

NumericSliderDouble::NumericSliderDouble()
{
}

NumericSliderDouble::NumericSliderDouble(wxWindow* parent,
    wxWindowID id,
    const wxString& value,
    const wxPoint& pos,
    const wxSize& size,
    long style,
    double min,
    double max,
    double initial,
    const wxString& name)
{
    Create(parent, id, pos, size, style);

    wxFlexGridSizer* mainSizer;
    mainSizer = new wxFlexGridSizer(0, 2, 0, 0);
    mainSizer->AddGrowableCol(0);
    mainSizer->SetFlexibleDirection(wxHORIZONTAL);

    mScroll = new wxScrollBar(this, wxNewId(), wxDefaultPosition, wxDefaultSize, wxSB_HORIZONTAL);
    mScroll->SetMinSize(wxSize(128, -1));
    mainSizer->Add(mScroll, 0, wxALIGN_CENTER_VERTICAL | wxEXPAND, 0);
    int sliderStep = (max - min) / 20;
    if (sliderStep == 0)
        sliderStep = 1;
    mScroll->SetScrollbar(initial, 1, max - min + 1, sliderStep);
    mScroll->Connect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(NumericSliderDouble::OnScrollChange), nullptr, this);

    mSpinner = new wxSpinCtrlDouble(
        this, wxNewId(), wxEmptyString, wxDefaultPosition, wxSize(-1, -1), wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, min, max, initial);
    //mSpinner->SetMinSize(wxSize(112, -1));
    mainSizer->Add(mSpinner, 0, wxALIGN_CENTER_VERTICAL, 0);
    mSpinner->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinDoubleEventHandler(NumericSliderDouble::OnSpinnerChange), nullptr, this);
    mSpinner->Connect(wxEVT_TEXT_ENTER, wxSpinDoubleEventHandler(NumericSliderDouble::OnSpinnerChangeEnter), nullptr, this);

    mScroll->SetThumbPosition(mSpinner->GetValue() - mSpinner->GetMin());

    SetSizer(mainSizer);
    Layout();
    mainSizer->Fit(this);
}

NumericSliderDouble::~NumericSliderDouble()
{
}

void NumericSliderDouble::OnSpinnerChangeEnter(wxSpinDoubleEvent& event)
{
    wxSpinDoubleEvent evt(wxEVT_COMMAND_SPINCTRLDOUBLE_UPDATED);
    evt.SetId(GetId());
    double value = 0;
    event.GetString().ToDouble(&value);
    evt.SetValue(value);
    evt.SetEventObject(this);
    mScroll->SetThumbPosition((event.GetValue() - mSpinner->GetMin()) / mSpinner->GetIncrement());
    wxPostEvent(this, evt);
}

void NumericSliderDouble::OnSpinnerChange(wxSpinDoubleEvent& event)
{
    wxSpinDoubleEvent evt(wxEVT_COMMAND_SPINCTRLDOUBLE_UPDATED);
    evt.SetId(GetId());
    evt.SetValue(event.GetValue());
    evt.SetEventObject(this);
    mScroll->SetThumbPosition((event.GetValue() - mSpinner->GetMin()) / mSpinner->GetIncrement());
    wxPostEvent(this, evt);
}

void NumericSliderDouble::OnScrollChange(wxScrollEvent& event)
{
    wxSpinDoubleEvent evt(wxEVT_COMMAND_SPINCTRLDOUBLE_UPDATED);
    evt.SetId(GetId());
    evt.SetEventObject(this);
    double value = event.GetInt() * mSpinner->GetIncrement() + mSpinner->GetMin();
    mSpinner->SetValue(value);
    evt.SetValue(value);
    wxPostEvent(this, evt);
}

void NumericSliderDouble::SetValue(double value)
{
    if (value < mSpinner->GetMin())
        value = mSpinner->GetMin();
    else if (value > mSpinner->GetMax())
        value = mSpinner->GetMax();
    mSpinner->SetValue(value);
    mScroll->SetThumbPosition(value - mSpinner->GetMin());
}

double NumericSliderDouble::GetValue()
{
    return mSpinner->GetValue();
}

void NumericSliderDouble::SetToolTip(const wxString& tipString)
{
#if wxUSE_TOOLTIPS
    mSpinner->UnsetToolTip();
    mSpinner->SetToolTip(tipString);
    mScroll->SetToolTip(tipString);
#endif
}

void NumericSliderDouble::SetRange(double min, double max, double step)
{
    mSpinner->SetRange(min, max);
    mSpinner->SetIncrement(step);
    int scrollValue = ((max - min) / 2) / step;
    mScroll->SetScrollbar(scrollValue, 1, (max - min) / step + 1, 1);
}

BEGIN_EVENT_TABLE(NumericSliderDouble, wxPanel)
END_EVENT_TABLE()
