#include "ILMS8001Tab.h"

#include "chips/LMS8001/LMS8001.h"
#include "limesuiteng/Logger.h"

#include <wx/spinctrl.h>

#include "lms8001_gui_utilities.h"

using namespace lime;
using namespace std::literals::string_literals;

ILMS8001Tab::ILMS8001Tab(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : wxPanel(parent, id, pos, size, style)
    , chip(nullptr)
    , mChannel(0)
{
}

void ILMS8001Tab::Initialize(lime::LMS8001* pControl)
{
    chip = pControl;
}

void ILMS8001Tab::UpdateGUI()
{
    if (chip == nullptr)
        return;
    LMS8001_WXGUI::UpdateControlsByMap(this, chip, wndId2Enum);
}

void ILMS8001Tab::ParameterChangeHandler(wxCommandEvent& event)
{
    assert(chip != nullptr);
    LMS8Parameter parameter;
    try
    {
        parameter = wndId2Enum.at(reinterpret_cast<wxWindow*>(event.GetEventObject()));
    } catch (std::exception& e)
    {
        lime::error("Control element(ID = "s + std::to_string(event.GetId()) + ") don't have assigned LMS parameter."s);
        return;
    }
    WriteParam(parameter, event.GetInt());
}

void ILMS8001Tab::ParameterChangeHandler(wxSpinEvent& event)
{
    wxCommandEvent evt;
    evt.SetInt(event.GetInt());
    evt.SetId(event.GetId());
    evt.SetEventObject(event.GetEventObject());
    ParameterChangeHandler(evt);
}

void ILMS8001Tab::SetChannel(uint8_t channel)
{
    mChannel = channel;
}

void ILMS8001Tab::WriteParam(const lime::LMS8Parameter param, uint16_t val)
{
    chip->Modify_SPI_Reg_bits(param, val, true, chip->channel, chip->PLLprofile);
}

uint16_t ILMS8001Tab::ReadParam(const lime::LMS8Parameter param)
{
    return chip->Get_SPI_Reg_bits(param, true, chip->channel, chip->PLLprofile);
}
