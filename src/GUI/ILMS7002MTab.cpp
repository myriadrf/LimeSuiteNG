#include "ILMS7002MTab.h"

#include "lms7002_gui_utilities.h"

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"

#include <wx/spinctrl.h>

using namespace lime;
using namespace std::literals::string_literals;

ILMS7002MTab::ILMS7002MTab(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : wxPanel(parent, id, pos, size, style)
    , lmsControl(nullptr)
    , mChannel(0)
{
}

void ILMS7002MTab::Initialize(LMS7002M* pControl)
{
    lmsControl = pControl;
}

void ILMS7002MTab::UpdateGUI()
{
    if (lmsControl == nullptr)
        return;
    LMS7002_WXGUI::UpdateControlsByMap(this, lmsControl, wndId2Enum, mChannel);
}

void ILMS7002MTab::ParameterChangeHandler(wxCommandEvent& event)
{
    assert(lmsControl != nullptr);
    LMS7Parameter parameter;
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

void ILMS7002MTab::SpinParameterChangeHandler(wxSpinEvent& event)
{
    wxCommandEvent evt;
    evt.SetInt(event.GetInt());
    evt.SetId(event.GetId());
    evt.SetEventObject(event.GetEventObject());
    ParameterChangeHandler(evt);
}

void ILMS7002MTab::SetChannel(uint8_t channel)
{
    mChannel = channel;
}

void ILMS7002MTab::WriteParam(const LMS7Parameter& param, uint16_t val)
{
    lmsControl->SetActiveChannel(mChannel == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);
    lmsControl->Modify_SPI_Reg_bits(param, val);
}

uint16_t ILMS7002MTab::ReadParam(const LMS7Parameter& param)
{
    lmsControl->SetActiveChannel(mChannel == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);
    return lmsControl->Get_SPI_Reg_bits(param);
}

uint16_t ILMS7002MTab::ReadLMSReg(uint16_t address)
{
    return lmsControl->SPI_read(address);
}

void ILMS7002MTab::WriteLMSReg(uint16_t address, uint16_t value)
{
    lmsControl->SPI_write(address, value);
}
