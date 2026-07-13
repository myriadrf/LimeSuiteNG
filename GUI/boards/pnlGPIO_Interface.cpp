#include "pnlGPIO_Interface.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRDescriptor.hpp"
#include "protocols/LMSBoards.h"

#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/checkbox.h>
#include <wx/statbox.h>

#include "limesuiteng/SDRDevice.hpp"

#include "boards/GPIO.h"

using namespace lime;
using namespace std;

pnlGPIO_Interface::pnlGPIO_Interface(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, int style)
    : ISOCPanel(parent, id, pos, size, style)
    , control(nullptr)
{
#ifdef WIN32
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
#endif
}

void pnlGPIO_Interface::Initialize(lime::GPIO_Interface* pControl)
{
    control = pControl;
    if (!control)
        return;

    auto gpioSizer = new wxFlexGridSizer(0, 4, 0, 0);

    auto pins = control->GetPins();
    for (auto& pin : pins)
    {
        wxCheckBox* inputBox = new wxCheckBox(this, wxNewId(), pin.name);
        inputBox->Connect(
            wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(pnlGPIO_Interface::GPIOValueChangeHandler), nullptr, this);
        gpioSizer->Add(inputBox);
        checkboxGPIOmap[inputBox] = pin.id;
    }

    auto groupSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("GPIO Control")), wxVERTICAL);
    groupSizer->Add(gpioSizer, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    SetSizer(groupSizer);
    groupSizer->Fit(this);
    groupSizer->SetSizeHints(this);
    Layout();

    UpdateGUI();
}

pnlGPIO_Interface::~pnlGPIO_Interface()
{
}

void pnlGPIO_Interface::UpdateGUI()
{
    assert(control);
    OpStatus status = control->ReadAll();
    if (status != OpStatus::Success)
        lime::error("Failed to read GPIO."s);

    for (auto& element : checkboxGPIOmap)
    {
        int value = control->GetValue(element.second);
        element.first->SetValue(value);
    }
}

void pnlGPIO_Interface::GPIOValueChangeHandler(wxCommandEvent& event)
{
    assert(control != nullptr);
    try
    {
        wxCheckBox* inputBox = reinterpret_cast<wxCheckBox*>(event.GetEventObject());
        uint32_t pinid = checkboxGPIOmap.at(inputBox);
        const int value = event.GetInt();
        control->SetValue(pinid, value);
        OpStatus status = control->WriteAll();
        if (status != OpStatus::Success)
            lime::error("Failed to write GPIO."s);
    } catch (std::exception& e)
    {
        lime::error("Control element(ID = "s + std::to_string(event.GetId()) + ") don't have assigned GPIO."s);
        return;
    }
}
