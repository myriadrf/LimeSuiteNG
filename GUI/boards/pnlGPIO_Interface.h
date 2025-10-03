#ifndef PNL_GPIO_INTERFACE_H
#define PNL_GPIO_INTERFACE_H

#include "limesuiteng/SDRDevice.h"
#include <wx/panel.h>

class wxStaticText;
class wxCheckBox;

#include "ISOCPanel.h"

namespace lime {
class GPIO;
class GPIO_Interface;
} // namespace lime

class pnlGPIO_Interface : public ISOCPanel
{
  public:
    pnlGPIO_Interface(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        int style = 0);
    void Initialize(lime::GPIO_Interface* pControl);
    virtual ~pnlGPIO_Interface();
    void UpdateGUI() override;

  protected:
    void GPIOValueChangeHandler(wxCommandEvent& event);

    lime::GPIO_Interface* control;
    std::vector<wxCheckBox*> checkboxes;
    std::unordered_map<wxCheckBox*, uint32_t> checkboxGPIOmap;
};

#endif
