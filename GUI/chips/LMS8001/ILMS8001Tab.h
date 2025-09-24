#pragma once

#include <wx/panel.h>
#include <map>
#include <cstdint>
#include "commonWxForwardDeclarations.h"

namespace lime {
class LMS8001;
class LMS8Parameter;
} // namespace lime

class ILMS8001Tab : public wxPanel
{
  public:
    ILMS8001Tab(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);
    virtual void Initialize(lime::LMS8001* pControl);
    virtual void UpdateGUI();
    void SetChannel(uint8_t channel);

  protected:
    virtual void ParameterChangeHandler(wxCommandEvent& event);
    virtual void ParameterChangeHandler(wxSpinEvent& event);

    virtual void WriteParam(const lime::LMS8Parameter param, uint16_t val);
    virtual uint16_t ReadParam(const lime::LMS8Parameter param);

    lime::LMS8001* chip;
    std::map<wxWindow*, lime::LMS8Parameter> wndId2Enum;
    uint8_t mChannel;
};
