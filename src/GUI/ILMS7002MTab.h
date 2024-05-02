#pragma once

#include <wx/panel.h>
#include <map>
#include <cstdint>
#include "commonWxForwardDeclarations.h"

struct LMS7Parameter;
namespace lime {
class LMS7002M;
}

class ILMS7002MTab : public wxPanel
{
  public:
    ILMS7002MTab(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);
    virtual void Initialize(lime::LMS7002M* pControl);
    virtual void UpdateGUI();
    void SetChannel(uint8_t channel);

  protected:
    uint16_t ReadLMSReg(uint16_t address);
    void WriteLMSReg(uint16_t address, uint16_t value);
    virtual void ParameterChangeHandler(wxCommandEvent& event);
    virtual void SpinParameterChangeHandler(wxSpinEvent& event);

    virtual void WriteParam(const LMS7Parameter& param, uint16_t val);
    virtual uint16_t ReadParam(const LMS7Parameter& param);

    lime::LMS7002M* lmsControl;
    std::map<wxWindow*, LMS7Parameter> wndId2Enum;
    uint8_t mChannel;
};
