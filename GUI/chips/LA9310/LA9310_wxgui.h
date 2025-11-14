#pragma once

#include <iostream>
#include <vector>

#include "ISOCPanel.h"

namespace lime {
class LA9310;
} // namespace lime

class DCCorrectorsPanel;
class QECPanel;

class LA9310_wxgui : public ISOCPanel
{
  public:
    static ISOCPanel* Create(wxWindow* parent, wxWindowID id);
    LA9310_wxgui(wxWindow* parent,
        wxWindowID id,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~LA9310_wxgui();
    bool Initialize(lime::LA9310* soc);
    bool Initialize(void* soc) override;
    void UpdateGUI() override;

  private:
    void TxToneToggle(wxCommandEvent& event);
    std::unique_ptr<DCCorrectorsPanel> rxdcpanel;
    std::unique_ptr<DCCorrectorsPanel> txdcpanel;

    std::unique_ptr<QECPanel> rxqecpanel;
    std::unique_ptr<QECPanel> txqecpanel;

    lime::LA9310* la9310;
};
