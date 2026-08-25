#pragma once

#include <iostream>
#include <vector>

#include "ISOCPanel.h"

namespace lime {
class LA9310_IQStreamer;
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
    bool Initialize(lime::LA9310_IQStreamer* soc);
    bool Initialize(void* soc) override;
    void UpdateGUI() override;

  private:
    std::unique_ptr<DCCorrectorsPanel> rxdcpanel;
    std::unique_ptr<DCCorrectorsPanel> txdcpanel;

    std::unique_ptr<QECPanel> rxqecpanel;
    std::unique_ptr<QECPanel> txqecpanel;

    wxCheckBox* t11Trigger;
    wxCheckBox* txEnable;

    wxTextCtrl* txtVSPAAddr;
    wxTextCtrl* txtVSPAValue;
    wxCheckBox* chkAxiqEn;

    wxSpinCtrl* spinTxToneBin;
    wxCheckBox* chkTxToneGenerator;
    void onTxToneGeneratorClick(wxCommandEvent& event);

    wxCheckBox* chkDAC_IQ;
    void onPhytimer(wxCommandEvent& event);

    lime::LA9310_IQStreamer* iqstreamer;
};
