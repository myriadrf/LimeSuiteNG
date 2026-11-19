#pragma once

#include "ISOCPanel.h"

namespace lime {
class ITrimDAC;
} // namespace lime

class wxButton;
class wxSpinCtrl;
class wxStaticText;

/// @brief Panel for a trim DAC (lime::ITrimDAC): sets the current value and reads and stores the non-volatile one.
class TrimDAC_wxgui : public ISOCPanel
{
  public:
    static ISOCPanel* Create(wxWindow* parent, wxWindowID id);
    TrimDAC_wxgui(wxWindow* parent,
        wxWindowID id,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    bool Initialize(lime::ITrimDAC* soc);
    bool Initialize(void* soc) override;
    void UpdateGUI() override;

  private:
    void OnSet(wxCommandEvent& event);
    void OnRead(wxCommandEvent& event);
    void OnStore(wxCommandEvent& event);
    void OnLoadStored(wxCommandEvent& event);

    lime::ITrimDAC* dac;

    wxStaticText* lblName;
    wxStaticText* lblRange;
    wxSpinCtrl* spinValue;
    wxStaticText* lblValue;
    wxStaticText* lblStored;
    wxButton* btnStore;
    wxButton* btnLoadStored;
};
