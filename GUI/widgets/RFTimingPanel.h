#ifndef WIDGET_RFTIMING_H
#define WIDGET_RFTIMING_H

#include <string>
#include <vector>

#include "ISOCPanel.h"

class wxChoice;
class wxFlexGridSizer;
class wxSpinCtrl;
class wxStaticBoxSizer;

namespace lime {
class IRFTiming;
}

/// @brief Device tree panel for lime::IRFTiming nodes: start/end offset and action of each RF front end control.
class RFTimingPanel : public ISOCPanel
{
  public:
    static ISOCPanel* Create(wxWindow* parent, wxWindowID id);
    RFTimingPanel(wxWindow* parent,
        wxWindowID id,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~RFTimingPanel();
    bool Initialize(lime::IRFTiming* soc);
    bool Initialize(void* soc) override;
    void UpdateGUI() override;

  private:
    struct ControlRow {
        std::string name;
        bool sharedOffset;
        wxSpinCtrl* startOffset;
        wxChoice* startAction;
        wxSpinCtrl* endOffset;
        wxChoice* endAction;
    };

    void WriteControl(ControlRow& row);
    void OnRead(wxCommandEvent& event);

    lime::IRFTiming* timing;
    std::vector<ControlRow> rows;
    wxStaticBoxSizer* timingBox;
    wxFlexGridSizer* controlsGrid;
};

#endif
