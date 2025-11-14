#pragma once

#include <wx/panel.h>

class ISOCPanel : public wxPanel
{
  public:
    ISOCPanel(wxWindow* parent,
        wxWindowID id,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long styles = 0)
        : wxPanel(parent, id, pos, size, styles){};
    virtual ~ISOCPanel(){};

    virtual bool Initialize(void* soc) = 0;
    virtual void UpdateGUI() = 0;

  private:
    ISOCPanel() = delete;
};
