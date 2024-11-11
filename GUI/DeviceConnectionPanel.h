#pragma once

#include <wx/panel.h>
#include <cstdint>

class wxButton;
class wxChoice;

namespace lime {

class DeviceConnectionPanel : public wxPanel
{
  public:
    DeviceConnectionPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);
    ~DeviceConnectionPanel();
    void SetSelection(uint32_t index);

  protected:
    void SendDisconnectEvent(wxCommandEvent& inEvent);
    void SendHandleChangeEvent(wxCommandEvent& inEvent);
    void EnumerateDevicesToChoice(wxCommandEvent& inEvent);

    wxChoice* cmbDevHandle;
    wxButton* btnEnumerate;
    wxButton* btnDisconnect;
};

} // namespace lime