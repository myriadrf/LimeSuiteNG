#pragma once

#include <wx/panel.h>
#include <cstdint>

class wxBoxSizer;
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

    std::size_t EnumerateDevicesToChoice();
    void OnDeviceHotplug(wxCommandEvent& inEvent);

    static void OnDeviceHotplugCallback(void* data);

    wxChoice* cmbDevHandle;
    wxButton* btnDisconnect;
    wxBoxSizer* szBox;
};

} // namespace lime