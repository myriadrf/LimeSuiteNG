#ifndef WIDGET_OVERSAMPLER_H
#define WIDGET_OVERSAMPLER_H

#include <cstdint>
#include <memory>

#include <wx/panel.h>

class wxChoice;
class wxCommandEvent;

namespace lime {
class IOversampler;
}

/// @brief Controls for a lime::IOversampler, offering oversampling factors of 2^0 to 2^maxOversamplePow2.
class OversamplerPanel : public wxPanel
{
  public:
    OversamplerPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = wxT("Oversample"),
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~OversamplerPanel();

    /// @param dev Not owned, must outlive the panel or be replaced by another Initialize() call.
    void Initialize(lime::IOversampler* dev, uint32_t maxOversamplePow2 = 5);
    /// @param dev Kept alive by the panel.
    void Initialize(std::shared_ptr<lime::IOversampler> dev, uint32_t maxOversamplePow2 = 5);
    /// @brief Reads the value back from the device.
    void UpdateGUI();

  private:
    void OnSelect(wxCommandEvent& event);

    wxChoice* cmbOversample;
    lime::IOversampler* device;
    std::shared_ptr<lime::IOversampler> deviceOwner;
};

#endif
