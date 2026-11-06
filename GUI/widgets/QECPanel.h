#ifndef WIDGET_QECCORRECTOR_H
#define WIDGET_QECCORRECTOR_H

#include <memory>

#include <wx/panel.h>

class wxSpinCtrlDouble;
class wxStaticText;
class wxWindow;

namespace lime {
class IQuadratureErrorCorrector;
}

/// @brief Controls for a lime::IQuadratureErrorCorrector. The correction is written as soon as it is changed.
class QECPanel : public wxPanel
{
  public:
    QECPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = wxT("QEC"),
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = 0);
    ~QECPanel();
    /// @param dev Shared ownership is kept by the panel.
    void Initialize(std::shared_ptr<lime::IQuadratureErrorCorrector> dev);
    /// @param dev Not owned, must outlive the panel or be replaced by another Initialize() call.
    void Initialize(lime::IQuadratureErrorCorrector* dev);
    /// @brief Reads the correction back from the device. Keeps the shown values if the device can not read it.
    void UpdateGUI();

  private:
    void WriteValues();

    wxWindow* box;
    wxSpinCtrlDouble* gainImbalance;
    wxSpinCtrlDouble* phaseImbalance;
    wxStaticText* lblGainRange;
    wxStaticText* lblPhaseRange;
    std::shared_ptr<lime::IQuadratureErrorCorrector> owner;
    lime::IQuadratureErrorCorrector* device{};
};

#endif
