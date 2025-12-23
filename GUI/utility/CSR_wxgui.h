#ifndef __CSR_wxgui__
#define __CSR_wxgui__

#include "utilities_gui.h"
#include "IModuleFrame.h"
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace lime {
class SDRDevice;
class ICSR;
} // namespace lime

#include <wx/combobox.h>

class wxTextCtrl;
class wxChoice;
class wxStaticText;
class wxSpinEvent;

class NumericSlider;

class CSR_wxgui : public IModuleFrame
{
  protected:
    struct CSRFields {
        wxChoice* devSelection;
        wxTextCtrl* address;
        wxTextCtrl* value;
        wxStaticText* status;
    };

    // Handlers for SPI_view events.
    void onCSRwrite(wxCommandEvent& event);
    void onCSRread(wxCommandEvent& event);

    wxFlexGridSizer* CreateCsrControls(wxWindow* parent, uint8_t rowCount);
    void InsertCsrControlsRow(wxWindow* parent, wxWindowID id, wxFlexGridSizer* row, CSR_wxgui::CSRFields* controls);

  public:
    CSR_wxgui(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxString& title = "CSR",
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long styles = wxDEFAULT_DIALOG_STYLE);

    virtual bool Initialize(lime::SDRDevice* device);
    virtual void Update() {};

  protected:
    std::vector<wxChoice*> mCSRselection;
    std::unordered_map<wxWindowID, CSRFields> mCSRElements;
    lime::ICSR* CSR_interface;
};

#endif // __CSR_wxgui__
