#include "pnlX8.h"

#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/button.h>
#include <wx/string.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/msgdlg.h>
#include "lms7suiteEvents.h"
#include "limesuiteng/SDRDevice.h"

#include "pnlXTRX.h"

#include <ciso646>

using namespace std;
using namespace lime;

BEGIN_EVENT_TABLE(pnlX8, wxPanel)
END_EVENT_TABLE()

pnlX8::pnlX8(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, int style, wxString name)
{
    Create(parent, id, pos, size, style, name);
#ifdef WIN32
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
#endif

    wxFlexGridSizer* submodulesSizer = new wxFlexGridSizer(4, 0, 0, 0);
    for (int i = 0; i < 8; ++i)
    {
        std::string spiSlaveName = "FPGA@"s + std::to_string(i + 1);
        pnlXTRX* pnl = new pnlXTRX(this, wxNewId(), wxDefaultPosition, wxDefaultSize, 0, spiSlaveName);
        subPanels.push_back(pnl);
        submodulesSizer->Add(pnl);
    }
    Layout();
    SetSizer(submodulesSizer);

    Bind(READ_ALL_VALUES, &pnlX8::OnReadAll, this, this->GetId());
    Bind(WRITE_ALL_VALUES, &pnlX8::OnWriteAll, this, this->GetId());
}

void pnlX8::Initialize(lime::SDRDevice* pControl)
{
    int i=0;
    for (auto& pnl : subPanels)
    {
        std::string spiSlaveName = "FPGA@"s + std::to_string(i + 1);
        pnl->Initialize(pControl, spiSlaveName);
        ++i;
    }
}

pnlX8::~pnlX8()
{
    Unbind(READ_ALL_VALUES, &pnlX8::OnReadAll, this, this->GetId());
    Unbind(WRITE_ALL_VALUES, &pnlX8::OnWriteAll, this, this->GetId());
}

void pnlX8::UpdatePanel()
{
    for (auto& pnl : subPanels)
        pnl->UpdatePanel();
}

void pnlX8::OnReadAll(wxCommandEvent& event)
{
    for (auto& pnl : subPanels)
        pnl->OnReadAll(event);
}

void pnlX8::OnWriteAll(wxCommandEvent& event)
{
    for (auto& pnl : subPanels)
        pnl->OnWriteAll(event);
}
