#include "pnlsSDR.h"

#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/button.h>
#include <wx/string.h>
#include <wx/checkbox.h>
#include <wx/statbox.h>
#include <wx/spinctrl.h>
#include <wx/msgdlg.h>
#include "events.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"

#include <ciso646>

using namespace std;
using namespace std::literals::string_literals;
using namespace lime;

BEGIN_EVENT_TABLE(pnlsSDR, wxPanel)
END_EVENT_TABLE()

pnlsSDR::pnlsSDR(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, int style, wxString name)
    : chipSelect(-1)
    , device(nullptr)
{
    Create(parent, id, pos, size, style, name);
#ifdef WIN32
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
#endif
    wxFlexGridSizer* mainSizer = new wxFlexGridSizer(0, 2, 1, 1);

    mainSizer->Add(new wxStaticText(this, wxID_ANY, _("RFSW_TX ")), 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    cmbTxPath = new wxChoice(this, wxNewId());
    cmbTxPath->Append("HF");
    cmbTxPath->Append("LF");
    cmbTxPath->SetSelection(0);
    Connect(cmbTxPath->GetId(), wxEVT_CHOICE, wxCommandEventHandler(pnlsSDR::OnInputChange), nullptr, this);
    mainSizer->Add(cmbTxPath, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    mainSizer->Add(new wxStaticText(this, wxID_ANY, _("RFSW_RX ")), 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    cmbRxPath = new wxChoice(this, wxNewId());
    cmbRxPath->Append("HF");
    cmbRxPath->Append("LF");
    cmbRxPath->SetSelection(0);
    Connect(cmbRxPath->GetId(), wxEVT_CHOICE, wxCommandEventHandler(pnlsSDR::OnInputChange), nullptr, this);
    mainSizer->Add(cmbRxPath, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    // TDDCntrl = new wxCheckBox(this, wxNewId(), _("TDD controls switches"));
    // mainSizer->Add(TDDCntrl, 1, wxEXPAND | wxALIGN_RIGHT | wxALIGN_TOP, 5);
    // Connect(TDDCntrl->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(pnlsSDR::OnInputChange), nullptr, this);

    wxStaticBoxSizer* mainBoxSizer;
    mainBoxSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, "RF controls " + name), wxHORIZONTAL);
    mainBoxSizer->Add(mainSizer, 0, 0, 5);

    mainBoxSizer->Fit(this);
    mainBoxSizer->SetSizeHints(this);
    SetSizer(mainBoxSizer);
    Layout();
    Bind(READ_ALL_VALUES, &pnlsSDR::OnReadAll, this, GetId());
    Bind(WRITE_ALL_VALUES, &pnlsSDR::OnWriteAll, this, GetId());
}

void pnlsSDR::Initialize(lime::SDRDevice* dev, const string& spiSlaveName)
{
    chipSelect = -1;
    device = dev;
    if (!device)
        return;

    const SDRDescriptor& desc = device->GetDescriptor();
    for (const auto& nameIds : desc.spiSlaveIds)
    {
        if (nameIds.first == spiSlaveName)
        {
            chipSelect = nameIds.second;
            break;
        }
    }
}

pnlsSDR::~pnlsSDR()
{
}

void pnlsSDR::OnInputChange(wxCommandEvent& event)
{
    uint16_t addr = 0x000a;
    uint16_t value = 0;

    if (pnlsSDR::LMS_ReadFPGAReg(device, addr, &value))
        wxMessageBox(_("Failed to read FPGA registers"), _("Error"), wxICON_ERROR | wxOK);
    value &= ~(1 << 4 | 1 << 2);
    value |= cmbTxPath->GetSelection() << 4;
    value |= cmbRxPath->GetSelection() << 2;

    if (LMS_WriteFPGAReg(device, addr, value))
        wxMessageBox(_("Failed to write FPGA registers"), _("Error"), wxICON_ERROR | wxOK);
}

int pnlsSDR::LMS_WriteFPGAReg(lime::SDRDevice* device, uint32_t address, uint16_t val)
{
    if (!device)
        return -1;

    const uint32_t mosi = (1 << 31) | address << 16 | val;
    try
    {
        device->SPI(chipSelect, &mosi, nullptr, 1);
        return 0;
    } catch (...)
    {
        return -1;
    }
}

int pnlsSDR::LMS_ReadFPGAReg(lime::SDRDevice* device, uint32_t address, uint16_t* val)
{
    if (!device)
        return -1;
    const uint32_t mosi = address;
    uint32_t miso = 0;

    try
    {
        device->SPI(chipSelect, &mosi, &miso, 1);
        *val = miso & 0xFFFF;
        return 0;
    } catch (...)
    {
        return -1;
    }
}

void pnlsSDR::UpdatePanel()
{
    uint16_t addr = 0x000A;
    uint16_t value = 0;

    if (LMS_ReadFPGAReg(device, addr, &value))
    {
        wxMessageBox(_("Failed to read FPGA registers"), _("Error"), wxICON_ERROR | wxOK);
        return;
    }
    cmbTxPath->SetSelection((value >> 4) & 1);
    cmbRxPath->SetSelection((value >> 2) & 3);
}

void pnlsSDR::OnReadAll(wxCommandEvent& event)
{
    UpdatePanel();
}

void pnlsSDR::OnWriteAll(wxCommandEvent& event)
{
    OnInputChange(event);
}
