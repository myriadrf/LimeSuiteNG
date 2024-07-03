#include "pnlLimeSDR.h"
#include "events.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "pnlGPIO.h"

#include <wx/sizer.h>
#include <wx/checkbox.h>
#include <wx/statbox.h>
#include <wx/choice.h>
#include <wx/msgdlg.h>

using namespace lime;
using namespace std;

BEGIN_EVENT_TABLE(pnlLimeSDR, wxPanel)
END_EVENT_TABLE()

pnlLimeSDR::pnlLimeSDR(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, int style, wxString name)
    : controlsSizer(new wxFlexGridSizer(0, 2, 5, 5))
    , mainSizer(new wxFlexGridSizer(0, 2, 5, 5))
    , fpgaSelect(-1)
    , device(nullptr)
{
    Create(parent, id, pos, size, style, name);
#ifdef WIN32
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
#endif

    SetSizer(mainSizer);
    chkRFLB_A_EN = new wxCheckBox(this, wxNewId(), _("RF loopback ch.A"));
    chkRFLB_A_EN->SetToolTip(_("[RFLB_A_EN] External RF loopback TxBAND2->RxLNAH channel A"));
    Connect(chkRFLB_A_EN->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    controlsSizer->Add(chkRFLB_A_EN, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    chkRFLB_B_EN = new wxCheckBox(this, wxNewId(), _("RF loopback ch.B"));
    chkRFLB_B_EN->SetToolTip(_("[RFLB_B_EN] External RF loopback TxBAND2->RxLNAH channel B"));
    Connect(chkRFLB_B_EN->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    controlsSizer->Add(chkRFLB_B_EN, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    chkTX1_2_LB_SH = new wxCheckBox(this, wxNewId(), _("Ch.A shunt"));
    chkTX1_2_LB_SH->SetToolTip(_("[TX1_2_LB_SH]"));
    Connect(chkTX1_2_LB_SH->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    controlsSizer->Add(chkTX1_2_LB_SH, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    chkTX2_2_LB_SH = new wxCheckBox(this, wxNewId(), _("Ch.B shunt"));
    chkTX2_2_LB_SH->SetToolTip(_("[TX2_2_LB_SH]"));
    Connect(chkTX2_2_LB_SH->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    controlsSizer->Add(chkTX2_2_LB_SH, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    chkTX1_2_LB_AT = new wxCheckBox(this, wxNewId(), _("Ch.A attenuator"));
    chkTX1_2_LB_AT->SetToolTip(_("[TX1_2_LB_AT]"));
    Connect(chkTX1_2_LB_AT->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    controlsSizer->Add(chkTX1_2_LB_AT, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    chkTX2_2_LB_AT = new wxCheckBox(this, wxNewId(), _("Ch.B attenuator"));
    chkTX2_2_LB_AT->SetToolTip(_("[TX2_2_LB_AT]"));
    Connect(chkTX2_2_LB_AT->GetId(), wxEVT_CHECKBOX, wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    controlsSizer->Add(chkTX2_2_LB_AT, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    auto groupSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("External loopback controls")), wxVERTICAL);
    groupSizer->Add(controlsSizer, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    mainSizer->Add(groupSizer, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
    pnl_gpio = new pnlGPIO(this, wxNewId());
    mainSizer->Add(pnl_gpio, 1, wxEXPAND | wxALL, 5);
    Bind(READ_ALL_VALUES, &pnlLimeSDR::OnReadAll, this, GetId());
    Bind(WRITE_ALL_VALUES, &pnlLimeSDR::OnWriteAll, this, GetId());
}

void pnlLimeSDR::Initialize(lime::SDRDevice* pControl)
{
    device = pControl;
    if (device)
    {
        auto controls = controlsSizer->GetChildren();
        for (auto i : controls)
            i->GetWindow()->Enable();

        for (const auto& id : device->GetDescriptor().spiSlaveIds)
        {
            if (id.first == "FPGA"s)
            {
                fpgaSelect = id.second;
                break;
            }
        }
    }
    else
        fpgaSelect = -1;

    pnl_gpio->Initialize(device);
    mainSizer->Fit(this);
    mainSizer->SetSizeHints(this);
    Layout();
}

void pnlLimeSDR::OnGPIOChange(wxCommandEvent& event)
{
    uint16_t addr = 0x0017;
    uint16_t value = 0;
    value |= chkRFLB_A_EN->GetValue() << 0;
    value |= chkTX1_2_LB_AT->GetValue() << 1;
    value |= chkTX1_2_LB_SH->GetValue() << 2;

    value |= chkRFLB_B_EN->GetValue() << 4;
    value |= chkTX2_2_LB_AT->GetValue() << 5;
    value |= chkTX2_2_LB_SH->GetValue() << 6;

    uint32_t mosi = (1 << 31) | addr << 16 | value;
    if (device)
    {
        try
        {
            device->SPI(fpgaSelect, &mosi, nullptr, 1);
        } catch (...)
        {
            wxMessageBox(_("Board loopback change failed"), _("Error"), wxICON_ERROR | wxOK);
        }
    }
}

pnlLimeSDR::~pnlLimeSDR()
{
    chkRFLB_A_EN->Disconnect(wxEVT_CHECKBOX, chkRFLB_A_EN->GetId(), wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    chkRFLB_B_EN->Disconnect(wxEVT_CHECKBOX, chkRFLB_B_EN->GetId(), wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    chkTX1_2_LB_SH->Disconnect(
        wxEVT_CHECKBOX, chkTX1_2_LB_SH->GetId(), wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    chkTX1_2_LB_AT->Disconnect(
        wxEVT_CHECKBOX, chkTX1_2_LB_AT->GetId(), wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    chkTX2_2_LB_SH->Disconnect(
        wxEVT_CHECKBOX, chkTX2_2_LB_SH->GetId(), wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
    chkTX2_2_LB_AT->Disconnect(
        wxEVT_CHECKBOX, chkTX2_2_LB_AT->GetId(), wxCommandEventHandler(pnlLimeSDR::OnGPIOChange), nullptr, this);
}

void pnlLimeSDR::UpdatePanel()
{
    uint16_t addr = 0x0017;
    uint16_t value = 0;

    uint32_t mosi = addr;
    uint32_t miso = 0;
    if (device)
    {
        try
        {
            device->SPI(fpgaSelect, &mosi, &miso, 1);
            value = miso & 0xFFFF;
            chkRFLB_A_EN->SetValue((value >> 0) & 0x1);
            chkTX1_2_LB_AT->SetValue((value >> 1) & 0x1);
            chkTX1_2_LB_SH->SetValue((value >> 2) & 0x1);

            chkRFLB_B_EN->SetValue((value >> 4) & 0x1);
            chkTX2_2_LB_AT->SetValue((value >> 5) & 0x1);
            chkTX2_2_LB_SH->SetValue((value >> 6) & 0x1);
            pnl_gpio->UpdatePanel();
        } catch (...)
        {
            wxMessageBox(_("Board loopback read failed"), _("Error"), wxICON_ERROR | wxOK);
            return;
        }
    }
}

void pnlLimeSDR::OnReadAll(wxCommandEvent& event)
{
    UpdatePanel();
}

void pnlLimeSDR::OnWriteAll(wxCommandEvent& event)
{
    OnGPIOChange(event);
    pnl_gpio->OnUsrGPIODirChange(event);
    pnl_gpio->OnUsrGPIOChange(event);
}
