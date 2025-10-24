#include "pnlMicro.h"

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

BEGIN_EVENT_TABLE(pnlMicro, wxPanel)
END_EVENT_TABLE()

pnlMicro::pnlMicro(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, int style, wxString name)
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
    cmbTxPath->Append("Band 2");
    cmbTxPath->Append("Band 1");
    cmbTxPath->SetSelection(0);
    Connect(cmbTxPath->GetId(), wxEVT_CHOICE, wxCommandEventHandler(pnlMicro::OnInputChange), nullptr, this);
    mainSizer->Add(cmbTxPath, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    mainSizer->Add(new wxStaticText(this, wxID_ANY, _("RFSW_RX ")), 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    cmbRxPath = new wxChoice(this, wxNewId());
    cmbRxPath->Append("W Band");
    cmbRxPath->Append("L Band");
    cmbRxPath->Append("H Band");
    cmbRxPath->Append("-unconnected-");
    cmbRxPath->SetSelection(0);
    Connect(cmbRxPath->GetId(), wxEVT_CHOICE, wxCommandEventHandler(pnlMicro::OnInputChange), nullptr, this);
    mainSizer->Add(cmbRxPath, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);

    wxStaticBoxSizer* mainBoxSizer;
    mainBoxSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, "RF controls " + name), wxHORIZONTAL);
    mainBoxSizer->Add(mainSizer, 0, 0, 5);

    wxString rgrEXT_CLK_CTRLChoices[] = { wxT("CLK_XO"), wxT("CLK_IN") };
    int rgrEXT_CLK_CTRLChoices_count = sizeof(rgrEXT_CLK_CTRLChoices) / sizeof(wxString);
    rgrEXT_CLK_CTRL = new wxRadioBox(this,
        wxNewId(),
        wxT("EXT_CLK_CTRL"),
        wxDefaultPosition,
        wxDefaultSize,
        rgrEXT_CLK_CTRLChoices_count,
        rgrEXT_CLK_CTRLChoices,
        2,
        wxRA_SPECIFY_COLS);
    rgrEXT_CLK_CTRL->SetSelection(0);
    mainBoxSizer->Add(rgrEXT_CLK_CTRL, 0, 0, 5);
    Connect(rgrEXT_CLK_CTRL->GetId(),
        wxEVT_COMMAND_RADIOBOX_SELECTED,
        wxCommandEventHandler(pnlMicro::OnClockSourceChanged),
        nullptr,
        this);

    mainBoxSizer->Fit(this);
    mainBoxSizer->SetSizeHints(this);
    SetSizer(mainBoxSizer);
    Layout();
    Bind(READ_ALL_VALUES, &pnlMicro::OnReadAll, this, GetId());
    Bind(WRITE_ALL_VALUES, &pnlMicro::OnWriteAll, this, GetId());
}

void pnlMicro::Initialize(lime::SDRDevice* dev, const string& spiSlaveName)
{
    device = dev;
    if (!device)
        return;
}

pnlMicro::~pnlMicro()
{
}

void pnlMicro::OnInputChange(wxCommandEvent& event)
{
    const uint8_t i2c_expander_address = 0x20;
    uint8_t value = 0;
    device->I2CRead(0, i2c_expander_address, 0x19, 1, &value, 1);

    value &= ~(1 << 1); // clear TX_SW
    if (cmbTxPath->GetSelection() == 1)
        value |= (1 << 1); // set TX_SW, Band1
    else
        value |= (0 << 1); // set TX_SW, Band2

    value &= ~(0x5); // clear RX_SW2, RX_SW3
    uint8_t rxsw2 = 0;
    uint8_t rxsw3 = 0;
    switch (cmbRxPath->GetSelection())
    {
    case 0:
        rxsw2 = 0;
        rxsw3 = 0;
        break;
    case 1:
        rxsw2 = 0;
        rxsw3 = 1;
        break;
    case 2:
        rxsw2 = 1;
        rxsw3 = 0;
        break;
    default:
        rxsw2 = 1;
        rxsw3 = 1;
        break;
    }
    value |= (rxsw2 << 0) | (rxsw3 << 2); // set TX_SW, Band2
    device->I2CWrite(0, i2c_expander_address, 0x19, 1, &value, 1);
}

void pnlMicro::OnClockSourceChanged(wxCommandEvent& event)
{
    const uint8_t i2c_expander_address = 0x20;
    uint8_t gpioa = 0;
    device->I2CRead(0, i2c_expander_address, 0x09, 1, &gpioa, 1);
    gpioa &= ~(1 << 5);
    gpioa |= ((rgrEXT_CLK_CTRL->GetSelection() & 1) << 5);
    device->I2CWrite(0, i2c_expander_address, 0x09, 1, &gpioa, 1);
}

void pnlMicro::UpdatePanel()
{
    const uint8_t i2c_expander_address = 0x20;
    uint8_t value = 0;
    device->I2CRead(0, i2c_expander_address, 0x19, 1, &value, 1);
    cmbTxPath->SetSelection((value >> 1) & 1);
    int rx_sw_value = ((value >> 1) & 0x2) | (value & 1);
    int rx_sw_to_combobox[4] = { 0, 2, 1, 3 };
    cmbRxPath->SetSelection(rx_sw_to_combobox[rx_sw_value]);

    uint8_t gpioa = 0;
    device->I2CRead(0, i2c_expander_address, 0x09, 1, &gpioa, 1);
    rgrEXT_CLK_CTRL->SetSelection((gpioa >> 5) & 1);
}

void pnlMicro::OnReadAll(wxCommandEvent& event)
{
    UpdatePanel();
}

void pnlMicro::OnWriteAll(wxCommandEvent& event)
{
    OnInputChange(event);
}
