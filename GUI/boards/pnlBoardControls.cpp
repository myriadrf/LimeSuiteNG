#include "pnlBoardControls.h"
#include <wx/wxprec.h>
#ifdef __BORLANDC__
    #pragma hdrstop
#endif //__BORLANDC__

#include <wx/spinctrl.h>
#include <wx/msgdlg.h>

#include "lime/LimeSuite.h"
#include "pnluLimeSDR.h"
#include "pnlLimeSDR.h"
#include "pnlBuffers.h"
#include "pnlX3.h"
#include "pnlX8.h"
#include "pnlXTRX.h"

#include "protocols/ADCUnits.h"
#include <cassert>
#include <vector>
#include "lms7suiteEvents.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "utilities/toString.h"

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

// wrapper class for assigning user data to wxWidgets event handling
struct UserDataContainer : public wxObject {
    UserDataContainer(void* ptr)
        : wxObject()
        , ptr(ptr)
    {
    }
    void* ptr;
};

static wxString power2unitsString(int powerx3)
{
    switch (powerx3)
    {
    case -8:
        return "y";
    case -7:
        return "z";
    case -6:
        return "a";
    case -5:
        return "f";
    case -4:
        return "p";
    case -3:
        return "n";
    case -2:
        return "u";
    case -1:
        return "m";
    case 0:
        return "";
    case 1:
        return "k";
    case 2:
        return "M";
    case 3:
        return "G";
    case 4:
        return "T";
    case 5:
        return "P";
    case 6:
        return "E";
    case 7:
        return "Y";
    default:
        return "";
    }
}

static OpStatus ReadCustomBoardParam(SDRDevice* device, std::vector<CustomParameterIO>& parameters)
{
    if (device == nullptr)
        return OpStatus::IOFailure;
    return device->CustomParameterRead(parameters);
}

static OpStatus WriteCustomBoardParam(SDRDevice* device, const std::vector<CustomParameterIO>& parameters)
{
    if (device == nullptr)
        return OpStatus::IOFailure;

    return device->CustomParameterWrite(parameters);
}

std::vector<pnlBoardControls::ADC_DAC> pnlBoardControls::mParameters;
const std::vector<eLMS_DEV> pnlBoardControls::board_list = { LMS_DEV_UNKNOWN,
    LMS_DEV_LIMESDR,
    LMS_DEV_LIMESDR_PCIE,
    LMS_DEV_LIMESDR_QPCIE,
    LMS_DEV_LIMESDRMINI,
    LMS_DEV_LIMESDRMINI_V2,
    LMS_DEV_LIMESDR_X3,
    LMS_DEV_LIMESDR_XTRX,
    LMS_DEV_LIMESDR_MMX8 };

pnlBoardControls::pnlBoardControls(
    wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
    : IModuleFrame(parent, id, title, pos, size, style)
    , additionalControls(nullptr)
    , txtDACTitle(nullptr)
    , txtDACValue(nullptr)
    , btnDAC(nullptr)
    , sizerDAC(nullptr)
{
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    wxFlexGridSizer* fgSizer247;
    fgSizer247 = new wxFlexGridSizer(0, 1, 10, 10);
    fgSizer247->AddGrowableCol(0);
    fgSizer247->AddGrowableRow(1);
    fgSizer247->SetFlexibleDirection(wxBOTH);
    fgSizer247->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer248;
    fgSizer248 = new wxFlexGridSizer(0, 4, 0, 0);
    fgSizer248->SetFlexibleDirection(wxBOTH);
    fgSizer248->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    btnReadAll = new wxButton(this, wxID_ANY, wxT("Read all"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer248->Add(btnReadAll, 0, wxALL, 5);

    btnWriteAll = new wxButton(this, wxID_ANY, wxT("Write all"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer248->Add(btnWriteAll, 0, wxALL, 5);

    m_staticText349 = new wxStaticText(this, wxID_ANY, wxT("Labels:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText349->Wrap(-1);
    fgSizer248->Add(m_staticText349, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

    wxArrayString cmbBoardSelectionChoices;
    cmbBoardSelection = new wxChoice(this, wxNewId(), wxDefaultPosition, wxDefaultSize, cmbBoardSelectionChoices, 0);
    cmbBoardSelection->SetSelection(0);
    fgSizer248->Add(cmbBoardSelection, 0, wxALL, 5);

    for (const auto& board : board_list)
    {
        cmbBoardSelection->AppendString(std::string{ GetDeviceName(board) });
    }

    fgSizer247->Add(fgSizer248, 1, wxEXPAND, 5);

    pnlCustomControls = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, _("Custom controls"));
    wxFlexGridSizer* sizerCustomControls = new wxFlexGridSizer(0, 5, 5, 5);

    sizerCustomControls->Add(new wxStaticText(pnlCustomControls, wxID_ANY, _("ID")));
    sizerCustomControls->Add(new wxStaticText(pnlCustomControls, wxID_ANY, _("Value")));
    sizerCustomControls->Add(new wxStaticText(pnlCustomControls, wxID_ANY, _("Power")));
    sizerCustomControls->Add(new wxStaticText(pnlCustomControls, wxID_ANY, _("Units")));
    sizerCustomControls->Add(new wxStaticText(pnlCustomControls, wxID_ANY, _("")));

    //reading
    spinCustomChannelRd =
        new wxSpinCtrl(pnlCustomControls, wxNewId(), _("0"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 0);
    sizerCustomControls->Add(spinCustomChannelRd, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    txtCustomValueRd = new wxStaticText(pnlCustomControls, wxID_ANY, _("0"));
    sizerCustomControls->Add(txtCustomValueRd, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    txtCustomPowerOf10Rd = new wxStaticText(pnlCustomControls, wxID_ANY, _(""));
    sizerCustomControls->Add(txtCustomPowerOf10Rd, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    txtCustomUnitsRd = new wxStaticText(pnlCustomControls, wxID_ANY, _(""));
    sizerCustomControls->Add(txtCustomUnitsRd, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    btnCustomRd = new wxButton(pnlCustomControls, wxNewId(), _("Read"));
    btnCustomRd->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(pnlBoardControls::OnCustomRead), NULL, this);
    sizerCustomControls->Add(btnCustomRd, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);

    //writing
    spinCustomChannelWr = new wxSpinCtrl(
        pnlCustomControls, wxNewId(), _("0"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, 0, 255, 0);
    sizerCustomControls->Add(spinCustomChannelWr, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    spinCustomValueWr = new wxSpinCtrl(
        pnlCustomControls, wxNewId(), _(""), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER, 0, 65535, 0);
    sizerCustomControls->Add(spinCustomValueWr, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);
    spinCustomValueWr->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(pnlBoardControls::OnSetDACvalues), NULL, this);

    wxArrayString powerChoices;
    for (int i = -8; i <= 7; ++i)
        powerChoices.push_back(power2unitsString(i));
    cmbCustomPowerOf10Wr = new wxChoice(pnlCustomControls, wxNewId(), wxDefaultPosition, wxDefaultSize, powerChoices, 0);
    cmbCustomPowerOf10Wr->SetSelection(0);
    sizerCustomControls->Add(cmbCustomPowerOf10Wr, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);

    wxArrayString unitChoices;
    for (int i = 0; i < ADC_UNITS_COUNT; ++i) //add all defined units
        unitChoices.push_back(std::string{ adcUnits2string(i) });
    for (int i = ADC_UNITS_COUNT; i < ADC_UNITS_COUNT + 4; ++i) //add some options to use undefined units
        unitChoices.push_back(std::to_string(i));
    cmbCustomUnitsWr = new wxChoice(pnlCustomControls, wxNewId(), wxDefaultPosition, wxDefaultSize, unitChoices, 0);
    cmbCustomUnitsWr->SetSelection(0);
    sizerCustomControls->Add(cmbCustomUnitsWr, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);

    btnCustomWr = new wxButton(pnlCustomControls, wxNewId(), _("Write"));
    btnCustomWr->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(pnlBoardControls::OnCustomWrite), NULL, this);
    sizerCustomControls->Add(btnCustomWr, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 0);

    pnlCustomControls->SetSizer(sizerCustomControls);
    pnlCustomControls->Fit();

    fgSizer247->Add(pnlCustomControls, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer249 = new wxFlexGridSizer(0, 2, 5, 5);
    fgSizer249->AddGrowableCol(0);
    fgSizer249->AddGrowableCol(1);
    fgSizer249->SetFlexibleDirection(wxBOTH);
    fgSizer249->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    pnlReadControls = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, _("General"));
    wxStaticBoxSizer* sbSizer133 = new wxStaticBoxSizer(new wxStaticBox(pnlReadControls, wxID_ANY, wxT("General")), wxVERTICAL);
    sizerAnalogRd = new wxFlexGridSizer(0, 3, 2, 2);
    sizerAnalogRd->SetFlexibleDirection(wxBOTH);
    sizerAnalogRd->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);
    sizerAnalogRd->Add(new wxStaticText(pnlReadControls, wxID_ANY, _("Name")), 1, wxALL, 5);
    sizerAnalogRd->Add(new wxStaticText(pnlReadControls, wxID_ANY, _("Value")), 1, wxALL, 5);
    sizerAnalogRd->Add(new wxStaticText(pnlReadControls, wxID_ANY, _("Units")), 1, wxALL, 5);
    sbSizer133->Add(sizerAnalogRd, 1, wxEXPAND, 5);
    pnlReadControls->SetSizer(sbSizer133);
    pnlReadControls->Fit();
    pnlReadControls->Hide();
    fgSizer249->Add(pnlReadControls, 1, wxEXPAND, 5);

    pnlEEPROMControls = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    wxStaticBoxSizer* eepromBoxSizer =
        new wxStaticBoxSizer(new wxStaticBox(pnlEEPROMControls, wxID_ANY, wxT("Non Volatile memory")), wxVERTICAL);
    pnlEEPROMControls->SetSizer(eepromBoxSizer);
    EEPROMsizer = new wxFlexGridSizer(0, 4, 0, 0);
    eepromBoxSizer->Add(EEPROMsizer, 1, wxEXPAND | wxALL, 5);
    fgSizer249->Add(pnlEEPROMControls, 1, wxEXPAND, 5);

    fgSizer247->Add(fgSizer249, 1, wxEXPAND, 5);

    sizerAdditionalControls = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer247->Add(sizerAdditionalControls, 1, wxEXPAND, 5);
    SetSizer(fgSizer247);

    // Connect Events
    cmbBoardSelection->Connect(
        wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler(pnlBoardControls::OnUserChangedBoardType), NULL, this);
    btnReadAll->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(pnlBoardControls::OnReadAll), NULL, this);
    btnWriteAll->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(pnlBoardControls::OnWriteAll), NULL, this);

    SetupControls(GetDeviceName(LMS_DEV_UNKNOWN));

    Layout();
    fgSizer247->Fit(this);
}

pnlBoardControls::~pnlBoardControls()
{
    // Disconnect Events
    btnReadAll->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(pnlBoardControls::OnReadAll), NULL, this);
    btnWriteAll->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(pnlBoardControls::OnWriteAll), NULL, this);
    cmbBoardSelection->Disconnect(
        wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler(pnlBoardControls::OnUserChangedBoardType), NULL, this);

    for (auto& widget : mGUI_widgets)
    {
        delete widget;
        widget = nullptr;
    }

    for (auto& widget : mMemoryGUI_widgets)
    {
        delete widget;
        widget = nullptr;
    }
}

void pnlBoardControls::OnReadAll(wxCommandEvent& event)
{
    vector<uint8_t> ids;
    vector<double> values;
    vector<string> units;

    for (const auto& param : mParameters)
    {
        ids.push_back(param.channel);
        values.push_back(0);
        units.push_back(""s);
    }

    std::vector<CustomParameterIO> params;
    params.reserve(mParameters.size());

    for (size_t i = 0; i < mParameters.size(); ++i)
    {
        params.push_back({ mParameters[i].channel, 0, ""s });
    }

    OpStatus status = ReadCustomBoardParam(mDevice, params);
    if (status != OpStatus::Success)
    {
        wxMessageBox(_("Error reading board parameters"), _("Warning"));
        return;
    }

    for (size_t i = 0; i < mParameters.size(); ++i)
    {
        mParameters[i].units = params[i].units;
        mParameters[i].value = params[i].value;
    }

    if (additionalControls)
    {
        wxCommandEvent evt;
        evt.SetEventType(READ_ALL_VALUES);
        evt.SetId(additionalControls->GetId());
        wxPostEvent(additionalControls, evt);
    }
    if (mMemoryGUI_widgets.size() > 0)
    {
        for (auto& row : mMemoryGUI_widgets)
        {
            OpStatus status = ReadMemory(row);

            if (status != OpStatus::Success)
            {
                wxMessageBox(_("Memory read failed"), _("Error"));
                break;
            }
        }
        // uint16_t val;
        // TODO: LMS_VCTCXORead(mDevice, &val);
        // txtDACValue->SetValue(std::to_string(val));
    }

    Update();
}

void pnlBoardControls::OnWriteAll(wxCommandEvent& event)
{
    std::vector<CustomParameterIO> params;
    params.reserve(mParameters.size());

    for (size_t i = 0; i < mParameters.size(); ++i)
    {
        if (!mParameters[i].writable)
            continue;

        params.push_back({ mParameters[i].channel, mParameters[i].value, ""s });
    }

    OpStatus status = WriteCustomBoardParam(mDevice, params);
    if (status != OpStatus::Success)
    {
        wxMessageBox(_("Failed to write values"), _("Warning"));
        return;
    }

    if (additionalControls)
    {
        wxCommandEvent evt;
        evt.SetEventType(WRITE_ALL_VALUES);
        evt.SetId(additionalControls->GetId());
        wxPostEvent(additionalControls, evt);
    }
}

bool pnlBoardControls::Initialize(lime::SDRDevice* device)
{
    mDevice = device;
    if (mDevice == nullptr)
        return false;

    const SDRDescriptor& desc = mDevice->GetDescriptor();

    SetupControls(desc.name);
    wxCommandEvent evt;
    OnReadAll(evt);
    return true;
}

void pnlBoardControls::Update()
{
    assert(mParameters.size() == mGUI_widgets.size());
    for (size_t i = 0; i < mParameters.size(); ++i)
    {
        mGUI_widgets[i]->title->SetLabel(wxString(mParameters[i].name));
        if (mGUI_widgets[i]->wValue)
            mGUI_widgets[i]->wValue->SetValue(mParameters[i].value);
        else
            mGUI_widgets[i]->rValue->SetLabel(wxString::Format(_("%1.0f"), mParameters[i].value));
        mGUI_widgets[i]->units->SetLabelText(mParameters[i].units);
    }

    if (additionalControls)
    {
        wxCommandEvent evt;
        evt.SetEventType(READ_ALL_VALUES);
        evt.SetId(additionalControls->GetId());
        wxPostEvent(additionalControls, evt);
    }
}

std::vector<pnlBoardControls::ADC_DAC> pnlBoardControls::getBoardParams(std::string_view boardID)
{
    std::vector<ADC_DAC> paramList;
    if (!mDevice)
        return paramList;

    const SDRDescriptor& desc = mDevice->GetDescriptor();

    for (const auto& param : desc.customParameters)
        paramList.push_back(ADC_DAC{
            param.name, !param.readOnly, 0, param.id, std::string{ adcUnits2string(RAW) }, 0, param.minValue, param.maxValue });
    return paramList;
}

void pnlBoardControls::OnMemoryWrite(wxCommandEvent& event)
{
    UserDataContainer* ud = dynamic_cast<UserDataContainer*>(event.GetEventUserData());
    MemoryParamGUI* gui = static_cast<MemoryParamGUI*>(ud->ptr);
    long val = 0;
    gui->txtValue->GetValue().ToLong(&val);
    assert(size_t(gui->memoryRegion.size) <= sizeof(val));
    OpStatus rez = mDevice->MemoryWrite(gui->dataStorage, gui->memoryRegion, &val);
    if (rez != OpStatus::Success)
        wxMessageBox(_("Memory write failed"), _("Error"));
}

OpStatus pnlBoardControls::ReadMemory(MemoryParamGUI* gui)
{
    long val = 0;
    assert(sizeof(val) >= size_t(gui->memoryRegion.size));
    OpStatus rez = mDevice->MemoryRead(gui->dataStorage, gui->memoryRegion, &val);
    if (rez == OpStatus::Success)
        gui->txtValue->SetValue(std::to_string(val));
    return rez;
}

void pnlBoardControls::OnMemoryRead(wxCommandEvent& event)
{
    UserDataContainer* ud = dynamic_cast<UserDataContainer*>(event.GetEventUserData());
    MemoryParamGUI* gui = static_cast<MemoryParamGUI*>(ud->ptr);
    ReadMemory(gui);
}

void pnlBoardControls::OnDACWrite(wxCommandEvent& event)
{
    long val;
    txtDACValue->GetValue().ToLong(&val);
    // TODO: LMS_VCTCXOWrite(mDevice, val);
    OnReadAll(event);
}

void pnlBoardControls::SetupControls(const std::string_view boardID)
{

    if (additionalControls)
    {
        additionalControls->Destroy();
        additionalControls = nullptr;
    }

    if (txtDACTitle)
    {
        txtDACTitle->Destroy();
        txtDACTitle = nullptr;
    }

    if (txtDACValue)
    {
        txtDACValue->Destroy();
        txtDACValue = nullptr;
    }

    if (btnDAC)
    {
        btnDAC->Destroy();
        btnDAC = nullptr;
    }

    if (sizerDAC)
    {
        sizerAnalogRd->Remove(sizerDAC);
        sizerDAC = nullptr;
    }

    cmbBoardSelection->SetSelection(0);
    for (unsigned i = 0; i < board_list.size(); ++i)
    {
        if (boardID == GetDeviceName(board_list[i]))
        {
            cmbBoardSelection->SetSelection(i);
            break;
        }
    }

    if (cmbBoardSelection->GetSelection() == 0)
        pnlCustomControls->Show();
    else
        pnlCustomControls->Hide();

    for (auto& widget : mGUI_widgets)
        delete widget;
    mGUI_widgets.clear(); //delete previously existing controls

    if (cmbBoardSelection->GetSelection() != 0)
    {
        mParameters = getBoardParams(boardID); //update controls list by board type
        if (mParameters.size() != 0)
            pnlReadControls->Show();
        else
            pnlReadControls->Hide();

        for (size_t i = 0; i < mParameters.size(); ++i)
        {
            Param_GUI* gui = new Param_GUI();
            gui->title = new wxStaticText(pnlReadControls, wxID_ANY, wxString(mParameters[i].name));
            if (mParameters[i].writable)
            {
                gui->wValue = new wxSpinCtrl(pnlReadControls,
                    wxNewId(),
                    _(""),
                    wxDefaultPosition,
                    wxDefaultSize,
                    wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER,
                    mParameters[i].minValue,
                    mParameters[i].maxValue,
                    mParameters[i].minValue);
                gui->wValue->Connect(
                    wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(pnlBoardControls::OnSetDACvalues), NULL, this);
                gui->wValue->Connect(wxEVT_TEXT_ENTER, wxCommandEventHandler(pnlBoardControls::OnSetDACvaluesENTER), NULL, this);
            }
            else
                gui->rValue = new wxStaticText(pnlReadControls, wxID_ANY, _(""));
            gui->units = new wxStaticText(pnlReadControls, wxID_ANY, mParameters[i].units);
            mGUI_widgets.push_back(gui);

            sizerAnalogRd->Add(gui->title, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 5);
            if (mParameters[i].writable)
                sizerAnalogRd->Add(gui->wValue, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 5);
            else
                sizerAnalogRd->Add(gui->rValue, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 5);
            sizerAnalogRd->Add(gui->units, 1, wxLEFT | wxRIGHT | wxALIGN_CENTER_VERTICAL, 5);
        }
    }

    for (auto& widget : mMemoryGUI_widgets)
        delete widget;
    mMemoryGUI_widgets.clear();
    //pnlEEPROMControls->Hide();

    if (mDevice)
    {
        lime::SDRDescriptor desc = mDevice->GetDescriptor();
        for (const auto& mem : desc.memoryDevices)
        {
            const auto& regions = mem.second->regions;
            for (const auto& region : regions)
            {
                MemoryParamGUI* gui = new MemoryParamGUI();
                gui->title = new wxStaticText(pnlEEPROMControls, wxID_ANY, region.first);
                gui->txtValue = new wxTextCtrl(pnlEEPROMControls, wxNewId(), _("0"), wxDefaultPosition, wxDefaultSize);
#if wxUSE_TOOLTIPS
                wxString tooltip = wxString::Format(_("[%s]\n%s @ 0x%08X : %i Bytes"),
                    mem.first.c_str(),
                    region.first.c_str(),
                    region.second.address,
                    region.second.size);
                gui->txtValue->SetToolTip(tooltip);
#endif
                gui->btnRead = new wxButton(pnlEEPROMControls, wxNewId(), _("Read"), wxDefaultPosition, wxDefaultSize);
                gui->btnWrite = new wxButton(pnlEEPROMControls, wxNewId(), _("Write"), wxDefaultPosition, wxDefaultSize);
                gui->dataStorage = mem.second;
                gui->memoryRegion = region.second;

                UserDataContainer* userData = new UserDataContainer(gui); // gets deleted when Event handler is disconnected
                gui->btnRead->Connect(gui->btnRead->GetId(),
                    wxEVT_COMMAND_BUTTON_CLICKED,
                    wxCommandEventHandler(pnlBoardControls::OnMemoryRead),
                    userData,
                    this);
                userData = new UserDataContainer(gui); // gets deleted when Event handler is disconnected
                gui->btnWrite->Connect(gui->btnWrite->GetId(),
                    wxEVT_COMMAND_BUTTON_CLICKED,
                    wxCommandEventHandler(pnlBoardControls::OnMemoryWrite),
                    userData,
                    this);
                mMemoryGUI_widgets.push_back(gui);

                EEPROMsizer->Add(gui->title, 1, wxALIGN_CENTER_VERTICAL, 5);
                EEPROMsizer->Add(gui->txtValue, 1, wxALIGN_CENTER_VERTICAL, 5);
                EEPROMsizer->Add(gui->btnRead, 1, wxALIGN_CENTER_VERTICAL, 5);
                EEPROMsizer->Add(gui->btnWrite, 1, wxALIGN_CENTER_VERTICAL, 5);
                EEPROMsizer->Layout();
                pnlEEPROMControls->Show();
            }
        }
    }
    pnlEEPROMControls->Layout();
    EEPROMsizer->Fit(pnlEEPROMControls);

    sizerAnalogRd->Layout();

    if (boardID == GetDeviceName(LMS_DEV_LIMESDRMINI) || boardID == GetDeviceName(LMS_DEV_LIMESDRMINI_V2))
    {
        pnluLimeSDR* pnl = new pnluLimeSDR(this, wxNewId());
        pnl->Initialize(mDevice);
        additionalControls = pnl;
        sizerAdditionalControls->Add(additionalControls);
    }
    if (boardID == GetDeviceName(LMS_DEV_LIMESDR))
    //  || boardID == GetDeviceName(LMS_DEV_LIMESDR_PCIE))
    {
        pnlLimeSDR* pnl = new pnlLimeSDR(this, wxNewId());
        pnl->Initialize(mDevice);
        additionalControls = pnl;
        sizerAdditionalControls->Add(additionalControls);
    }
    else if (boardID == GetDeviceName(LMS_DEV_LIMESDR_X3))
    {
        pnlX3* pnl = new pnlX3(this, wxNewId());
        pnl->Initialize(mDevice);
        additionalControls = pnl;
        sizerAdditionalControls->Add(additionalControls);
    }
    else if (boardID == GetDeviceName(LMS_DEV_LIMESDR_XTRX))
    {
        pnlXTRX* pnl = new pnlXTRX(this, wxNewId());
        pnl->Initialize(mDevice);
        additionalControls = pnl;
        sizerAdditionalControls->Add(additionalControls);
    }
    else if (boardID == GetDeviceName(LMS_DEV_LIMESDR_MMX8))
    {
        pnlX8* pnl = new pnlX8(this, wxNewId());
        pnl->Initialize(mDevice);
        additionalControls = pnl;
        sizerAdditionalControls->Add(additionalControls);
    }
    if (additionalControls)
    {
        additionalControls->Fit();
        additionalControls->Layout();
    }
    sizerAdditionalControls->Layout();
    Layout();
    Fit();
}

void pnlBoardControls::OnSetDACvaluesENTER(wxCommandEvent& event)
{
    wxSpinEvent evt;
    evt.SetEventObject(event.GetEventObject());
    OnSetDACvalues(evt);
}

void pnlBoardControls::OnSetDACvalues(wxSpinEvent& event)
{
    for (size_t i = 0; i < mGUI_widgets.size(); ++i)
    {
        if (event.GetEventObject() == mGUI_widgets[i]->wValue)
        {
            mParameters[i].value = mGUI_widgets[i]->wValue->GetValue();
            if (mDevice == nullptr)
                return;

            OpStatus status = WriteCustomBoardParam(mDevice, { { mParameters[i].channel, mParameters[i].value, ""s } });
            if (status != OpStatus::Success)
                wxMessageBox(_("Failed to set value"), _("Warning"));
            return;
        }
    }
}

void pnlBoardControls::OnUserChangedBoardType(wxCommandEvent& event)
{
    SetupControls(GetDeviceName(board_list[cmbBoardSelection->GetSelection()]));
}

void pnlBoardControls::OnCustomRead(wxCommandEvent& event)
{
    uint8_t id = spinCustomChannelRd->GetValue();
    std::vector<CustomParameterIO> param{ { id, 0, ""s } };

    OpStatus status = ReadCustomBoardParam(mDevice, param);
    if (status != OpStatus::Success)
    {
        wxMessageBox(_("Failed to read value"), _("Warning"));
        return;
    }

    txtCustomUnitsRd->SetLabel(param[0].units);
    txtCustomValueRd->SetLabel(wxString::Format(_("%1.1f"), param[0].value));
}

void pnlBoardControls::OnCustomWrite(wxCommandEvent& event)
{
    uint8_t id = spinCustomChannelWr->GetValue();
    int powerOf10 = (cmbCustomPowerOf10Wr->GetSelection() - 8) * 3;

    double value = spinCustomValueWr->GetValue() * pow(10, powerOf10);

    OpStatus status =
        WriteCustomBoardParam(mDevice, { { id, value, std::string{ adcUnits2string(cmbCustomUnitsWr->GetSelection()) } } });
    if (status != OpStatus::Success)
    {
        wxMessageBox(_("Failed to write value"), _("Warning"));
        return;
    }
}
