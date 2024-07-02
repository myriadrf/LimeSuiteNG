#include "SPI_wxgui.h"
#include <vector>

#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/Logger.h"

#include "numericSlider/numericSlider.h"

#include <wx/spinctrl.h>

using namespace lime;
using namespace std::literals::string_literals;

void SPI_wxgui::InsertSPIControlsRow(wxWindow* parent, wxWindowID id, wxFlexGridSizer* row, SPI_wxgui::SPIFields* controls)
{
    wxStaticText* addrText = new wxStaticText(parent, wxID_ANY, wxT("Address(Hex):"));
    row->Add(addrText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxTextCtrl* txtLMSwriteAddr = new wxTextCtrl(parent, wxID_ANY, wxT("FFFF"));
    //txtLMSwriteAddr->SetMinSize( wxSize( 48,-1 ) );
    controls->address = txtLMSwriteAddr;

    row->Add(txtLMSwriteAddr, 1, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    wxStaticText* valueText = new wxStaticText(parent, wxID_ANY, wxT("Value(Hex):"));
    row->Add(valueText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxTextCtrl* txtLMSwriteValue = new wxTextCtrl(parent, wxID_ANY, wxT("FFFF"));
    //txtLMSwriteValue->SetMinSize( wxSize( 48,-1 ) );
    row->Add(txtLMSwriteValue, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);
    controls->value = txtLMSwriteValue;

    wxButton* btnLMSwrite = new wxButton(parent, id, wxT("Write"));
    btnLMSwrite->SetDefault();
    row->Add(btnLMSwrite, 0, wxALIGN_CENTER_VERTICAL, 0);
    btnLMSwrite->Bind(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(SPI_wxgui::onSPIwrite), this, id);

    wxButton* btnLMSread = new wxButton(parent, id, wxT("Read"));
    btnLMSread->SetDefault();
    row->Add(btnLMSread, 0, wxALIGN_CENTER_VERTICAL, 0);
    btnLMSread->Bind(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(SPI_wxgui::onSPIread), this, id);

    wxStaticText* statusText = new wxStaticText(parent, wxID_ANY, wxT("Status:"));
    row->Add(statusText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxStaticText* lblLMSwriteStatus = new wxStaticText(parent, wxID_ANY, wxT("???"), wxDefaultPosition, wxSize(134, 13));
    row->Add(lblLMSwriteStatus, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    controls->status = lblLMSwriteStatus;
}

void SPI_wxgui::InsertQuickSPIControlsRow(
    wxWindow* parent, wxWindowID id, wxFlexGridSizer* row, SPI_wxgui::QuickSPIFields* controls)
{
    wxStaticText* addrText = new wxStaticText(parent, wxID_ANY, wxT("Address(Hex):"));
    row->Add(addrText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxTextCtrl* txtLMSwriteAddr = new wxTextCtrl(parent, wxID_ANY, wxT("FFFF"));
    //txtLMSwriteAddr->SetMinSize( wxSize( 48,-1 ) );
    controls->address = txtLMSwriteAddr;
    row->Add(txtLMSwriteAddr, 1, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    wxStaticText* maskText = new wxStaticText(parent, wxID_ANY, wxT("Mask(Hex):"));
    row->Add(maskText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxTextCtrl* txtMask = new wxTextCtrl(parent, wxID_ANY, wxT("FFFF"));
    row->Add(txtMask, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);
    controls->mask = txtMask;

    NumericSlider* ctrlValue =
        new NumericSlider(parent, id, wxEmptyString, wxDefaultPosition, wxSize(-1, -1), wxSP_ARROW_KEYS, -32768, 32767, 0);
    ctrlValue->Bind(wxEVT_COMMAND_SPINCTRL_UPDATED, &SPI_wxgui::onQuickSPIwrite, this, id);
    controls->value = ctrlValue;
    row->Add(ctrlValue, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxStaticText* statusText = new wxStaticText(parent, wxID_ANY, wxT("Written:"));
    row->Add(statusText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxStaticText* lblLMSwriteStatus = new wxStaticText(parent, wxID_ANY, wxT("???"), wxDefaultPosition, wxSize(134, 13));
    row->Add(lblLMSwriteStatus, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    controls->status = lblLMSwriteStatus;
}

wxFlexGridSizer* SPI_wxgui::CreateSPIControls(wxWindow* parent, uint8_t rowCount, wxChoice* deviceSelector)
{
    wxFlexGridSizer* mainSizer;
    mainSizer = new wxFlexGridSizer(0, 1, 0, 0);
    mainSizer->SetFlexibleDirection(wxBOTH);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    mainSizer->Add(deviceSelector, 0, wxALIGN_LEFT, 0);

    wxFlexGridSizer* szRows = new wxFlexGridSizer(0, 8, 0, 0);
    szRows->SetFlexibleDirection(wxBOTH);
    szRows->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);
    for (int i = 0; i < rowCount; ++i)
    {
        SPI_wxgui::SPIFields controls{};
        wxWindowID lineID = wxNewId();
        InsertSPIControlsRow(parent, lineID, szRows, &controls);
        controls.devSelection = deviceSelector;
        mSPIElements[lineID] = controls;
    }
    mainSizer->Add(szRows, 0, wxEXPAND, 0);

    szRows = new wxFlexGridSizer(0, 7, 0, 0);
    szRows->SetFlexibleDirection(wxBOTH);
    szRows->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);
    for (int i = 0; i < 2; ++i)
    {
        SPI_wxgui::QuickSPIFields controls{};
        wxWindowID lineID = wxNewId();
        InsertQuickSPIControlsRow(parent, lineID, szRows, &controls);
        controls.devSelection = deviceSelector;
        mQuickSPIElements[lineID] = controls;
    }
    mainSizer->Add(szRows, 0, wxEXPAND, 0);
    return mainSizer;
}

SPI_wxgui::SPI_wxgui(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long styles)
    : IModuleFrame(parent, id, title, pos, size, styles)
{
    mDevice = nullptr;

    wxFlexGridSizer* mainSizer;
    mainSizer = new wxFlexGridSizer(0, 1, 0, 0);
    mainSizer->SetFlexibleDirection(wxBOTH);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    mSPIselection.push_back(new wxChoice(this, wxNewId()));
    wxFlexGridSizer* spiControls = CreateSPIControls(this, 4, mSPIselection.back());
    mainSizer->Add(spiControls, 0, wxALIGN_LEFT, 0);

    mSPIselection.push_back(new wxChoice(this, wxNewId()));
    spiControls = CreateSPIControls(this, 4, mSPIselection.back());
    mainSizer->Add(spiControls, 0, wxALIGN_LEFT, 0);

    SetSizer(mainSizer);
    Layout();
    mainSizer->Fit(this);

    Centre(wxBOTH);
}

bool SPI_wxgui::Initialize(SDRDevice* pCtrPort)
{
    mDevice = pCtrPort;
    if (mDevice == nullptr)
    {
        wxArrayString emptyList;
        emptyList.Add("No comms");
        for (auto iter : mSPIselection)
        {
            if (iter)
                iter->Set(emptyList);
        }
        return false;
    }

    wxArrayString spiSlavesList;
    const SDRDescriptor& desc = mDevice->GetDescriptor();

    for (const auto& nameIds : desc.spiSlaveIds)
        spiSlavesList.Add(nameIds.first);

    for (auto iter : mSPIselection)
        if (iter)
            iter->Set(spiSlavesList);

    // by default for first SPI section select LMS chip
    if (mSPIselection[0])
    {
        bool found = mSPIselection[0]->SetStringSelection(_("LMS7002M"));
        if (!found)
            found = mSPIselection[0]->SetStringSelection(_("LMS7002M_1"));
    }

    // by default for second SPI section select FPGA
    if (mSPIselection[1])
        mSPIselection[1]->SetStringSelection(_("FPGA"));

    return true;
}

void SPI_wxgui::onSPIwrite(wxCommandEvent& event)
{
    try
    {
        SPIFields& fields = mSPIElements.at(event.GetId());
        if (!mDevice)
        {
            fields.status->SetLabel("Not connected");
            return;
        }

        const wxString strAddress = fields.address->GetValue();
        long addr = 0;
        strAddress.ToLong(&addr, 16);

        const wxString strValue = fields.value->GetValue();
        long value = 0;
        strValue.ToLong(&value, 16);

        uint32_t devAddr = 0;
        int listSelection = fields.devSelection->GetSelection();
        if (listSelection < 0)
            return;
        const wxString strDevAddr = fields.devSelection->GetString(listSelection);
        const SDRDescriptor& desc = mDevice->GetDescriptor();
        auto iter = desc.spiSlaveIds.find(std::string(strDevAddr.mb_str()));
        if (iter == desc.spiSlaveIds.end())
        {
            lime::warning("Connected device does not have SPI for "s + strDevAddr.mb_str().data());
            return;
        }
        devAddr = iter->second;

        const uint32_t mosi = (1 << 31) | addr << 16 | value;

        try
        {
            mDevice->SPI(devAddr, &mosi, nullptr, 1);
            fields.status->SetLabel("OK");
        } catch (std::runtime_error& e)
        {
            fields.status->SetLabel(e.what());
        }
    } catch (...)
    {
        lime::error("No spi controls created for event id: %i", event.GetId());
    }
}

void SPI_wxgui::onQuickSPIwrite(wxSpinEvent& event)
{
    try
    {
        QuickSPIFields& fields = mQuickSPIElements.at(event.GetId());
        if (!mDevice)
        {
            fields.status->SetLabel("Not connected");
            return;
        }

        const wxString strAddress = fields.address->GetValue();
        long addr = 0;
        strAddress.ToLong(&addr, 16);

        const wxString strMask = fields.mask->GetValue();
        long mask = 0;
        strMask.ToLong(&mask, 16);

        int lsb = 0;
        for (int i = 0; i < 15; ++i)
        {
            if (mask & (1 << i))
            {
                lsb = i;
                break;
            }
        }

        long value = fields.value->GetValue();

        uint32_t devAddr = 0;
        int listSelection = fields.devSelection->GetSelection();
        if (listSelection < 0)
            return;
        const wxString strDevAddr = fields.devSelection->GetString(listSelection);
        const SDRDescriptor& desc = mDevice->GetDescriptor();
        auto iter = desc.spiSlaveIds.find(std::string(strDevAddr.mb_str()));
        if (iter == desc.spiSlaveIds.end())
        {
            lime::warning("Connected device does not have SPI for "s + strDevAddr.mb_str().data());
            return;
        }
        devAddr = iter->second;

        try
        {
            uint32_t read_mosi = addr;
            uint32_t regValue = 0;
            mDevice->SPI(devAddr, &read_mosi, &regValue, 1);

            regValue &= 0xFFFF;
            regValue &= ~mask;
            regValue |= ((value << lsb) & mask);

            const uint32_t mosi = (1 << 31) | addr << 16 | regValue;
            mDevice->SPI(devAddr, &mosi, nullptr, 1);
            fields.status->SetLabel(wxString::Format("%04X", regValue));
        } catch (std::runtime_error& e)
        {
            fields.status->SetLabel(e.what());
        }
    } catch (...)
    {
        lime::error("No spi controls created for event id: %i", event.GetId());
    }
}

void SPI_wxgui::onSPIread(wxCommandEvent& event)
{
    try
    {
        SPIFields& fields = mSPIElements.at(event.GetId());
        if (!mDevice)
        {
            fields.status->SetLabel("Not connected");
            return;
        }

        const wxString strAddress = fields.address->GetValue();
        long addr = 0;
        strAddress.ToLong(&addr, 16);

        const wxString strValue = fields.value->GetValue();
        long value = 0;
        strValue.ToLong(&value, 16);

        uint32_t devAddr = 0;
        int listSelection = fields.devSelection->GetSelection();
        if (listSelection < 0)
            return;
        const wxString strDevAddr = fields.devSelection->GetString(listSelection);
        const SDRDescriptor& desc = mDevice->GetDescriptor();
        auto iter = desc.spiSlaveIds.find(std::string(strDevAddr.mb_str()));
        if (iter == desc.spiSlaveIds.end())
        {
            lime::warning("Connected device does not have SPI for "s + strDevAddr.mb_str().data());
            return;
        }
        devAddr = iter->second;

        const uint32_t mosi = addr;
        uint32_t miso = 0;

        try
        {
            mDevice->SPI(devAddr, &mosi, &miso, 1);
            fields.status->SetLabel("OK");
            fields.value->SetValue(wxString::Format("%04X", miso));
        } catch (std::runtime_error& e)
        {
            fields.status->SetLabel(e.what());
        }
    } catch (...)
    {
        lime::error("No spi controls created for event id: %i", event.GetId());
    }
}
