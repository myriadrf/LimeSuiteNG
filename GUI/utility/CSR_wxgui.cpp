#include "CSR_wxgui.h"
#include <vector>

#include "limesuiteng/SDRDevice.hpp"
#include "limesuiteng/SDRDescriptor.hpp"
#include "limesuiteng/Logger.h"
#include "limesuiteng/ToString.h"
#include "comms/ICSR.h"

#include "numericSlider/numericSlider.h"

#include <wx/spinctrl.h>

using namespace lime;
using namespace std::literals::string_literals;

void CSR_wxgui::InsertCsrControlsRow(wxWindow* parent, wxWindowID id, wxFlexGridSizer* row, CSR_wxgui::CSRFields* controls)
{
    wxStaticText* addrText = new wxStaticText(parent, wxID_ANY, wxT("Address(Hex):"));
    row->Add(addrText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxTextCtrl* txtLMSwriteAddr = new wxTextCtrl(parent, wxID_ANY, wxT("FFFFFFFFFFFFFFFF"));
    //txtLMSwriteAddr->SetMinSize( wxSize( 48,-1 ) );
    controls->address = txtLMSwriteAddr;

    row->Add(txtLMSwriteAddr, 1, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    wxStaticText* valueText = new wxStaticText(parent, wxID_ANY, wxT("Value(Hex):"));
    row->Add(valueText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxTextCtrl* txtLMSwriteValue = new wxTextCtrl(parent, wxID_ANY, wxT("FFFFFFFFFFFFFFFF"));
    //txtLMSwriteValue->SetMinSize( wxSize( 48,-1 ) );
    row->Add(txtLMSwriteValue, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);
    controls->value = txtLMSwriteValue;

    wxButton* btnLMSwrite = new wxButton(parent, id, wxT("Write"));
    btnLMSwrite->SetDefault();
    row->Add(btnLMSwrite, 0, wxALIGN_CENTER_VERTICAL, 0);
    btnLMSwrite->Bind(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(CSR_wxgui::onCSRwrite), this, id);

    wxButton* btnLMSread = new wxButton(parent, id, wxT("Read"));
    btnLMSread->SetDefault();
    row->Add(btnLMSread, 0, wxALIGN_CENTER_VERTICAL, 0);
    btnLMSread->Bind(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(CSR_wxgui::onCSRread), this, id);

    wxStaticText* statusText = new wxStaticText(parent, wxID_ANY, wxT("Status:"));
    row->Add(statusText, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    wxStaticText* lblLMSwriteStatus = new wxStaticText(parent, wxID_ANY, wxT("???"), wxDefaultPosition, wxSize(134, 13));
    row->Add(lblLMSwriteStatus, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
    controls->status = lblLMSwriteStatus;
}

wxFlexGridSizer* CSR_wxgui::CreateCsrControls(wxWindow* parent, uint8_t rowCount)
{
    wxFlexGridSizer* mainSizer;
    mainSizer = new wxFlexGridSizer(0, 1, 0, 0);
    mainSizer->SetFlexibleDirection(wxBOTH);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* szRows = new wxFlexGridSizer(0, 8, 0, 0);
    szRows->SetFlexibleDirection(wxBOTH);
    szRows->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);
    for (int i = 0; i < rowCount; ++i)
    {
        CSR_wxgui::CSRFields controls{};
        wxWindowID lineID = wxNewId();
        InsertCsrControlsRow(parent, lineID, szRows, &controls);
        mCSRElements[lineID] = controls;
    }
    mainSizer->Add(szRows, 0, wxEXPAND, 0);

    return mainSizer;
}

CSR_wxgui::CSR_wxgui(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long styles)
    : IModuleFrame(parent, id, title, pos, size, styles)
{
    mDevice = nullptr;
    CSR_interface = nullptr;

    wxFlexGridSizer* mainSizer;
    mainSizer = new wxFlexGridSizer(0, 1, 0, 0);
    mainSizer->SetFlexibleDirection(wxBOTH);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* csrControls = CreateCsrControls(this, 4);
    mainSizer->Add(csrControls, 0, wxALIGN_LEFT, 0);

    SetSizer(mainSizer);
    Layout();
    mainSizer->Fit(this);

    Centre(wxBOTH);
}

bool CSR_wxgui::Initialize(SDRDevice* pCtrPort)
{
    mDevice = pCtrPort;
    if (mDevice == nullptr)
    {
        delete CSR_interface;
        CSR_interface = nullptr;
        wxArrayString emptyList;
        emptyList.Add("No comms");
        for (auto iter : mCSRselection)
        {
            if (iter)
                iter->Set(emptyList);
        }
        return false;
    }

    CSR_interface = pCtrPort->getICSR();
    return true;
}

void CSR_wxgui::onCSRwrite(wxCommandEvent& event)
{
    try
    {

        CSRFields& fields = mCSRElements.at(event.GetId());
        if (!mDevice)
        {
            fields.status->SetLabel("Not connected");
            return;
        }

        if (!CSR_interface)
        {
            fields.status->SetLabel("CSR interface not available!");
            return;
        }

        const wxString strAddress = fields.address->GetValue();
        unsigned long long addr = 0;
        strAddress.ToULongLong(&addr, 16);

        const wxString strValue = fields.value->GetValue();
        unsigned long long value = 0;
        strValue.ToULongLong(&value, 16);

        OpStatus status = CSR_interface->ioWrite64(addr, value);
        fields.status->SetLabel(ToString(status));

    } catch (...)
    {
        lime::error("No csr controls created for event id: %i", event.GetId());
    }
}

void CSR_wxgui::onCSRread(wxCommandEvent& event)
{
    try
    {
        CSRFields& fields = mCSRElements.at(event.GetId());
        if (!mDevice)
        {
            fields.status->SetLabel("Not connected");
            return;
        }

        if (!CSR_interface)
        {
            fields.status->SetLabel("CSR interface not available!");
            return;
        }

        const wxString strAddress = fields.address->GetValue();
        unsigned long long addr = 0;
        strAddress.ToULongLong(&addr, 16);

        //  const wxString strValue = fields.value->GetValue();
        //  unsigned long long value = 0;
        //  strValue.ToULongLong(&value, 16);

        //  uint64_t data_wr = addr;
        uint64_t value = 0;
        OpStatus status = OpStatus::Success;

        try
        {
            value = CSR_interface->ioRead64(addr, &status);
            fields.status->SetLabel(ToString(status));
            if (status != OpStatus::Success)
                return;
            fields.value->SetValue(wxString::Format("%0llX", value));

        } catch (std::runtime_error& e)
        {
            fields.status->SetLabel(e.what());
        }
    } catch (...)
    {
        lime::error("No csr controls created for event id: %i", event.GetId());
    }
}
