#include "lms7002_pnlR3.h"
#include "commonWxHeaders.h"
#include <wx/spinctrl.h>
#include <wx/msgdlg.h>
#include "lms7002_gui_utilities.h"
#include <chrono>
#include <thread>
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/LMS7002MCSR.h"
#include "limesuiteng/Logger.h"

#include <vector>

using namespace lime;
using namespace lime::LMS7002MCSR_Data;
using namespace std;
using namespace std::literals::string_literals;

lms7002_pnlR3_view::lms7002_pnlR3_view(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ILMS7002MTab(parent, id, pos, size, style)
{
    lmsControl = nullptr;
    wxFlexGridSizer* mainSizer;
    mainSizer = new wxFlexGridSizer(0, 1, 5, 5);
    {
        wxStaticBoxSizer* dcCalibGroup = new wxStaticBoxSizer(wxHORIZONTAL, this, wxT("DC calibration"));
        wxWindow* panel = dcCalibGroup->GetStaticBox();
        {
            wxFlexGridSizer* sizer = new wxFlexGridSizer(0, 1, 0, 5);
            constexpr std::array<LMS7002MCSR, 8> params = { LMS7002MCSR::PD_DCDAC_RXB,
                LMS7002MCSR::PD_DCDAC_RXA,
                LMS7002MCSR::PD_DCDAC_TXB,
                LMS7002MCSR::PD_DCDAC_TXA,
                LMS7002MCSR::PD_DCCMP_RXB,
                LMS7002MCSR::PD_DCCMP_RXA,
                LMS7002MCSR::PD_DCCMP_TXB,
                LMS7002MCSR::PD_DCCMP_TXA };
            int row = 0;
            for (auto i : params)
            {
                const CSRegister& r = GetRegister(i);
                sizer->AddGrowableRow(row++);
                wxCheckBox* chkbox = new wxCheckBox(dcCalibGroup->GetStaticBox(), wxNewId(), r.name);
                chkbox->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED,
                    wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler),
                    nullptr,
                    this);
                sizer->Add(chkbox, 1, wxEXPAND, 5);
                wndId2Enum[chkbox] = i;
            }
            dcCalibGroup->Add(sizer, 0, wxEXPAND, 5);
        }
        {
            wxFlexGridSizer* sizer = new wxFlexGridSizer(0, 2, 0, 0);
            constexpr std::array<LMS7002MCSR, 12> paramsRx = { LMS7002MCSR::DCWR_RXBQ,
                LMS7002MCSR::DCRD_RXBQ,
                LMS7002MCSR::DC_RXBQ,
                LMS7002MCSR::DCWR_RXBI,
                LMS7002MCSR::DCRD_RXBI,
                LMS7002MCSR::DC_RXBI,
                LMS7002MCSR::DCWR_RXAQ,
                LMS7002MCSR::DCRD_RXAQ,
                LMS7002MCSR::DC_RXAQ,
                LMS7002MCSR::DCWR_RXAI,
                LMS7002MCSR::DCRD_RXAI,
                LMS7002MCSR::DC_RXAI };
            for (size_t i = 0; i < paramsRx.size(); i += 3)
            {
                wxButton* btnReadDC = new wxButton(dcCalibGroup->GetStaticBox(), wxNewId(), _("Read"));
                btnReadDC->Connect(
                    wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlR3_view::OnReadDC), nullptr, this);
                sizer->Add(btnReadDC, 1, wxEXPAND, 5);
                NumericSlider* slider = new NumericSlider(dcCalibGroup->GetStaticBox(),
                    wxNewId(),
                    wxEmptyString,
                    wxDefaultPosition,
                    wxDefaultSize,
                    wxSP_ARROW_KEYS,
                    -63,
                    63,
                    0);
                cmbDCControlsRx.push_back(slider);
                slider->Connect(
                    wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::OnWriteRxDC), nullptr, this);
                sizer->Add(slider, 1, wxEXPAND, 5);
                wndId2Enum[slider] = paramsRx[i + 2];
            }
            constexpr std::array<LMS7002MCSR, 12> paramsTx = { LMS7002MCSR::DCWR_TXBQ,
                LMS7002MCSR::DCRD_TXBQ,
                LMS7002MCSR::DC_TXBQ,
                LMS7002MCSR::DCWR_TXBI,
                LMS7002MCSR::DCRD_TXBI,
                LMS7002MCSR::DC_TXBI,
                LMS7002MCSR::DCWR_TXAQ,
                LMS7002MCSR::DCRD_TXAQ,
                LMS7002MCSR::DC_TXAQ,
                LMS7002MCSR::DCWR_TXAI,
                LMS7002MCSR::DCRD_TXAI,
                LMS7002MCSR::DC_TXAI };
            for (size_t i = 0; i < paramsTx.size(); i += 3)
            {
                wxButton* btnReadDC = new wxButton(dcCalibGroup->GetStaticBox(), wxNewId(), _("Read"));
                btnReadDC->Connect(
                    wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlR3_view::OnReadDC), nullptr, this);
                sizer->Add(btnReadDC, 1, wxEXPAND, 5);
                NumericSlider* slider = new NumericSlider(dcCalibGroup->GetStaticBox(),
                    wxNewId(),
                    wxEmptyString,
                    wxDefaultPosition,
                    wxDefaultSize,
                    wxSP_ARROW_KEYS,
                    -1023,
                    1023,
                    0);
                cmbDCControlsTx.push_back(slider);
                slider->Connect(
                    wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::OnWriteTxDC), nullptr, this);
                sizer->Add(slider, 1, wxEXPAND, 5);
                wndId2Enum[slider] = paramsTx[i + 2];
            }
            dcCalibGroup->Add(sizer, 0, 0, 5);
        }
        {
            const std::vector<wxString> names = {
                _("RXBQ"), _("RXBI"), _("RXAQ"), _("RXAI"), _("TXBQ"), _("TXBI"), _("TXAQ"), _("TXAI")
            };
            constexpr std::array<LMS7002MCSR, 8> cmpcfg = { LMS7002MCSR::DCCAL_CMPCFG_RXBQ,
                LMS7002MCSR::DCCAL_CMPCFG_RXBI,
                LMS7002MCSR::DCCAL_CMPCFG_RXAQ,
                LMS7002MCSR::DCCAL_CMPCFG_RXAI,
                LMS7002MCSR::DCCAL_CMPCFG_TXBQ,
                LMS7002MCSR::DCCAL_CMPCFG_TXBI,
                LMS7002MCSR::DCCAL_CMPCFG_TXAQ,
                LMS7002MCSR::DCCAL_CMPCFG_TXAI };
            constexpr std::array<LMS7002MCSR, 8> cmpstatus = { LMS7002MCSR::DCCAL_CMPSTATUS_RXBQ,
                LMS7002MCSR::DCCAL_CMPSTATUS_RXBI,
                LMS7002MCSR::DCCAL_CMPSTATUS_RXAQ,
                LMS7002MCSR::DCCAL_CMPSTATUS_RXAI,
                LMS7002MCSR::DCCAL_CMPSTATUS_TXBQ,
                LMS7002MCSR::DCCAL_CMPSTATUS_TXBI,
                LMS7002MCSR::DCCAL_CMPSTATUS_TXAQ,
                LMS7002MCSR::DCCAL_CMPSTATUS_TXAI };
            constexpr std::array<LMS7002MCSR, 8> calstatus = { LMS7002MCSR::DCCAL_CALSTATUS_RXBQ,
                LMS7002MCSR::DCCAL_CALSTATUS_RXBI,
                LMS7002MCSR::DCCAL_CALSTATUS_RXAQ,
                LMS7002MCSR::DCCAL_CALSTATUS_RXAI,
                LMS7002MCSR::DCCAL_CALSTATUS_TXBQ,
                LMS7002MCSR::DCCAL_CALSTATUS_TXBI,
                LMS7002MCSR::DCCAL_CALSTATUS_TXAQ,
                LMS7002MCSR::DCCAL_CMPSTATUS_TXAI };
            constexpr std::array<LMS7002MCSR, 8> start = { LMS7002MCSR::DCCAL_START_RXBQ,
                LMS7002MCSR::DCCAL_START_RXBI,
                LMS7002MCSR::DCCAL_START_RXAQ,
                LMS7002MCSR::DCCAL_START_RXAI,
                LMS7002MCSR::DCCAL_START_TXBQ,
                LMS7002MCSR::DCCAL_START_TXBI,
                LMS7002MCSR::DCCAL_START_TXAQ,
                LMS7002MCSR::DCCAL_START_TXAI };

            wxFlexGridSizer* sizer = new wxFlexGridSizer(0, 5, 0, 5);
            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("Name:")), 1, wxEXPAND, 0);
            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("START:")), 1, wxEXPAND, 0);
            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("CMP invert:")), 1, wxEXPAND, 0);
            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("CMP:")), 1, wxEXPAND, 0);
            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("Status:")), 1, wxEXPAND, 0);
            for (size_t i = 0; i < names.size(); ++i)
            {
                sizer->AddGrowableRow(i);
                wxCheckBox* chkbox;
                sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, names[i]), 1, wxEXPAND, 0);

                chkbox = new wxCheckBox(dcCalibGroup->GetStaticBox(), wxNewId(), wxEmptyString);
                chkbox->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED,
                    wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler),
                    nullptr,
                    this);
                wndId2Enum[chkbox] = start[i];
                sizer->Add(chkbox, 0, wxALIGN_CENTER_HORIZONTAL, 0);

                chkbox = new wxCheckBox(dcCalibGroup->GetStaticBox(), wxNewId(), wxEmptyString);
                chkbox->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED,
                    wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler),
                    nullptr,
                    this);
                wndId2Enum[chkbox] = cmpcfg[i];
                sizer->Add(chkbox, 0, wxALIGN_CENTER_HORIZONTAL, 0);

                dccal_cmpstatuses[i] = new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("0"));
                wndId2Enum[dccal_cmpstatuses[i]] = cmpstatus[i];
                sizer->Add(dccal_cmpstatuses[i], 1, wxEXPAND, 0);
                dccal_statuses[i] = new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("Not running"));
                sizer->Add(dccal_statuses[i], 1, wxEXPAND, 0);
                wndId2Enum[dccal_statuses[i]] = calstatus[i];
            }
            for (int i = 0; i < 4; ++i)
                sizer->Add(new wxFlexGridSizer(0, 0, 0, 0));
            wxButton* btnUpdateCMP = new wxButton(panel, wxNewId(), _("Read"));
            btnUpdateCMP->Connect(
                wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlR3_view::OnReadDCCMP), nullptr, this);
            sizer->Add(btnUpdateCMP);
            dcCalibGroup->Add(sizer, 0, wxLEFT, 5);
        }
        {
            wxFlexGridSizer* sizer = new wxFlexGridSizer(0, 2, 0, 5);
            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("DC_RXCDIV")), 1, wxALIGN_CENTER_VERTICAL, 0);
            NumericSlider* ctrl = new NumericSlider(dcCalibGroup->GetStaticBox(),
                wxNewId(),
                wxEmptyString,
                wxDefaultPosition,
                wxDefaultSize,
                wxSP_ARROW_KEYS,
                0,
                255,
                0);
            ctrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(ctrl, 1, wxEXPAND, 5);
            wndId2Enum[ctrl] = LMS7002MCSR::DC_RXCDIV;
            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("DC_TXCDIV")), 1, wxALIGN_CENTER_VERTICAL, 0);
            ctrl = new NumericSlider(dcCalibGroup->GetStaticBox(),
                wxNewId(),
                wxEmptyString,
                wxDefaultPosition,
                wxDefaultSize,
                wxSP_ARROW_KEYS,
                0,
                255,
                0);
            ctrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(ctrl, 1, wxEXPAND, 5);
            wndId2Enum[ctrl] = LMS7002MCSR::DC_TXCDIV;

            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("HYSCMP_RXB")), 1, wxALIGN_CENTER_VERTICAL, 0);
            ctrl = new NumericSlider(
                dcCalibGroup->GetStaticBox(), wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 7, 0);
            ctrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(ctrl, 1, wxEXPAND, 5);
            wndId2Enum[ctrl] = LMS7002MCSR::HYSCMP_RXB;

            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("HYSCMP_RXA")), 1, wxALIGN_CENTER_VERTICAL, 0);
            ctrl = new NumericSlider(
                dcCalibGroup->GetStaticBox(), wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 7, 0);
            ctrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(ctrl, 1, wxEXPAND, 5);
            wndId2Enum[ctrl] = LMS7002MCSR::HYSCMP_RXA;

            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("HYSCMP_TXB")), 1, wxALIGN_CENTER_VERTICAL, 0);
            ctrl = new NumericSlider(
                dcCalibGroup->GetStaticBox(), wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 7, 0);
            ctrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(ctrl, 1, wxEXPAND, 5);
            wndId2Enum[ctrl] = LMS7002MCSR::HYSCMP_TXB;

            sizer->Add(new wxStaticText(dcCalibGroup->GetStaticBox(), wxID_ANY, _("HYSCMP_TXA")), 1, wxALIGN_CENTER_VERTICAL, 0);
            ctrl = new NumericSlider(
                dcCalibGroup->GetStaticBox(), wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 7, 0);
            ctrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(ctrl, 1, wxEXPAND, 5);
            wndId2Enum[ctrl] = LMS7002MCSR::HYSCMP_TXA;
            dcCalibGroup->Add(sizer, 0, wxLEFT | wxEXPAND, 5);
        }
        mainSizer->Add(dcCalibGroup, 1, wxEXPAND, 5);
    }
    {
        wxFlexGridSizer* rowGroup = new wxFlexGridSizer(0, 4, 0, 5);
        {
            wxStaticBoxSizer* RSSIPDETGroup = new wxStaticBoxSizer(wxHORIZONTAL, this, wxT("RSSI, PDET, TEMP"));
            wxWindow* panel = RSSIPDETGroup->GetStaticBox();

            wxFlexGridSizer* sizer = new wxFlexGridSizer(0, 2, 0, 5);

            wxCheckBox* chkbox = new wxCheckBox(panel, wxNewId(), _("Power down modules"));
            chkbox->Connect(
                wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(chkbox);
            wndId2Enum[chkbox] = LMS7002MCSR::RSSI_PD;

            chkbox = new wxCheckBox(panel, wxNewId(), _("Manual operation mode"));
            chkbox->Connect(
                wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(chkbox);
            wndId2Enum[chkbox] = LMS7002MCSR::RSSI_RSSIMODE;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("DAC_CLKDIV")), 1, wxALIGN_CENTER_VERTICAL, 0);
            NumericSlider* spnCtrl =
                new NumericSlider(panel, wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 0);
            spnCtrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(spnCtrl);
            wndId2Enum[spnCtrl] = LMS7002MCSR::DAC_CLKDIV;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("Reference bias current to test ADC")), 1, wxALIGN_CENTER_VERTICAL, 0);
            spnCtrl =
                new NumericSlider(panel, wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 0);
            spnCtrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(spnCtrl);
            wndId2Enum[spnCtrl] = LMS7002MCSR::RSSI_BIAS;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("HYSCMP")), 1, wxALIGN_CENTER_VERTICAL, 0);
            spnCtrl =
                new NumericSlider(panel, wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 7, 0);
            spnCtrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(spnCtrl);
            wndId2Enum[spnCtrl] = LMS7002MCSR::RSSI_HYSCMP;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("DAC_VAL")), 1, wxALIGN_CENTER_VERTICAL, 0);
            spnCtrl =
                new NumericSlider(panel, wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 0);
            spnCtrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(spnCtrl);
            wndId2Enum[spnCtrl] = LMS7002MCSR::RSSI_DAC_VAL;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("PDET2_VAL")), 1, wxALIGN_CENTER_VERTICAL, 0);
            pdet_vals[1] = new wxStaticText(panel, wxID_ANY, _("????"));
            sizer->Add(pdet_vals[1]);
            wndId2Enum[pdet_vals[1]] = LMS7002MCSR::RSSI_PDET2_VAL;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("PDET1_VAL")), 1, wxALIGN_CENTER_VERTICAL, 0);
            pdet_vals[0] = new wxStaticText(panel, wxID_ANY, _("????"));
            sizer->Add(pdet_vals[0]);
            wndId2Enum[pdet_vals[0]] = LMS7002MCSR::RSSI_PDET1_VAL;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("RSSI2_VAL")), 1, wxALIGN_CENTER_VERTICAL, 0);
            rssi_vals[1] = new wxStaticText(panel, wxID_ANY, _("????"));
            sizer->Add(rssi_vals[1]);
            wndId2Enum[rssi_vals[1]] = LMS7002MCSR::RSSI_RSSI2_VAL;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("RSSI1_VAL")), 1, wxALIGN_CENTER_VERTICAL, 0);
            rssi_vals[0] = new wxStaticText(panel, wxID_ANY, _("????"));
            sizer->Add(rssi_vals[0]);
            wndId2Enum[rssi_vals[0]] = LMS7002MCSR::RSSI_RSSI1_VAL;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("TREF_VAL")), 1, wxALIGN_CENTER_VERTICAL, 0);
            tref_val = new wxStaticText(panel, wxID_ANY, _("????"));
            sizer->Add(tref_val);
            wndId2Enum[tref_val] = LMS7002MCSR::RSSI_TREF_VAL;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("TVPTAT_VAL")), 1, wxALIGN_CENTER_VERTICAL, 0);
            tvptat_val = new wxStaticText(panel, wxID_ANY, _("????"));
            wndId2Enum[tvptat_val] = LMS7002MCSR::RSSI_TVPTAT_VAL;
            sizer->Add(tvptat_val);

            RSSIPDETGroup->Add(sizer);

            wxFlexGridSizer* sizerCMP = new wxFlexGridSizer(0, 2, 0, 5);
            constexpr std::array<LMS7002MCSR, 6> paramStatus = { LMS7002MCSR::INTADC_CMPSTATUS_TEMPREF,
                LMS7002MCSR::INTADC_CMPSTATUS_TEMPVPTAT,
                LMS7002MCSR::INTADC_CMPSTATUS_RSSI2,
                LMS7002MCSR::INTADC_CMPSTATUS_RSSI1,
                LMS7002MCSR::INTADC_CMPSTATUS_PDET2,
                LMS7002MCSR::INTADC_CMPSTATUS_PDET1 };
            constexpr std::array<LMS7002MCSR, 6> params = { LMS7002MCSR::INTADC_CMPCFG_TEMPREF,
                LMS7002MCSR::INTADC_CMPCFG_TEMPVPTAT,
                LMS7002MCSR::INTADC_CMPCFG_RSSI2,
                LMS7002MCSR::INTADC_CMPCFG_RSSI1,
                LMS7002MCSR::INTADC_CMPCFG_PDET2,
                LMS7002MCSR::INTADC_CMPCFG_PDET1 };
            sizerCMP->Add(new wxStaticText(panel, wxID_ANY, _("Invert:")));
            sizerCMP->Add(new wxStaticText(panel, wxID_ANY, _("CMP:")));

            for (int i = 0; i < 6; ++i)
            {
                const CSRegister& r = GetRegister(params[i]);
                rssiCMPCFG[i] = new wxCheckBox(panel, wxNewId(), r.name);
                sizerCMP->Add(rssiCMPCFG[i]);
                wndId2Enum[rssiCMPCFG[i]] = params[i];
                rssiCMPCFG[i]->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED,
                    wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler),
                    nullptr,
                    this);
                rssiCMPSTATUS[i] = new wxStaticText(panel, wxNewId(), _("?"));
                sizerCMP->Add(rssiCMPSTATUS[i], 0, wxALIGN_CENTER_VERTICAL, 0);
                wndId2Enum[rssiCMPSTATUS[i]] = paramStatus[i];
            }
            sizerCMP->Add(new wxFlexGridSizer(0, 0, 0, 0));
            wxButton* btnUpdateRSSICMP = new wxButton(panel, wxNewId(), _("Read"));
            btnUpdateRSSICMP->Connect(
                wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlR3_view::OnReadRSSICMP), nullptr, this);
            sizerCMP->Add(btnUpdateRSSICMP);
            RSSIPDETGroup->Add(sizerCMP, 0, wxLEFT, 5);

            rowGroup->Add(RSSIPDETGroup);
        }
        {
            wxStaticBoxSizer* RSSIGroup = new wxStaticBoxSizer(wxHORIZONTAL, this, wxT("RSSI DC Calibration"));
            wxWindow* panel = RSSIGroup->GetStaticBox();

            wxFlexGridSizer* sizer = new wxFlexGridSizer(0, 2, 0, 5);

            sizer->Add(new wxFlexGridSizer(0, 1, 0, 0), 0, 0, 0);

            wxCheckBox* chkbox = new wxCheckBox(panel, wxNewId(), _("Power down modules"));
            chkbox->Connect(
                wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(chkbox);
            wndId2Enum[chkbox] = LMS7002MCSR::RSSIDC_PD;
            chkRSSI_PD = chkbox;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("HYSCMP")), 1, wxALIGN_CENTER_VERTICAL, 0);
            NumericSlider* spnCtrl =
                new NumericSlider(panel, wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 7, 0);
            cmbRSSIDC_HYSCMP = spnCtrl;
            spnCtrl->Connect(
                wxEVT_COMMAND_SPINCTRL_UPDATED, wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandler), nullptr, this);
            sizer->Add(spnCtrl, 0, wxEXPAND, 0);
            wndId2Enum[spnCtrl] = LMS7002MCSR::RSSIDC_HYSCMP;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("DCO2")), 1, wxALIGN_CENTER_VERTICAL, 0);
            spnCtrl =
                new NumericSlider(panel, wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 127, 0);
            spinDCO2 = spnCtrl;
            spnCtrl->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED,
                wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandlerCMPRead),
                nullptr,
                this);
            sizer->Add(spnCtrl, 0, wxEXPAND, 0);
            wndId2Enum[spnCtrl] = LMS7002MCSR::RSSIDC_DCO2;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("DCO1")), 1, wxALIGN_CENTER_VERTICAL, 0);
            spnCtrl =
                new NumericSlider(panel, wxNewId(), wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 127, 0);
            spinDCO1 = spnCtrl;
            spnCtrl->Connect(wxEVT_COMMAND_SPINCTRL_UPDATED,
                wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandlerCMPRead),
                nullptr,
                this);
            sizer->Add(spnCtrl, 0, wxEXPAND, 0);
            wndId2Enum[spnCtrl] = LMS7002MCSR::RSSIDC_DCO1;

            wxArrayString rselArray;
            float value_mV = 800;
            int i = 0;
            for (; i < 4; ++i)
            {
                rselArray.push_back(wxString::Format("%.1f mV", value_mV));
                value_mV -= 50;
            }
            for (; i < 12; ++i)
            {
                rselArray.push_back(wxString::Format("%.1f mV", value_mV));
                value_mV -= 21.5;
            }
            for (; i <= 31; ++i)
            {
                rselArray.push_back(wxString::Format("%.1f mV", value_mV));
                value_mV -= 10;
            }
            sizer->Add(new wxStaticText(panel, wxID_ANY, _("RSEL:")), 1, wxALIGN_CENTER_VERTICAL, 0);
            cmbRSEL = new wxComboBox(panel, wxID_ANY);
            cmbRSEL->Append(rselArray);
            cmbRSEL->Connect(wxEVT_COMMAND_COMBOBOX_SELECTED,
                wxCommandEventHandler(lms7002_pnlR3_view::ParameterChangeHandlerCMPRead),
                nullptr,
                this);
            sizer->Add(cmbRSEL);
            wndId2Enum[cmbRSEL] = LMS7002MCSR::RSSIDC_RSEL;

            sizer->Add(new wxStaticText(panel, wxID_ANY, _("CMPSTATUS")), 1, wxALIGN_CENTER_VERTICAL, 0);
            rssidc_cmpstatus = new wxStaticText(panel, wxID_ANY, _("?"));
            sizer->Add(rssidc_cmpstatus, 0, wxALIGN_CENTER_VERTICAL, 0);

            wxButton* calibrateAnalogRSSIDC = new wxButton(panel, wxNewId(), _("Calibrate Analog RSSI"));
            calibrateAnalogRSSIDC->Connect(
                wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_pnlR3_view::OnCalibrateAnalogRSSI), nullptr, this);
            sizer->Add(calibrateAnalogRSSIDC);

            RSSIGroup->Add(sizer);
            rowGroup->Add(RSSIGroup);
        }
        mainSizer->Add(rowGroup);
    }
    SetSizer(mainSizer);
    Layout();
    mainSizer->Fit(this);

    LMS7002_WXGUI::UpdateTooltips(wndId2Enum, true);
}

lms7002_pnlR3_view::~lms7002_pnlR3_view()
{
}

void lms7002_pnlR3_view::Initialize(LMS7002M* pControl)
{
    ILMS7002MTab::Initialize(pControl);
    if (pControl == nullptr)
        return;
    uint16_t value = ReadParam(LMS7002MCSR::MASK);
    Enable(value);
}

void lms7002_pnlR3_view::UpdateGUI()
{
    if (!lmsControl)
        return;
    lmsControl->DownloadAll();
    LMS7002_WXGUI::UpdateControlsByMap(this, lmsControl, wndId2Enum, mChannel);

    wxCommandEvent evt;

    for (size_t i = 0; i < cmbDCControlsRx.size(); ++i)
    {
        uint16_t value = 0;
        auto parameter = wndId2Enum[cmbDCControlsRx[i]];
        uint16_t baseAddress = GetRegister(parameter).address;
        lmsControl->SPI_write(baseAddress, 0);
        lmsControl->SPI_write(baseAddress, 0x4000);
        value = ReadParam(wndId2Enum[cmbDCControlsRx[i]]);
        lmsControl->SPI_write(baseAddress, value & ~0xC000);
        int absval = (value & 0x3F);
        if (value & 0x40)
            absval *= -1;
        cmbDCControlsRx[i]->SetValue(absval);
    }
    for (size_t i = 0; i < cmbDCControlsTx.size(); ++i)
    {
        uint16_t value = 0;
        auto parameter = wndId2Enum[cmbDCControlsTx[i]];
        uint16_t baseAddress = GetRegister(parameter).address;
        lmsControl->SPI_write(baseAddress, 0);
        lmsControl->SPI_write(baseAddress, 0x4000);
        value = ReadParam(wndId2Enum[cmbDCControlsTx[i]]);
        lmsControl->SPI_write(baseAddress, value & ~0xC000);
        int absval = (value & 0x3FF);
        if (value & 0x400)
            absval *= -1;
        cmbDCControlsTx[i]->SetValue(absval);
    }

    UpdateGUISlow();
}

void lms7002_pnlR3_view::MCU_RunProcedure(uint8_t id)
{
    uint16_t temp;
    temp = lmsControl->SPI_read(0x0002);

    const uint16_t x0002reg = temp & 0xFF;
    const uint16_t interrupt6 = 0x0008;
    const uint16_t addrs[5] = { 0x0006, 0x0, 0x0002, 0x0002, 0x0002 };
    const uint16_t values[5] = {
        static_cast<uint16_t>(id != 0),
        static_cast<uint16_t>(id),
        static_cast<uint16_t>(x0002reg & ~interrupt6),
        static_cast<uint16_t>(x0002reg | interrupt6),
        static_cast<uint16_t>(x0002reg & ~interrupt6),
    };
    for (int i = 0; i < 5; ++i)
        lmsControl->SPI_write(addrs[i], values[i]);
}

uint8_t lms7002_pnlR3_view::MCU_WaitForStatus(uint16_t timeout_ms)
{
    if (!lmsControl)
        return 0;
    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = t1;
    uint16_t value = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    do
    {
        //value = SPI_read(0x0001) & 0xFF;
        value = lmsControl->SPI_read(0x0001);
        value &= 0xFF;
        if (value != 0xFF) //working
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        t2 = std::chrono::high_resolution_clock::now();
    } while (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() < timeout_ms);
    lmsControl->SPI_write(0x0006, 0);
    //SPI_write(0x0006, 0); //return SPI control to PC
    return value & 0x7F;
}

void lms7002_pnlR3_view::ParameterChangeHandler(wxCommandEvent& event)
{
    if (!lmsControl)
        return;
    LMS7002MCSR parameter;
    try
    {
        parameter = wndId2Enum.at(reinterpret_cast<wxWindow*>(event.GetEventObject()));
    } catch (std::exception& e)
    {
        lime::error("Control element(ID = "s + std::to_string(event.GetId()) + ") don't have assigned LMS parameter."s);
        return;
    }

    // if(parameter.address == 0x0640 || parameter.address == 0x0641)
    // {
    //     MCU_RunProcedure(MCU_FUNCTION_GET_PROGRAM_ID);
    //     if(MCU_WaitForStatus(100) != MCU_ID_CALIBRATIONS_SINGLE_IMAGE)
    //         LMS_Program(lmsControl, (const char*)mcu_program_lms7_dc_iq_calibration_bin, sizeof(mcu_program_lms7_dc_iq_calibration_bin), lime::program_mode::mcuRAM, nullptr);

    //     //run mcu write
    //     lmsControl->SPI_write(0x002D, parameter.address);
    //     uint16_t wrVal = 0;

    //     //read reg
    //     MCU_RunProcedure(8);
    //     uint16_t rdVal = 0;
    //     MCU_WaitForStatus(100);
    //     rdVal = lmsControl->SPI_read(0x040B);

    //     uint16_t mask = (~0u) << (parameter.msb-parameter.lsb+1);
    //     mask = ~mask;
    //     mask <<= parameter.lsb;
    //     rdVal &= ~mask;
    //     wrVal = rdVal | (event.GetInt() << parameter.lsb);

    //     lmsControl->SPI_write(0x020C, wrVal);
    //     MCU_RunProcedure(7);
    //     MCU_WaitForStatus(100);

    //     //check if correct
    //     MCU_RunProcedure(8);

    //     MCU_WaitForStatus(100);
    //     rdVal = lmsControl->SPI_read(0x040B);
    //     if(rdVal != wrVal)
    //     {
    //         printf("Reg 0x%04X value mismatch, written 0x%04X, got 0x%04X\n",
    //                parameter.address, wrVal, rdVal);
    //     }
    // }
    // else
    WriteParam(parameter, event.GetInt());
}

void lms7002_pnlR3_view::ParameterChangeHandler(wxSpinEvent& event)
{
    wxCommandEvent evt;
    evt.SetInt(event.GetInt());
    evt.SetId(event.GetId());
    evt.SetEventObject(event.GetEventObject());
    ParameterChangeHandler(evt);
}

void lms7002_pnlR3_view::OnDCCMPCFGRead()
{
    UpdateGUI();
}

void lms7002_pnlR3_view::OnReadRSSICMP(wxCommandEvent& event)
{
    UpdateGUI();
}

void lms7002_pnlR3_view::OnReadDCCMP(wxCommandEvent& event)
{
    UpdateGUI();
}

void lms7002_pnlR3_view::OnWriteTxDC(wxCommandEvent& event)
{
    if (!lmsControl)
        return;
    LMS7002MCSR parameter;
    try
    {
        parameter = wndId2Enum.at(reinterpret_cast<wxWindow*>(event.GetEventObject()));
        uint16_t regVal = 0;
        uint16_t baseAddress = GetRegister(parameter).address;
        regVal = lmsControl->SPI_read(baseAddress);
        regVal &= 0xF800;
        int dcVal = event.GetInt();
        if (dcVal < 0)
        {
            --dcVal;
            regVal |= 0x0400;
        }
        regVal |= (abs(dcVal + 0x400) & 0x3FF);
        lmsControl->SPI_write(baseAddress, regVal);
        lmsControl->SPI_write(baseAddress, regVal | 0x8000);
        lmsControl->SPI_write(baseAddress, regVal);
        return;
    } catch (std::exception& e)
    {
        lime::error("Control element(ID = "s + std::to_string(event.GetId()) + ") don't have assigned LMS parameter."s);
        return;
    }
}

void lms7002_pnlR3_view::OnWriteRxDC(wxCommandEvent& event)
{
    if (!lmsControl)
        return;
    LMS7002MCSR parameter;
    try
    {
        parameter = wndId2Enum.at(reinterpret_cast<wxWindow*>(event.GetEventObject()));
        uint16_t regVal = 0;
        uint16_t baseAddress = GetRegister(parameter).address;
        regVal = lmsControl->SPI_read(baseAddress);
        regVal &= 0xFF80;
        int dcVal = event.GetInt();
        if (dcVal < 0)
        {
            --dcVal;
            regVal |= 0x0040;
        }
        regVal |= (abs(dcVal + 0x40) & 0x3F);
        lmsControl->SPI_write(baseAddress, regVal & ~0x8000);
        lmsControl->SPI_write(baseAddress, regVal | 0x8000);
        lmsControl->SPI_write(baseAddress, regVal);
        return;
    } catch (std::exception& e)
    {
        lime::error("Control element(ID = "s + std::to_string(event.GetId()) + ") don't have assigned LMS parameter."s);
        return;
    }
}

void lms7002_pnlR3_view::OnReadDC(wxCommandEvent& event)
{
    UpdateGUI();
}

void lms7002_pnlR3_view::ParameterChangeHandlerCMPRead(wxCommandEvent& event)
{
    ParameterChangeHandler(event);
    UpdateGUISlow();
}

void lms7002_pnlR3_view::UpdateGUISlow()
{
    if (!lmsControl)
        return;
    vector<uint16_t> addrs = { 0x0640, 0x0641 };
    vector<uint16_t> rez;
    for (auto i : addrs)
    {
        lmsControl->SPI_write(0x002D, i);
        uint16_t value = 0;
        MCU_RunProcedure(8);
        MCU_WaitForStatus(100);
        value = lmsControl->SPI_read(0x040B);
        rez.push_back(value);
    }
    rssidc_cmpstatus->SetLabel(std::to_string((rez[0] >> 15)));
    cmbRSEL->SetSelection((rez[0] >> 4) & 0x1F);
    cmbRSSIDC_HYSCMP->SetValue((rez[0] >> 1) & 0x7);
    chkRSSI_PD->SetValue(rez[0] & 0x1);
    spinDCO2->SetValue((rez[1] >> 7) & 0x7F);
    spinDCO1->SetValue((rez[1] >> 0) & 0x7F);
}

void lms7002_pnlR3_view::OnCalibrateAnalogRSSI(wxCommandEvent& event)
{
    OpStatus status = lmsControl->CalibrateAnalogRSSI_DC_Offset();
    if (status != OpStatus::Success)
        wxMessageBox(wxString::Format(_("Analog RSSI DC calibration failed.")));
    UpdateGUI();
}
