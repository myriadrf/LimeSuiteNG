#include <assert.h>
#include "commonWxHeaders.h"
#include "lms7002_mainPanel.h"
#include "lms7002_pnlAFE_view.h"
#include "lms7002_pnlBIAS_view.h"
#include "lms7002_pnlBIST_view.h"
#include "lms7002_pnlCDS_view.h"
#include "lms7002_pnlCLKGEN_view.h"
#include "lms7002_pnlLDO_view.h"
#include "lms7002_pnlLimeLightPAD_view.h"
#include "lms7002_pnlTxTSP_view.h"
#include "lms7002_pnlRxTSP_view.h"
#include "lms7002_pnlRBB_view.h"
#include "lms7002_pnlRFE_view.h"
#include "lms7002_pnlSX_view.h"
#include "lms7002_pnlTBB_view.h"
#include "lms7002_pnlTRF_view.h"
#include "lms7002_pnlXBUF_view.h"
#include "lms7002_pnlGains_view.h"
#include "lms7002_pnlCalibrations_view.h"
#include "SDRConfiguration_view.h"
#include <wx/time.h>
#include <wx/msgdlg.h>
#include <iostream>
#include <wx/filedlg.h>
#include <wx/notebook.h>
#include <wx/bookctrl.h>
#include "lms7suiteEvents.h"
#include "lms7002_pnlMCU_BD_view.h"
#include "lms7002_pnlR3.h"

#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

lms7002_mainPanel::lms7002_mainPanel(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
    , soc(nullptr)
{
    wxFlexGridSizer* mainSizer;
    mainSizer = new wxFlexGridSizer(3, 1, 0, 0);
    mainSizer->AddGrowableCol(0);
    mainSizer->AddGrowableRow(1);
    mainSizer->SetFlexibleDirection(wxBOTH);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* szToolbar;
    szToolbar = new wxFlexGridSizer(7, 0, 0);
    szToolbar->AddGrowableCol(3);
    szToolbar->SetFlexibleDirection(wxHORIZONTAL);
    szToolbar->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    btnOpen = new wxButton(this, wxID_ANY, wxT("Open"));
    szToolbar->Add(btnOpen, 1, 0, 5);

    btnSave = new wxButton(this, wxID_ANY, wxT("Save"));
    szToolbar->Add(btnSave, 1, 0, 5);

    wxFlexGridSizer* fgSizer300;
    fgSizer300 = new wxFlexGridSizer(0, 5, 0, 0);
    fgSizer300->AddGrowableRow(0);
    fgSizer300->SetFlexibleDirection(wxBOTH);
    fgSizer300->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer248;
    fgSizer248 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer248->SetFlexibleDirection(wxBOTH);
    fgSizer248->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer249;
    fgSizer249 = new wxFlexGridSizer(0, 4, 0, 0);
    fgSizer249->SetFlexibleDirection(wxHORIZONTAL);
    fgSizer249->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    cmbLmsDevice = new wxChoice(this, wxNewId());
    cmbLmsDevice->Append(wxT("LMS 1"));
    cmbLmsDevice->Append(wxT("LMS 2"));
    cmbLmsDevice->SetSelection(0);
    cmbLmsDevice->SetToolTip(wxT("Controls the gain of the LNA"));

    fgSizer249->Add(cmbLmsDevice, 0, wxALL, 0);

    rbChannelA = new wxRadioButton(this, ID_BTN_CH_A, wxT("A CHANNEL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer249->Add(rbChannelA, 0, wxALIGN_CENTER_VERTICAL, 5);

    rbChannelB = new wxRadioButton(this, ID_BTN_CH_B, wxT("B CHANNEL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer249->Add(rbChannelB, 0, wxALIGN_CENTER_VERTICAL, 5);

    chkEnableMIMO = new wxCheckBox(this, wxID_ANY, wxT("Enable MIMO"), wxDefaultPosition, wxDefaultSize, 0);
    chkEnableMIMO->SetToolTip(wxT("Enables required registers for MIMO mode"));

    fgSizer249->Add(chkEnableMIMO, 0, wxALIGN_CENTER_VERTICAL, 5);

    fgSizer248->Add(fgSizer249, 0, wxEXPAND, 5);

    fgSizer300->Add(fgSizer248, 0, wxEXPAND, 5);

    btnDownloadAll = new wxButton(this, ID_BTN_CHIP_TO_GUI, wxT("Chip-->GUI"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer300->Add(btnDownloadAll, 0, 0, 5);

    btnUploadAll = new wxButton(this, wxID_ANY, wxT("GUI-->Chip"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer300->Add(btnUploadAll, 0, 0, 5);

    btnResetChip = new wxButton(this, ID_BTN_RESET_CHIP, wxT("Reset"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer300->Add(btnResetChip, 0, 0, 5);

    wxFlexGridSizer* fgSizer247;
    fgSizer247 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer247->SetFlexibleDirection(wxBOTH);
    fgSizer247->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer300->Add(fgSizer247, 1, wxEXPAND, 5);

    szToolbar->Add(fgSizer300, 1, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL | wxLEFT, 10);

    txtTemperature = new wxStaticText(this, wxID_ANY, wxT("Temperature: ?????"), wxDefaultPosition, wxDefaultSize, 0);
    txtTemperature->Wrap(-1);
    szToolbar->Add(txtTemperature, 0, wxTOP | wxRIGHT | wxLEFT | wxALIGN_CENTER_VERTICAL, 5);

    btnReadTemperature = new wxButton(this, wxID_ANY, wxT("Read Temp"), wxDefaultPosition, wxDefaultSize, 0);
    szToolbar->Add(btnReadTemperature, 0, wxALIGN_CENTER_VERTICAL, 5);

    mainSizer->Add(szToolbar, 0, wxALIGN_LEFT | wxALIGN_TOP, 0);

    tabsNotebook = new wxNotebook(this, ID_TABS_NOTEBOOK);

    ILMS7002MTab* tab;
    tab = new lms7002_pnlCalibrations_view(tabsNotebook, ID_TAB_CALIBRATIONS);
    tabsNotebook->AddPage(tab, _("Calibration"), false);
    mTabs[ID_TAB_CALIBRATIONS] = tab;
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define CreatePage(tabClass, title) \
    { \
        tab = new lms7002_pnl##tabClass##_view(tabsNotebook, ID_TAB_##tabClass); \
        tabsNotebook->AddPage(tab, wxT(title), false); \
        mTabs[ID_TAB_##tabClass] = tab; \
    }
    CreatePage(RFE, "RFE");
    CreatePage(RBB, "RBB");
    CreatePage(TRF, "TRF");
    CreatePage(TBB, "TBB");
    CreatePage(AFE, "AFE");
    CreatePage(BIAS, "BIAS");
    CreatePage(LDO, "LDO");
    CreatePage(XBUF, "XBUF");
    CreatePage(CLKGEN, "CLKGEN");

    tab = new lms7002_pnlSX_view(tabsNotebook, ID_TAB_SXR);
    tab->SetChannel(0);
    tabsNotebook->AddPage(tab, wxT("SXR"), false);
    mTabs[ID_TAB_SXR] = tab;

    lms7002_pnlSX_view* sxtTab = new lms7002_pnlSX_view(tabsNotebook, ID_TAB_SXT);
    sxtTab->direction = TRXDir::Tx;
    sxtTab->SetChannel(1);
    tabsNotebook->AddPage(sxtTab, wxT("SXT"), false);
    mTabs[ID_TAB_SXT] = sxtTab;

    tab = new lms7002_pnlLimeLightPAD_view(tabsNotebook, ID_TAB_LIMELIGHT);
    tabsNotebook->AddPage(tab, wxT("LimeLight && PAD"), false);
    mTabs[ID_TAB_LIMELIGHT] = tab;

    CreatePage(TXTSP, "TXTSP");
    CreatePage(RXTSP, "RXTSP");
    CreatePage(CDS, "CDS");
    CreatePage(BIST, "BIST");

    tab = new lms7002_pnlGains_view(tabsNotebook, ID_TAB_GAINS);
    tabsNotebook->AddPage(tab, wxT("TRX Gain"), false);
    mTabs[ID_TAB_GAINS] = tab;

    lms7002_pnlMCU_BD_view* mTabMCU = new lms7002_pnlMCU_BD_view(tabsNotebook, ID_TAB_MCU);
    tabsNotebook->AddPage(mTabMCU, _("MCU"));
    mTabs[ID_TAB_MCU] = mTabMCU;

    lms7002_pnlR3_view* mTabR3 = new lms7002_pnlR3_view(tabsNotebook, ID_TAB_R3);
    tabsNotebook->AddPage(mTabR3, _("R3 Controls"));
    mTabs[ID_TAB_R3] = mTabR3;
#undef CreatePage

    mainSizer->Add(tabsNotebook, 0, wxEXPAND, 5);
    SetSizerAndFit(mainSizer);

    // Connect Events
    btnOpen->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_mainPanel::OnOpenProject), NULL, this);
    btnSave->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_mainPanel::OnSaveProject), NULL, this);
    cmbLmsDevice->Connect(
        wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler(lms7002_mainPanel::OnChannelOrSOCChange), NULL, this);
    rbChannelA->Connect(
        wxEVT_COMMAND_RADIOBUTTON_SELECTED, wxCommandEventHandler(lms7002_mainPanel::OnChannelOrSOCChange), NULL, this);
    rbChannelB->Connect(
        wxEVT_COMMAND_RADIOBUTTON_SELECTED, wxCommandEventHandler(lms7002_mainPanel::OnChannelOrSOCChange), NULL, this);
    chkEnableMIMO->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms7002_mainPanel::OnEnableMIMOchecked), NULL, this);
    btnDownloadAll->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_mainPanel::OnDownloadAll), NULL, this);
    btnUploadAll->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_mainPanel::OnUploadAll), NULL, this);
    btnResetChip->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_mainPanel::OnResetChip), NULL, this);
    btnReadTemperature->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_mainPanel::OnReadTemperature), NULL, this);
    tabsNotebook->Bind(wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED, &lms7002_mainPanel::Onnotebook_modulesPageChanged, this);
}

lms7002_mainPanel::~lms7002_mainPanel()
{
    tabsNotebook->Unbind(wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED, &lms7002_mainPanel::Onnotebook_modulesPageChanged, this);
}

void lms7002_mainPanel::UpdateVisiblePanel()
{
    if (soc == nullptr)
        return;

    wxWindow* currentPage = tabsNotebook->GetCurrentPage();
    const wxWindowID pageId = currentPage->GetId();
    uint16_t spisw_ctrl = 0;

    LMS7002M* chip = GetSelectedChip();
    assert(chip);

    if (pageId == ID_TAB_SXR) //change active channel to A
        chip->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 1);
    else if (pageId == ID_TAB_SXT) //change active channel to B
        chip->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 2);
    else
        chip->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, rbChannelA->GetValue() == 1 ? 1 : 2);
    spisw_ctrl = chip->SPI_read(0x0006);

    if (spisw_ctrl & 1) // transceiver controlled by MCU
    {
        if (pageId != ID_TAB_MCU && pageId != ID_TAB_GAINS)
            currentPage->Disable();
        return;
    }
    else
        currentPage->Enable();

    wxLongLong t1, t2;
    t1 = wxGetUTCTimeMillis();
    auto currentTab = mTabs.find(pageId);
    if (currentTab != mTabs.end())
        currentTab->second->UpdateGUI();
    t2 = wxGetUTCTimeMillis();
#ifndef NDEBUG
    lime::debug("Visible GUI update time: "s + (t2 - t1).ToString());
#endif
}

void lms7002_mainPanel::Initialize(lime::LMS7002M* socPtr)
{
    soc = socPtr;
    if (soc == nullptr)
    {
        for (auto& tab : mTabs)
            tab.second->Initialize(nullptr);
        return;
    }
    cmbLmsDevice->SetSelection(0);
    cmbLmsDevice->Hide();
    int ch = soc->GetActiveChannelIndex();
    rbChannelA->SetValue(ch == 0);
    rbChannelB->SetValue(ch == 1);

    for (auto& tab : mTabs)
        tab.second->Initialize(soc);
    UpdateGUI();
    Layout();
}

void lms7002_mainPanel::OnResetChip(wxCommandEvent& event)
{
    try
    {
        soc->ResetChip();
    } catch (std::runtime_error& e)
    {
        wxMessageBox("Reset failed: "s + e.what(), _("Warning"));
        return;
    }
    wxNotebookEvent evt;
    chkEnableMIMO->SetValue(false);
    Onnotebook_modulesPageChanged(evt); //after reset chip active channel might change, this refresh channel for active tab
}

void lms7002_mainPanel::UpdateGUI()
{
    wxLongLong t1, t2;
    t1 = wxGetUTCTimeMillis();
    t2 = wxGetUTCTimeMillis();
    UpdateVisiblePanel();
}

void lms7002_mainPanel::OnOpenProject(wxCommandEvent& event)
{
    wxFileDialog dlg(this, _("Open config file"), "", "", "Project-File (*.ini)|*.ini", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() == wxID_CANCEL)
        return;

    LMS7002M* chip = GetSelectedChip();
    if (chip == nullptr)
    {
        wxMessageBox(_("Failed to load file: invalid chip index"), _("Warning"));
        return;
    }
    try
    {
        if (chip->LoadConfig(dlg.GetPath().ToStdString()) != OpStatus::Success)
            wxMessageBox(_("Failed to load file"), _("Warning"));
    } catch (std::runtime_error& e)
    {
        wxMessageBox("Failed to load file: "s + e.what(), _("Warning"));
        return;
    }
    wxCommandEvent tevt;
    // LMS_WriteParam(sdrDevice, LMS7002MCSR::MAC, rbChannelA->GetValue() == 1 ? 1 : 2);
    UpdateGUI();
}

void lms7002_mainPanel::OnSaveProject(wxCommandEvent& event)
{
    wxFileDialog dlg(this, _("Save config file"), "", "", "Project-File (*.ini)|*.ini", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() == wxID_CANCEL)
        return;

    LMS7002M* chip = GetSelectedChip();
    if (chip == nullptr)
    {
        wxMessageBox(_("Failed to save file: invalid chip index"), _("Warning"));
        return;
    }

    if (chip->SaveConfig(dlg.GetPath().ToStdString()) != OpStatus::Success)
        wxMessageBox(_("Failed to save file"), _("Warning"));
}

void lms7002_mainPanel::OnChannelOrSOCChange(wxCommandEvent& event)
{
    int channel = -1;
    if (rbChannelA->GetValue())
        channel = 0;
    else if (rbChannelB->GetValue())
        channel = 1;

    LMS7002M* chip = GetSelectedChip();
    for (auto iter : mTabs)
    {
        iter.second->Initialize(chip);
        if (iter.first == ID_TAB_SXR || iter.first == ID_TAB_SXT)
            continue; // do not change assigned channel for SXR/SXT tabs
        iter.second->SetChannel(channel);
    }
    UpdateVisiblePanel();
}

void lms7002_mainPanel::Onnotebook_modulesPageChanged(wxNotebookEvent& event)
{
    const wxWindowID pageId = tabsNotebook->GetCurrentPage()->GetId();
    switch (pageId)
    {
    case ID_TAB_AFE:
    case ID_TAB_BIAS:
    case ID_TAB_LDO:
    case ID_TAB_XBUF:
    case ID_TAB_CLKGEN:
    case ID_TAB_CDS:
    case ID_TAB_BIST:
        rbChannelA->Disable();
        rbChannelB->Disable();
        break;
    case ID_TAB_SXR: //change active channel to A
    case ID_TAB_SXT: //change active channel to B
        rbChannelA->Disable();
        rbChannelB->Disable();
        break;
    default:
        rbChannelA->Enable();
        rbChannelB->Enable();
    }

#ifdef __APPLE__
    //force show the page selected by the event (needed on apple)
    if (event.GetSelection() != -1)
    {
        dynamic_cast<wxNotebook*>(event.GetEventObject())->GetPage(event.GetSelection())->Show(true);
    }
#endif

    UpdateVisiblePanel();
}

void lms7002_mainPanel::OnDownloadAll(wxCommandEvent& event)
{
    try
    {
        soc->DownloadAll();
    } catch (std::runtime_error& e)
    {
        wxMessageBox("Download all registers failed: "s + e.what(), _("Warning"));
        return;
    }
    UpdateVisiblePanel();
}

void lms7002_mainPanel::OnUploadAll(wxCommandEvent& event)
{
    try
    {
        soc->UploadAll();
    } catch (std::runtime_error& e)
    {
        wxMessageBox("Download all registers failed: "s + e.what(), _("Warning"));
        return;
    }
    UpdateVisiblePanel();
}

void lms7002_mainPanel::OnReadTemperature(wxCommandEvent& event)
{
    double t = soc->GetTemperature();
    if (t == 0)
        wxMessageBox(_("Failed to read chip temperature"), _("Warning"));
    txtTemperature->SetLabel(wxString::Format("Temperature: %.0f C", t));
}

void lms7002_mainPanel::OnEnableMIMOchecked(wxCommandEvent& event)
{
    // TODO:
    // uint16_t chBck;
    // LMS_ReadParam(sdrDevice, LMS7002MCSR::MAC, &chBck);
    // bool enable = chkEnableMIMO->IsChecked();
    // for (int ch = enable ? 0 : 1; ch < LMS_GetNumChannels(sdrDevice, false); ch++) {
    //     LMS_EnableChannel(sdrDevice, LMS_CH_RX, ch, enable);
    //     LMS_EnableChannel(sdrDevice, LMS_CH_TX, ch, enable);
    // }
    // LMS_WriteParam(sdrDevice, LMS7002MCSR::MAC, chBck);
    // UpdateVisiblePanel();
}

LMS7002M* lms7002_mainPanel::GetSelectedChip() const
{
    return soc;
}
