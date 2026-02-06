#include <assert.h>
#include "lms8001_mainPanel.h"
//msavic here
#include "lms8001_pnlConfig_view.h"
#include "lms8001_pnlLDO_view.h"
#include "lms8001_pnlChannel_view.h"
#include "lms8001_pnlHLMIX_view.h"
#include "lms8001_pnlPLLConfig_view.h"
#include "lms8001_pnlPLLProfiles_view.h"

#include "SOC_GUIFactory.h"

#include "chips/LMS8001/LMS8001.h"
#include <wx/time.h>
#include <wx/msgdlg.h>
#include <iostream>
#include <wx/filedlg.h>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/ToString.h"

using namespace std;
using namespace lime;

static bool isRegistered = RegisterToFactory<SOC_GUIFactory, &lms8001_mainPanel::Create>("LMS8001");

ISOCPanel* lms8001_mainPanel::Create(wxWindow* parent, wxWindowID id)
{
    return new lms8001_mainPanel(parent, id);
}

lms8001_mainPanel::lms8001_mainPanel(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
    , lmsControl(nullptr)
{
    wxFlexGridSizer* fgSizer298;
    fgSizer298 = new wxFlexGridSizer(3, 1, 0, 0);
    fgSizer298->AddGrowableCol(0);
    fgSizer298->AddGrowableRow(1);
    fgSizer298->SetFlexibleDirection(wxBOTH);
    fgSizer298->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer299;
    fgSizer299 = new wxFlexGridSizer(0, 8, 0, 0);
    fgSizer299->AddGrowableCol(3);
    fgSizer299->SetFlexibleDirection(wxBOTH);
    fgSizer299->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    btnNew = new wxButton(this, wxID_ANY, wxT("New"), wxDefaultPosition, wxSize(-1, -1), 0);
    btnNew->SetDefault();
    fgSizer299->Add(btnNew, 1, wxEXPAND, 0);

    btnOpen = new wxButton(this, wxID_ANY, wxT("Open"), wxDefaultPosition, wxSize(-1, -1), 0);
    btnOpen->SetDefault();
    fgSizer299->Add(btnOpen, 1, wxEXPAND | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 0);

    btnSave = new wxButton(this, wxID_ANY, wxT("Save"), wxDefaultPosition, wxSize(-1, -1), 0);
    btnSave->SetDefault();
    fgSizer299->Add(btnSave, 1, wxEXPAND | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 0);

    wxString rgrChannelChoices[] = { wxT("A"), wxT("B"), wxT("C"), wxT("D") };
    int rgrChannelNChoices = sizeof(rgrChannelChoices) / sizeof(wxString);
    rgrChannel = new wxRadioBox(this,
        wxID_ANY,
        wxT("CHANNEL"),
        wxDefaultPosition,
        wxDefaultSize,
        rgrChannelNChoices,
        rgrChannelChoices,
        1,
        wxRA_SPECIFY_ROWS);
    rgrChannel->SetSelection(0);
    rgrChannel->Enable(false);

    fgSizer299->Add(rgrChannel, 0, wxALIGN_CENTER | wxALL, 0);

    btnResetChip = new wxButton(this, ID_BTN_RESET_CHIP, wxT("Reset"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer299->Add(btnResetChip, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxEXPAND, 0);

    btnDownloadAll = new wxButton(this, ID_BTN_CHIP_TO_GUI, wxT("Chip-->GUI"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer299->Add(btnDownloadAll, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxEXPAND, 0);

    btnUploadAll = new wxButton(this, wxID_ANY, wxT("GUI-->Chip"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer299->Add(btnUploadAll, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxEXPAND, 0);

    btnUploadPanel = new wxButton(this, ID_BTN_UPLOAD_PANEL, wxT("Reload Panel"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer299->Add(btnUploadPanel, 0, wxALL | wxEXPAND, 0);

    fgSizer298->Add(fgSizer299, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP | wxBOTTOM, 5);

    tabsNotebook = new wxNotebook(this, ID_TABS_NOTEBOOK, wxDefaultPosition, wxDefaultSize, 0);
    mTabChipConfig =
        new lms8001_pnlConfig_view(tabsNotebook, ID_TAB_CHIP_CONFIG, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    tabsNotebook->AddPage(mTabChipConfig, wxT("Chip Configuration"), true);
    mTabLDO = new lms8001_pnlLDO_view(tabsNotebook, ID_TAB_LDO, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    tabsNotebook->AddPage(mTabLDO, wxT("LDOs Configuration"), false);
    mTabChannel = new lms8001_pnlChannel_view(tabsNotebook, ID_TAB_CHANNEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    tabsNotebook->AddPage(mTabChannel, wxT("LMS8001A - Channel"), false);
    mTabHLMIX = new lms8001_pnlHLMIX_view(tabsNotebook, ID_TAB_HLMIX, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    tabsNotebook->AddPage(mTabHLMIX, wxT("LMS8001B - High-Linearity Mixer"), false);
    mTabPLLConfig =
        new lms8001_pnlPLLConfig_view(tabsNotebook, ID_TAB_PLLCONFIG, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    tabsNotebook->AddPage(mTabPLLConfig, wxT("PLL Configuration"), false);
    mTabPLLProfiles =
        new lms8001_pnlPLLProfiles_view(tabsNotebook, ID_TAB_PLLPROFILES, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    tabsNotebook->AddPage(mTabPLLProfiles, wxT("PLL Profiles"), false);

    fgSizer298->Add(tabsNotebook, 1, wxALIGN_LEFT | wxALIGN_TOP | wxEXPAND, 0);

    this->SetSizer(fgSizer298);
    this->Layout();
    fgSizer298->Fit(this);

    // Connect Events
    btnNew->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_mainPanel::OnNewProject), NULL, this);
    btnOpen->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_mainPanel::OnOpenProject), NULL, this);
    btnSave->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_mainPanel::OnSaveProject), NULL, this);
    rgrChannel->Connect(wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler(lms8001_mainPanel::OnSwitchChannel), NULL, this);
    btnResetChip->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_mainPanel::OnResetChip), NULL, this);
    btnDownloadAll->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_mainPanel::OnDownloadAll), NULL, this);
    btnUploadAll->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_mainPanel::OnUploadAll), NULL, this);
    btnUploadPanel->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_mainPanel::OnUploadPanel), NULL, this);
    tabsNotebook->Connect(
        wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED, wxNotebookEventHandler(lms8001_mainPanel::Onnotebook_modulesPageChanged), NULL, this);
}

lms8001_mainPanel::~lms8001_mainPanel()
{
}

void lms8001_mainPanel::UpdateVisiblePanel()
{
    wxLongLong t1, t2;
    t1 = wxGetUTCTimeMillis();
    long visibleTabId = tabsNotebook->GetCurrentPage()->GetId();
    switch (visibleTabId)
    {
    case ID_TAB_CHIP_CONFIG:
        mTabChipConfig->UpdateGUI();
        break;
    case ID_TAB_LDO:
        mTabLDO->UpdateGUI();
        break;
    case ID_TAB_CHANNEL:
        mTabChannel->UpdateGUI();
        break;
    case ID_TAB_HLMIX:
        mTabHLMIX->UpdateGUI();
        break;
    case ID_TAB_PLLCONFIG:
        mTabPLLConfig->UpdateGUI();
        break;
    case ID_TAB_PLLPROFILES:
        mTabPLLProfiles->UpdateGUI();
        break;
    }
    t2 = wxGetUTCTimeMillis();
#ifndef NDEBUG
    cout << "Visible GUI update time: " << (t2 - t1).ToString() << endl;
#endif
}

bool lms8001_mainPanel::Initialize(lime::LMS8001* pControl)
{
    assert(pControl != nullptr);
    if (!pControl)
        return false;

    lmsControl = pControl;
    //msavic ABCD
    lmsControl->channel = rgrChannel->GetSelection();
    //msavic PLLprofile
    lmsControl->PLLprofile = 0;
    //msavic here
    mTabChipConfig->Initialize(lmsControl);
    mTabLDO->Initialize(lmsControl);
    mTabChannel->Initialize(lmsControl);
    mTabHLMIX->Initialize(lmsControl);
    mTabPLLConfig->Initialize(lmsControl);
    mTabPLLProfiles->Initialize(lmsControl);
    UpdateGUI();
    return true;
}

bool lms8001_mainPanel::Initialize(void* pControl)
{
    return Initialize(reinterpret_cast<lime::LMS8001*>(pControl));
}

void lms8001_mainPanel::OnResetChip(wxCommandEvent& event)
{
    OpStatus status = lmsControl->ResetChip();
    if (status != OpStatus::Success)
        wxMessageBox(wxString::Format(_("Chip reset: %s"), wxString(ToString(status))), _("Warning"));

    wxNotebookEvent evt;
    Onnotebook_modulesPageChanged(evt); //after reset chip active channel might change, this refresh channel for active tab
}

void lms8001_mainPanel::UpdateGUI()
{
    UpdateVisiblePanel();
}

void lms8001_mainPanel::OnNewProject(wxCommandEvent& event)
{
    lmsControl->ResetChip();
    // lmsControl->DownloadAll();
    //msavic ABCD
    //    lmsControl->Modify_SPI_Reg_bits(MAC, rbChannelA->GetValue() == 1 ? 1 : 2);
    //    lmsControl->Modify_SPI_Reg_bits(MAC, rgrChannel->GetSelection());
    UpdateGUI();
}

void lms8001_mainPanel::OnOpenProject(wxCommandEvent& event)
{
    wxFileDialog dlg(this, _("Open config file"), "", "", "Project-File (*.ini)|*.ini", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() == wxID_CANCEL)
        return;
    OpStatus status = lmsControl->LoadConfig(dlg.GetPath().ToStdString());
    if (status != OpStatus::Success)
    {
        wxMessageBox(_("Failed to load file"), _("Warning"));
    }
    wxCommandEvent tevt;

    UpdateGUI();
}

void lms8001_mainPanel::OnSaveProject(wxCommandEvent& event)
{
    wxFileDialog dlg(this, _("Save config file"), "", "", "Project-File (*.ini)|*.ini", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() == wxID_CANCEL)
        return;
    OpStatus status = lmsControl->SaveConfig(dlg.GetPath().ToStdString());
    if (status != OpStatus::Success)
        wxMessageBox(_("Failed to save file"), _("Warning"));
}

void lms8001_mainPanel::OnSwitchChannel(wxCommandEvent& event)
{
    lmsControl->channel = rgrChannel->GetSelection();
    UpdateVisiblePanel();
}

void lms8001_mainPanel::Onnotebook_modulesPageChanged(wxNotebookEvent& event)
{
    wxNotebookPage* page = tabsNotebook->GetCurrentPage();
    if (page == mTabChannel || page == mTabHLMIX)
        rgrChannel->Enable();
    else
        rgrChannel->Disable();
    UpdateVisiblePanel();
}

void lms8001_mainPanel::OnDownloadAll(wxCommandEvent& event)
{
    // OpStatus status = lmsControl->DownloadAll();
    // if (status != OpStatus::Success)
    //     wxMessageBox(wxString::Format(_("Download all registers: %s"), wxString::From8BitData(OpStatus2string(status))), _("Warning"));
    UpdateVisiblePanel();
}

void lms8001_mainPanel::OnUploadAll(wxCommandEvent& event)
{
    // OpStatus status = lmsControl->UploadAll();
    // if (status != OpStatus::Success)
    //     wxMessageBox(wxString::Format(_("Upload all registers: %s"), wxString::From8BitData(OpStatus2string(status))), _("Warning"));
    UpdateVisiblePanel();
}
//mb added:
void lms8001_mainPanel::OnUploadPanel(wxCommandEvent& event)
{
    UpdateVisiblePanel();
}