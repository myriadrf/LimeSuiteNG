#ifndef __lms8001_mainPanel__
#define __lms8001_mainPanel__

#include "ISOCPanel.h"

namespace lime {
class LMS8001;
}

class lms8001_pnlMCU_BD_view;
class lms8001_pnlConfig_view;
class lms8001_pnlLDO_view;
class lms8001_pnlChannel_view;
class lms8001_pnlHLMIX_view;
class lms8001_pnlPLLProfiles_view;
class lms8001_pnlPLLConfig_view;

/** Implementing mainPanel */
class lms8001_mainPanel : public ISOCPanel
{
  public:
    lms8001_mainPanel(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);
    void UpdateVisiblePanel();
    virtual ~lms8001_mainPanel();
    void UpdateGUI();
    void Initialize(lime::LMS8001* pControl);

  protected:
    lime::LMS8001* lmsControl;
    void OnNewProject(wxCommandEvent& event);
    void OnOpenProject(wxCommandEvent& event);
    void OnSaveProject(wxCommandEvent& event);
    void OnUploadAll(wxCommandEvent& event);
    void OnDownloadAll(wxCommandEvent& event);
    void OnReset(wxCommandEvent& event);
    void OnSwitchChannel(wxCommandEvent& event);
    void OnUploadPanel(wxCommandEvent& event);

    void Onnotebook_modulesPageChanged(wxNotebookEvent& event);
    void OnResetChip(wxCommandEvent& event);

    enum {
        ID_BTN_RESET_CHIP = 2048,
        ID_BTN_CHIP_TO_GUI,
        ID_BTN_UPLOAD_PANEL,
        ID_TABS_NOTEBOOK,
        ID_TAB_CHIP_CONFIG,
        ID_TAB_LDO,
        ID_TAB_CHANNEL,
        ID_TAB_HLMIX,
        ID_TAB_PLLCONFIG,
        ID_TAB_PLLPROFILES
    };

    wxButton* btnNew;
    wxButton* btnOpen;
    wxButton* btnSave;
    wxRadioBox* rgrChannel;
    wxButton* btnResetChip;
    wxButton* btnDownloadAll;
    wxButton* btnUploadAll;
    wxButton* btnUploadPanel;
    wxNotebook* tabsNotebook;

  public:
    lms8001_pnlConfig_view* mTabChipConfig;
    lms8001_pnlLDO_view* mTabLDO;
    lms8001_pnlChannel_view* mTabChannel;
    lms8001_pnlHLMIX_view* mTabHLMIX;
    lms8001_pnlPLLConfig_view* mTabPLLConfig;
    lms8001_pnlPLLProfiles_view* mTabPLLProfiles;
};

#endif // __lms8001_mainPanel__
