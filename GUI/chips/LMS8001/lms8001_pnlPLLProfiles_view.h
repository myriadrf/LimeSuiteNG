#ifndef __lms8001_pnlPLLProfiles_view__
#define __lms8001_pnlPLLProfiles_view__

#include "ILMS8001Tab.h"
#include "limesuiteng/OpStatus.h"

class lms8001_pnlPLLProfiles_view : public ILMS8001Tab
{
  public:
    lms8001_pnlPLLProfiles_view(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);

    void OnSwitchPLLProfile(wxCommandEvent& event);

    void chkPLL_LODIST_FSP_OUTCH2_n_Change(wxCommandEvent& event);

    void OnTuneClick(wxCommandEvent& event);

    void OnSmartTuneClick(wxCommandEvent& event);

    void On_FF_MOD_Change(wxCommandEvent& event);

    void OnUpdateUI_cmbPLL_LODIST_FSP_OUTCH10_n(wxUpdateUIEvent& event);

    void OnupdateUI_sttxtPLLFreq(wxUpdateUIEvent& event);
    void OnUpdateUI_sttxtFLockN(wxUpdateUIEvent& event);

    void OnUpdateUI_cmbFLOCK_PULSE_n(wxUpdateUIEvent& event);

    void OnTextRefClock(wxCommandEvent& event);

    lime::OpStatus configPLL(double fLO,
        int fref,
        bool slfbenXBUF,
        bool genIQ,
        bool intMode,
        int loopBW,
        double pm,
        bool fitKVCO,
        double bwef,
        int flock_N);
    lime::OpStatus setLOFREQ(double fLO, int fref, bool slfbenXBUF, bool genIQ, bool intMode);
    lime::OpStatus setFFDIV(int FFMOD);
    lime::OpStatus vco_auto_ctune(double fVCO,
        int fref,
        bool slfbenXBUF,
        bool intMode,
        bool pdiv2 = false,
        int vtune_vct = 1,
        int vco_sel_force = 0,
        int vco_sel_init = 2,
        int freq_init_pos = 7,
        int freq_init = 0,
        int freq_settling_N = 4,
        int vtune_wait_N = 128,
        int vco_sel_freq_max = 250,
        int vco_sel_freq_min = 5);
    void calc_fbdiv(double fVCO, int fref, bool intMode, bool pdiv2, int* nint, int* nfrac, int* nfix);
    void enablePLL(bool pdiv2, bool intMode, bool slfbenXBUF);
    lime::OpStatus centerVTUNE2(int fref, bool slfbenXBUF);
    double getNDIV();
    lime::OpStatus optim_PLL_LoopBW(double pm, double fc, bool fitKVCO);
    void setCP(int PULSE, int OFS, int ICT_CP);
    void setLPF(int C1, int C2, int R2, int C3, int R3);
    lime::OpStatus optimCPandLD();
    void setLD(int LD_VCT);
    lime::OpStatus setFLOCK(double bwef, int flock_N);

    void UpdateGUI() override;

  protected:
    enum {
        ID_VTUNE_HIGH = 2048,
        ID_VTUNE_LOW,
        ID_PLL_LOCK,
        ID_TEST,
        ID_CHx_PA_ILIN2X,
        ID_VCO_SEL_FINAL,
        ID_RDIV_LOBUFA,
        ID_FLOCK_VCO_SPDUP_n,
        ID_chkPLL_LODIST_FSP_OUT02_n
    };

    wxRadioBox* rgrPLLProfile;
    wxStaticText* m_staticText272;
    wxCheckBox* chkVTUNE_HIGH;
    wxCheckBox* chkVTUNE_LOW;
    wxCheckBox* chkPLL_LOCK;
    wxStaticText* sttxtPLLFreq;
    wxStaticText* sttxtPLLFreq1;
    wxCheckBox* chkPLL_LODIST_EN_BIAS_n;
    wxCheckBox* chkPLL_LODIST_EN_DIV2IQ_n;
    wxCheckBox* chkPLL_EN_VTUNE_COMP_n;
    wxCheckBox* chkPLL_EN_LD_n;
    wxCheckBox* chkPLL_EN_PFD_n;
    wxCheckBox* chkPLL_EN_CP_n;
    wxCheckBox* chkPLL_EN_CPOFS_n;
    wxCheckBox* chkPLL_EN_VCO_n;
    wxCheckBox* chkPLL_EN_FFDIV_n;
    wxCheckBox* chkPLL_EN_FB_PDIV2_n;
    wxCheckBox* chkPLL_EN_FFCORE_n;
    wxCheckBox* chkPLL_EN_FBDIV_n;
    wxCheckBox* chkPLL_SDM_CLK_EN_n;
    wxCheckBox* chkFLIP_n;
    wxStaticText* m_staticText1328;
    wxSpinCtrl* cmbDEL_n;
    wxStaticText* m_staticText13281;
    wxComboBox* cmbPULSE_n;
    wxStaticText* m_staticText132811;
    wxComboBox* cmbOFS_n;
    wxStaticText* m_staticText1328111;
    wxComboBox* cmbLD_VCT_n;
    wxStaticText* m_staticText13281111;
    wxSpinCtrl* cmbICT_CP_n;
    wxStaticText* m_staticText1341;
    wxSpinCtrl* cmbVCO_FREQ_n;
    wxCheckBox* chkSPDUP_VCO_n;
    wxCheckBox* chkVCO_AAC_EN_n;
    wxStaticText* m_staticText1342;
    wxComboBox* cmbVDIV_SWVDD_n;
    wxStaticText* m_staticText1343;
    wxComboBox* cmbVCO_SEL_n;
    wxStaticText* m_staticText1344;
    wxSpinCtrl* cmbVCO_AMP_n;
    wxStaticText* m_staticText13113;
    wxComboBox* cmbFLOCK_R3_n;
    wxStaticText* m_staticText131111;
    wxComboBox* cmbFLOCK_R2_n;
    wxStaticText* m_staticText1311221;
    wxComboBox* cmbFLOCK_C3_n;
    wxStaticText* m_staticText131123;
    wxComboBox* cmbFLOCK_C2_n;
    wxStaticText* m_staticText1311211;
    wxComboBox* cmbFLOCK_C1_n;
    wxStaticText* m_staticText132812;
    wxComboBox* cmbFLOCK_PULSE_n;
    wxStaticText* m_staticText1328112;
    wxComboBox* cmbFLOCK_OFS_n;
    wxCheckBox* chkFLOCK_VCO_SPDUP_n;
    wxStaticText* m_staticText134211221;
    wxSpinCtrl* cmbFLOCK_N_n;
    wxStaticText* sttxtFLockN;
    wxStaticText* m_staticText1328122;
    wxCheckBox* chkFLOCK_LODIST_EN_OUT0_n;
    wxCheckBox* chkFLOCK_LODIST_EN_OUT1_n;
    wxCheckBox* chkFLOCK_LODIST_EN_OUT2_n;
    wxCheckBox* chkFLOCK_LODIST_EN_OUT3_n;
    wxStaticText* m_staticText1311;
    wxComboBox* cmbR3_n;
    wxStaticText* m_staticText13111;
    wxComboBox* cmbR2_n;
    wxStaticText* m_staticText131122;
    wxComboBox* cmbC3_n;
    wxStaticText* m_staticText13112;
    wxComboBox* cmbC2_n;
    wxStaticText* m_staticText131121;
    wxComboBox* cmbC1_n;
    wxStaticText* m_staticText1324;
    wxComboBox* cmbVTUNE_VCT_n;
    wxCheckBox* chkLPFSW_n;
    wxStaticText* m_staticText134211;
    wxComboBox* cmbFF_MOD_n;
    wxStaticText* m_staticText1342112;
    wxCheckBox* chkEnableFFDIVDebug;
    wxCheckBox* chkFFDIV_SEL_n;
    wxStaticText* m_staticText13421;
    wxComboBox* cmbFFCORE_MOD_n;
    wxCheckBox* chkINTMOD_EN_n;
    wxCheckBox* chkDITHER_EN_n;
    wxCheckBox* chkSEL_SDMCLK_n;
    wxCheckBox* chkREV_SDMCLK_n;
    wxStaticText* m_staticText1342111;
    wxSpinCtrl* cmbINTMOD_n;
    wxStaticText* m_staticText13421111;
    wxSpinCtrl* cmbFRACMODL_n;
    wxStaticText* m_staticText134211111;
    wxSpinCtrl* cmbFRACMODH_n;
    wxStaticText* m_staticText13421123;
    wxStaticText* m_staticText134211231;
    wxStaticText* m_staticText1342112311;
    wxStaticText* m_staticText1342112312;
    wxStaticText* m_staticText262;
    wxStaticText* m_staticText13421123111;
    wxCheckBox* chkPLL_LODIST_EN_OUT0_n;
    wxCheckBox* chkPLL_LODIST_FSP_OUT02_n;
    wxComboBox* cmbPLL_LODIST_FSP_OUT010_n;
    wxStaticText* sttxtLODistrChAFreq;
    wxStaticText* m_staticText134211231111;
    wxCheckBox* chkPLL_LODIST_EN_OUT1_n;
    wxCheckBox* chkPLL_LODIST_FSP_OUT12_n;
    wxComboBox* cmbPLL_LODIST_FSP_OUT110_n;
    wxStaticText* sttxtLODistrChBFreq;
    wxStaticText* m_staticText1342112311111;
    wxCheckBox* chkPLL_LODIST_EN_OUT2_n;
    wxCheckBox* chkPLL_LODIST_FSP_OUT22_n;
    wxComboBox* cmbPLL_LODIST_FSP_OUT210_n;
    wxStaticText* sttxtLODistrChCFreq;
    wxStaticText* m_staticText13421123111111;
    wxCheckBox* chkPLL_LODIST_EN_OUT3_n;
    wxCheckBox* chkPLL_LODIST_FSP_OUT32_n;
    wxComboBox* cmbPLL_LODIST_FSP_OUT310_n;
    wxStaticText* sttxtLODistrChDFreq;
    wxTextCtrl* txtRefClock;
    wxStaticText* m_staticText273;
    wxTextCtrl* txtVCOFrequency;
    wxButton* btnTune;
    wxStaticText* m_staticText2731;
    wxTextCtrl* txtSmartTunePLLFrequency;
    wxCheckBox* chkSmartTuneGenIQ;
    wxStaticText* m_staticText27311;
    wxTextCtrl* txtSmartTuneLoopBW;
    wxStaticText* m_staticText273111;
    wxTextCtrl* txtSmartTunePhaseMargin;
    wxStaticText* m_staticText2731111;
    wxTextCtrl* txtSmartTuneBWEF;
    wxButton* btnSmartTune;
};

#endif // __lms8001_pnlPLLProfiles_view__
