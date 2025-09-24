#ifndef __lms8001_pnlPLLConfig_view__
#define __lms8001_pnlPLLConfig_view__

#include "ILMS8001Tab.h"

class lms8001_pnlPLLConfig_view : public ILMS8001Tab
{
  public:
    lms8001_pnlPLLConfig_view(wxWindow* parent);
    lms8001_pnlPLLConfig_view(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);

    void OnSelectProfileClick(wxCommandEvent& event);

  protected:
    enum {
        ID_CHx_PA_ILIN2X = 2048,
        ID_RDIV_LOBUFA,
        ID_VCO_SEL_FINAL,
        ID_VTUNE_HIGH,
        ID_VTUNE_LOW,
        ID_PLL_LOCK,
        ID_PLL_CFG_SEL0_MASK0,
        ID_PLL_CFG_SEL0_MASK1,
        ID_PLL_CFG_SEL0_MASK2,
        ID_PLL_CFG_SEL0_MASK3,
        ID_PLL_CFG_SEL0_MASK4,
        ID_PLL_CFG_SEL0_MASK5,
        ID_PLL_CFG_SEL0_MASK6,
        ID_PLL_CFG_SEL0_MASK7,
        ID_PLL_CFG_SEL0_MASK8,
        ID_PLL_CFG_SEL1_MASK0,
        ID_PLL_CFG_SEL1_MASK1,
        ID_PLL_CFG_SEL1_MASK2,
        ID_PLL_CFG_SEL1_MASK3,
        ID_PLL_CFG_SEL1_MASK4,
        ID_PLL_CFG_SEL1_MASK5,
        ID_PLL_CFG_SEL1_MASK6,
        ID_PLL_CFG_SEL1_MASK7,
        ID_PLL_CFG_SEL1_MASK8,
        ID_PLL_CFG_SEL2_INTERNAL,
        ID_PLL_CFG_SEL2_INVERT,
        ID_PLL_CFG_SEL2_MASK0,
        ID_PLL_CFG_SEL2_MASK1,
        ID_PLL_CFG_SEL2_MASK2,
        ID_PLL_CFG_SEL2_MASK3,
        ID_PLL_CFG_SEL2_MASK4,
        ID_PLL_CFG_SEL2_MASK5,
        ID_PLL_CFG_SEL2_MASK6,
        ID_PLL_CFG_SEL2_MASK7,
        ID_PLL_CFG_SEL2_MASK8,
        ID_PLL_CFG_INT_SEL0,
        ID_PLL_CFG_INT_SEL1,
        ID_PLL_CFG_INT_SEL2,
        ID_PLL_LODIST_ICT_CORE,
        ID_PLL_LODIST_ICT_BUF,
        ID_BSTATE,
        ID_EN_SDM_TSTO
    };

    wxNotebook* ID_NOTEBOOK_PLLCONFIG;
    wxPanel* ID_PANEL_PLLCONFIG;
    wxCheckBox* chkEN_VCOBIAS;
    wxCheckBox* chkBYP_VCOREG;
    wxCheckBox* chkCURLIM_VCOREG;
    wxCheckBox* chkSPDUP_VCOREG;
    wxStaticText* m_staticText991;
    wxComboBox* cmbVDIV_VCOREG;
    wxCheckBox* chkPLL_XBUF_SLFBEN;
    wxCheckBox* chkPLL_XBUF_BYPEN;
    wxCheckBox* chkPLL_XBUF_EN;
    wxStaticText* m_staticText99211;
    wxSpinCtrl* cmbVCO_FREQ_MAN;
    wxStaticText* m_staticText9923;
    wxSpinCtrl* cmbVCO_SEL_MAN;
    wxStaticText* m_staticText99231;
    wxStaticText* m_staticText99232;
    wxStaticText* m_staticText99233;
    wxCheckBox* chkFREQ_LOW;
    wxCheckBox* chkFREQ_EQUAL;
    wxCheckBox* chkFREQ_HIGH;
    wxCheckBox* chkCTUNE_STEP_DONE;
    wxCheckBox* chkCTUNE_START;
    wxCheckBox* chkCTUNE_EN;
    wxStaticText* m_staticText1184;
    wxCheckBox* chkVTUNE_HIGH;
    wxCheckBox* chkVTUNE_LOW;
    wxCheckBox* chkPLL_LOCK;
    wxCheckBox* chkFCAL_START;
    wxCheckBox* chkVCO_SEL_FINAL_VAL;
    wxStaticText* m_staticText992;
    wxSpinCtrl* cmbVCO_SEL_FINAL;
    wxCheckBox* chkFREQ_FINAL_VAL;
    wxStaticText* m_staticText9921;
    wxSpinCtrl* cmbFREQ_FINAL;
    wxCheckBox* chkVCO_SEL_FORCE;
    wxStaticText* m_staticText9922;
    wxSpinCtrl* cmbVCO_SEL_INIT;
    wxStaticText* m_staticText99221;
    wxSpinCtrl* cmbFREQ_INIT_POS;
    wxStaticText* m_staticText992211;
    wxSpinCtrl* cmbFREQ_INIT;
    wxStaticText* m_staticText9922111;
    wxSpinCtrl* cmbFREQ_SETTLING_N;
    wxStaticText* m_staticText99221111;
    wxSpinCtrl* cmbVTUNE_WAIT_N;
    wxStaticText* m_staticText992211111;
    wxSpinCtrl* cmbVCO_SEL_FREQ_MAX;
    wxStaticText* m_staticText9922111111;
    wxSpinCtrl* cmbVCO_SEL_FREQ_MIN;
    wxCheckBox* chkSelectProfile;
    wxComboBox* chkPLL_CFG_INT_SEL;
    wxNotebook* ID_NOTEBOOK_MUX_CONTROL_PLL;
    wxPanel* ID_NOTEBOOK_MUX_CONTROL_PLL_SEL0;
    wxCheckBox* chkPLL_CFG_SEL0_INTERNAL;
    wxCheckBox* chkPLL_CFG_SEL0_INVERT;
    wxStaticText* m_staticText26510;
    wxStaticText* m_staticText26512;
    wxStaticText* m_staticText26522;
    wxStaticText* m_staticText26532;
    wxStaticText* m_staticText26542;
    wxStaticText* m_staticText26552;
    wxStaticText* m_staticText26562;
    wxStaticText* m_staticText26572;
    wxStaticText* m_staticText26582;
    wxCheckBox* chkPLL_CFG_SEL0_MASK0;
    wxCheckBox* chkPLL_CFG_SEL0_MASK1;
    wxCheckBox* chkPLL_CFG_SEL0_MASK2;
    wxCheckBox* chkPLL_CFG_SEL0_MASK3;
    wxCheckBox* chkPLL_CFG_SEL0_MASK4;
    wxCheckBox* chkPLL_CFG_SEL0_MASK5;
    wxCheckBox* chkPLL_CFG_SEL0_MASK6;
    wxCheckBox* chkPLL_CFG_SEL0_MASK7;
    wxCheckBox* chkPLL_CFG_SEL0_MASK8;
    wxPanel* ID_NOTEBOOK_MUX_CONTROL_PLL_SEL1;
    wxCheckBox* chkPLL_CFG_SEL1_INTERNAL;
    wxCheckBox* chkPLL_CFG_SEL1_INVERT;
    wxStaticText* m_staticText265101;
    wxStaticText* m_staticText265121;
    wxStaticText* m_staticText265221;
    wxStaticText* m_staticText265321;
    wxStaticText* m_staticText265421;
    wxStaticText* m_staticText265521;
    wxStaticText* m_staticText265621;
    wxStaticText* m_staticText265721;
    wxStaticText* m_staticText265821;
    wxCheckBox* chkPLL_CFG_SEL1_MASK0;
    wxCheckBox* chkPLL_CFG_SEL1_MASK1;
    wxCheckBox* chkPLL_CFG_SEL1_MASK2;
    wxCheckBox* chkPLL_CFG_SEL1_MASK3;
    wxCheckBox* chkPLL_CFG_SEL1_MASK4;
    wxCheckBox* chkPLL_CFG_SEL1_MASK5;
    wxCheckBox* chkPLL_CFG_SEL1_MASK6;
    wxCheckBox* chkPLL_CFG_SEL1_MASK7;
    wxCheckBox* chkPLL_CFG_SEL1_MASK8;
    wxPanel* ID_NOTEBOOK_MUX_CONTROL_PLL_SEL2;
    wxCheckBox* chkPLL_CFG_SEL2_INTERNAL;
    wxCheckBox* chkPLL_CFG_SEL2_INVERT;
    wxStaticText* m_staticText2651011;
    wxStaticText* m_staticText2651211;
    wxStaticText* m_staticText2652211;
    wxStaticText* m_staticText2653211;
    wxStaticText* m_staticText2654211;
    wxStaticText* m_staticText2655211;
    wxStaticText* m_staticText2656211;
    wxStaticText* m_staticText2657211;
    wxStaticText* m_staticText2658211;
    wxCheckBox* chkPLL_CFG_SEL2_MASK0;
    wxCheckBox* chkPLL_CFG_SEL2_MASK1;
    wxCheckBox* chkPLL_CFG_SEL2_MASK2;
    wxCheckBox* chkPLL_CFG_SEL2_MASK3;
    wxCheckBox* chkPLL_CFG_SEL2_MASK4;
    wxCheckBox* chkPLL_CFG_SEL2_MASK5;
    wxCheckBox* chkPLL_CFG_SEL2_MASK6;
    wxCheckBox* chkPLL_CFG_SEL2_MASK7;
    wxCheckBox* chkPLL_CFG_SEL2_MASK8;
    wxCheckBox* chkPLL_CFG_INT_SEL0;
    wxCheckBox* chkPLL_CFG_INT_SEL1;
    wxCheckBox* chkPLL_CFG_INT_SEL2;
    wxRadioBox* rgrPLL_RSTN;
    wxStaticText* m_staticText992111;
    wxComboBox* cmbCTUNE_RES;
    wxCheckBox* chkPLL_CALIBRATION_MODE;
    wxCheckBox* chkPLL_CALIBRATION_EN;
    wxCheckBox* chkPLL_FLOCK_INTERNAL;
    wxCheckBox* chkPLL_FLOCK_INTVAL;
    wxCheckBox* chkSEL_BIAS_CORE;
    wxStaticText* m_staticText1191;
    wxSpinCtrl* cmbPLL_LODIST_ICT_CORE;
    wxStaticText* m_staticText11911;
    wxSpinCtrl* cmbPLL_LODIST_ICT_BUF;
    wxChoice* choPLL_ICT_OUT0;
    wxStaticText* m_staticText1195;
    wxChoice* choPLL_ICT_OUT1;
    wxStaticText* m_staticText11951;
    wxChoice* choPLL_ICT_OUT2;
    wxStaticText* m_staticText119511;
    wxChoice* choPLL_ICT_OUT3;
    wxStaticText* m_staticText1195111;
    wxStaticText* m_staticText1205;
    wxSpinCtrl* cmbBSIGL;
    wxCheckBox* chkBSTATE;
    wxCheckBox* chkEN_SDM_TSTO;
    wxCheckBox* chkBEN;
    wxCheckBox* chkBSTART;
    wxStaticText* m_staticText1206;
    wxSpinCtrl* cmbBSIGH;
};

#endif // __lms8001_pnlPLLConfig_view__
