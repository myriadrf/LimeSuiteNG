#ifndef __lms8001_pnlHLMIX_view__
#define __lms8001_pnlHLMIX_view__

#include "ILMS8001Tab.h"

class lms8001_pnlHLMIX_view : public ILMS8001Tab
{
  public:
    lms8001_pnlHLMIX_view(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);

  protected:
    enum { ID_HLMIXx_VGCAS0 = 2048, ID_RP_CALIB_BIAS, ID_CHx_PA_ILIN2X, ID_HLMIXx_VGCAS1, ID_HLMIXx_VGCAS2, ID_HLMIXx_VGCAS3 };

    wxNotebook* ID_NOTEBOOK_HLMIX;
    wxPanel* ID_PANEL_HLMIX;
    wxNotebook* ID_NOTEBOOK_HLMIX_PROGRAM;
    wxPanel* ID_PANEL_HLMIX_P0;
    wxStaticText* m_staticText6493;
    wxSpinCtrl* cmbHLMIXx_VGCAS0;
    wxStaticText* m_staticText649;
    wxSpinCtrl* cmbHLMIXx_ICT_BIAS0;
    wxCheckBox* chkHLMIXx_BIAS_PD0;
    wxCheckBox* chkHLMIXx_LOBUF_PD0;
    wxStaticText* m_staticText6491;
    wxSpinCtrl* cmbHLMIXx_MIXLOSS0;
    wxPanel* ID_PANEL_HLMIX_P1;
    wxStaticText* m_staticText64922;
    wxSpinCtrl* cmbHLMIXx_VGCAS1;
    wxStaticText* m_staticText6492;
    wxSpinCtrl* cmbHLMIXx_ICT_BIAS1;
    wxCheckBox* chkHLMIXx_BIAS_PD1;
    wxCheckBox* chkHLMIXx_LOBUF_PD1;
    wxStaticText* m_staticText64912;
    wxSpinCtrl* cmbHLMIXx_MIXLOSS1;
    wxPanel* ID_PANEL_HLMIX_P2;
    wxStaticText* m_staticText649212;
    wxSpinCtrl* cmbHLMIXx_VGCAS2;
    wxStaticText* m_staticText64921;
    wxSpinCtrl* cmbHLMIXx_ICT_BIAS2;
    wxCheckBox* chkHLMIXx_BIAS_PD2;
    wxCheckBox* chkHLMIXx_LOBUF_PD2;
    wxStaticText* m_staticText649121;
    wxSpinCtrl* cmbHLMIXx_MIXLOSS2;
    wxPanel* ID_PANEL_HLMIX_P3;
    wxStaticText* m_staticText6492121;
    wxSpinCtrl* cmbHLMIXx_VGCAS3;
    wxStaticText* m_staticText649211;
    wxSpinCtrl* cmbHLMIXx_ICT_BIAS3;
    wxCheckBox* chkHLMIXx_BIAS_PD3;
    wxCheckBox* chkHLMIXx_LOBUF_PD3;
    wxStaticText* m_staticText6491211;
    wxSpinCtrl* cmbHLMIXx_MIXLOSS3;
    wxPanel* ID_PANEL_HLMIX_RB;
    wxStaticText* m_staticText64921211;
    wxSpinCtrl* cmbHLMIXx_VGCAS_RB;
    wxStaticText* m_staticText6492111;
    wxSpinCtrl* cmbHLMIXx_ICT_BIAS_RB;
    wxCheckBox* chkHLMIXx_BIAS_PD_RB;
    wxCheckBox* chkHLMIXx_LOBUF_PD_RB;
    wxStaticText* m_staticText64912111;
    wxSpinCtrl* cmbHLMIXx_MIXLOSS_RB;
    wxNotebook* ID_NOTEBOOK_MUX_CONTROL_HLMIX_CONFIG;
    wxPanel* ID_PANEL_MUX_HLMIX_CONFIG_SEL0;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_INTERNAL;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_INVERT;
    wxStaticText* m_staticText26510;
    wxStaticText* m_staticText26512;
    wxStaticText* m_staticText26522;
    wxStaticText* m_staticText26532;
    wxStaticText* m_staticText26542;
    wxStaticText* m_staticText26552;
    wxStaticText* m_staticText26562;
    wxStaticText* m_staticText26572;
    wxStaticText* m_staticText26582;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_MASK0;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_MASK1;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_MASK2;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_MASK3;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_MASK4;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_MASK5;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_MASK6;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_MASK7;
    wxCheckBox* chkHLMIXx_CONFIG_SEL0_MASK8;
    wxPanel* ID_PANEL_MUX_HLMIX_CONFIG_SEL1;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_INTERNAL;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_INVERT;
    wxStaticText* m_staticText26591;
    wxStaticText* m_staticText265111;
    wxStaticText* m_staticText265211;
    wxStaticText* m_staticText265311;
    wxStaticText* m_staticText265411;
    wxStaticText* m_staticText265511;
    wxStaticText* m_staticText265611;
    wxStaticText* m_staticText265711;
    wxStaticText* m_staticText265811;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_MASK0;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_MASK1;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_MASK2;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_MASK3;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_MASK4;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_MASK5;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_MASK6;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_MASK7;
    wxCheckBox* chkHLMIXx_CONFIG_SEL1_MASK8;
    wxCheckBox* chkHLMIXx_CONFIG_INT_SEL0;
    wxCheckBox* chkHLMIXx_CONFIG_INT_SEL1;
    wxNotebook* ID_NOTEBOOK_MUX_CONTROL_HLMIX;
    wxPanel* ID_PANEL_MUX_HLMIX_SEL0;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_INTERNAL;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_INVERT;
    wxStaticText* m_staticText265;
    wxStaticText* m_staticText2651;
    wxStaticText* m_staticText2652;
    wxStaticText* m_staticText2653;
    wxStaticText* m_staticText2654;
    wxStaticText* m_staticText2655;
    wxStaticText* m_staticText2656;
    wxStaticText* m_staticText2657;
    wxStaticText* m_staticText2658;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_MASK0;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_MASK1;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_MASK2;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_MASK3;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_MASK4;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_MASK5;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_MASK6;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_MASK7;
    wxCheckBox* chkHLMIXx_LOSS_SEL0_MASK8;
    wxPanel* ID_PANEL_MUX_HLMIX_SEL1;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_INTERNAL;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_INVERT;
    wxStaticText* m_staticText2659;
    wxStaticText* m_staticText26511;
    wxStaticText* m_staticText26521;
    wxStaticText* m_staticText26531;
    wxStaticText* m_staticText26541;
    wxStaticText* m_staticText26551;
    wxStaticText* m_staticText26561;
    wxStaticText* m_staticText26571;
    wxStaticText* m_staticText26581;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_MASK0;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_MASK1;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_MASK2;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_MASK3;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_MASK4;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_MASK5;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_MASK6;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_MASK7;
    wxCheckBox* chkHLMIXx_LOSS_SEL1_MASK8;
    wxCheckBox* chkHLMIXx_LOSS_INT_SEL0;
    wxCheckBox* chkHLMIXx_LOSS_INT_SEL1;
};

#endif // __lms8001_pnlHLMIX_view__
