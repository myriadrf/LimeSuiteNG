#ifndef __lms8001_pnlLDO_view__
#define __lms8001_pnlLDO_view__

#include "ILMS8001Tab.h"

class lms8001_pnlLDO_view : public ILMS8001Tab
{
  public:
    lms8001_pnlLDO_view(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);

  protected:
    enum {
        ID_PD_FRP_BIAS = 2048,
        ID_PD_F_BIAS,
        ID_PD_PTRP_BIAS,
        ID_PD_PT_BIAS,
        ID_PD_BIAS,
        ID_EN_LDO_LOBUFA,
        ID_EN_LDO_LOBUFB,
        ID_EN_LDO_LOBUFC,
        ID_EN_LDO_HFLNAA,
        ID_EN_LDO_HFLNAB,
        ID_EN_LDO_HFLNAC,
        ID_EN_LDO_HFLNAD,
        ID_EN_LDO_CLK_BUF,
        ID_EN_LDO_PLL_DIV,
        ID_EN_LDO_PLL_CP,
        ID_PD_LDO_DIG_CORE,
        ID_SPDUP_LDO_LOBUFA,
        ID_SPDUP_LDO_LOBUFB,
        ID_SPDUP_LDO_LOBUFC,
        ID_SPDUP_LDO_LOBUFD,
        ID_SPDUP_LDO_HFLNAA,
        ID_SPDUP_LDO_HFLNAB,
        ID_SPDUP_LDO_HFLNAC,
        ID_SPDUP_LDO_HFLNAD,
        ID_SPDUP_LDO_CLK_BUF,
        ID_SPDUP_LDO_PLL_DIV,
        ID_SPDUP_LDO_PLL_CP,
        ID_SPDUP_LDO_DIG_CORE,
        ID_EN_LOADIMP_LDO_LOBUFA,
        ID_EN_LOADIMP_LDO_LOBUFB,
        ID_EN_LOADIMP_LDO_LOBUFC,
        ID_EN_LOADIMP_LDO_LOBUFD,
        ID_EN_LOADIMP_LDO_HFLNAA,
        ID_EN_LOADIMP_LDO_HFLNAB,
        ID_EN_LOADIMP_LDO_HFLNAD,
        ID_EN_LOADIMP_LDO_CLK_BUF,
        ID_EN_LOADIMP_LDO_PLL_DIV,
        ID_EN_LOADIMP_LDO_PLL_CP,
        ID_EN_LOADIMP_LDO_DIG_CORE,
        ID_RDIV_LOBUFA,
        ID_RDIV_LOBUFB,
        ID_RDIV_LOBUFC,
        ID_RDIV_LOBUFD,
        ID_RDIV_HFLNAA,
        ID_RDIV_HFLNAB,
        ID_RDIV_HFLNAC,
        ID_RDIV_HFLNAD,
        ID_RDIV_CLK_BUF,
        ID_RDIV_PLL_DIV,
        ID_RDIV_PLL_CP,
        ID_RDIV_DIG_CORE,
        ID_PD_CALIB_COMP,
        ID_RP_CALIB_BIAS
    };

    wxPanel* ID_PANEL_LDO;
    wxCheckBox* chkPD_FRP_BIAS;
    wxCheckBox* chkPD_F_BIAS;
    wxCheckBox* chkPD_PTRP_BIAS;
    wxCheckBox* chkPD_PT_BIAS;
    wxCheckBox* chkPD_BIAS;
    wxCheckBox* chkEN_LDO_LOBUFA;
    wxCheckBox* chkEN_LDO_LOBUFB;
    wxCheckBox* chkEN_LDO_LOBUFC;
    wxCheckBox* chkEN_LDO_LOBUFD;
    wxCheckBox* chkEN_LDO_HFLNAA;
    wxCheckBox* chkEN_LDO_HFLNAB;
    wxCheckBox* chkEN_LDO_HFLNAC;
    wxCheckBox* chkEN_LDO_HFLNAD;
    wxCheckBox* chkEN_LDO_CLK_BUF;
    wxCheckBox* chkEN_LDO_PLL_DIV;
    wxCheckBox* chkEN_LDO_PLL_CP;
    wxCheckBox* chkPD_LDO_DIG_CORE;
    wxCheckBox* chkSPDUP_LDO_LOBUFA;
    wxCheckBox* chkSPDUP_LDO_LOBUFB;
    wxCheckBox* chkSPDUP_LDO_LOBUFC;
    wxCheckBox* chkSPDUP_LDO_LOBUFD;
    wxCheckBox* chkSPDUP_LDO_HFLNAA;
    wxCheckBox* chkSPDUP_LDO_HFLNAB;
    wxCheckBox* chkSPDUP_LDO_HFLNAC;
    wxCheckBox* chkSPDUP_LDO_HFLNAD;
    wxCheckBox* chkSPDUP_LDO_CLK_BUF;
    wxCheckBox* chkSPDUP_LDO_PLL_DIV;
    wxCheckBox* chkSPDUP_LDO_PLL_CP;
    wxCheckBox* chkSPDUP_LDO_DIG_CORE;
    wxCheckBox* chkEN_LOADIMP_LDO_LOBUFA;
    wxCheckBox* chkEN_LOADIMP_LDO_LOBUFB;
    wxCheckBox* chkEN_LOADIMP_LDO_LOBUFC;
    wxCheckBox* chkEN_LOADIMP_LDO_LOBUFD;
    wxCheckBox* chkEN_LOADIMP_LDO_HFLNAA;
    wxCheckBox* chkEN_LOADIMP_LDO_HFLNAB;
    wxCheckBox* chkEN_LOADIMP_LDO_HFLNAC;
    wxCheckBox* chkEN_LOADIMP_LDO_HFLNAD;
    wxCheckBox* chkEN_LOADIMP_LDO_CLK_BUF;
    wxCheckBox* chkEN_LOADIMP_LDO_PLL_DIV;
    wxCheckBox* chkEN_LOADIMP_LDO_PLL_CP;
    wxCheckBox* chkEN_LOADIMP_LDO_DIG_CORE;
    wxStaticText* m_staticText44;
    wxComboBox* cmbRDIV_LOBUFA;
    wxStaticText* m_staticText441;
    wxComboBox* cmbRDIV_LOBUFB;
    wxStaticText* m_staticText4411;
    wxComboBox* cmbRDIV_LOBUFC;
    wxStaticText* m_staticText44111;
    wxComboBox* cmbRDIV_LOBUFD;
    wxStaticText* m_staticText442;
    wxComboBox* cmbRDIV_HFLNAA;
    wxStaticText* m_staticText4421;
    wxComboBox* cmbRDIV_HFLNAB;
    wxStaticText* m_staticText44211;
    wxComboBox* cmbRDIV_HFLNAC;
    wxStaticText* m_staticText442111;
    wxComboBox* cmbRDIV_HFLNAD;
    wxStaticText* m_staticText4421111;
    wxComboBox* cmbRDIV_CLK_BUF;
    wxStaticText* m_staticText44211111;
    wxComboBox* cmbRDIV_PLL_DIV;
    wxStaticText* m_staticText442111111;
    wxComboBox* cmbRDIV_PLL_CP;
    wxStaticText* m_staticText4421111111;
    wxComboBox* cmbRDIV_DIG_CORE;
    wxCheckBox* chkPD_CALIB_COMP;
    wxCheckBox* chkRP_CALIB_COMP;
    wxStaticText* m_staticText43;
    wxSpinCtrl* cmbRP_CALIB_BIAS;
};

#endif // __lms8001_pnlLDO_view__
