#include "lms8001_pnlLDO_view.h"
#include <map>
#include <iostream>
#include "lms8001_gui_utilities.h"

#include "chips/LMS8001/LMS8001.h"

using namespace lime;

lms8001_pnlLDO_view::lms8001_pnlLDO_view(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ILMS8001Tab(parent, id, pos, size, style)
{
    wxFlexGridSizer* fgSizer69;
    fgSizer69 = new wxFlexGridSizer(0, 4, 0, 0);
    fgSizer69->SetFlexibleDirection(wxBOTH);
    fgSizer69->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer29;
    fgSizer29 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer29->SetFlexibleDirection(wxBOTH);
    fgSizer29->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer45;
    sbSizer45 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Power Controls")), wxVERTICAL);

    wxStaticBoxSizer* sbSizer35;
    sbSizer35 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Bias")), wxVERTICAL);

    chkPD_FRP_BIAS = new wxCheckBox(this, ID_PD_FRP_BIAS, wxT("PD_FRP_BIAS"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer35->Add(chkPD_FRP_BIAS, 0, wxALL, 0);

    chkPD_F_BIAS = new wxCheckBox(this, ID_PD_F_BIAS, wxT("PD_F_BIAS"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer35->Add(chkPD_F_BIAS, 0, wxALL, 0);

    chkPD_PTRP_BIAS = new wxCheckBox(this, ID_PD_PTRP_BIAS, wxT("PD_PTRP_BIAS"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer35->Add(chkPD_PTRP_BIAS, 0, wxALL, 0);

    chkPD_PT_BIAS = new wxCheckBox(this, ID_PD_PT_BIAS, wxT("PD_PT_BIAS"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer35->Add(chkPD_PT_BIAS, 0, wxALL, 0);

    chkPD_BIAS = new wxCheckBox(this, ID_PD_BIAS, wxT("PD_BIAS"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer35->Add(chkPD_BIAS, 0, wxALL, 0);

    sbSizer45->Add(sbSizer35, 0, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer481;
    sbSizer481 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("LOBUF")), wxVERTICAL);

    chkEN_LDO_LOBUFA = new wxCheckBox(this, ID_EN_LDO_LOBUFA, wxT("EN_LDO_LOBUFA"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer481->Add(chkEN_LDO_LOBUFA, 0, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    chkEN_LDO_LOBUFB = new wxCheckBox(this, ID_EN_LDO_LOBUFB, wxT("EN_LDO_LOBUFB"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer481->Add(chkEN_LDO_LOBUFB, 0, wxALL, 0);

    chkEN_LDO_LOBUFC = new wxCheckBox(this, ID_EN_LDO_LOBUFC, wxT("EN_LDO_LOBUFC"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer481->Add(chkEN_LDO_LOBUFC, 0, wxALL, 0);

    chkEN_LDO_LOBUFD = new wxCheckBox(this, ID_EN_LDO_LOBUFC, wxT("EN_LDO_LOBUFD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer481->Add(chkEN_LDO_LOBUFD, 0, wxALL, 0);

    sbSizer45->Add(sbSizer481, 0, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer4811;
    sbSizer4811 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("LNA")), wxVERTICAL);

    chkEN_LDO_HFLNAA = new wxCheckBox(this, ID_EN_LDO_HFLNAA, wxT("EN_LDO_HFLNAA"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer4811->Add(chkEN_LDO_HFLNAA, 0, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    chkEN_LDO_HFLNAB = new wxCheckBox(this, ID_EN_LDO_HFLNAB, wxT("EN_LDO_HFLNAB"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer4811->Add(chkEN_LDO_HFLNAB, 0, wxALL, 0);

    chkEN_LDO_HFLNAC = new wxCheckBox(this, ID_EN_LDO_HFLNAC, wxT("EN_LDO_HFLNAC"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer4811->Add(chkEN_LDO_HFLNAC, 0, wxALL, 0);

    chkEN_LDO_HFLNAD = new wxCheckBox(this, ID_EN_LDO_HFLNAD, wxT("EN_LDO_HFLNAD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer4811->Add(chkEN_LDO_HFLNAD, 0, wxALL, 0);

    sbSizer45->Add(sbSizer4811, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer48111;
    sbSizer48111 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("PLL")), wxVERTICAL);

    chkEN_LDO_CLK_BUF = new wxCheckBox(this, ID_EN_LDO_CLK_BUF, wxT("EN_LDO_CLK_BUF"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer48111->Add(chkEN_LDO_CLK_BUF, 0, wxALL, 0);

    chkEN_LDO_PLL_DIV = new wxCheckBox(this, ID_EN_LDO_PLL_DIV, wxT("EN_LDO_PLL_DIV"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer48111->Add(chkEN_LDO_PLL_DIV, 0, wxALL, 0);

    chkEN_LDO_PLL_CP = new wxCheckBox(this, ID_EN_LDO_PLL_CP, wxT("EN_LDO_PLL_CP"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer48111->Add(chkEN_LDO_PLL_CP, 0, wxALL, 0);

    sbSizer45->Add(sbSizer48111, 0, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer481111;
    sbSizer481111 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Digital core")), wxVERTICAL);

    chkPD_LDO_DIG_CORE = new wxCheckBox(this, ID_PD_LDO_DIG_CORE, wxT("PD_LDO_DIG_CORE"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer481111->Add(chkPD_LDO_DIG_CORE, 0, wxALL, 0);

    sbSizer45->Add(sbSizer481111, 0, wxEXPAND, 5);

    fgSizer29->Add(sbSizer45, 1, wxEXPAND, 5);

    fgSizer69->Add(fgSizer29, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer30;
    fgSizer30 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer30->SetFlexibleDirection(wxBOTH);
    fgSizer30->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer46;
    sbSizer46 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Short noise filter resistor")), wxVERTICAL);

    chkSPDUP_LDO_LOBUFA = new wxCheckBox(this, ID_SPDUP_LDO_LOBUFA, wxT("SPDUP_LDO_LOBUFA"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_LOBUFA, 0, wxALL, 0);

    chkSPDUP_LDO_LOBUFB = new wxCheckBox(this, ID_SPDUP_LDO_LOBUFB, wxT("SPDUP_LDO_LOBUFB"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_LOBUFB, 0, wxALL, 0);

    chkSPDUP_LDO_LOBUFC = new wxCheckBox(this, ID_SPDUP_LDO_LOBUFC, wxT("SPDUP_LDO_LOBUFC"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_LOBUFC, 0, wxALL, 0);

    chkSPDUP_LDO_LOBUFD = new wxCheckBox(this, ID_SPDUP_LDO_LOBUFD, wxT("SPDUP_LDO_LOBUFD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_LOBUFD, 0, wxALL, 0);

    chkSPDUP_LDO_HFLNAA = new wxCheckBox(this, ID_SPDUP_LDO_HFLNAA, wxT("SPDUP_LDO_HFLNAA"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_HFLNAA, 0, wxALL, 0);

    chkSPDUP_LDO_HFLNAB = new wxCheckBox(this, ID_SPDUP_LDO_HFLNAB, wxT("SPDUP_LDO_HFLNAB"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_HFLNAB, 0, wxALL, 0);

    chkSPDUP_LDO_HFLNAC = new wxCheckBox(this, ID_SPDUP_LDO_HFLNAC, wxT("SPDUP_LDO_HFLNAC"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_HFLNAC, 0, wxALL, 0);

    chkSPDUP_LDO_HFLNAD = new wxCheckBox(this, ID_SPDUP_LDO_HFLNAD, wxT("SPDUP_LDO_HFLNAD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_HFLNAD, 0, wxALL, 0);

    chkSPDUP_LDO_CLK_BUF =
        new wxCheckBox(this, ID_SPDUP_LDO_CLK_BUF, wxT("SPDUP_LDO_CLK_BUF"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_CLK_BUF, 0, wxALL, 0);

    chkSPDUP_LDO_PLL_DIV =
        new wxCheckBox(this, ID_SPDUP_LDO_PLL_DIV, wxT("SPDUP_LDO_PLL_DIV"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_PLL_DIV, 0, wxALL, 0);

    chkSPDUP_LDO_PLL_CP = new wxCheckBox(this, ID_SPDUP_LDO_PLL_CP, wxT("SPDUP_LDO_PLL_CP"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_PLL_CP, 0, wxALL, 0);

    chkSPDUP_LDO_DIG_CORE =
        new wxCheckBox(this, ID_SPDUP_LDO_DIG_CORE, wxT("SPDUP_LDO_DIG_CORE"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer46->Add(chkSPDUP_LDO_DIG_CORE, 0, wxALL, 0);

    fgSizer30->Add(sbSizer46, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer461;
    sbSizer461 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Bias")), wxVERTICAL);

    chkEN_LOADIMP_LDO_LOBUFA =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_LOBUFA, wxT("EN_LOADIMP_LDO_LOBUFA"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_LOBUFA, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_LOBUFB =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_LOBUFB, wxT("EN_LOADIMP_LDO_LOBUFB"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_LOBUFB, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_LOBUFC =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_LOBUFC, wxT("EN_LOADIMP_LDO_LOBUFC"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_LOBUFC, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_LOBUFD =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_LOBUFD, wxT("EN_LOADIMP_LDO_LOBUFD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_LOBUFD, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_HFLNAA =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_HFLNAA, wxT("EN_LOADIMP_LDO_HFLNAA"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_HFLNAA, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_HFLNAB =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_HFLNAB, wxT("EN_LOADIMP_LDO_HFLNAB"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_HFLNAB, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_HFLNAC =
        new wxCheckBox(this, ID_SPDUP_LDO_HFLNAC, wxT("EN_LOADIMP_LDO_HFLNAC"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_HFLNAC, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_HFLNAD =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_HFLNAD, wxT("EN_LOADIMP_LDO_HFLNAD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_HFLNAD, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_CLK_BUF =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_CLK_BUF, wxT("EN_LOADIMP_LDO_CLK_BUF"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_CLK_BUF, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_PLL_DIV =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_PLL_DIV, wxT("EN_LOADIMP_LDO_PLL_DIV"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_PLL_DIV, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_PLL_CP =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_PLL_CP, wxT("EN_LOADIMP_LDO_PLL_CP"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_PLL_CP, 0, wxALL, 0);

    chkEN_LOADIMP_LDO_DIG_CORE =
        new wxCheckBox(this, ID_EN_LOADIMP_LDO_DIG_CORE, wxT("EN_LOADIMP_LDO_DIG_CORE"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer461->Add(chkEN_LOADIMP_LDO_DIG_CORE, 0, wxALL, 0);

    fgSizer30->Add(sbSizer461, 1, wxEXPAND, 5);

    fgSizer69->Add(fgSizer30, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer31;
    fgSizer31 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer31->SetFlexibleDirection(wxBOTH);
    fgSizer31->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer50;
    sbSizer50 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Output Voltage")), wxVERTICAL);

    wxFlexGridSizer* fgSizer32;
    fgSizer32 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer32->SetFlexibleDirection(wxBOTH);
    fgSizer32->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText44 = new wxStaticText(this, wxID_ANY, wxT("RDIV_LOBUFA"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText44->Wrap(-1);
    fgSizer32->Add(m_staticText44, 0, wxALL, 4);

    cmbRDIV_LOBUFA = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_LOBUFA, 0, wxALL, 0);

    m_staticText441 = new wxStaticText(this, wxID_ANY, wxT("RDIV_LOBUFB"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText441->Wrap(-1);
    fgSizer32->Add(m_staticText441, 0, wxALL, 4);

    cmbRDIV_LOBUFB = new wxComboBox(this, ID_RDIV_LOBUFB, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_LOBUFB, 0, wxALL, 0);

    m_staticText4411 = new wxStaticText(this, wxID_ANY, wxT("RDIV_LOBUFC"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4411->Wrap(-1);
    fgSizer32->Add(m_staticText4411, 0, wxALL, 4);

    cmbRDIV_LOBUFC = new wxComboBox(this, ID_RDIV_LOBUFC, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_LOBUFC, 0, wxALL, 0);

    m_staticText44111 = new wxStaticText(this, wxID_ANY, wxT("RDIV_LOBUFD"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText44111->Wrap(-1);
    fgSizer32->Add(m_staticText44111, 0, wxALL, 4);

    cmbRDIV_LOBUFD = new wxComboBox(this, ID_RDIV_LOBUFD, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_LOBUFD, 0, wxALL, 0);

    m_staticText442 = new wxStaticText(this, wxID_ANY, wxT("RDIV_HFLNAA"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText442->Wrap(-1);
    fgSizer32->Add(m_staticText442, 0, wxALL, 5);

    cmbRDIV_HFLNAA = new wxComboBox(this, ID_RDIV_HFLNAA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_HFLNAA, 0, wxALL, 0);

    m_staticText4421 = new wxStaticText(this, wxID_ANY, wxT("RDIV_HFLNAB"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4421->Wrap(-1);
    fgSizer32->Add(m_staticText4421, 0, wxALL, 4);

    cmbRDIV_HFLNAB = new wxComboBox(this, ID_RDIV_HFLNAB, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_HFLNAB, 0, wxALL, 0);

    m_staticText44211 = new wxStaticText(this, wxID_ANY, wxT("RDIV_HFLNAC"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText44211->Wrap(-1);
    fgSizer32->Add(m_staticText44211, 0, wxALL, 4);

    cmbRDIV_HFLNAC = new wxComboBox(this, ID_RDIV_HFLNAC, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_HFLNAC, 0, wxALL, 0);

    m_staticText442111 = new wxStaticText(this, wxID_ANY, wxT("RDIV_HFLNAD"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText442111->Wrap(-1);
    fgSizer32->Add(m_staticText442111, 0, wxALL, 4);

    cmbRDIV_HFLNAD = new wxComboBox(this, ID_RDIV_HFLNAD, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_HFLNAD, 0, wxALL, 0);

    m_staticText4421111 = new wxStaticText(this, wxID_ANY, wxT("RDIV_CLK_BUF"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4421111->Wrap(-1);
    fgSizer32->Add(m_staticText4421111, 0, wxALL, 4);

    cmbRDIV_CLK_BUF = new wxComboBox(this, ID_RDIV_CLK_BUF, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_CLK_BUF, 0, wxALL, 0);

    m_staticText44211111 = new wxStaticText(this, wxID_ANY, wxT("RDIV_PLL_DIV"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText44211111->Wrap(-1);
    fgSizer32->Add(m_staticText44211111, 0, wxALL, 4);

    cmbRDIV_PLL_DIV = new wxComboBox(this, ID_RDIV_PLL_DIV, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_PLL_DIV, 0, wxALL, 0);

    m_staticText442111111 = new wxStaticText(this, wxID_ANY, wxT("RDIV_PLL_CP"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText442111111->Wrap(-1);
    fgSizer32->Add(m_staticText442111111, 0, wxALL, 4);

    cmbRDIV_PLL_CP = new wxComboBox(this, ID_RDIV_PLL_CP, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_PLL_CP, 0, wxALL, 0);

    m_staticText4421111111 = new wxStaticText(this, wxID_ANY, wxT("RDIV_DIG_CORE"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4421111111->Wrap(-1);
    fgSizer32->Add(m_staticText4421111111, 0, wxALL, 4);

    cmbRDIV_DIG_CORE = new wxComboBox(this, ID_RDIV_DIG_CORE, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    fgSizer32->Add(cmbRDIV_DIG_CORE, 0, wxALL, 0);

    sbSizer50->Add(fgSizer32, 1, wxEXPAND, 5);

    fgSizer31->Add(sbSizer50, 1, wxEXPAND, 5);

    fgSizer69->Add(fgSizer31, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer34;
    fgSizer34 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer34->SetFlexibleDirection(wxBOTH);
    fgSizer34->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer48;
    sbSizer48 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Calibration rppolywo")), wxVERTICAL);

    chkPD_CALIB_COMP = new wxCheckBox(this, ID_PD_CALIB_COMP, wxT("PD_CALIB_COMP"), wxDefaultPosition, wxDefaultSize, 0);
    chkPD_CALIB_COMP->SetValue(true);
    sbSizer48->Add(chkPD_CALIB_COMP, 0, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 0);

    chkRP_CALIB_COMP = new wxCheckBox(this, ID_PD_CALIB_COMP, wxT("RP_CALIB_COMP"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer48->Add(chkRP_CALIB_COMP, 0, wxALL, 0);

    m_staticText43 = new wxStaticText(this, wxID_ANY, wxT("Code"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43->Wrap(-1);
    sbSizer48->Add(m_staticText43, 0, wxALL, 0);

    cmbRP_CALIB_BIAS =
        new wxSpinCtrl(this, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    sbSizer48->Add(cmbRP_CALIB_BIAS, 0, wxALL, 0);

    fgSizer34->Add(sbSizer48, 1, wxEXPAND | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    fgSizer69->Add(fgSizer34, 1, wxEXPAND, 5);

    this->SetSizer(fgSizer69);
    this->Layout();
    fgSizer69->Fit(this);
    // ID_NOTEBOOK_LDO->AddPage( this, wxT("Bias"), false );

    this->SetSizer(fgSizer69);
    this->Layout();
    fgSizer69->Fit(this);

    // Connect Events
    chkPD_FRP_BIAS->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkPD_F_BIAS->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkPD_PTRP_BIAS->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkPD_PT_BIAS->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkPD_BIAS->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_LOBUFA->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_LOBUFB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_LOBUFC->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_LOBUFD->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_HFLNAA->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_HFLNAB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_HFLNAC->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_HFLNAD->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_CLK_BUF->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_PLL_DIV->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LDO_PLL_CP->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkPD_LDO_DIG_CORE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_LOBUFA->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_LOBUFB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_LOBUFC->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_LOBUFD->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_HFLNAA->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_HFLNAB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_HFLNAC->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_HFLNAD->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_CLK_BUF->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_PLL_DIV->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_PLL_CP->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_LDO_DIG_CORE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_LOBUFA->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_LOBUFB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_LOBUFC->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_LOBUFD->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_HFLNAA->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_HFLNAB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_HFLNAC->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_HFLNAD->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_CLK_BUF->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_PLL_DIV->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_PLL_CP->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkEN_LOADIMP_LDO_DIG_CORE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_LOBUFA->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_LOBUFB->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_LOBUFC->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_LOBUFD->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_HFLNAA->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_HFLNAB->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_HFLNAC->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_HFLNAD->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_CLK_BUF->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_PLL_DIV->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_PLL_CP->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRDIV_DIG_CORE->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkPD_CALIB_COMP->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    chkRP_CALIB_COMP->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRP_CALIB_BIAS->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRP_CALIB_BIAS->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);
    cmbRP_CALIB_BIAS->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlLDO_view::ParameterChangeHandler), NULL, this);

    wndId2Enum[chkPD_CALIB_COMP] = PD_CALIB_COMP;
    wndId2Enum[chkRP_CALIB_COMP] = RP_CALIB_COMP;
    //msavic: I had problems with wxSpinCtrl, it was not invoking handler for wxEVT_COMMAND_SPINCTRL_UPDATED, so I also added handlers for wxEVT_COMMAND_TEXT_UPDATED and wxEVT_COMMAND_TEXT_ENTER
    wndId2Enum[cmbRP_CALIB_BIAS] = RP_CALIB_BIAS;
    wndId2Enum[chkPD_FRP_BIAS] = PD_FRP_BIAS;
    wndId2Enum[chkPD_F_BIAS] = PD_F_BIAS;
    wndId2Enum[chkPD_PTRP_BIAS] = PD_PTRP_BIAS;
    wndId2Enum[chkPD_PT_BIAS] = PD_PT_BIAS;
    wndId2Enum[chkPD_BIAS] = PD_BIAS;

    wndId2Enum[chkEN_LDO_LOBUFA] = EN_LDO_LOBUFA;
    wndId2Enum[chkEN_LDO_LOBUFB] = EN_LDO_LOBUFB;
    wndId2Enum[chkEN_LDO_LOBUFC] = EN_LDO_LOBUFC;
    wndId2Enum[chkEN_LDO_LOBUFD] = EN_LDO_LOBUFD;

    wndId2Enum[chkEN_LDO_HFLNAA] = EN_LDO_HFLNAA;
    wndId2Enum[chkEN_LDO_HFLNAB] = EN_LDO_HFLNAB;
    wndId2Enum[chkEN_LDO_HFLNAC] = EN_LDO_HFLNAC;
    wndId2Enum[chkEN_LDO_HFLNAD] = EN_LDO_HFLNAD;

    wndId2Enum[chkEN_LDO_CLK_BUF] = EN_LDO_CLK_BUF;
    wndId2Enum[chkEN_LDO_PLL_DIV] = EN_LDO_PLL_DIV;
    wndId2Enum[chkEN_LDO_PLL_CP] = EN_LDO_PLL_CP;
    wndId2Enum[chkPD_LDO_DIG_CORE] = PD_LDO_DIG_CORE;

    wndId2Enum[chkSPDUP_LDO_LOBUFA] = SPDUP_LDO_LOBUFA;
    wndId2Enum[chkSPDUP_LDO_LOBUFB] = SPDUP_LDO_LOBUFB;
    wndId2Enum[chkSPDUP_LDO_LOBUFC] = SPDUP_LDO_LOBUFC;
    wndId2Enum[chkSPDUP_LDO_LOBUFD] = SPDUP_LDO_LOBUFD;

    wndId2Enum[chkSPDUP_LDO_HFLNAA] = SPDUP_LDO_HFLNAA;
    wndId2Enum[chkSPDUP_LDO_HFLNAB] = SPDUP_LDO_HFLNAB;
    wndId2Enum[chkSPDUP_LDO_HFLNAC] = SPDUP_LDO_HFLNAC;
    wndId2Enum[chkSPDUP_LDO_HFLNAD] = SPDUP_LDO_HFLNAD;

    wndId2Enum[chkSPDUP_LDO_CLK_BUF] = SPDUP_LDO_CLK_BUF;
    wndId2Enum[chkSPDUP_LDO_PLL_DIV] = SPDUP_LDO_PLL_DIV;
    wndId2Enum[chkSPDUP_LDO_PLL_CP] = SPDUP_LDO_PLL_CP;
    wndId2Enum[chkSPDUP_LDO_DIG_CORE] = SPDUP_LDO_DIG_CORE;

    wndId2Enum[chkEN_LOADIMP_LDO_LOBUFA] = EN_LOADIMP_LDO_LOBUFA;
    wndId2Enum[chkEN_LOADIMP_LDO_LOBUFB] = EN_LOADIMP_LDO_LOBUFB;
    wndId2Enum[chkEN_LOADIMP_LDO_LOBUFC] = EN_LOADIMP_LDO_LOBUFC;
    wndId2Enum[chkEN_LOADIMP_LDO_LOBUFD] = EN_LOADIMP_LDO_LOBUFD;

    wndId2Enum[chkEN_LOADIMP_LDO_HFLNAA] = EN_LOADIMP_LDO_HFLNAA;
    wndId2Enum[chkEN_LOADIMP_LDO_HFLNAB] = EN_LOADIMP_LDO_HFLNAB;
    wndId2Enum[chkEN_LOADIMP_LDO_HFLNAC] = EN_LOADIMP_LDO_HFLNAC;
    wndId2Enum[chkEN_LOADIMP_LDO_HFLNAD] = EN_LOADIMP_LDO_HFLNAD;

    wndId2Enum[chkEN_LOADIMP_LDO_CLK_BUF] = EN_LOADIMP_LDO_CLK_BUF;
    wndId2Enum[chkEN_LOADIMP_LDO_PLL_DIV] = EN_LOADIMP_LDO_PLL_DIV;
    wndId2Enum[chkEN_LOADIMP_LDO_PLL_CP] = EN_LOADIMP_LDO_PLL_CP;
    wndId2Enum[chkEN_LOADIMP_LDO_DIG_CORE] = EN_LOADIMP_LDO_DIG_CORE;

    wndId2Enum[cmbRDIV_LOBUFA] = RDIV_LOBUFA;
    wndId2Enum[cmbRDIV_LOBUFB] = RDIV_LOBUFB;
    wndId2Enum[cmbRDIV_LOBUFC] = RDIV_LOBUFC;
    wndId2Enum[cmbRDIV_LOBUFD] = RDIV_LOBUFD;

    wndId2Enum[cmbRDIV_HFLNAA] = RDIV_HFLNAA;
    wndId2Enum[cmbRDIV_HFLNAB] = RDIV_HFLNAB;
    wndId2Enum[cmbRDIV_HFLNAC] = RDIV_HFLNAC;
    wndId2Enum[cmbRDIV_HFLNAD] = RDIV_HFLNAD;

    wndId2Enum[cmbRDIV_CLK_BUF] = RDIV_CLK_BUF;
    wndId2Enum[cmbRDIV_PLL_DIV] = RDIV_PLL_DIV;
    wndId2Enum[cmbRDIV_PLL_CP] = RDIV_PLL_CP;
    wndId2Enum[cmbRDIV_DIG_CORE] = RDIV_DIG_CORE;

    wxArrayString temp;
    temp.clear();
    for (int i = 0; i < 256; ++i)
    {
        temp.push_back(wxString::Format(_("%.3f V"), (860.0 + 3.92 * i) / 1000.0));
    }
    cmbRDIV_LOBUFA->Append(temp);
    cmbRDIV_LOBUFB->Append(temp);
    cmbRDIV_LOBUFC->Append(temp);
    cmbRDIV_LOBUFD->Append(temp);

    cmbRDIV_HFLNAA->Append(temp);
    cmbRDIV_HFLNAB->Append(temp);
    cmbRDIV_HFLNAC->Append(temp);
    cmbRDIV_HFLNAD->Append(temp);

    cmbRDIV_CLK_BUF->Append(temp);
    cmbRDIV_PLL_DIV->Append(temp);
    cmbRDIV_PLL_CP->Append(temp);
    cmbRDIV_DIG_CORE->Append(temp);

    LMS8001_WXGUI::UpdateTooltips(wndId2Enum, true);
}
