#include "lms8001_pnlPLLProfiles_view.h"
#include "lms8001_gui_utilities.h"
#include "events.h"

#include <map>
#include <chrono>
#include <thread>

#include "chips/LMS8001/LMS8001.h"

using namespace lime;

#define max(a, b) ((a > b) ? a : b)
#define min(a, b) ((a < b) ? a : b)

lms8001_pnlPLLProfiles_view::lms8001_pnlPLLProfiles_view(
    wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ILMS8001Tab(parent, id, pos, size, style)
{
    wxFlexGridSizer* fgSizer68;
    fgSizer68 = new wxFlexGridSizer(1, 3, 0, 0);
    fgSizer68->SetFlexibleDirection(wxBOTH);
    fgSizer68->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer154;
    fgSizer154 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer154->SetFlexibleDirection(wxBOTH);
    fgSizer154->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxString rgrPLLProfileChoices[] = { wxT("0"), wxT("1"), wxT("2"), wxT("3"), wxT("4"), wxT("5"), wxT("6"), wxT("7") };
    int rgrPLLProfileNChoices = sizeof(rgrPLLProfileChoices) / sizeof(wxString);
    rgrPLLProfile = new wxRadioBox(this,
        wxID_ANY,
        wxT("Profile"),
        wxDefaultPosition,
        wxDefaultSize,
        rgrPLLProfileNChoices,
        rgrPLLProfileChoices,
        8,
        wxRA_SPECIFY_ROWS);
    rgrPLLProfile->SetSelection(0);
    fgSizer154->Add(rgrPLLProfile, 0, wxALL | wxEXPAND, 0);

    wxStaticBoxSizer* sbSizer505;
    sbSizer505 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer5651;
    fgSizer5651 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer5651->SetFlexibleDirection(wxBOTH);
    fgSizer5651->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText272 = new wxStaticText(this, wxID_ANY, wxT("Active\nProfile\nStatus:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText272->Wrap(-1);
    fgSizer5651->Add(m_staticText272, 0, wxBOTTOM, 5);

    chkVTUNE_HIGH = new wxCheckBox(this, ID_VTUNE_HIGH, wxT("High"), wxDefaultPosition, wxDefaultSize, 0);
    chkVTUNE_HIGH->Enable(false);

    fgSizer5651->Add(chkVTUNE_HIGH, 0, wxALL, 0);

    chkVTUNE_LOW = new wxCheckBox(this, ID_VTUNE_LOW, wxT("Low"), wxDefaultPosition, wxDefaultSize, 0);
    chkVTUNE_LOW->Enable(false);

    fgSizer5651->Add(chkVTUNE_LOW, 0, wxALL, 0);

    chkPLL_LOCK = new wxCheckBox(this, ID_PLL_LOCK, wxT("Lock"), wxDefaultPosition, wxDefaultSize, 0);
    chkPLL_LOCK->Enable(false);

    fgSizer5651->Add(chkPLL_LOCK, 0, wxALL, 0);

    sbSizer505->Add(fgSizer5651, 1, wxEXPAND, 5);

    fgSizer154->Add(sbSizer505, 1, wxEXPAND, 0);

    wxStaticBoxSizer* sbSizer115;
    sbSizer115 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("PLL Freq.")), wxVERTICAL);

    sttxtPLLFreq = new wxStaticText(this, wxID_ANY, wxT("0.0000"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtPLLFreq->Wrap(-1);
    sbSizer115->Add(sttxtPLLFreq, 0, wxALL, 0);

    sttxtPLLFreq1 = new wxStaticText(this, wxID_ANY, wxT("GHz"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtPLLFreq1->Wrap(-1);
    sbSizer115->Add(sttxtPLLFreq1, 0, wxALIGN_TOP, 0);

    fgSizer154->Add(sbSizer115, 1, wxEXPAND, 5);

    fgSizer68->Add(fgSizer154, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer697;
    fgSizer697 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer697->SetFlexibleDirection(wxBOTH);
    fgSizer697->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer543;
    sbSizer543 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Profile Configuration")), wxVERTICAL);

    wxFlexGridSizer* fgSizer698;
    fgSizer698 = new wxFlexGridSizer(0, 4, 0, 0);
    fgSizer698->SetFlexibleDirection(wxBOTH);
    fgSizer698->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer545;
    sbSizer545 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Enable")), wxVERTICAL);

    chkPLL_LODIST_EN_BIAS_n = new wxCheckBox(this, ID_TEST, wxT("LODIST_EN_BIAS"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_LODIST_EN_BIAS_n, 0, wxALL, 0);

    chkPLL_LODIST_EN_DIV2IQ_n = new wxCheckBox(this, ID_TEST, wxT("LODIST_EN_DIV2IQ"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_LODIST_EN_DIV2IQ_n, 0, wxALL, 0);

    chkPLL_EN_VTUNE_COMP_n = new wxCheckBox(this, ID_TEST, wxT("EN_VTUNE_COMP"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_VTUNE_COMP_n, 0, wxALL, 0);

    chkPLL_EN_LD_n = new wxCheckBox(this, ID_TEST, wxT("EN_LD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_LD_n, 0, wxALL, 0);

    chkPLL_EN_PFD_n = new wxCheckBox(this, ID_TEST, wxT("EN_PFD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_PFD_n, 0, wxALL, 0);

    chkPLL_EN_CP_n = new wxCheckBox(this, ID_TEST, wxT("EN_CP"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_CP_n, 0, wxALL, 0);

    chkPLL_EN_CPOFS_n = new wxCheckBox(this, ID_TEST, wxT("EN_CPOFS"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_CPOFS_n, 0, wxALL, 0);

    chkPLL_EN_VCO_n = new wxCheckBox(this, ID_TEST, wxT("EN_VCO"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_VCO_n, 0, wxALL, 0);

    chkPLL_EN_FFDIV_n = new wxCheckBox(this, ID_TEST, wxT("EN_FFDIV"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_FFDIV_n, 0, wxALL, 0);

    chkPLL_EN_FB_PDIV2_n = new wxCheckBox(this, ID_TEST, wxT("EN_FB_PDIV2"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_FB_PDIV2_n, 0, wxALL, 0);

    chkPLL_EN_FFCORE_n = new wxCheckBox(this, ID_TEST, wxT("EN_FFCORE"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_FFCORE_n, 0, wxALL, 0);

    chkPLL_EN_FBDIV_n = new wxCheckBox(this, ID_TEST, wxT("EN_FBDIV"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_EN_FBDIV_n, 0, wxALL, 0);

    chkPLL_SDM_CLK_EN_n = new wxCheckBox(this, ID_TEST, wxT("SDM_CLK_EN"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer545->Add(chkPLL_SDM_CLK_EN_n, 0, wxALL, 0);

    fgSizer698->Add(sbSizer545, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer547;
    sbSizer547 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Charge Pump")), wxVERTICAL);

    chkFLIP_n = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("FLIP"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer547->Add(chkFLIP_n, 0, wxALL, 0);

    m_staticText1328 = new wxStaticText(this, wxID_ANY, wxT("DEL"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1328->Wrap(-1);
    sbSizer547->Add(m_staticText1328, 0, wxALL, 0);

    cmbDEL_n = new wxSpinCtrl(this, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 3, 0);
    cmbDEL_n->SetMinSize(wxSize(70, -1));

    sbSizer547->Add(cmbDEL_n, 0, wxALL, 0);

    m_staticText13281 = new wxStaticText(this, wxID_ANY, wxT("PULSE [µA]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13281->Wrap(-1);
    sbSizer547->Add(m_staticText13281, 0, wxALL, 0);

    cmbPULSE_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbPULSE_n->SetSelection(0);
    sbSizer547->Add(cmbPULSE_n, 0, wxALL, 0);

    m_staticText132811 = new wxStaticText(this, wxID_ANY, wxT("OFS  [µA]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText132811->Wrap(-1);
    sbSizer547->Add(m_staticText132811, 0, wxALL, 0);

    cmbOFS_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbOFS_n->SetSelection(0);
    sbSizer547->Add(cmbOFS_n, 0, wxALL, 0);

    m_staticText1328111 = new wxStaticText(this, wxID_ANY, wxT("LD_VCT [mV]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1328111->Wrap(-1);
    sbSizer547->Add(m_staticText1328111, 0, wxALL, 0);

    cmbLD_VCT_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbLD_VCT_n->Append(wxT("600"));
    cmbLD_VCT_n->Append(wxT("700"));
    cmbLD_VCT_n->Append(wxT("800"));
    cmbLD_VCT_n->Append(wxT("900"));
    cmbLD_VCT_n->SetSelection(1);
    sbSizer547->Add(cmbLD_VCT_n, 0, wxALL, 0);

    m_staticText13281111 = new wxStaticText(this, wxID_ANY, wxT("ICT_CP"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13281111->Wrap(-1);
    sbSizer547->Add(m_staticText13281111, 0, wxALL, 0);

    cmbICT_CP_n =
        new wxSpinCtrl(this, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    cmbICT_CP_n->SetMinSize(wxSize(70, -1));

    sbSizer547->Add(cmbICT_CP_n, 0, wxALL, 0);

    fgSizer698->Add(sbSizer547, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer548;
    sbSizer548 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("VCO")), wxVERTICAL);

    wxFlexGridSizer* fgSizer701;
    fgSizer701 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer701->SetFlexibleDirection(wxBOTH);
    fgSizer701->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText1341 = new wxStaticText(this, wxID_ANY, wxT("VCO_FREQ"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1341->Wrap(-1);
    fgSizer701->Add(m_staticText1341, 0, wxALL, 0);

    cmbVCO_FREQ_n =
        new wxSpinCtrl(this, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 255, 128);
    fgSizer701->Add(cmbVCO_FREQ_n, 0, wxALL, 0);

    chkSPDUP_VCO_n = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("SPDUP_VCO"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer701->Add(chkSPDUP_VCO_n, 0, wxALL, 0);

    chkVCO_AAC_EN_n = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("VCO_AAC_EN"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer701->Add(chkVCO_AAC_EN_n, 0, wxALL, 0);

    m_staticText1342 = new wxStaticText(this, wxID_ANY, wxT("VDIV_SWVDD [mV]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1342->Wrap(-1);
    fgSizer701->Add(m_staticText1342, 0, wxALL, 0);

    cmbVDIV_SWVDD_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbVDIV_SWVDD_n->Append(wxT("600"));
    cmbVDIV_SWVDD_n->Append(wxT("800"));
    cmbVDIV_SWVDD_n->Append(wxT("1000"));
    cmbVDIV_SWVDD_n->Append(wxT("1200"));
    cmbVDIV_SWVDD_n->SetSelection(2);
    fgSizer701->Add(cmbVDIV_SWVDD_n, 0, wxALL, 0);

    m_staticText1343 = new wxStaticText(this, wxID_ANY, wxT("VCO_SEL"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1343->Wrap(-1);
    fgSizer701->Add(m_staticText1343, 0, wxALL, 0);

    cmbVCO_SEL_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbVCO_SEL_n->Append(wxT("External"));
    cmbVCO_SEL_n->Append(wxT("Core 1"));
    cmbVCO_SEL_n->Append(wxT("Core 2"));
    cmbVCO_SEL_n->Append(wxT("Core 3"));
    cmbVCO_SEL_n->SetSelection(3);
    fgSizer701->Add(cmbVCO_SEL_n, 0, wxALL, 0);

    m_staticText1344 = new wxStaticText(this, wxID_ANY, wxT("VCO_AMP"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1344->Wrap(-1);
    fgSizer701->Add(m_staticText1344, 0, wxALL, 0);

    cmbVCO_AMP_n =
        new wxSpinCtrl(this, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 127, 1);
    fgSizer701->Add(cmbVCO_AMP_n, 0, wxALL, 0);

    sbSizer548->Add(fgSizer701, 1, wxEXPAND, 5);

    fgSizer698->Add(sbSizer548, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer553;
    sbSizer553 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Fast Lock")), wxVERTICAL);

    wxFlexGridSizer* fgSizer703;
    fgSizer703 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer703->SetFlexibleDirection(wxBOTH);
    fgSizer703->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText13113 = new wxStaticText(this, wxID_ANY, wxT("R3 [kΩ]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13113->Wrap(-1);
    fgSizer703->Add(m_staticText13113, 0, wxALL, 4);

    cmbFLOCK_R3_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbFLOCK_R3_n->SetSelection(0);
    fgSizer703->Add(cmbFLOCK_R3_n, 0, wxALL, 0);

    m_staticText131111 = new wxStaticText(this, wxID_ANY, wxT("R2 [kΩ]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText131111->Wrap(-1);
    fgSizer703->Add(m_staticText131111, 0, wxALL, 4);

    cmbFLOCK_R2_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbFLOCK_R2_n->SetSelection(0);
    fgSizer703->Add(cmbFLOCK_R2_n, 0, wxALL, 0);

    m_staticText1311221 = new wxStaticText(this, wxID_ANY, wxT("C3 [pF]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1311221->Wrap(-1);
    fgSizer703->Add(m_staticText1311221, 0, wxALL, 4);

    cmbFLOCK_C3_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbFLOCK_C3_n->SetSelection(0);
    fgSizer703->Add(cmbFLOCK_C3_n, 0, wxALL, 0);

    m_staticText131123 = new wxStaticText(this, wxID_ANY, wxT("C2 [pF]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText131123->Wrap(-1);
    fgSizer703->Add(m_staticText131123, 0, wxALL, 4);

    cmbFLOCK_C2_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbFLOCK_C2_n->SetSelection(0);
    fgSizer703->Add(cmbFLOCK_C2_n, 0, wxALL, 0);

    m_staticText1311211 = new wxStaticText(this, wxID_ANY, wxT("C1 [pF]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1311211->Wrap(-1);
    fgSizer703->Add(m_staticText1311211, 0, wxALL, 4);

    cmbFLOCK_C1_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbFLOCK_C1_n->SetSelection(0);
    fgSizer703->Add(cmbFLOCK_C1_n, 0, wxALL, 0);

    m_staticText132812 = new wxStaticText(this, wxID_ANY, wxT("PULSE [µA]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText132812->Wrap(-1);
    fgSizer703->Add(m_staticText132812, 0, wxALL, 4);

    cmbFLOCK_PULSE_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbFLOCK_PULSE_n->SetSelection(0);
    fgSizer703->Add(cmbFLOCK_PULSE_n, 0, wxALL, 0);

    m_staticText1328112 = new wxStaticText(this, wxID_ANY, wxT("OFS  [µA]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1328112->Wrap(-1);
    fgSizer703->Add(m_staticText1328112, 0, wxALL, 4);

    cmbFLOCK_OFS_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbFLOCK_OFS_n->SetSelection(0);
    fgSizer703->Add(cmbFLOCK_OFS_n, 0, wxALL, 0);

    sbSizer553->Add(fgSizer703, 1, wxEXPAND, 5);

    chkFLOCK_VCO_SPDUP_n = new wxCheckBox(this, ID_FLOCK_VCO_SPDUP_n, wxT("VCO_SPDUP"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer553->Add(chkFLOCK_VCO_SPDUP_n, 0, wxLEFT, 4);

    wxFlexGridSizer* fgSizer186;
    fgSizer186 = new wxFlexGridSizer(0, 4, 0, 0);
    fgSizer186->SetFlexibleDirection(wxBOTH);
    fgSizer186->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText134211221 = new wxStaticText(this, wxID_ANY, wxT("FLOCK_N   "), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText134211221->Wrap(-1);
    fgSizer186->Add(m_staticText134211221, 0, wxALL, 4);

    cmbFLOCK_N_n =
        new wxSpinCtrl(this, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 1023, 400);
    cmbFLOCK_N_n->SetMinSize(wxSize(60, -1));

    fgSizer186->Add(cmbFLOCK_N_n, 0, wxALL, 0);

    sttxtFLockN = new wxStaticText(this, wxID_ANY, wxT("0.0000"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtFLockN->Wrap(-1);
    fgSizer186->Add(sttxtFLockN, 0, wxBOTTOM | wxLEFT | wxTOP, 5);

    m_staticText1328122 = new wxStaticText(this, wxID_ANY, wxT("[µs]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1328122->Wrap(-1);
    fgSizer186->Add(m_staticText1328122, 0, wxBOTTOM | wxLEFT | wxTOP, 5);

    sbSizer553->Add(fgSizer186, 0, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer151;
    fgSizer151 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer151->SetFlexibleDirection(wxBOTH);
    fgSizer151->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer116;
    sbSizer116 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("LO Distr. Enable Ch.")), wxVERTICAL);

    wxFlexGridSizer* fgSizer152;
    fgSizer152 = new wxFlexGridSizer(0, 4, 0, 0);
    fgSizer152->SetFlexibleDirection(wxBOTH);
    fgSizer152->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkFLOCK_LODIST_EN_OUT0_n = new wxCheckBox(this, ID_FLOCK_VCO_SPDUP_n, wxT("A "), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer152->Add(chkFLOCK_LODIST_EN_OUT0_n, 0, wxALL, 0);

    chkFLOCK_LODIST_EN_OUT1_n = new wxCheckBox(this, ID_FLOCK_VCO_SPDUP_n, wxT("B "), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer152->Add(chkFLOCK_LODIST_EN_OUT1_n, 0, wxALL, 0);

    chkFLOCK_LODIST_EN_OUT2_n = new wxCheckBox(this, ID_FLOCK_VCO_SPDUP_n, wxT("C "), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer152->Add(chkFLOCK_LODIST_EN_OUT2_n, 0, wxALL, 0);

    chkFLOCK_LODIST_EN_OUT3_n = new wxCheckBox(this, ID_FLOCK_VCO_SPDUP_n, wxT("D"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer152->Add(chkFLOCK_LODIST_EN_OUT3_n, 0, wxALL, 0);

    sbSizer116->Add(fgSizer152, 1, wxEXPAND, 0);

    fgSizer151->Add(sbSizer116, 0, wxEXPAND, 0);

    sbSizer553->Add(fgSizer151, 0, wxEXPAND, 0);

    fgSizer698->Add(sbSizer553, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer546;
    sbSizer546 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("LPF Configuration")), wxVERTICAL);

    wxFlexGridSizer* fgSizer699;
    fgSizer699 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer699->SetFlexibleDirection(wxBOTH);
    fgSizer699->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText1311 = new wxStaticText(this, wxID_ANY, wxT("R3 [kΩ]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1311->Wrap(-1);
    fgSizer699->Add(m_staticText1311, 0, wxALL, 4);

    cmbR3_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbR3_n->SetSelection(0);
    fgSizer699->Add(cmbR3_n, 0, wxALL, 0);

    m_staticText13111 = new wxStaticText(this, wxID_ANY, wxT("R2 [kΩ]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13111->Wrap(-1);
    fgSizer699->Add(m_staticText13111, 0, wxALL, 4);

    cmbR2_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbR2_n->SetSelection(0);
    fgSizer699->Add(cmbR2_n, 0, wxALL, 0);

    m_staticText131122 = new wxStaticText(this, wxID_ANY, wxT("C3 [pF]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText131122->Wrap(-1);
    fgSizer699->Add(m_staticText131122, 0, wxALL, 4);

    cmbC3_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbC3_n->SetSelection(0);
    fgSizer699->Add(cmbC3_n, 0, wxALL, 0);

    m_staticText13112 = new wxStaticText(this, wxID_ANY, wxT("C2 [pF]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13112->Wrap(-1);
    fgSizer699->Add(m_staticText13112, 0, wxALL, 4);

    cmbC2_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbC2_n->SetSelection(0);
    fgSizer699->Add(cmbC2_n, 0, wxALL, 0);

    m_staticText131121 = new wxStaticText(this, wxID_ANY, wxT("C1 [pF]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText131121->Wrap(-1);
    fgSizer699->Add(m_staticText131121, 0, wxALL, 4);

    cmbC1_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbC1_n->SetSelection(0);
    fgSizer699->Add(cmbC1_n, 0, wxALL, 0);

    sbSizer546->Add(fgSizer699, 0, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer700;
    fgSizer700 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer700->SetFlexibleDirection(wxBOTH);
    fgSizer700->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText1324 = new wxStaticText(this, wxID_ANY, wxT("VTUNE_VCT [mV]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1324->Wrap(-1);
    fgSizer700->Add(m_staticText1324, 0, wxALL, 5);

    cmbVTUNE_VCT_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbVTUNE_VCT_n->Append(wxT("300"));
    cmbVTUNE_VCT_n->Append(wxT("600"));
    cmbVTUNE_VCT_n->Append(wxT("750"));
    cmbVTUNE_VCT_n->Append(wxT("900"));
    cmbVTUNE_VCT_n->SetSelection(1);
    fgSizer700->Add(cmbVTUNE_VCT_n, 0, wxALL, 0);

    chkLPFSW_n = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("LPFSW"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer700->Add(chkLPFSW_n, 0, wxLEFT, 4);

    sbSizer546->Add(fgSizer700, 1, wxALL | wxEXPAND, 0);

    fgSizer698->Add(sbSizer546, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer549;
    sbSizer549 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Feed-forward divider")), wxVERTICAL);

    m_staticText134211 = new wxStaticText(this, wxID_ANY, wxT("FF_MOD"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText134211->Wrap(-1);
    sbSizer549->Add(m_staticText134211, 0, wxALL, 0);

    cmbFF_MOD_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbFF_MOD_n->Append(wxT("No division"));
    cmbFF_MOD_n->Append(wxT("Div by 2"));
    cmbFF_MOD_n->Append(wxT("Div by 4"));
    cmbFF_MOD_n->Append(wxT("Div by 8"));
    cmbFF_MOD_n->SetSelection(0);
    sbSizer549->Add(cmbFF_MOD_n, 0, wxALL, 0);

    m_staticText1342112 = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1342112->Wrap(-1);
    sbSizer549->Add(m_staticText1342112, 0, wxALL, 0);

    chkEnableFFDIVDebug = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("Enable debug"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer549->Add(chkEnableFFDIVDebug, 0, wxALL, 0);

    wxStaticBoxSizer* sbSizer117;
    sbSizer117 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Debug")), wxVERTICAL);

    chkFFDIV_SEL_n = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("FFDIV_SEL"), wxDefaultPosition, wxDefaultSize, 0);
    chkFFDIV_SEL_n->Enable(false);

    sbSizer117->Add(chkFFDIV_SEL_n, 0, wxALL, 0);

    m_staticText13421 = new wxStaticText(this, wxID_ANY, wxT("Modulus"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13421->Wrap(-1);
    sbSizer117->Add(m_staticText13421, 0, wxALL, 0);

    cmbFFCORE_MOD_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbFFCORE_MOD_n->Append(wxT("No division"));
    cmbFFCORE_MOD_n->Append(wxT("Div by 2"));
    cmbFFCORE_MOD_n->Append(wxT("Div by 4"));
    cmbFFCORE_MOD_n->Append(wxT("Div by 8"));
    cmbFFCORE_MOD_n->SetSelection(0);
    cmbFFCORE_MOD_n->Enable(false);

    sbSizer117->Add(cmbFFCORE_MOD_n, 0, wxALL, 0);

    sbSizer549->Add(sbSizer117, 1, wxEXPAND, 5);

    fgSizer698->Add(sbSizer549, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer551;
    sbSizer551 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Sigma-Delta Modulator")), wxVERTICAL);

    chkINTMOD_EN_n = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("INTMOD_EN"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer551->Add(chkINTMOD_EN_n, 0, wxALL, 0);

    chkDITHER_EN_n = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("DITHER_EN"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer551->Add(chkDITHER_EN_n, 0, wxALL, 0);

    chkSEL_SDMCLK_n = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("SEL_SDMCLK"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer551->Add(chkSEL_SDMCLK_n, 0, wxALL, 0);

    chkREV_SDMCLK_n = new wxCheckBox(this, ID_CHx_PA_ILIN2X, wxT("REV_SDMCLK"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer551->Add(chkREV_SDMCLK_n, 0, wxALL, 0);

    wxFlexGridSizer* fgSizer145;
    fgSizer145 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer145->SetFlexibleDirection(wxBOTH);
    fgSizer145->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText1342111 = new wxStaticText(this, wxID_ANY, wxT("INTMOD"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1342111->Wrap(-1);
    fgSizer145->Add(m_staticText1342111, 0, wxALL, 0);

    cmbINTMOD_n =
        new wxSpinCtrl(this, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 1023, 216);
    cmbINTMOD_n->SetMinSize(wxSize(80, -1));

    fgSizer145->Add(cmbINTMOD_n, 0, wxALL, 0);

    m_staticText13421111 = new wxStaticText(this, wxID_ANY, wxT("FRACMODL"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13421111->Wrap(-1);
    fgSizer145->Add(m_staticText13421111, 0, wxALL, 0);

    cmbFRACMODL_n =
        new wxSpinCtrl(this, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 65535, 22320);
    cmbFRACMODL_n->SetMinSize(wxSize(80, -1));

    fgSizer145->Add(cmbFRACMODL_n, 0, wxALL, 0);

    m_staticText134211111 = new wxStaticText(this, wxID_ANY, wxT("FRACMODH"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText134211111->Wrap(-1);
    fgSizer145->Add(m_staticText134211111, 0, wxALL, 0);

    cmbFRACMODH_n =
        new wxSpinCtrl(this, ID_VCO_SEL_FINAL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 5);
    cmbFRACMODH_n->SetMinSize(wxSize(80, -1));

    fgSizer145->Add(cmbFRACMODH_n, 0, wxALL, 0);

    sbSizer551->Add(fgSizer145, 1, wxEXPAND | wxTOP, 5);

    fgSizer698->Add(sbSizer551, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer5521;
    sbSizer5521 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("LO Distribution")), wxVERTICAL);

    wxFlexGridSizer* fgSizer1421;
    fgSizer1421 = new wxFlexGridSizer(0, 5, 0, 0);
    fgSizer1421->AddGrowableCol(0);
    fgSizer1421->SetFlexibleDirection(wxBOTH);
    fgSizer1421->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText13421123 = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13421123->Wrap(-1);
    fgSizer1421->Add(m_staticText13421123, 0, wxALL, 0);

    m_staticText134211231 = new wxStaticText(this, wxID_ANY, wxT("EN"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText134211231->Wrap(-1);
    fgSizer1421->Add(m_staticText134211231, 0, wxALIGN_BOTTOM | wxALL, 5);

    m_staticText1342112311 =
        new wxStaticText(this, wxID_ANY, wxT("Div by:\n0 - /2\n1 - /1"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE);
    m_staticText1342112311->Wrap(-1);
    fgSizer1421->Add(m_staticText1342112311, 0, wxALL, 5);

    m_staticText1342112312 = new wxStaticText(this, wxID_ANY, wxT("Phase"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1342112312->Wrap(-1);
    fgSizer1421->Add(m_staticText1342112312, 0, wxALIGN_BOTTOM | wxALL, 5);

    m_staticText262 = new wxStaticText(this, wxID_ANY, wxT("f [GHz]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText262->Wrap(-1);
    fgSizer1421->Add(m_staticText262, 0, wxALIGN_BOTTOM | wxALL, 5);

    m_staticText13421123111 = new wxStaticText(this, wxID_ANY, wxT("Ch. A"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13421123111->Wrap(-1);
    fgSizer1421->Add(m_staticText13421123111, 0, wxALL, 4);

    chkPLL_LODIST_EN_OUT0_n = new wxCheckBox(this, ID_TEST, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1421->Add(chkPLL_LODIST_EN_OUT0_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 4);

    chkPLL_LODIST_FSP_OUT02_n =
        new wxCheckBox(this, ID_chkPLL_LODIST_FSP_OUT02_n, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1421->Add(chkPLL_LODIST_FSP_OUT02_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 4);

    cmbPLL_LODIST_FSP_OUT010_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbPLL_LODIST_FSP_OUT010_n->Append(wxT("0"));
    cmbPLL_LODIST_FSP_OUT010_n->Append(wxT("90"));
    cmbPLL_LODIST_FSP_OUT010_n->Append(wxT("180"));
    cmbPLL_LODIST_FSP_OUT010_n->Append(wxT("270"));
    cmbPLL_LODIST_FSP_OUT010_n->SetSelection(0);
    fgSizer1421->Add(cmbPLL_LODIST_FSP_OUT010_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 0);

    sttxtLODistrChAFreq = new wxStaticText(this, wxID_ANY, wxT("0.0000"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtLODistrChAFreq->Wrap(-1);
    fgSizer1421->Add(sttxtLODistrChAFreq, 0, wxALL, 4);

    m_staticText134211231111 = new wxStaticText(this, wxID_ANY, wxT("Ch. B"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText134211231111->Wrap(-1);
    fgSizer1421->Add(m_staticText134211231111, 0, wxALL, 4);

    chkPLL_LODIST_EN_OUT1_n = new wxCheckBox(this, ID_TEST, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1421->Add(chkPLL_LODIST_EN_OUT1_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 4);

    chkPLL_LODIST_FSP_OUT12_n =
        new wxCheckBox(this, ID_chkPLL_LODIST_FSP_OUT02_n, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1421->Add(chkPLL_LODIST_FSP_OUT12_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 4);

    cmbPLL_LODIST_FSP_OUT110_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbPLL_LODIST_FSP_OUT110_n->Append(wxT("0"));
    cmbPLL_LODIST_FSP_OUT110_n->Append(wxT("90"));
    cmbPLL_LODIST_FSP_OUT110_n->Append(wxT("180"));
    cmbPLL_LODIST_FSP_OUT110_n->Append(wxT("270"));
    cmbPLL_LODIST_FSP_OUT110_n->SetSelection(0);
    fgSizer1421->Add(cmbPLL_LODIST_FSP_OUT110_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 0);

    sttxtLODistrChBFreq = new wxStaticText(this, wxID_ANY, wxT("0.0000"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtLODistrChBFreq->Wrap(-1);
    fgSizer1421->Add(sttxtLODistrChBFreq, 0, wxALL, 4);

    m_staticText1342112311111 = new wxStaticText(this, wxID_ANY, wxT("Ch. C"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1342112311111->Wrap(-1);
    fgSizer1421->Add(m_staticText1342112311111, 0, wxALL, 4);

    chkPLL_LODIST_EN_OUT2_n = new wxCheckBox(this, ID_TEST, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1421->Add(chkPLL_LODIST_EN_OUT2_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 4);

    chkPLL_LODIST_FSP_OUT22_n =
        new wxCheckBox(this, ID_chkPLL_LODIST_FSP_OUT02_n, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1421->Add(chkPLL_LODIST_FSP_OUT22_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 4);

    cmbPLL_LODIST_FSP_OUT210_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbPLL_LODIST_FSP_OUT210_n->Append(wxT("0"));
    cmbPLL_LODIST_FSP_OUT210_n->Append(wxT("90"));
    cmbPLL_LODIST_FSP_OUT210_n->Append(wxT("180"));
    cmbPLL_LODIST_FSP_OUT210_n->Append(wxT("270"));
    cmbPLL_LODIST_FSP_OUT210_n->SetSelection(0);
    fgSizer1421->Add(cmbPLL_LODIST_FSP_OUT210_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 0);

    sttxtLODistrChCFreq = new wxStaticText(this, wxID_ANY, wxT("0.0000"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtLODistrChCFreq->Wrap(-1);
    fgSizer1421->Add(sttxtLODistrChCFreq, 0, wxALL, 4);

    m_staticText13421123111111 = new wxStaticText(this, wxID_ANY, wxT("Ch. D"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText13421123111111->Wrap(-1);
    fgSizer1421->Add(m_staticText13421123111111, 0, wxALL, 4);

    chkPLL_LODIST_EN_OUT3_n = new wxCheckBox(this, ID_TEST, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1421->Add(chkPLL_LODIST_EN_OUT3_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 4);

    chkPLL_LODIST_FSP_OUT32_n =
        new wxCheckBox(this, ID_chkPLL_LODIST_FSP_OUT02_n, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1421->Add(chkPLL_LODIST_FSP_OUT32_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 4);

    cmbPLL_LODIST_FSP_OUT310_n = new wxComboBox(this, ID_RDIV_LOBUFA, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbPLL_LODIST_FSP_OUT310_n->Append(wxT("0"));
    cmbPLL_LODIST_FSP_OUT310_n->Append(wxT("90"));
    cmbPLL_LODIST_FSP_OUT310_n->Append(wxT("180"));
    cmbPLL_LODIST_FSP_OUT310_n->Append(wxT("270"));
    cmbPLL_LODIST_FSP_OUT310_n->SetSelection(0);
    fgSizer1421->Add(cmbPLL_LODIST_FSP_OUT310_n, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 0);

    sttxtLODistrChDFreq = new wxStaticText(this, wxID_ANY, wxT("0.0000"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtLODistrChDFreq->Wrap(-1);
    fgSizer1421->Add(sttxtLODistrChDFreq, 0, wxALL, 4);

    sbSizer5521->Add(fgSizer1421, 1, wxEXPAND, 5);

    fgSizer698->Add(sbSizer5521, 1, wxEXPAND, 5);

    sbSizer543->Add(fgSizer698, 1, wxEXPAND, 0);

    wxFlexGridSizer* fgSizer147;
    fgSizer147 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer147->SetFlexibleDirection(wxBOTH);
    fgSizer147->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    sbSizer543->Add(fgSizer147, 0, wxEXPAND, 5);

    fgSizer697->Add(sbSizer543, 1, wxEXPAND, 5);

    fgSizer68->Add(fgSizer697, 0, wxEXPAND, 0);

    wxFlexGridSizer* fgSizer185;
    fgSizer185 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer185->SetFlexibleDirection(wxBOTH);
    fgSizer185->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer1151;
    sbSizer1151 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Tuning")), wxVERTICAL);

    wxStaticBoxSizer* sbSizer1131;
    sbSizer1131 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Ref. Clock [MHz]")), wxVERTICAL);

    txtRefClock = new wxTextCtrl(this, wxID_ANY, wxT("40.00"), wxDefaultPosition, wxDefaultSize, 0);
    txtRefClock->SetToolTip(wxT("Reference clock frequency [MHz]"));
    txtRefClock->SetMinSize(wxSize(90, -1));

    sbSizer1131->Add(txtRefClock, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    sbSizer1151->Add(sbSizer1131, 0, wxEXPAND, 0);

    wxStaticBoxSizer* sbSizer113;
    sbSizer113 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Simple Tune")), wxVERTICAL);

    wxFlexGridSizer* fgSizer148;
    fgSizer148 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer148->SetFlexibleDirection(wxBOTH);
    fgSizer148->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText273 = new wxStaticText(this, wxID_ANY, wxT("VCO Freq. [GHz]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText273->Wrap(-1);
    fgSizer148->Add(m_staticText273, 0, wxLEFT | wxTOP, 5);

    txtVCOFrequency = new wxTextCtrl(this, wxID_ANY, wxT("1.2"), wxDefaultPosition, wxDefaultSize, 0);
    txtVCOFrequency->SetToolTip(wxT("VCO frequency [GHz]"));
    txtVCOFrequency->SetMinSize(wxSize(90, -1));

    fgSizer148->Add(txtVCOFrequency, 0, wxBOTTOM | wxLEFT, 5);

    btnTune = new wxButton(this, wxID_ANY, wxT("Simple Tune"), wxDefaultPosition, wxDefaultSize, 0);
    btnTune->SetMinSize(wxSize(90, -1));

    fgSizer148->Add(btnTune, 0, wxALL, 5);

    sbSizer113->Add(fgSizer148, 0, wxEXPAND, 5);

    sbSizer1151->Add(sbSizer113, 0, wxEXPAND, 0);

    wxStaticBoxSizer* sbSizer1132;
    sbSizer1132 = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, wxT("Smart Tune")), wxVERTICAL);

    wxFlexGridSizer* fgSizer1481;
    fgSizer1481 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer1481->SetFlexibleDirection(wxBOTH);
    fgSizer1481->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText2731 = new wxStaticText(this, wxID_ANY, wxT("LO Freq. [GHz]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2731->Wrap(-1);
    fgSizer1481->Add(m_staticText2731, 0, wxLEFT | wxTOP, 5);

    txtSmartTunePLLFrequency = new wxTextCtrl(this, wxID_ANY, wxT("4.5"), wxDefaultPosition, wxDefaultSize, 0);
    txtSmartTunePLLFrequency->SetToolTip(wxT("LO frequency [GHz]"));
    txtSmartTunePLLFrequency->SetMinSize(wxSize(90, -1));

    fgSizer1481->Add(txtSmartTunePLLFrequency, 0, wxBOTTOM | wxLEFT, 5);

    chkSmartTuneGenIQ = new wxCheckBox(this, ID_PLL_LOCK, wxT("Generate IQ"), wxDefaultPosition, wxDefaultSize, 0);
    chkSmartTuneGenIQ->SetToolTip(wxT("Generate quadrature in LO distribution"));

    fgSizer1481->Add(chkSmartTuneGenIQ, 0, wxALL, 5);

    m_staticText27311 = new wxStaticText(this, wxID_ANY, wxT("Loop BW [kHz]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText27311->Wrap(-1);
    fgSizer1481->Add(m_staticText27311, 0, wxLEFT | wxTOP, 5);

    txtSmartTuneLoopBW = new wxTextCtrl(this, wxID_ANY, wxT("300"), wxDefaultPosition, wxDefaultSize, 0);
    txtSmartTuneLoopBW->SetToolTip(wxT("PLL loop bandwidth"));
    txtSmartTuneLoopBW->SetMinSize(wxSize(90, -1));

    fgSizer1481->Add(txtSmartTuneLoopBW, 0, wxBOTTOM | wxLEFT, 5);

    m_staticText273111 = new wxStaticText(this, wxID_ANY, wxT("Phase Margin [°]"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText273111->Wrap(-1);
    fgSizer1481->Add(m_staticText273111, 0, wxLEFT | wxTOP, 5);

    txtSmartTunePhaseMargin = new wxTextCtrl(this, wxID_ANY, wxT("50"), wxDefaultPosition, wxDefaultSize, 0);
    txtSmartTunePhaseMargin->SetToolTip(wxT("PLL phase margin"));
    txtSmartTunePhaseMargin->SetMinSize(wxSize(90, -1));

    fgSizer1481->Add(txtSmartTunePhaseMargin, 0, wxBOTTOM | wxLEFT, 5);

    m_staticText2731111 = new wxStaticText(this, wxID_ANY, wxT("BWEF"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2731111->Wrap(-1);
    fgSizer1481->Add(m_staticText2731111, 0, wxLEFT | wxTOP, 5);

    txtSmartTuneBWEF = new wxTextCtrl(this, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    txtSmartTuneBWEF->SetToolTip(wxT("Bandwidth extension factor in fast lock mode"));
    txtSmartTuneBWEF->SetMinSize(wxSize(90, -1));

    fgSizer1481->Add(txtSmartTuneBWEF, 0, wxBOTTOM | wxLEFT, 5);

    btnSmartTune = new wxButton(this, wxID_ANY, wxT("Smart Tune"), wxDefaultPosition, wxDefaultSize, 0);
    btnSmartTune->SetMinSize(wxSize(90, -1));

    fgSizer1481->Add(btnSmartTune, 0, wxALL, 5);

    sbSizer1132->Add(fgSizer1481, 0, wxEXPAND, 5);

    sbSizer1151->Add(sbSizer1132, 0, wxEXPAND, 5);

    fgSizer185->Add(sbSizer1151, 1, wxEXPAND, 5);

    fgSizer68->Add(fgSizer185, 1, wxEXPAND, 5);

    this->SetSizer(fgSizer68);
    this->Layout();
    fgSizer68->Fit(this);

    // Connect Events
    rgrPLLProfile->Connect(
        wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::OnSwitchPLLProfile), NULL, this);
    chkVTUNE_HIGH->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkVTUNE_LOW->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_LOCK->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    sttxtPLLFreq->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnupdateUI_sttxtPLLFreq), NULL, this);
    sttxtPLLFreq1->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnupdateUI_sttxtPLLFreq), NULL, this);
    chkPLL_LODIST_EN_BIAS_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_LODIST_EN_DIV2IQ_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_VTUNE_COMP_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_LD_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_PFD_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_CP_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_CPOFS_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_VCO_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_FFDIV_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_FB_PDIV2_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_FFCORE_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_EN_FBDIV_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_SDM_CLK_EN_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkFLIP_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbDEL_n->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbDEL_n->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbDEL_n->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbPULSE_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbPULSE_n->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbFLOCK_PULSE_n), NULL, this);
    cmbOFS_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbOFS_n->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbFLOCK_PULSE_n), NULL, this);
    cmbLD_VCT_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbICT_CP_n->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbICT_CP_n->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbICT_CP_n->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbVCO_FREQ_n->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbVCO_FREQ_n->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbVCO_FREQ_n->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkSPDUP_VCO_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkVCO_AAC_EN_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbVDIV_SWVDD_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbVCO_SEL_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbVCO_AMP_n->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbVCO_AMP_n->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbVCO_AMP_n->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_R3_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_R2_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_C3_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_C2_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_C1_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_PULSE_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_PULSE_n->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbFLOCK_PULSE_n), NULL, this);
    cmbFLOCK_OFS_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_OFS_n->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbFLOCK_PULSE_n), NULL, this);
    chkFLOCK_VCO_SPDUP_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_N_n->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_N_n->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFLOCK_N_n->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    sttxtFLockN->Connect(wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnUpdateUI_sttxtFLockN), NULL, this);
    chkFLOCK_LODIST_EN_OUT0_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkFLOCK_LODIST_EN_OUT1_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkFLOCK_LODIST_EN_OUT2_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkFLOCK_LODIST_EN_OUT3_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbR3_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbR2_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbC3_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbC2_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbC1_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbVTUNE_VCT_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkLPFSW_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFF_MOD_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::On_FF_MOD_Change), NULL, this);
    chkEnableFFDIVDebug->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::On_FF_MOD_Change), NULL, this);
    chkFFDIV_SEL_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFFCORE_MOD_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkINTMOD_EN_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkDITHER_EN_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkSEL_SDMCLK_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkREV_SDMCLK_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbINTMOD_n->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbINTMOD_n->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbINTMOD_n->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFRACMODL_n->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFRACMODL_n->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFRACMODL_n->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFRACMODH_n->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFRACMODH_n->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbFRACMODH_n->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_LODIST_EN_OUT0_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_LODIST_FSP_OUT02_n->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED,
        wxCommandEventHandler(lms8001_pnlPLLProfiles_view::chkPLL_LODIST_FSP_OUTCH2_n_Change),
        NULL,
        this);
    cmbPLL_LODIST_FSP_OUT010_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_FSP_OUT010_n->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbPLL_LODIST_FSP_OUTCH10_n), NULL, this);
    chkPLL_LODIST_EN_OUT1_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_LODIST_FSP_OUT12_n->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED,
        wxCommandEventHandler(lms8001_pnlPLLProfiles_view::chkPLL_LODIST_FSP_OUTCH2_n_Change),
        NULL,
        this);
    cmbPLL_LODIST_FSP_OUT110_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_FSP_OUT110_n->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbPLL_LODIST_FSP_OUTCH10_n), NULL, this);
    chkPLL_LODIST_EN_OUT2_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_LODIST_FSP_OUT22_n->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED,
        wxCommandEventHandler(lms8001_pnlPLLProfiles_view::chkPLL_LODIST_FSP_OUTCH2_n_Change),
        NULL,
        this);
    cmbPLL_LODIST_FSP_OUT210_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_FSP_OUT210_n->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbPLL_LODIST_FSP_OUTCH10_n), NULL, this);
    chkPLL_LODIST_EN_OUT3_n->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    chkPLL_LODIST_FSP_OUT32_n->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED,
        wxCommandEventHandler(lms8001_pnlPLLProfiles_view::chkPLL_LODIST_FSP_OUTCH2_n_Change),
        NULL,
        this);
    cmbPLL_LODIST_FSP_OUT310_n->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::ParameterChangeHandler), NULL, this);
    cmbPLL_LODIST_FSP_OUT310_n->Connect(
        wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbPLL_LODIST_FSP_OUTCH10_n), NULL, this);
    txtRefClock->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::OnTextRefClock), NULL, this);
    btnTune->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::OnTuneClick), NULL, this);
    btnSmartTune->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_pnlPLLProfiles_view::OnSmartTuneClick), NULL, this);

    wndId2Enum[chkPLL_LODIST_EN_BIAS_n] = PLL_LODIST_EN_BIAS_n;
    wndId2Enum[chkPLL_LODIST_EN_DIV2IQ_n] = PLL_LODIST_EN_DIV2IQ_n;
    wndId2Enum[chkPLL_EN_VTUNE_COMP_n] = PLL_EN_VTUNE_COMP_n;
    wndId2Enum[chkPLL_EN_LD_n] = PLL_EN_LD_n;
    wndId2Enum[chkPLL_EN_PFD_n] = PLL_EN_PFD_n;
    wndId2Enum[chkPLL_EN_CP_n] = PLL_EN_CP_n;
    wndId2Enum[chkPLL_EN_CPOFS_n] = PLL_EN_CPOFS_n;
    wndId2Enum[chkPLL_EN_VCO_n] = PLL_EN_VCO_n;
    wndId2Enum[chkPLL_EN_FFDIV_n] = PLL_EN_FFDIV_n;
    wndId2Enum[chkPLL_EN_FB_PDIV2_n] = PLL_EN_FB_PDIV2_n;
    wndId2Enum[chkPLL_EN_FFCORE_n] = PLL_EN_FFCORE_n;
    wndId2Enum[chkPLL_EN_FBDIV_n] = PLL_EN_FBDIV_n;
    wndId2Enum[chkPLL_SDM_CLK_EN_n] = PLL_SDM_CLK_EN_n;

    wndId2Enum[cmbR3_n] = R3_n;
    wndId2Enum[cmbR2_n] = R2_n;
    wndId2Enum[cmbC2_n] = C2_n;
    wndId2Enum[cmbC1_n] = C1_n;
    wndId2Enum[cmbVTUNE_VCT_n] = VTUNE_VCT_n;
    wndId2Enum[chkLPFSW_n] = LPFSW_n;
    wndId2Enum[cmbC3_n] = C3_n;

    wndId2Enum[chkFLIP_n] = FLIP_n;
    wndId2Enum[cmbDEL_n] = DEL_n;
    wndId2Enum[cmbPULSE_n] = PULSE_n;
    wndId2Enum[cmbOFS_n] = OFS_n;
    wndId2Enum[cmbLD_VCT_n] = LD_VCT_n;
    wndId2Enum[cmbICT_CP_n] = ICT_CP_n;

    wndId2Enum[cmbVCO_FREQ_n] = VCO_FREQ_n;
    wndId2Enum[chkSPDUP_VCO_n] = SPDUP_VCO_n;
    wndId2Enum[chkVCO_AAC_EN_n] = VCO_AAC_EN_n;
    wndId2Enum[cmbVDIV_SWVDD_n] = VDIV_SWVDD_n;
    wndId2Enum[cmbVCO_SEL_n] = VCO_SEL_n;
    wndId2Enum[cmbVCO_AMP_n] = VCO_AMP_n;

    wndId2Enum[chkFFDIV_SEL_n] = FFDIV_SEL_n;
    wndId2Enum[cmbFFCORE_MOD_n] = FFCORE_MOD_n;
    wndId2Enum[cmbFF_MOD_n] = FF_MOD_n;
    wndId2Enum[chkINTMOD_EN_n] = INTMOD_EN_n;
    wndId2Enum[chkDITHER_EN_n] = DITHER_EN_n;
    wndId2Enum[chkSEL_SDMCLK_n] = SEL_SDMCLK_n;
    wndId2Enum[chkREV_SDMCLK_n] = REV_SDMCLK_n;
    wndId2Enum[cmbINTMOD_n] = INTMOD_n;
    wndId2Enum[cmbFRACMODL_n] = FRACMODL_n;
    wndId2Enum[cmbFRACMODH_n] = FRACMODH_n;

    //	wndId2Enum[cmbPLL_LODIST_EN_OUT_n] = PLL_LODIST_EN_OUT_n;
    //	wndId2Enum[cmbPLL_LODIST_FSP_OUT3_n] = PLL_LODIST_FSP_OUT3_n;
    //	wndId2Enum[cmbPLL_LODIST_FSP_OUT2_n] = PLL_LODIST_FSP_OUT2_n;
    //	wndId2Enum[cmbPLL_LODIST_FSP_OUT1_n] = PLL_LODIST_FSP_OUT1_n;
    //	wndId2Enum[cmbPLL_LODIST_FSP_OUT0_n] = PLL_LODIST_FSP_OUT0_n;

    wndId2Enum[cmbFLOCK_R3_n] = FLOCK_R3_n;
    wndId2Enum[cmbFLOCK_R2_n] = FLOCK_R2_n;
    wndId2Enum[cmbFLOCK_C3_n] = FLOCK_C3_n;
    wndId2Enum[cmbFLOCK_C2_n] = FLOCK_C2_n;
    wndId2Enum[cmbFLOCK_C1_n] = FLOCK_C1_n;
    wndId2Enum[cmbFLOCK_PULSE_n] = FLOCK_PULSE_n;
    wndId2Enum[cmbFLOCK_OFS_n] = FLOCK_OFS_n;
    //	wndId2Enum[cmbFLOCK_LODIST_EN_OUT_n] = FLOCK_LODIST_EN_OUT_n;
    wndId2Enum[chkFLOCK_VCO_SPDUP_n] = FLOCK_VCO_SPDUP_n;
    wndId2Enum[cmbFLOCK_N_n] = FLOCK_N_n;

    wndId2Enum[chkFLOCK_LODIST_EN_OUT0_n] = FLOCK_LODIST_EN_OUT0_n;
    wndId2Enum[chkFLOCK_LODIST_EN_OUT1_n] = FLOCK_LODIST_EN_OUT1_n;
    wndId2Enum[chkFLOCK_LODIST_EN_OUT2_n] = FLOCK_LODIST_EN_OUT2_n;
    wndId2Enum[chkFLOCK_LODIST_EN_OUT3_n] = FLOCK_LODIST_EN_OUT3_n;

    wndId2Enum[chkPLL_LODIST_EN_OUT0_n] = PLL_LODIST_EN_OUT0_n;
    wndId2Enum[chkPLL_LODIST_EN_OUT1_n] = PLL_LODIST_EN_OUT1_n;
    wndId2Enum[chkPLL_LODIST_EN_OUT2_n] = PLL_LODIST_EN_OUT2_n;
    wndId2Enum[chkPLL_LODIST_EN_OUT3_n] = PLL_LODIST_EN_OUT3_n;

    wndId2Enum[chkPLL_LODIST_FSP_OUT02_n] = PLL_LODIST_FSP_OUT02_n;
    wndId2Enum[chkPLL_LODIST_FSP_OUT12_n] = PLL_LODIST_FSP_OUT12_n;
    wndId2Enum[chkPLL_LODIST_FSP_OUT22_n] = PLL_LODIST_FSP_OUT22_n;
    wndId2Enum[chkPLL_LODIST_FSP_OUT32_n] = PLL_LODIST_FSP_OUT32_n;

    wndId2Enum[cmbPLL_LODIST_FSP_OUT010_n] = PLL_LODIST_FSP_OUT010_n;
    wndId2Enum[cmbPLL_LODIST_FSP_OUT110_n] = PLL_LODIST_FSP_OUT110_n;
    wndId2Enum[cmbPLL_LODIST_FSP_OUT210_n] = PLL_LODIST_FSP_OUT210_n;
    wndId2Enum[cmbPLL_LODIST_FSP_OUT310_n] = PLL_LODIST_FSP_OUT310_n;

    wndId2Enum[chkVTUNE_HIGH] = VTUNE_HIGH;
    wndId2Enum[chkVTUNE_LOW] = VTUNE_LOW;
    wndId2Enum[chkPLL_LOCK] = PLL_LOCK;

    /*
	wxArrayString temp;
	temp.clear();
	for (int i = 0; i<256; ++i)
	{
		temp.push_back(wxString::Format(_("%.3f V"), 1.8*(257 - i)/(265 - i)));
	}
	cmbVDIV_VCOREG->Append(temp);
*/

    wxArrayString temp;

    temp.clear();
    temp.push_back("Open");
    for (int i = 1; i < 16; ++i)
    {
        temp.push_back(wxString::Format(_("%.3f"), 14.9 / i));
    }
    cmbR3_n->Append(temp);
    cmbFLOCK_R3_n->Append(temp);

    temp.clear();
    temp.push_back("Open");
    for (int i = 1; i < 16; ++i)
    {
        temp.push_back(wxString::Format(_("%.3f"), 24.6 / i));
    }
    cmbR2_n->Append(temp);
    cmbFLOCK_R2_n->Append(temp);

    temp.clear();
    for (int i = 0; i < 16; ++i)
    {
        temp.push_back(wxString::Format(_("%.3f"), 1.2 * i));
    }
    cmbC1_n->Append(temp);
    cmbFLOCK_C1_n->Append(temp);

    temp.clear();
    for (int i = 0; i < 16; ++i)
    {
        temp.push_back(wxString::Format(_("%.2f"), 10.0 * i + 150.0));
    }
    cmbC2_n->Append(temp);
    cmbFLOCK_C2_n->Append(temp);

    temp.clear();
    for (int i = 0; i < 16; ++i)
    {
        temp.push_back(wxString::Format(_("%.3f"), 1.2 * i + 4.8));
    }
    cmbC3_n->Append(temp);
    cmbFLOCK_C3_n->Append(temp);

    // This is just to define the width of the combo
    temp.clear();
    for (int i = 0; i < 64; ++i)
    {
        temp.push_back("000.00");
    }

    cmbFLOCK_PULSE_n->Append(temp);
    cmbPULSE_n->Append(temp);
    cmbFLOCK_OFS_n->Append(temp);
    cmbOFS_n->Append(temp);

    /*
	temp.clear();
	temp.push_back("Open");
	for (int i = 1; i<16; ++i)
	{
		temp.push_back(wxString::Format(_("%.3f kOhm"), 14.9 / i));
	}
	cmbR3_n->Append(temp);
*/
    LMS8001_WXGUI::UpdateTooltips(wndId2Enum, true);
}

void lms8001_pnlPLLProfiles_view::UpdateGUI()
{
    ILMS8001Tab::UpdateGUI();

    txtRefClock->SetValue(wxString::Format(_("%.4f"), chip->mRefClk_MHz));

    //	wxString SRefClock = txtRefClock->GetValue();
    //	double currentFreq;
    //	double RefClock_MHz, RefClock;
    //	SRefClock.ToDouble(&RefClock_MHz);
    //	RefClock = RefClock_MHz*1E6;
    double RefClock;
    RefClock = chip->mRefClk_MHz * 1E6;

    //	int chkPLL_EN_FB_PDIV2_n_val, cmbINTMOD_n_val, cmbFRACMODL_n_val, cmbFRACMODH_n_val;
    //	chkPLL_EN_FB_PDIV2_n_val = chkPLL_EN_FB_PDIV2_n->GetValue();
    //	cmbINTMOD_n_val = cmbINTMOD_n->GetValue();
    //	cmbFRACMODL_n_val = cmbFRACMODL_n->GetValue();
    //	cmbFRACMODH_n_val = cmbFRACMODH_n->GetValue();

    double VCOFrequency =
        pow(2, chkPLL_EN_FB_PDIV2_n->GetValue()) *
        (cmbINTMOD_n->GetValue() + (cmbFRACMODL_n->GetValue() + pow(2, 16) * cmbFRACMODH_n->GetValue()) / pow(2, 20)) * (RefClock);
    double VCOFrequency_GHz = VCOFrequency / 1E9;
    txtVCOFrequency->SetValue(wxString::Format(_("%.4f"), VCOFrequency_GHz));
}

void lms8001_pnlPLLProfiles_view::OnSwitchPLLProfile(wxCommandEvent& event)
{
    //	LMS8001_WXGUI::UpdateControlsByMap(this, chip, wndId2Enum);
    chip->PLLprofile = rgrPLLProfile->GetSelection();
    //	UpdateVisiblePanel();
    UpdateGUI();
}

void lms8001_pnlPLLProfiles_view::chkPLL_LODIST_FSP_OUTCH2_n_Change(wxCommandEvent& event)
{

    wxCheckBox* object = reinterpret_cast<wxCheckBox*>(event.GetEventObject());
    /*
	wxArrayString temp;

	if (event.GetInt() == 1) {
		temp.push_back("0");
		temp.push_back("0");
		temp.push_back("180");
		temp.push_back("180");
	}
	if (event.GetInt() == 0) {
		temp.push_back("0");
		temp.push_back("90");
		temp.push_back("180");
		temp.push_back("270");
	}

	wxComboBox* currCombo = cmbPLL_LODIST_FSP_OUT010_n;

	if (object == chkPLL_LODIST_FSP_OUT02_n)
		currCombo = cmbPLL_LODIST_FSP_OUT010_n;
	if (object == chkPLL_LODIST_FSP_OUT12_n)
		currCombo = cmbPLL_LODIST_FSP_OUT110_n;
	if (object == chkPLL_LODIST_FSP_OUT22_n)
		currCombo = cmbPLL_LODIST_FSP_OUT210_n;
	if (object == chkPLL_LODIST_FSP_OUT32_n)
		currCombo = cmbPLL_LODIST_FSP_OUT310_n;

	currCombo->Set(temp);
	currCombo->Select(0);

	LMS8Parameter parCurrCombo;
	parCurrCombo = wndId2Enum.at(reinterpret_cast<wxWindow*>(currCombo));
	chip->Modify_SPI_Reg_bits(parCurrCombo, currCombo->GetSelection(), true, chip->channel, chip->PLLprofile);
*/
    LMS8Parameter parObject;
    parObject = wndId2Enum.at(reinterpret_cast<wxWindow*>(object));
    chip->Modify_SPI_Reg_bits(parObject, object->GetValue(), true, chip->channel, chip->PLLprofile);

    bool en_DIVIQ2 = true;
    // Enable quadrature generation of divide by 2 is active
    if (chkPLL_LODIST_FSP_OUT02_n->GetValue() && chkPLL_LODIST_FSP_OUT12_n->GetValue() && chkPLL_LODIST_FSP_OUT22_n->GetValue() &&
        chkPLL_LODIST_FSP_OUT32_n->GetValue())
    {
        en_DIVIQ2 = false;
    }
    chip->Modify_SPI_Reg_bits(PLL_LODIST_EN_DIV2IQ_n, en_DIVIQ2, true, chip->channel, chip->PLLprofile);
    chkPLL_LODIST_EN_DIV2IQ_n->SetValue(en_DIVIQ2);

    //	UpdateGUI();
}

void lms8001_pnlPLLProfiles_view::OnTuneClick(wxCommandEvent& event)
{
    OpStatus status;

    // Hold the PLL in reset
    status = chip->Modify_SPI_Reg_bits(PLL_RSTN, 0, true, chip->channel, chip->PLLprofile);
    if (status != OpStatus::Success)
        return;

    // Turn off divide by 2 in LO distribution
    // We decided later than it is better not to change these settings
    //	status = chip->Modify_SPI_Reg_bits(PLL_LODIST_FSP_OUT02_n, 1, true, chip->channel, chip->PLLprofile);
    //	status = chip->Modify_SPI_Reg_bits(PLL_LODIST_FSP_OUT12_n, 1, true, chip->channel, chip->PLLprofile);
    //	status = chip->Modify_SPI_Reg_bits(PLL_LODIST_FSP_OUT22_n, 1, true, chip->channel, chip->PLLprofile);
    //	status = chip->Modify_SPI_Reg_bits(PLL_LODIST_FSP_OUT32_n, 1, true, chip->channel, chip->PLLprofile);
    /*
	wxArrayString temp;

	temp.push_back("0");
	temp.push_back("0");
	temp.push_back("180");
	temp.push_back("180");

	int sel;

	sel = cmbPLL_LODIST_FSP_OUT010_n->GetSelection();
	cmbPLL_LODIST_FSP_OUT010_n->Set(temp);
	cmbPLL_LODIST_FSP_OUT010_n->Select(sel);
	sel = cmbPLL_LODIST_FSP_OUT110_n->GetSelection();
	cmbPLL_LODIST_FSP_OUT110_n->Set(temp);
	cmbPLL_LODIST_FSP_OUT110_n->Select(sel);
	sel = cmbPLL_LODIST_FSP_OUT210_n->GetSelection();
	cmbPLL_LODIST_FSP_OUT210_n->Set(temp);
	cmbPLL_LODIST_FSP_OUT210_n->Select(sel);
	sel = cmbPLL_LODIST_FSP_OUT310_n->GetSelection();
	cmbPLL_LODIST_FSP_OUT310_n->Set(temp);
	cmbPLL_LODIST_FSP_OUT310_n->Select(sel);
*/
    //	int sel_PLL_profile_internal0 = chip->Get_SPI_Reg_bits(PLL_CFG_INT_SEL0, true, chip->channel, chip->PLLprofile);
    //	int sel_PLL_profile_internal1 = chip->Get_SPI_Reg_bits(PLL_CFG_INT_SEL1, true, chip->channel, chip->PLLprofile);
    //	int sel_PLL_profile_internal2 = chip->Get_SPI_Reg_bits(PLL_CFG_INT_SEL2, true, chip->channel, chip->PLLprofile);

    // Save the controls of the PLL profile multiplexer & restore it after the calibration

    int pll_cfg_sel0 = chip->Get_SPI_Reg_bits(PLL_CFG_SEL0, true, chip->channel);
    int pll_cfg_sel1 = chip->Get_SPI_Reg_bits(PLL_CFG_SEL1, true, chip->channel);
    int pll_cfg_sel2 = chip->Get_SPI_Reg_bits(PLL_CFG_SEL2, true, chip->channel);

    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, PLL_CFG_SEL0.defaultValue, true, chip->channel);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, PLL_CFG_SEL1.defaultValue, true, chip->channel);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, PLL_CFG_SEL2.defaultValue, true, chip->channel);

    // Save the internal PLL profile control before the calibration, to be restored after calibration

    int sel_PLL_profile_internal = chip->Get_SPI_Reg_bits(PLL_CFG_INT_SEL, true, chip->channel, chip->PLLprofile);
    status = chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, chip->PLLprofile, true, chip->channel, chip->PLLprofile);

    double VCOFrequency_GHz, VCOFrequency;
    txtVCOFrequency->GetValue().ToDouble(&VCOFrequency_GHz);
    VCOFrequency = VCOFrequency_GHz * 1E9;
    VCOFrequency = round(VCOFrequency);
    double refClock_MHz, refClock;
    txtRefClock->GetValue().ToDouble(&refClock_MHz);
    refClock = refClock_MHz * 1E6;
    int FBDIV2 = chkPLL_EN_FB_PDIV2_n->GetValue();

    int INTMOD;
    int FRACMOD, FRACMODH, FRACMODL;

    //	double fFRACMOD = round(pow(2, 20) * (VCOFrequency / (refClock*pow(2, FBDIV2))));
    //	int FRACMOD = fFRACMOD;
    /*
	if (chkINTMOD_EN_n->GetValue()) // Integer mode
	{
		INTMOD = round(VCOFrequency / (refClock*pow(2, FBDIV2)));
		FRACMODL = 0;
		FRACMODH = 0;
		double intModeVCOFrequency_GHz;
		intModeVCOFrequency_GHz = INTMOD*refClock*pow(2, FBDIV2) / 1E9;
		txtVCOFrequency->SetValue(wxString::Format(_("%.4f"), VCOFrequency_GHz));
	}
	else {
		INTMOD = floor(VCOFrequency / (refClock*pow(2, FBDIV2)));
		FRACMOD = round(pow(2, 20) * (VCOFrequency / (refClock*pow(2, FBDIV2))));
		FRACMODL = FRACMOD % (int)pow(2, 16);
		FRACMODH = FRACMOD / (int)pow(2, 16);
	}
*/
    int NFIX;
    calc_fbdiv(VCOFrequency, refClock, chkINTMOD_EN_n->GetValue(), FBDIV2, &INTMOD, &FRACMOD, &NFIX);
    FRACMODH = (floor(FRACMOD / pow(2, 16)));
    FRACMODL = (FRACMOD - FRACMODH * pow(2, 16));

    //	LMS8Parameter parameter;
    //	parameter = chip->findLMS8Param("INTMOD_n");
    //	status = chip->Modify_SPI_Reg_bits(parameter, INTMOD, true, chip->channel, chip->PLLprofile);

    status = chip->Modify_SPI_Reg_bits(INTMOD_n, INTMOD, true, chip->channel, chip->PLLprofile);
    status = chip->Modify_SPI_Reg_bits(FRACMODL_n, FRACMODL, true, chip->channel, chip->PLLprofile);
    status = chip->Modify_SPI_Reg_bits(FRACMODH_n, FRACMODH, true, chip->channel, chip->PLLprofile);

    //	if (strcmp(parameter.name, "NOT_FOUND") != 0) {};

    // EN_VCOBIAS = 1
    status = chip->Modify_SPI_Reg_bits(EN_VCOBIAS, 1, true, chip->channel, chip->PLLprofile);
    // PLL_XBUF_EN = 1
    status = chip->Modify_SPI_Reg_bits(PLL_XBUF_EN, 1, true, chip->channel, chip->PLLprofile);
    // PLL_EN_VTUNE_COMP = 1
    status = chip->Modify_SPI_Reg_bits(PLL_EN_VTUNE_COMP_n, 1, true, chip->channel, chip->PLLprofile);
    // EN_LD = 1
    status = chip->Modify_SPI_Reg_bits(PLL_EN_LD_n, 1, true, chip->channel, chip->PLLprofile);
    // EN_PFD = 1
    status = chip->Modify_SPI_Reg_bits(PLL_EN_PFD_n, 1, true, chip->channel, chip->PLLprofile);
    // EN_CP = 1
    status = chip->Modify_SPI_Reg_bits(PLL_EN_CP_n, 1, true, chip->channel, chip->PLLprofile);
    // EN_VCO = 1
    status = chip->Modify_SPI_Reg_bits(PLL_EN_VCO_n, 1, true, chip->channel, chip->PLLprofile);
    // EN_FFDIV = 1
    status = chip->Modify_SPI_Reg_bits(PLL_EN_FFDIV_n, 1, true, chip->channel, chip->PLLprofile);
    // EN_FB_PDIV2 set by the user
    // EN_FFCORE = 1 if the FF_DIV modulus is > 1
    if (cmbFF_MOD_n->GetSelection() > 0)
        status = chip->Modify_SPI_Reg_bits(PLL_EN_FFCORE_n, 1, true, chip->channel, chip->PLLprofile);
    else
        status = chip->Modify_SPI_Reg_bits(PLL_EN_FFCORE_n, 0, true, chip->channel, chip->PLLprofile);
    // EN_FBDIV = 1
    status = chip->Modify_SPI_Reg_bits(PLL_EN_FBDIV_n, 1, true, chip->channel, chip->PLLprofile);
    // SDM_CLK_EN = 1 if INTMOD_EN = 0
    if (!chkINTMOD_EN_n->GetValue())
        status = chip->Modify_SPI_Reg_bits(PLL_SDM_CLK_EN_n, 1, true, chip->channel, chip->PLLprofile);
    else
        status = chip->Modify_SPI_Reg_bits(PLL_SDM_CLK_EN_n, 0, true, chip->channel, chip->PLLprofile);

    // PLL_CALIBRATION_EN = 1
    status = chip->Modify_SPI_Reg_bits(PLL_CALIBRATION_EN, 1, true, chip->channel, chip->PLLprofile);
    // Un-reset the PLL
    status = chip->Modify_SPI_Reg_bits(PLL_RSTN, 1, true, chip->channel, chip->PLLprofile);
    // FCAL_START = 1
    status = chip->Modify_SPI_Reg_bits(FCAL_START, 1, true, chip->channel, chip->PLLprofile);

    //  Wait - 5ms?
    //  Check is FCAL_START = 0
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    //  If it is not, check again?

    //  If FCAL_START == 0 and
    //  VCO_SEL_FINAL_VAL == 1 and FREQ_FINAL_VAL == 1 then
    //  value from VCO_SEL_FINAL should be copied to VCO_SEL and
    //  value from FREQ_FINAL should be copied to VCO_FREQ

    int FCAL_start = chip->Get_SPI_Reg_bits(FCAL_START, true, chip->channel, chip->PLLprofile);
    int VCO_sel_final_val = chip->Get_SPI_Reg_bits(VCO_SEL_FINAL_VAL, true, chip->channel, chip->PLLprofile);
    int freq_sel_final_val = chip->Get_SPI_Reg_bits(FREQ_FINAL_VAL, true, chip->channel, chip->PLLprofile);
    if ((FCAL_start == 0) && (VCO_sel_final_val == 1) && (freq_sel_final_val == 1))
    {
        int VCO_final = chip->Get_SPI_Reg_bits(VCO_SEL_FINAL, true, chip->channel, chip->PLLprofile);
        int freq_final = chip->Get_SPI_Reg_bits(FREQ_FINAL, true, chip->channel, chip->PLLprofile);
        status = chip->Modify_SPI_Reg_bits(VCO_SEL_n, VCO_final, true, chip->channel, chip->PLLprofile);
        status = chip->Modify_SPI_Reg_bits(VCO_FREQ_n, freq_final, true, chip->channel, chip->PLLprofile);
    }
    else
    {
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        evt.SetString(_("WARNING: Requested frequency could not be tuned."));
        wxPostEvent(this, evt);
    }

    // PLL_CALIBRATION_EN = 0
    status = chip->Modify_SPI_Reg_bits(PLL_CALIBRATION_EN, 0, true, chip->channel, chip->PLLprofile);

    // PLL_LODIST_EN_BIAS_n = 1
    status = chip->Modify_SPI_Reg_bits(PLL_LODIST_EN_BIAS_n, 1, true, chip->channel, chip->PLLprofile);

    // Return the internal PLL profile control to the original state (before the calibration)
    //	status = chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, sel_PLL_profile_internal, true, chip->channel, chip->PLLprofile);
    status = chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, sel_PLL_profile_internal, true);

    // Restore the controls of the PLL profile multiplexer & restore it after the calibration
    status = chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, pll_cfg_sel0, true);
    status = chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, pll_cfg_sel1, true);
    status = chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, pll_cfg_sel2, true);

    UpdateGUI();
}

void lms8001_pnlPLLProfiles_view::OnSmartTuneClick(wxCommandEvent& event)
{
    int result = 0;
    double fLO_GHz = 0;
    wxString fLO_wxString = txtSmartTunePLLFrequency->GetValue();
    result = txtSmartTunePLLFrequency->GetValue().ToDouble(&fLO_GHz);
    if (!result)
    {
        wxMessageBox(_("LO Freq. value should be a number."), _("Error"));
        return;
    }
    if (fLO_GHz <= 0)
    {
        wxMessageBox(_("LO Freq. value should be > 0"), _("Error"));
        return;
    }
    double fLO = fLO_GHz * 1E9;
    fLO = round(fLO);
    int fref = chip->mRefClk_MHz * 1E6;
    //	bool slfbenXBUF = false; //CHANGE THIS FOR THE BOARD REDESIGN!!!
    // Do not change the self bias enable value!!! Read the current value and set it later to the same value in configPLL.
    bool slfbenXBUF = chip->Get_SPI_Reg_bits(PLL_XBUF_SLFBEN, true);
    bool genIQ = chkSmartTuneGenIQ->GetValue();
    //	bool intMode = false; //CHECK THIS!!!
    bool intMode = chkINTMOD_EN_n->GetValue();
    double loopBW_kHz;
    int loopBW;
    result = txtSmartTuneLoopBW->GetValue().ToDouble(&loopBW_kHz);
    if (!result)
    {
        wxMessageBox(_("Loop BW value should be a number"), _("Error"));
        return;
    }
    if (loopBW_kHz <= 0)
    {
        wxMessageBox(_("Loop BW value should be > 0"), _("Error"));
        return;
    }
    loopBW = loopBW_kHz * 1E3;
    double pm;
    result = txtSmartTunePhaseMargin->GetValue().ToDouble(&pm);
    if (!result)
    {
        wxMessageBox(_("Phase Margin value should be a number"), _("Error"));
        return;
    }
    if (pm <= 0)
    {
        wxMessageBox(_("Phase Margin value should be > 0"), _("Error"));
        return;
    }
    bool fitKVCO = true;
    double bwef;
    result = txtSmartTuneBWEF->GetValue().ToDouble(&bwef);
    if (!result)
    {
        wxMessageBox(_("Bandwidth Extension Factor (BWEF) value should be a number"), _("Error"));
        return;
    }
    if (bwef <= 0)
    {
        wxMessageBox(_("Bandwidth Extension Factor (BWEF) value should be > 0"), _("Error"));
        return;
    }

    int flock_N = cmbFLOCK_N_n->GetValue();

    OpStatus status = configPLL(fLO, fref, slfbenXBUF, genIQ, intMode, loopBW, pm, fitKVCO, bwef, flock_N);

    // Setup (enable or disable) quadrature generation
    chip->Modify_SPI_Reg_bits(PLL_LODIST_FSP_OUT02_n, !genIQ, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_LODIST_FSP_OUT12_n, !genIQ, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_LODIST_FSP_OUT22_n, !genIQ, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_LODIST_FSP_OUT32_n, !genIQ, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_LODIST_EN_DIV2IQ_n, genIQ, true, chip->channel, chip->PLLprofile);

    // Enable LO distribution
    if (status == OpStatus::Success)
        chip->Modify_SPI_Reg_bits(PLL_LODIST_EN_BIAS_n, 1, true, chip->channel, chip->PLLprofile);

    UpdateGUI();
}

/***************************************************************************************************************
This method does complete configuration of LMS8001 IC PLL in 5 steps:
1. Runs VCO Coarse Frequency Tuning and Sets FF - DIV Ratios needed for generation of F_LO frequency
2. Centers VCO VTUNE voltage if needed
3. Optimizes PLL configuration for targeted LoopBW and Phase Margin(PM)
4. Optimize CP offset current and Lock - Detector threashold settings depending on chosen PLL operating mode
5. Sets Fast - Lock Settings
****************************************************************************************************************/
OpStatus lms8001_pnlPLLProfiles_view::configPLL(
    double fLO, int fref, bool slfbenXBUF, bool genIQ, bool intMode, int loopBW, double pm, bool fitKVCO, double bwef, int flock_N)
{
    OpStatus status, status1, status2, status3, status4, status5;

    status = OpStatus::Success;

    // Calculate Loop - Crossover frequency
    double fc = loopBW / 1.65;

    // Set optimal VCO settings
    chip->Modify_SPI_Reg_bits(VDIV_SWVDD_n, 2, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(VCO_AMP_n, 3, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(VCO_AAC_EN_n, 1, true, chip->channel, chip->PLLprofile);

    // Step 1 - Tune PLL to generate F_LO frequency at LODIST outputs that should be manualy enabled outside this method
    status1 = setLOFREQ(fLO, fref, slfbenXBUF, genIQ, intMode);

    if (status1 != OpStatus::Success)
    {
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        char message[512];
        sprintf(message, "PLL Tuning to F_LO=%.2f GHz failed.", fLO / 1.0E9);
        evt.SetString(_(message));
        wxPostEvent(this, evt);

        return status1;
        //		status = OpStatus::Error;
    }

    // Step 2 - Center VCO Tuning Voltage if needed
    status2 = centerVTUNE2(fref, slfbenXBUF);

    if (status2 != OpStatus::Success)
    {
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        char message[512];
        sprintf(message, "Centering VTUNE at F_LO=%.2f GHz failed.", fLO / 1.0E9);
        evt.SetString(_(message));
        wxPostEvent(this, evt);

        status = OpStatus::Error;
    }

    // Step 3 - Optimize PLL settings for targeted LoopBW
    status3 = optim_PLL_LoopBW(pm, fc, fitKVCO);

    if (status3 != OpStatus::Success)
    {
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        char message[512];
        sprintf(message,
            "Optimization of PLL at F_LO=%.2f GHz, LoopBW=%.2f kHz and PM=%.2f deg failed.",
            fLO / 1.0e9,
            loopBW / 1.0e3,
            pm);
        evt.SetString(_(message));
        wxPostEvent(this, evt);

        status = OpStatus::Error;
    }

    // Step 4 - Optimize CP offset current Lock Detector Threashold depending on operating mode chosen(IntN or FracN)
    status4 = optimCPandLD();
    if (status4 != OpStatus::Success)
        return status4;

    // Step 5 - Configure Fast - Lock Mode Registers
    status5 = setFLOCK(bwef, flock_N);
    if (status5 != OpStatus::Success)
        return status5;

    return status;
}

/*****************************************************************************************************
Automatically calculates Fast-Lock Mode parameters from BWEF argument. BWEF-BandWidth Extension Factor
******************************************************************************************************/
OpStatus lms8001_pnlPLLProfiles_view::setFLOCK(double bwef, int flock_N)
{

    int R3 = chip->Get_SPI_Reg_bits(R3_n, true, chip->channel, chip->PLLprofile);
    int R2 = chip->Get_SPI_Reg_bits(R2_n, true, chip->channel, chip->PLLprofile);
    int C1 = chip->Get_SPI_Reg_bits(C1_n, true, chip->channel, chip->PLLprofile);
    int C2 = chip->Get_SPI_Reg_bits(C2_n, true, chip->channel, chip->PLLprofile);
    int C3 = chip->Get_SPI_Reg_bits(C3_n, true, chip->channel, chip->PLLprofile);

    int PULSE = chip->Get_SPI_Reg_bits(PULSE_n, true, chip->channel, chip->PLLprofile);
    int OFS = chip->Get_SPI_Reg_bits(OFS_n, true, chip->channel, chip->PLLprofile);

    int R3_FLOCK = min(R3 * bwef, 15);
    int R2_FLOCK = min(R2 * bwef, 15);

    int PULSE_FLOCK = min(PULSE * pow(bwef, 2), 63);
    int OFS_FLOCK = OFS;

    chip->Modify_SPI_Reg_bits(FLOCK_R3_n, R3_FLOCK, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FLOCK_R2_n, R2_FLOCK, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FLOCK_C1_n, C1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FLOCK_C2_n, C2, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FLOCK_C3_n, C3, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FLOCK_PULSE_n, PULSE_FLOCK, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FLOCK_OFS_n, OFS_FLOCK, true, chip->channel, chip->PLLprofile);

    chip->Modify_SPI_Reg_bits(FLOCK_VCO_SPDUP_n, 0, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FLOCK_N_n, min(flock_N, 1023), true, chip->channel, chip->PLLprofile);

    return OpStatus::Success;
}

/**************************************************************************************************************************************************************************
This method checks if PLL works in fractional-N Mode. If this condition is true, it sets the offset CP current to optimize phase noise performance in FracN operation mode.
When CP offset current is used, it is recommended to set ICP_OFS ~3 % of ICP_PULSE
***************************************************************************************************************************************************************************/
OpStatus lms8001_pnlPLLProfiles_view::optimCPandLD()
{
    // Check operating mode of LMS8001 PLL
    int INTMOD_EN = chip->Get_SPI_Reg_bits(INTMOD_EN_n, true, chip->channel, chip->PLLprofile);

    // Read CP current configuration
    int PULSE = chip->Get_SPI_Reg_bits(PULSE_n, true, chip->channel, chip->PLLprofile);
    int OFS = chip->Get_SPI_Reg_bits(OFS_n, true, chip->channel, chip->PLLprofile);
    int ICT_CP = chip->Get_SPI_Reg_bits(ICT_CP_n, true, chip->channel, chip->PLLprofile);

    // Read Lock Detector Threashold Voltage
    int LD_VCT = chip->Get_SPI_Reg_bits(LD_VCT_n, true, chip->channel, chip->PLLprofile);

    // Calculate OFS and LD_VCT optimal values
    if (INTMOD_EN)
    {
        // Set Offset Current and Lock Detector Threashold for IntN - Operating Mode
        LD_VCT = 2;
        OFS = 0;
    }
    else
    {
        // Set Offset Current and Lock Detector Threashold for IntN - Operating Mode
        LD_VCT = 0;
        double Icp = (25.0 * ICT_CP / 16.0) * PULSE;
        // Calculate Target Value for Offset Current, as 3 % of Pulse current value
        double Icp_OFS = 0.03 * Icp;
        double Icp_OFS_step = (25.0 * ICT_CP / 16.0) * 0.25;
        OFS = max(1, int(Icp_OFS / Icp_OFS_step));
    }

    setCP(PULSE, OFS, ICT_CP);
    setLD(LD_VCT);

    wxCommandEvent evt;
    evt.SetEventType(LOG_MESSAGE);
    char message[512];
    sprintf(message, "Optimization of CP-OFS and LD-VCT Settings: OFS=%d, LD_VCT=%d", OFS, LD_VCT);

    evt.SetString(_(message));
    wxPostEvent(this, evt);

    return OpStatus::Success;
}

/******************************************
Sets Lock-Detector's Comparator Threashold
*******************************************/
void lms8001_pnlPLLProfiles_view::setLD(int LD_VCT)
{
    chip->Modify_SPI_Reg_bits(PLL_EN_LD_n, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(LD_VCT_n, LD_VCT, true, chip->channel, chip->PLLprofile);
}

/*****************************************************************************************************************************************************
This method finds optimal PLL configuration, CP pulse current and LPF element values.
Optimization finds maximal CP current which can results with targeted PLL Loop BW using Loop - Filter elements which can be implemented in LMS8001 IC.
Result should be PLL configuration with best phase noise performance for targeted loop bandwidth.
******************************************************************************************************************************************************/
OpStatus lms8001_pnlPLLProfiles_view::optim_PLL_LoopBW(double PM_deg, double fc, bool fitKVCO)
{
    // Calculate Phase Margin in radians
    // Calculate angle frequency for fc
    double PM_rad = PM_deg * M_PI / 180.0;
    double wc = 2.0 * M_PI * fc;

    // Get initial CP current settings
    int PULSE_INIT = chip->Get_SPI_Reg_bits(PULSE_n, true, chip->channel, chip->PLLprofile);
    int OFS_INIT = chip->Get_SPI_Reg_bits(OFS_n, true, chip->channel, chip->PLLprofile);
    int ICT_CP_INIT = chip->Get_SPI_Reg_bits(ICT_CP_n, true, chip->channel, chip->PLLprofile);

    // Pulse control word of CP inside LMS8001 will be swept from 63 to 4.
    // First value that gives implementable PLL configuration will be used.
    int cp_pulse_vals[60];
    int cp_pulse_vals_No;
    int currindex = 0;
    for (int i = 63; i >= 4; i--)
    {
        cp_pulse_vals[currindex] = i;
        currindex++;
    }
    cp_pulse_vals_No = currindex;

    //msavic start 170213 - Moved from the loop to speed up
    // Check VCO_SEL and VCO_FREQ
    int vco_sel = chip->Get_SPI_Reg_bits(VCO_SEL_n, true, chip->channel, chip->PLLprofile);
    int vco_freq = chip->Get_SPI_Reg_bits(VCO_FREQ_n, true, chip->channel, chip->PLLprofile);

    double KVCO_avg;

    if (!fitKVCO)
    {
        if (vco_sel == 1)
            KVCO_avg = 44.404e6;
        else if (vco_sel == 2)
            KVCO_avg = 33.924e6;
        else if (vco_sel == 3)
            KVCO_avg = 41.455e6;
        else
        {
            wxCommandEvent evt;
            evt.SetEventType(LOG_MESSAGE);
            evt.SetString(_("Ext. LO selected in PLL_PROFILE."));
            wxPostEvent(this, evt);

            return OpStatus::Error;
        }
    }
    else
    {
        // Use Fitted Values for KVCO in Calculations
        int CBANK = vco_freq;
        if (vco_sel == 1)
            KVCO_avg = 24.71e6 * (2.09e-10 * pow(CBANK, 4) + 2.77e-09 * pow(CBANK, 3) + 1.13e-05 * pow(CBANK, 2) +
                                     3.73e-03 * CBANK + 1.01e+00);
        else if (vco_sel == 2)
            KVCO_avg = 21.05e6 * (-9.88e-11 * pow(CBANK, 4) + 1.46e-07 * pow(CBANK, 3) + (-2.14e-05) * pow(CBANK, 2) +
                                     5.08e-03 * CBANK + 9.99e-01);
        else if (vco_sel == 3)
            KVCO_avg = 32.00e6 * (-1.04e-10 * pow(CBANK, 4) + 8.72e-08 * pow(CBANK, 3) + (-4.68e-06) * pow(CBANK, 2) +
                                     3.68e-03 * CBANK + 1.00e+00);
        else
        {
            wxCommandEvent evt;
            evt.SetEventType(LOG_MESSAGE);
            evt.SetString(_("Ext. LO selected in PLL_PROFILE."));
            wxPostEvent(this, evt);

            return OpStatus::Error;
        }
    }

    // Read Feedback-Divider Modulus
    double N = getNDIV();

    double Kvco = 2 * M_PI * KVCO_avg;

    double LMS8001_C1_STEP = 1.2e-12;
    double LMS8001_C2_STEP = 10.0e-12;
    double LMS8001_C3_STEP = 1.2e-12;
    double LMS8001_C2_FIX = 150.0e-12;
    double LMS8001_C3_FIX = 5.0e-12;
    double LMS8001_R2_0 = 24.6e3;
    double LMS8001_R3_0 = 14.9e3;

    //msavic end 170213 - Moved from the loop to speed up

    for (int i = 0; i < cp_pulse_vals_No; i++)
    {
        int cp_pulse = cp_pulse_vals[i];
        /* msavic 170213 - This is moved outside of the loop to speed up
		// Check VCO_SEL and VCO_FREQ
		int vco_sel = chip->Get_SPI_Reg_bits(VCO_SEL_n, true, chip->channel, chip->PLLprofile);
		int vco_freq = chip->Get_SPI_Reg_bits(VCO_FREQ_n, true, chip->channel, chip->PLLprofile);

		double KVCO_avg;

		if (!fitKVCO) {
			if (vco_sel == 1)
				KVCO_avg = 44.404e6;
			else if (vco_sel == 2)
				KVCO_avg = 33.924e6;
			else if (vco_sel == 3)
				KVCO_avg = 41.455e6;
			else {
				wxCommandEvent evt;
				evt.SetEventType(LOG_MESSAGE);
				evt.SetString(_("Ext. LO selected in PLL_PROFILE."));
				wxPostEvent(this, evt);

				return OpStatus::Error;
			}
		}
		else {
			// Use Fitted Values for KVCO in Calculations
			int CBANK = vco_freq;
			if (vco_sel == 1)
				KVCO_avg = 24.71e6*(2.09e-10*pow(CBANK, 4) + 2.77e-09*pow(CBANK, 3) + 1.13e-05*pow(CBANK, 2) + 3.73e-03*CBANK + 1.01e+00);
			else if (vco_sel == 2)
				KVCO_avg = 21.05e6 * (-9.88e-11*pow(CBANK, 4) + 1.46e-07*pow(CBANK, 3) + (-2.14e-05)*pow(CBANK, 2) + 5.08e-03*CBANK + 9.99e-01);
			else if (vco_sel == 3)
				KVCO_avg = 32.00e6 * (-1.04e-10*pow(CBANK, 4) + 8.72e-08*pow(CBANK, 3) + (-4.68e-06)*pow(CBANK, 2) + 3.68e-03*CBANK + 1.00e+00);
			else {
				wxCommandEvent evt;
				evt.SetEventType(LOG_MESSAGE);
				evt.SetString(_("Ext. LO selected in PLL_PROFILE."));
				wxPostEvent(this, evt);

				return OpStatus::Error;
			}
		}
*/
        // Read CP Current Value
        double Icp = ICT_CP_INIT * 25.0e-6 / 16.0 * cp_pulse;

        /* msavic 170213 - Moved outside the loop to speed up
		// Read Feedback-Divider Modulus
		double N = getNDIV();
		
		double Kvco = 2 * M_PI*KVCO_avg;
*/
        double Kphase = Icp / (2 * M_PI);

        double gamma = 1.045;
        double T31 = 0.1;

        // Approx. formula, Dean Banerjee
        double T1 = (1.0 / cos(PM_rad) - tan(PM_rad)) / (wc * (1 + T31));

        double T3 = T1 * T31;
        double T2 = gamma / ((pow(wc, 2)) * (T1 + T3));

        double A0 =
            (Kphase * Kvco) / ((pow(wc, 2)) * N) *
            sqrt((1 + (pow(wc, 2)) * (pow(T2, 2))) / ((1 + (pow(wc, 2)) * (pow(T1, 2))) * (1 + (pow(wc, 2)) * (pow(T3, 2)))));
        double A2 = A0 * T1 * T3;
        double A1 = A0 * (T1 + T3);

        double C1 = A2 / (pow(T2, 2)) * (1 + sqrt(1 + T2 / A2 * (T2 * A0 - A1)));
        double C3 = (-(pow(T2, 2)) * (pow(C1, 2)) + T2 * A1 * C1 - A2 * A0) / ((pow(T2, 2)) * C1 - A2);
        double C2 = A0 - C1 - C3;
        double R2 = T2 / C2;
        double R3 = A2 / (C1 * C3 * T2);

        /* msavic 170213 - Moved out of the loop to speed up
		double LMS8001_C1_STEP = 1.2e-12;
		double LMS8001_C2_STEP = 10.0e-12;
		double LMS8001_C3_STEP = 1.2e-12;
		double LMS8001_C2_FIX = 150.0e-12;
		double LMS8001_C3_FIX = 5.0e-12;
		double LMS8001_R2_0 = 24.6e3;
		double LMS8001_R3_0 = 14.9e3;
*/

        bool C1_cond = (LMS8001_C1_STEP <= C1) && (C1 <= 15.0 * LMS8001_C1_STEP);
        bool C2_cond = (LMS8001_C2_FIX <= C2) && (C2 <= LMS8001_C2_FIX + 15.0 * LMS8001_C2_STEP);
        bool C3_cond = (LMS8001_C3_FIX + LMS8001_C3_STEP <= C3) && (C3 <= LMS8001_C3_FIX + 15.0 * LMS8001_C3_STEP);
        bool R2_cond = (LMS8001_R2_0 / 15.0 <= R2) && (R2 <= LMS8001_R2_0);
        bool R3_cond = (LMS8001_R3_0 / 15.0 <= R3) && (R3 <= LMS8001_R3_0);

        if (C1_cond && C2_cond && C3_cond && R2_cond && R3_cond)
        {

            int C1_CODE = int(round(C1 / LMS8001_C1_STEP));
            int C2_CODE = int(round((C2 - LMS8001_C2_FIX) / LMS8001_C2_STEP));
            int C3_CODE = int(round((C3 - LMS8001_C3_FIX) / LMS8001_C3_STEP));
            C1_CODE = int(min(max(C1_CODE, 0), 15));
            C2_CODE = int(min(max(C2_CODE, 0), 15));
            C3_CODE = int(min(max(C3_CODE, 0), 15));

            int R2_CODE = int(round(LMS8001_R2_0 / R2));
            int R3_CODE = int(round(LMS8001_R3_0 / R3));
            R2_CODE = min(max(R2_CODE, 1), 15);
            R3_CODE = min(max(R3_CODE, 1), 15);

            // Set CP Pulse Current to the optimized value
            setCP(cp_pulse, OFS_INIT, ICT_CP_INIT);

            // Set LPF Components to the optimized values
            setLPF(C1_CODE, C2_CODE, R2_CODE, C3_CODE, R3_CODE);

            return OpStatus::Success;
        }
    }

    wxCommandEvent evt;
    evt.SetEventType(LOG_MESSAGE);
    evt.SetString(_("PLL LoopBW Optimization failed. Some of the LPF components out of implementable range."));
    wxPostEvent(this, evt);

    // Set back to initial settings of CP
    setCP(PULSE_INIT, OFS_INIT, ICT_CP_INIT);

    return OpStatus::Error;
}

/***********************
Sets LPF Element Values
************************/
void lms8001_pnlPLLProfiles_view::setLPF(int C1, int C2, int R2, int C3, int R3)
{
    chip->Modify_SPI_Reg_bits(R3_n, R3, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(R2_n, R2, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(C2_n, C2, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(C1_n, C1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(C3_n, C3, true, chip->channel, chip->PLLprofile);
}

/******************
Sets CP Parameters
*******************/
void lms8001_pnlPLLProfiles_view::setCP(int PULSE, int OFS, int ICT_CP)
{

    chip->Modify_SPI_Reg_bits(PULSE_n, PULSE, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(OFS_n, OFS, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(ICT_CP_n, ICT_CP, true, chip->channel, chip->PLLprofile);

    if (OFS >= 1)
        chip->Modify_SPI_Reg_bits(PLL_EN_CPOFS_n, 1, true, chip->channel, chip->PLLprofile);
    else
        chip->Modify_SPI_Reg_bits(PLL_EN_CPOFS_n, 0, true, chip->channel, chip->PLLprofile);
}

/***************************************************************************************************************************************
This method should be used when coarse tuning algorithm converges to the subband at which PLL locks with VTUNE_HIGH = 1 or VTUNE_LOW = 1
If it's possible, this method tweaks different VCO setings in order to get PLL locked at desired frequency with VTUNE_HIGH=VTUNE_LOW=0
The purpose of this method is same as of centerVTUNE method.
Algorithm is different.
****************************************************************************************************************************************/
OpStatus lms8001_pnlPLLProfiles_view::centerVTUNE2(int fref, bool slfbenXBUF)
{
    // Save the controls of the PLL profile multiplexer & restore it after the calibration
    int pll_cfg_sel0 = chip->Get_SPI_Reg_bits(PLL_CFG_SEL0, true, chip->channel);
    int pll_cfg_sel1 = chip->Get_SPI_Reg_bits(PLL_CFG_SEL1, true, chip->channel);
    int pll_cfg_sel2 = chip->Get_SPI_Reg_bits(PLL_CFG_SEL2, true, chip->channel);

    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, PLL_CFG_SEL0.defaultValue, true, chip->channel);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, PLL_CFG_SEL1.defaultValue, true, chip->channel);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, PLL_CFG_SEL2.defaultValue, true, chip->channel);

    // Save the internal PLL profile control before the calibration, to be restored after calibration
    int sel_PLL_profile_internal = chip->Get_SPI_Reg_bits(PLL_CFG_INT_SEL, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, chip->PLLprofile, true, chip->channel, chip->PLLprofile);

    // Reset PLL
    chip->Modify_SPI_Reg_bits(PLL_RSTN, 0, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_RSTN, 1, true, chip->channel, chip->PLLprofile);

    // Get Initial value for VCO_FREQ<1:0> word
    int freq_init = chip->Get_SPI_Reg_bits(VCO_FREQ_n, true, chip->channel, chip->PLLprofile);

    // Get Initial value for VDIV_SWVDD<1:0> word
    int vdiv_swvdd_init = chip->Get_SPI_Reg_bits(VDIV_SWVDD_n, true, chip->channel, chip->PLLprofile);
    int sel_init = chip->Get_SPI_Reg_bits(VCO_SEL_n, true, chip->channel, chip->PLLprofile);

    // Get Initial Value for VCO_AMP<7:0> and VCO_AAC_EN
    int amp_init = chip->Get_SPI_Reg_bits(VCO_AMP_n, true, chip->channel, chip->PLLprofile);
    int aac_en_init = chip->Get_SPI_Reg_bits(VCO_AAC_EN_n, true, chip->channel, chip->PLLprofile);

    // Get Initial Value for INTMOD_EN
    int INTMOD_EN = chip->Get_SPI_Reg_bits(INTMOD_EN_n, true, chip->channel, chip->PLLprofile);

    // Get Initial Value for PDIV2_EN
    int PDIV2_EN = chip->Get_SPI_Reg_bits(PLL_EN_FB_PDIV2_n, true, chip->channel, chip->PLLprofile);

    // Get Initial Values stored in PLL_CAL_AUTO1 register
    int vco_sel_force_init = chip->Get_SPI_Reg_bits(VCO_SEL_FORCE, true, chip->channel, chip->PLLprofile);
    int vco_sel_init = chip->Get_SPI_Reg_bits(VCO_SEL_INIT, true, chip->channel, chip->PLLprofile);
    int vco_freq_init_pos = chip->Get_SPI_Reg_bits(FREQ_INIT_POS, true, chip->channel, chip->PLLprofile);
    int vco_freq_init = chip->Get_SPI_Reg_bits(FREQ_INIT, true, chip->channel, chip->PLLprofile);

    // Get VTUNE_HIGH, VTUNE_LOW, PLL_LOCK bit values
    int vtune_high = chip->Get_SPI_Reg_bits(VTUNE_HIGH, true, chip->channel, chip->PLLprofile);
    int vtune_low = chip->Get_SPI_Reg_bits(VTUNE_LOW, true, chip->channel, chip->PLLprofile);
    // int pll_lock = chip->Get_SPI_Reg_bits(PLL_LOCK, true, chip->channel, chip->PLLprofile);

    double fTarget = getNDIV() * fref; //msavic - Is it OK for freq. to be int (maybe it should be double)?
    // int freq = freq_init;

    if ((vtune_high == 0) && (vtune_low == 0))
    {
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        evt.SetString(_("Centering of VTUNE not needed."));
        wxPostEvent(this, evt);

        // Return the internal PLL profile control to the original state (before the calibration)
        chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, sel_PLL_profile_internal, true);

        // Restore the controls of the PLL profile multiplexer & restore it after the calibration
        chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, pll_cfg_sel0, true);
        chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, pll_cfg_sel1, true);
        chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, pll_cfg_sel2, true);

        return OpStatus::Success;
    }

    int swvdd_list[4] = { 3, 2, 1, 0 };
    int amp_list[4] = { 3, 2, 1, 0 };

    for (int i = 0; i < 4; i++)
    {
        int vdiv_swvdd = swvdd_list[i];
        if (!(vdiv_swvdd_init == vdiv_swvdd))
        {
            chip->Modify_SPI_Reg_bits(VDIV_SWVDD_n, vdiv_swvdd, true, chip->channel, chip->PLLprofile);
            int vtune_vct = 1;
            int vco_sel_force = 1;
            int freq_init_pos = 4;
            vco_auto_ctune(
                fTarget, fref, slfbenXBUF, INTMOD_EN, PDIV2_EN, vtune_vct, vco_sel_force, sel_init, freq_init_pos, freq_init);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            int vtune_high = chip->Get_SPI_Reg_bits(VTUNE_HIGH, true, chip->channel, chip->PLLprofile);
            int vtune_low = chip->Get_SPI_Reg_bits(VTUNE_LOW, true, chip->channel, chip->PLLprofile);
            // int pll_lock = chip->Get_SPI_Reg_bits(PLL_LOCK, true, chip->channel, chip->PLLprofile);

            if ((vtune_high == 0) && (vtune_low == 0))
            {
                wxCommandEvent evt;
                evt.SetEventType(LOG_MESSAGE);
                char message[512];
                sprintf(message,
                    "VTUNE voltage centered successfuly by changing VDIV_SWVDD value.\nNew value, VDIV_SWVDD<1:0> = %d",
                    vdiv_swvdd);
                evt.SetString(_(message));
                wxPostEvent(this, evt);

                // Set back PLL_CAL_AUTO1 to starting values
                chip->Modify_SPI_Reg_bits(VCO_SEL_FORCE, vco_sel_force_init, true, chip->channel, chip->PLLprofile);
                chip->Modify_SPI_Reg_bits(VCO_SEL_INIT, vco_sel_init, true, chip->channel, chip->PLLprofile);
                chip->Modify_SPI_Reg_bits(FREQ_INIT_POS, vco_freq_init_pos, true, chip->channel, chip->PLLprofile);
                chip->Modify_SPI_Reg_bits(FREQ_INIT, vco_freq_init, true, chip->channel, chip->PLLprofile);

                // Return the internal PLL profile control to the original state (before the calibration)
                chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, sel_PLL_profile_internal, true);

                // Restore the controls of the PLL profile multiplexer & restore it after the calibration
                chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, pll_cfg_sel0, true);
                chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, pll_cfg_sel1, true);
                chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, pll_cfg_sel2, true);

                return OpStatus::Success;
            }
        }
    }
    // Set back VDIV_SWVDD<1:0> and FREQ<7:0> to inital values
    chip->Modify_SPI_Reg_bits(VDIV_SWVDD_n, vdiv_swvdd_init, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(VCO_FREQ_n, freq_init, true, chip->channel, chip->PLLprofile);

    for (int i = 0; i < 4; i++)
    {
        int amp = amp_list[i];
        if (!(amp_init == amp))
        {
            chip->Modify_SPI_Reg_bits(VCO_AMP_n, amp, true, chip->channel, chip->PLLprofile);
            chip->Modify_SPI_Reg_bits(VCO_AAC_EN_n, 1, true, chip->channel, chip->PLLprofile);
            int vtune_vct = 1;
            int vco_sel_force = 1;
            int freq_init_pos = 4;
            vco_auto_ctune(
                fTarget, fref, slfbenXBUF, INTMOD_EN, PDIV2_EN, vtune_vct, vco_sel_force, sel_init, freq_init_pos, freq_init);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            int vtune_high = chip->Get_SPI_Reg_bits(VTUNE_HIGH, true, chip->channel, chip->PLLprofile);
            int vtune_low = chip->Get_SPI_Reg_bits(VTUNE_LOW, true, chip->channel, chip->PLLprofile);
            // int pll_lock = chip->Get_SPI_Reg_bits(PLL_LOCK, true, chip->channel, chip->PLLprofile);

            if ((vtune_high == 0) && (vtune_low == 0))
            {
                wxCommandEvent evt;
                evt.SetEventType(LOG_MESSAGE);
                char message[512];
                sprintf(message, "VTUNE voltage centered successfuly by changing VCO_AMP value.\nNew value, VCO_AMP = %d", amp);
                evt.SetString(_(message));
                wxPostEvent(this, evt);

                // Set back PLL_CAL_AUTO1 to starting values
                chip->Modify_SPI_Reg_bits(VCO_SEL_FORCE, vco_sel_force_init, true, chip->channel, chip->PLLprofile);
                chip->Modify_SPI_Reg_bits(VCO_SEL_INIT, vco_sel_init, true, chip->channel, chip->PLLprofile);
                chip->Modify_SPI_Reg_bits(FREQ_INIT_POS, vco_freq_init_pos, true, chip->channel, chip->PLLprofile);
                chip->Modify_SPI_Reg_bits(FREQ_INIT, vco_freq_init, true, chip->channel, chip->PLLprofile);

                // Return the internal PLL profile control to the original state (before the calibration)
                chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, sel_PLL_profile_internal, true);

                // Restore the controls of the PLL profile multiplexer & restore it after the calibration
                chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, pll_cfg_sel0, true);
                chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, pll_cfg_sel1, true);
                chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, pll_cfg_sel2, true);

                return OpStatus::Success;
            }
        }
    }
    // Set back VCO_AMP, VCO_AAC_EN, and FREQ<7:0> to inital values
    chip->Modify_SPI_Reg_bits(VCO_AMP_n, amp_init, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(VCO_AAC_EN_n, aac_en_init, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(VCO_FREQ_n, freq_init, true, chip->channel, chip->PLLprofile);

    // Return the internal PLL profile control to the original state (before the calibration)
    chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, sel_PLL_profile_internal, true);

    // Restore the controls of the PLL profile multiplexer & restore it after the calibration
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, pll_cfg_sel0, true);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, pll_cfg_sel1, true);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, pll_cfg_sel2, true);

    wxCommandEvent evt;
    evt.SetEventType(LOG_MESSAGE);
    evt.SetString(_("Centering VTUNE using VDIV_SWVDD and VCO_AMP failed. Returning to original settings and exiting."));
    wxPostEvent(this, evt);

    return OpStatus::Error;
}

/************************************************************************************************
Returns float that represents PLL feedback division ratio for configuration in PLL profile PINDEX
*************************************************************************************************/
double lms8001_pnlPLLProfiles_view::getNDIV()
{
    int PDIV2 = chip->Get_SPI_Reg_bits(PLL_EN_FB_PDIV2_n, true, chip->channel, chip->PLLprofile);

    int NINT = chip->Get_SPI_Reg_bits(INTMOD_n, true, chip->channel, chip->PLLprofile);
    int nfrach = chip->Get_SPI_Reg_bits(FRACMODH_n, true, chip->channel, chip->PLLprofile);
    int nfracl = chip->Get_SPI_Reg_bits(FRACMODL_n, true, chip->channel, chip->PLLprofile);
    int NFRAC = nfrach * pow(2, 16) + nfracl;

    return pow(2.0, PDIV2) * 1.0 * (NINT * 1.0 + NFRAC * 1.0 / pow(2.0, 20));
}

/********************************************************************************************
This methods configures PLL-LODIST subsystems of LMS8001 IC to generate desired LO frequency.
*********************************************************************************************/
OpStatus lms8001_pnlPLLProfiles_view::setLOFREQ(double fLO, int fref, bool slfbenXBUF, bool genIQ, bool intMode)
{
    int DIV2IQ = 0;
    int FFMOD = 0;

    if (genIQ)
    {
        if (!((fLO >= 260.0E6) && (fLO <= 4.55E9)))
        {
            wxCommandEvent evt;
            evt.SetEventType(LOG_MESSAGE);
            evt.SetString(_(
                "LO frequency should be between 260 MHz and 4.55 GHz, when Generate IQ is selected. Failed to set LO frequency."));
            wxPostEvent(this, evt);

            return OpStatus::Error;
        }
        DIV2IQ = 1;
    }
    else
    {
        if (!((fLO >= 260.0E6) && (fLO <= 9.11E9)))
        {
            wxCommandEvent evt;
            evt.SetEventType(LOG_MESSAGE);
            evt.SetString(_("LO frequency should be between 260 MHz and 9.11 GHz. Failed to set LO frequency."));
            wxPostEvent(this, evt);

            return OpStatus::Error;
        }
        if ((fLO >= 260.0E6) && (fLO <= 520E6))
        {
            wxCommandEvent evt;
            evt.SetEventType(LOG_MESSAGE);
            evt.SetString(
                _("LO frequency values between 260 MHz and 520 MHz can only be generated when IQ=True. Failed to set Lo Freq."));
            wxPostEvent(this, evt);

            return OpStatus::Error;
        }
        DIV2IQ = 0;
    }

    double fVCO = fLO * pow(2, DIV2IQ) * pow(2, FFMOD);
    while (!((fVCO >= 4.1E9) && (fVCO <= 9.11E9)))
    {
        FFMOD++;
        fVCO = fLO * pow(2.0, DIV2IQ) * pow(2.0, FFMOD);
    }

    // Set FF - DIV Control Signals
    setFFDIV(FFMOD);
    OpStatus status = vco_auto_ctune(fVCO, fref, slfbenXBUF, intMode);

    if (status == OpStatus::Error)
    {
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        evt.SetString(_("Setting LO Frequency failed."));
        wxPostEvent(this, evt);
    }

    return status;
}

/**************************************************************************************
Performs VCO Coarse Frequency Tuning Using On-Chip LMS8001 IC Calibration State-Machine
***************************************************************************************/

OpStatus lms8001_pnlPLLProfiles_view::vco_auto_ctune(double fVCO,
    int fref,
    bool slfbenXBUF,
    bool intMode,
    bool pdiv2,
    int vtune_vct,
    int vco_sel_force,
    int vco_sel_init,
    int freq_init_pos,
    int freq_init,
    int freq_settling_N,
    int vtune_wait_N,
    int vco_sel_freq_max,
    int vco_sel_freq_min)
{
    OpStatus result = OpStatus::Success;

    // Save the controls of the PLL profile multiplexer & restore it after the calibration
    int pll_cfg_sel0 = chip->Get_SPI_Reg_bits(PLL_CFG_SEL0, true, chip->channel);
    int pll_cfg_sel1 = chip->Get_SPI_Reg_bits(PLL_CFG_SEL1, true, chip->channel);
    int pll_cfg_sel2 = chip->Get_SPI_Reg_bits(PLL_CFG_SEL2, true, chip->channel);

    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, PLL_CFG_SEL0.defaultValue, true, chip->channel);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, PLL_CFG_SEL1.defaultValue, true, chip->channel);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, PLL_CFG_SEL2.defaultValue, true, chip->channel);

    // Save the internal PLL profile control before the calibration, to be restored after calibration
    int sel_PLL_profile_internal = chip->Get_SPI_Reg_bits(PLL_CFG_INT_SEL, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, chip->PLLprofile, true, chip->channel, chip->PLLprofile);

    // Determine the FB - DIV configuration for targeted VCO frequency and F_REF reference frequency
    int nint, nfrac, nfix;
    calc_fbdiv(fVCO, fref, intMode, pdiv2, &nint, &nfrac, &nfix);

    // The exact value of targetec VCO frequency that will be used in automatic coarse - tune algorithm
    // If IntN - Mode is chosen, VCO will be locked to the closest integer multiple of reference frequency
    // double fVCO_target = nfix * (nint + nfrac / pow(2.0, 20)) * fref;

    // Calculate the fractional division words
    int nfrach = int(floor(nfrac / pow(2.0, 16)));
    int nfracl = int(nfrac - nfrach * pow(2.0, 16));

    // Enable PLL
    enablePLL(pdiv2, intMode, slfbenXBUF);

    // Set the VCO tuning voltage value during coarse - tuning
    chip->Modify_SPI_Reg_bits(VTUNE_VCT_n, vtune_vct, true, chip->channel, chip->PLLprofile);

    // Define SDM & FB - DIV Modulus
    //ovdi!!!
    // We decided that in case that integer mode is possible, only the message is printed!
    //	bool en_INTMOD_EN = (intMode || (nfrac == 0));
    if (nfrac == 0)
    {
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        evt.SetString(_("MESSAGE: Requested LO frequency can be generated in integer mode."));
        wxPostEvent(this, evt);
    }

    bool en_INTMOD_EN = intMode;

    chip->Modify_SPI_Reg_bits(INTMOD_EN_n, en_INTMOD_EN, true, chip->channel, chip->PLLprofile);

    chip->Modify_SPI_Reg_bits(INTMOD_n, int(nint), true, chip->channel, chip->PLLprofile);

    chip->Modify_SPI_Reg_bits(FRACMODL_n, nfracl, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FRACMODH_n, nfrach, true, chip->channel, chip->PLLprofile);

    // Reset PLL, Enable Calibration Mode
    chip->Modify_SPI_Reg_bits(PLL_RSTN, 0, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_RSTN, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_CALIBRATION_EN, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(CTUNE_RES, 3, true, chip->channel, chip->PLLprofile);

    // Write VCO AUTO - CAL Registers
    chip->Modify_SPI_Reg_bits(VCO_SEL_FORCE, vco_sel_force, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(VCO_SEL_INIT, vco_sel_init, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FREQ_INIT_POS, freq_init_pos, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(FREQ_INIT, freq_init, true, chip->channel, chip->PLLprofile);

    chip->Modify_SPI_Reg_bits(FREQ_SETTLING_N, freq_settling_N, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(VTUNE_WAIT_N, vtune_wait_N, true, chip->channel, chip->PLLprofile);

    chip->Modify_SPI_Reg_bits(VCO_SEL_FREQ_MAX, vco_sel_freq_max, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(VCO_SEL_FREQ_MIN, vco_sel_freq_min, true, chip->channel, chip->PLLprofile);

    // Start VCO Auto - Tuning Process
    chip->Modify_SPI_Reg_bits(FCAL_START, 1, true, chip->channel, chip->PLLprofile);

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    //  If it is not, check again?

    //  If FCAL_START == 0 and
    //  VCO_SEL_FINAL_VAL == 1 and FREQ_FINAL_VAL == 1 then
    //  value from VCO_SEL_FINAL should be copied to VCO_SEL and
    //  value from FREQ_FINAL should be copied to VCO_FREQ

    int FCAL_start = chip->Get_SPI_Reg_bits(FCAL_START, true, chip->channel, chip->PLLprofile);
    int VCO_sel_final_val = chip->Get_SPI_Reg_bits(VCO_SEL_FINAL_VAL, true, chip->channel, chip->PLLprofile);
    int freq_sel_final_val = chip->Get_SPI_Reg_bits(FREQ_FINAL_VAL, true, chip->channel, chip->PLLprofile);
    if ((FCAL_start == 0) && (VCO_sel_final_val == 1) && (freq_sel_final_val == 1))
    {
        int VCO_final = chip->Get_SPI_Reg_bits(VCO_SEL_FINAL, true, chip->channel, chip->PLLprofile);
        int freq_final = chip->Get_SPI_Reg_bits(FREQ_FINAL, true, chip->channel, chip->PLLprofile);
        chip->Modify_SPI_Reg_bits(VCO_SEL_n, VCO_final, true, chip->channel, chip->PLLprofile);
        chip->Modify_SPI_Reg_bits(VCO_FREQ_n, freq_final, true, chip->channel, chip->PLLprofile);
    }
    else
    {
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        evt.SetString(_("WARNING: Requested frequency could not be tuned."));
        wxPostEvent(this, evt);

        result = OpStatus::Error;
    }

    chip->Modify_SPI_Reg_bits(PLL_CALIBRATION_EN, 0, true, chip->channel, chip->PLLprofile);

    // Return the internal PLL profile control to the original state (before the calibration)
    // status = chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, sel_PLL_profile_internal, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_CFG_INT_SEL, sel_PLL_profile_internal, true);

    // Restore the controls of the PLL profile multiplexer & restore it after the calibration
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL0, pll_cfg_sel0, true);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL1, pll_cfg_sel1, true);
    chip->Modify_SPI_Reg_bits(PLL_CFG_SEL2, pll_cfg_sel2, true);

    wxCommandEvent evt;
    evt.SetEventType(LOG_MESSAGE);
    evt.SetString(_("Calibration finished."));
    wxPostEvent(this, evt);

    return result;
}

/************************************
Enables VCO Bias, XBUF and PLL Blocks
*************************************/
void lms8001_pnlPLLProfiles_view::enablePLL(bool pdiv2, bool intMode, bool slfbenXBUF)
{
    // Define PLL Config
    // Enable VCO Biasing Block
    chip->Modify_SPI_Reg_bits(EN_VCOBIAS, 1, true, chip->channel, chip->PLLprofile);

    // Enable XBUF
    // Sets SLFBEN, when TCXO is AC - coupled to LMS8001 IC REFIN
    chip->Modify_SPI_Reg_bits(PLL_XBUF_EN, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_XBUF_SLFBEN, slfbenXBUF, true, chip->channel, chip->PLLprofile);

    // Define Desired PLL Profile
    // Enable Blocks
    chip->Modify_SPI_Reg_bits(PLL_EN_VTUNE_COMP_n, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_EN_LD_n, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_EN_PFD_n, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_EN_CP_n, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_EN_VCO_n, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_EN_FFDIV_n, 1, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_EN_FBDIV_n, 1, true, chip->channel, chip->PLLprofile);

    chip->Modify_SPI_Reg_bits(PLL_EN_FB_PDIV2_n, pdiv2, true, chip->channel, chip->PLLprofile);
    chip->Modify_SPI_Reg_bits(PLL_SDM_CLK_EN_n, !intMode, true, chip->channel, chip->PLLprofile);
}

/************************************************************************
Calculates Configuration Parameters for FB-DIV for targeted VCO Frequency
*************************************************************************/
void lms8001_pnlPLLProfiles_view::calc_fbdiv(double fVCO, int fref, bool intMode, bool pdiv2, int* nint, int* nfrac, int* nfix)
{
    if (pdiv2)
        *nfix = 2;
    else
        *nfix = 1;

    // Integer - N or Fractional - N Mode
    if (intMode)
    {
        *nint = round(fVCO / (*nfix) / fref);
        *nfrac = 0;
    }
    else
    {
        *nint = int(floor(fVCO / (*nfix) / fref));
        //		*nfrac = int(pow(2.0, 20) * (fVCO / (*nfix) / fref - *nint));
        *nfrac = int(round(pow(2.0, 20) * (fVCO / (*nfix) / fref - *nint)));
    }
}

/*******************
Sets FF-DIV Modulus
********************/
OpStatus lms8001_pnlPLLProfiles_view::setFFDIV(int FFMOD)
{
    int FF_DIV_val = FFMOD;

    OpStatus status;
    status = chip->Modify_SPI_Reg_bits(FF_MOD_n, FF_DIV_val, true, chip->channel, chip->PLLprofile);
    status = chip->Modify_SPI_Reg_bits(FFCORE_MOD_n, FF_DIV_val, true, chip->channel, chip->PLLprofile);

    bool en_PLL_EN_FFCORE_n = (FF_DIV_val > 0);
    bool en_FFDIV_SEL_n = (FF_DIV_val > 0);

    status = chip->Modify_SPI_Reg_bits(FFDIV_SEL_n, en_FFDIV_SEL_n, true, chip->channel, chip->PLLprofile);
    status = chip->Modify_SPI_Reg_bits(PLL_EN_FFCORE_n, en_PLL_EN_FFCORE_n, true, chip->channel, chip->PLLprofile);
    return status;
}

void lms8001_pnlPLLProfiles_view::On_FF_MOD_Change(wxCommandEvent& event)
{
    int FF_DIV_val = cmbFF_MOD_n->GetSelection();
    int debug = chkEnableFFDIVDebug->GetValue();
    chkFFDIV_SEL_n->Enable(debug);
    cmbFFCORE_MOD_n->Enable(debug);

    if (debug == 0)
    {
        chkFFDIV_SEL_n->SetValue(FF_DIV_val > 0);
        cmbFFCORE_MOD_n->SetSelection(FF_DIV_val);
    }

    OpStatus status;
    status = chip->Modify_SPI_Reg_bits(FF_MOD_n, cmbFF_MOD_n->GetSelection(), true, chip->channel, chip->PLLprofile);
    if (status != OpStatus::Success)
        return;
    status = chip->Modify_SPI_Reg_bits(FFCORE_MOD_n, cmbFFCORE_MOD_n->GetSelection(), true, chip->channel, chip->PLLprofile);
    status = chip->Modify_SPI_Reg_bits(FFDIV_SEL_n, chkFFDIV_SEL_n->GetValue(), true, chip->channel, chip->PLLprofile);

    bool en_PLL_EN_FFCORE_n = false;
    if (FF_DIV_val > 0)
        en_PLL_EN_FFCORE_n = true;

    status = chip->Modify_SPI_Reg_bits(PLL_EN_FFCORE_n, en_PLL_EN_FFCORE_n, true, chip->channel, chip->PLLprofile);
    chkPLL_EN_FFCORE_n->SetValue(en_PLL_EN_FFCORE_n);

    //	UpdateGUI(); //Maybe here it is not needed to update, all data should be OK...
}

/*
void lms8001_pnlPLLProfiles_view::OnSetFocus_cmbPLL_LODIST_FSP_OUTCH10_n(wxFocusEvent& event)
{
	wxComboBox* currCombo = reinterpret_cast<wxComboBox*>(event.GetEventObject());
	wxCheckBox* currCheck;

	if (currCombo == cmbPLL_LODIST_FSP_OUT010_n)
		currCheck = chkPLL_LODIST_FSP_OUT02_n;
	if (currCombo == cmbPLL_LODIST_FSP_OUT110_n)
		currCheck = chkPLL_LODIST_FSP_OUT12_n;
	if (currCombo == cmbPLL_LODIST_FSP_OUT210_n)
		currCheck = chkPLL_LODIST_FSP_OUT22_n;
	if (currCombo == cmbPLL_LODIST_FSP_OUT310_n)
		currCheck = chkPLL_LODIST_FSP_OUT32_n;

	wxArrayString temp;

	bool checked = currCheck->GetValue();

	if (checked) {
		temp.push_back("0");
		temp.push_back("0");
		temp.push_back("180");
		temp.push_back("180");
	} else {
		temp.push_back("0");
		temp.push_back("90");
		temp.push_back("180");
		temp.push_back("270");
	}

	int selection = currCombo->GetSelection();
	currCombo->Set(temp);
	currCombo->Select(selection);
}
*/

void lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbPLL_LODIST_FSP_OUTCH10_n(wxUpdateUIEvent& event)
{
    wxComboBox* currCombo = reinterpret_cast<wxComboBox*>(event.GetEventObject());
    wxCheckBox* currCheck = nullptr;

    if (currCombo == cmbPLL_LODIST_FSP_OUT010_n)
        currCheck = chkPLL_LODIST_FSP_OUT02_n;
    if (currCombo == cmbPLL_LODIST_FSP_OUT110_n)
        currCheck = chkPLL_LODIST_FSP_OUT12_n;
    if (currCombo == cmbPLL_LODIST_FSP_OUT210_n)
        currCheck = chkPLL_LODIST_FSP_OUT22_n;
    if (currCombo == cmbPLL_LODIST_FSP_OUT310_n)
        currCheck = chkPLL_LODIST_FSP_OUT32_n;

    wxArrayString newList;

    bool checked = currCheck->GetValue();

    if (checked)
    {
        newList.push_back("0");
        newList.push_back("0");
        newList.push_back("180");
        newList.push_back("180");
    }
    else
    {
        newList.push_back("0");
        newList.push_back("90");
        newList.push_back("180");
        newList.push_back("270");
    }

    wxArrayString oldList;

    oldList = currCombo->GetStrings();

    if (newList != oldList)
    {
        int selection = currCombo->GetSelection();
        currCombo->Set(newList);
        currCombo->Select(selection);
    }
}

void lms8001_pnlPLLProfiles_view::OnupdateUI_sttxtPLLFreq(wxUpdateUIEvent& event)
{
    int fracmod, fracmodl, fracmodh, intmod, pllPDiv2, ffMod;
    wxString SRefClock = txtRefClock->GetValue();
    double refClock_MHz, refClock;
    txtRefClock->GetValue().ToDouble(&refClock_MHz);
    refClock = refClock_MHz * 1E6;

    fracmodl = cmbFRACMODL_n->GetValue();
    fracmodh = cmbFRACMODH_n->GetValue();
    fracmod = fracmodl + fracmodh * pow(2, 16);
    intmod = cmbINTMOD_n->GetValue();
    pllPDiv2 = chkPLL_EN_FB_PDIV2_n->GetValue();
    ffMod = cmbFF_MOD_n->GetSelection();

    double pllFreq = pow(2, pllPDiv2) * (intmod + fracmod / (pow(2, 20))) * refClock * pow(2, -1 * ffMod);
    double pllFreq_GHz = pllFreq / 1E9;

    wxString newString = wxString::Format(_("%.4f"), pllFreq_GHz);
    wxString oldString = sttxtPLLFreq->GetLabelText();
    if (oldString != newString)
        sttxtPLLFreq->SetLabelText(newString);

    double chLOFreq_GHz;

    chLOFreq_GHz = (chkPLL_LODIST_EN_OUT0_n->GetValue()) ? pllFreq_GHz : 0.0;
    if (!chkPLL_LODIST_FSP_OUT02_n->GetValue())
        chLOFreq_GHz /= 2.0;
    newString = wxString::Format(_("%.4f"), chLOFreq_GHz);
    oldString = sttxtLODistrChAFreq->GetLabelText();
    if (oldString != newString)
        sttxtLODistrChAFreq->SetLabelText(newString);

    chLOFreq_GHz = (chkPLL_LODIST_EN_OUT1_n->GetValue()) ? pllFreq_GHz : 0.0;
    if (!chkPLL_LODIST_FSP_OUT12_n->GetValue())
        chLOFreq_GHz /= 2.0;
    newString = wxString::Format(_("%.4f"), chLOFreq_GHz);
    oldString = sttxtLODistrChBFreq->GetLabelText();
    if (oldString != newString)
        sttxtLODistrChBFreq->SetLabelText(newString);

    chLOFreq_GHz = (chkPLL_LODIST_EN_OUT2_n->GetValue()) ? pllFreq_GHz : 0.0;
    if (!chkPLL_LODIST_FSP_OUT22_n->GetValue())
        chLOFreq_GHz /= 2.0;
    newString = wxString::Format(_("%.4f"), chLOFreq_GHz);
    oldString = sttxtLODistrChCFreq->GetLabelText();
    if (oldString != newString)
        sttxtLODistrChCFreq->SetLabelText(newString);

    chLOFreq_GHz = (chkPLL_LODIST_EN_OUT3_n->GetValue()) ? pllFreq_GHz : 0.0;
    if (!chkPLL_LODIST_FSP_OUT32_n->GetValue())
        chLOFreq_GHz /= 2.0;
    newString = wxString::Format(_("%.4f"), chLOFreq_GHz);
    oldString = sttxtLODistrChDFreq->GetLabelText();
    if (oldString != newString)
        sttxtLODistrChDFreq->SetLabelText(newString);
}

void lms8001_pnlPLLProfiles_view::OnUpdateUI_sttxtFLockN(wxUpdateUIEvent& event)
{
    double fref_MHz = chip->mRefClk_MHz;
    int flock_N = cmbFLOCK_N_n->GetValue();
    double flock_us = flock_N * 1.0 / fref_MHz;

    wxString newString = wxString::Format(_("%.2f"), flock_us);
    wxString oldString = sttxtFLockN->GetLabelText();
    if (oldString != newString)
        sttxtFLockN->SetLabelText(newString);
}
void lms8001_pnlPLLProfiles_view::OnUpdateUI_cmbFLOCK_PULSE_n(wxUpdateUIEvent& event)
{
    int ict_cp = cmbICT_CP_n->GetValue();
    double newFirstValue = 1 * 25.0 * ict_cp / 16.0;

    wxString oldFirst = "";
    if ((cmbFLOCK_PULSE_n->GetStrings()).GetCount() > 1)
        oldFirst = (cmbFLOCK_PULSE_n->GetStrings()).Item(1);

    wxString newFirst = wxString::Format(_("%.2f"), newFirstValue);

    if (oldFirst != newFirst)
    {
        wxArrayString temp1, temp2;

        for (int i = 0; i < 64; ++i)
        {
            temp1.push_back(wxString::Format(_("%.2f"), i * 25.0 * ict_cp / 16.0));
            temp2.push_back(wxString::Format(_("%.2f"), i * 6.25 * ict_cp / 16.0));
        }
        int currSel;

        currSel = cmbFLOCK_PULSE_n->GetSelection();
        cmbFLOCK_PULSE_n->Set(temp1);
        cmbFLOCK_PULSE_n->SetSelection(currSel);

        currSel = cmbPULSE_n->GetSelection();
        cmbPULSE_n->Set(temp1);
        cmbPULSE_n->SetSelection(currSel);

        currSel = cmbFLOCK_OFS_n->GetSelection();
        cmbFLOCK_OFS_n->Set(temp2);
        cmbFLOCK_OFS_n->SetSelection(currSel);

        currSel = cmbOFS_n->GetSelection();
        cmbOFS_n->Set(temp2);
        cmbOFS_n->SetSelection(currSel);
    }
}

void lms8001_pnlPLLProfiles_view::OnTextRefClock(wxCommandEvent& event)
{
    txtRefClock->GetValue().ToDouble(&chip->mRefClk_MHz);
}