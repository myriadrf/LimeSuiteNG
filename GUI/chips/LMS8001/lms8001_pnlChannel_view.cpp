#include "lms8001_pnlChannel_view.h"
#include <map>
#include "lms8001_gui_utilities.h"

#include <wx/msgdlg.h>

#include <wx/dc.h>

#include "chips/LMS8001/LMS8001.h"

using namespace lime;

lms8001_pnlChannel_view::lms8001_pnlChannel_view(
    wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ILMS8001Tab(parent, id, pos, size, style)
{

    wxFlexGridSizer* fgSizer68;
    fgSizer68 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer68->SetFlexibleDirection(wxBOTH);
    fgSizer68->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_NOTEBOOK_CHANNEL = new wxNotebook(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_CHANNEL = new wxPanel(ID_NOTEBOOK_CHANNEL, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer69;
    fgSizer69 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer69->SetFlexibleDirection(wxBOTH);
    fgSizer69->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer125;
    fgSizer125 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer125->SetFlexibleDirection(wxBOTH);
    fgSizer125->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer29;
    fgSizer29 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer29->SetFlexibleDirection(wxBOTH);
    fgSizer29->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer35;
    sbSizer35 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CHANNEL, wxID_ANY, wxT("HFPAD Reference Current")), wxVERTICAL);

    wxFlexGridSizer* fgSizer83;
    fgSizer83 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer83->SetFlexibleDirection(wxBOTH);
    fgSizer83->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_PA_ILIN2X =
        new wxCheckBox(ID_PANEL_CHANNEL, ID_CHx_PA_ILIN2X, wxT("Double LIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer83->Add(chkCHx_PA_ILIN2X, 0, wxALL, 4);

    sbSizer35->Add(fgSizer83, 1, wxEXPAND, 0);

    wxFlexGridSizer* fgSizer84;
    fgSizer84 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer84->SetFlexibleDirection(wxBOTH);
    fgSizer84->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText4311 = new wxStaticText(ID_PANEL_CHANNEL, wxID_ANY, wxT("LIN"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311->Wrap(-1);
    fgSizer84->Add(m_staticText4311, 0, wxALL, 4);

    cmbCHx_PA_ICT_LIN = new wxSpinCtrl(
        ID_PANEL_CHANNEL, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer84->Add(cmbCHx_PA_ICT_LIN, 0, wxALL, 0);

    m_staticText43111 = new wxStaticText(ID_PANEL_CHANNEL, wxID_ANY, wxT("MAIN"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111->Wrap(-1);
    fgSizer84->Add(m_staticText43111, 0, wxALL, 4);

    cmbCHx_PA_ICT_MAIN = new wxSpinCtrl(
        ID_PANEL_CHANNEL, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer84->Add(cmbCHx_PA_ICT_MAIN, 0, wxALL, 0);

    sbSizer35->Add(fgSizer84, 0, wxEXPAND, 5);

    fgSizer29->Add(sbSizer35, 0, wxEXPAND, 5);

    fgSizer125->Add(fgSizer29, 1, wxEXPAND, 5);

    ID_NOTEBOOK_CH_PROGRAM = new wxNotebook(ID_PANEL_CHANNEL, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_CH_P0 = new wxPanel(ID_NOTEBOOK_CH_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer88;
    fgSizer88 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer88->SetFlexibleDirection(wxBOTH);
    fgSizer88->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer115;
    sbSizer115 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P0, wxID_ANY, wxT("Power-down")), wxVERTICAL);

    wxFlexGridSizer* fgSizer85;
    fgSizer85 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer85->SetFlexibleDirection(wxBOTH);
    fgSizer85->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    pnlChannel = new wxPanel(ID_PANEL_CH_P0, wxID_ANY, wxDefaultPosition, wxSize(-1, -1), wxRAISED_BORDER | wxTAB_TRAVERSAL);
    wxBoxSizer* bsChannel;
    bsChannel = new wxBoxSizer(wxVERTICAL);

    bsChannel->SetMinSize(wxSize(400, 187));
    wxFlexGridSizer* fgSizer156;
    fgSizer156 = new wxFlexGridSizer(0, 6, 0, 0);
    fgSizer156->SetFlexibleDirection(wxBOTH);
    fgSizer156->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer156->Add(30, 0, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer155;
    fgSizer155 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer155->SetFlexibleDirection(wxBOTH);
    fgSizer155->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer155->SetMinSize(wxSize(-1, 50));

    fgSizer155->Add(0, 120, 0, 0, 5);

    chkCHx_LNA_PD0 = new wxCheckBox(pnlChannel, ID_CHx_PA_ILIN2X, wxT("LNA_PD"), wxPoint(50, -1), wxDefaultSize, 0);
    fgSizer155->Add(chkCHx_LNA_PD0, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156->Add(fgSizer155, 0, 0, 5);

    wxFlexGridSizer* fgSizer1551;
    fgSizer1551 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1551->SetFlexibleDirection(wxBOTH);
    fgSizer1551->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1551->SetMinSize(wxSize(-1, 50));

    fgSizer1551->Add(0, 10, 0, 0, 5);

    chkCHx_MIXB_LOBUFF_PD0 = new wxCheckBox(pnlChannel, ID_CHx_PA_ILIN2X, wxT("MIXB_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1551->Add(chkCHx_MIXB_LOBUFF_PD0, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156->Add(fgSizer1551, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer15511;
    fgSizer15511 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer15511->SetFlexibleDirection(wxBOTH);
    fgSizer15511->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer15511->SetMinSize(wxSize(-1, 50));

    fgSizer15511->Add(0, 120, 0, 0, 5);

    chkCHx_MIXA_LOBUFF_PD0 = new wxCheckBox(pnlChannel, ID_CHx_PA_ILIN2X, wxT("MIXA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15511->Add(chkCHx_MIXA_LOBUFF_PD0, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156->Add(fgSizer15511, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer155111;
    fgSizer155111 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer155111->SetFlexibleDirection(wxBOTH);
    fgSizer155111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer155111->SetMinSize(wxSize(-1, 50));

    fgSizer155111->Add(60, 165, 0, 0, 5);

    chkCHx_PA_R50_EN0 = new wxCheckBox(pnlChannel, ID_CHx_PA_ILIN2X, wxT("PA_R50"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer155111->Add(chkCHx_PA_R50_EN0, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156->Add(fgSizer155111, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer1551111;
    fgSizer1551111 = new wxFlexGridSizer(4, 0, 0, 0);
    fgSizer1551111->SetFlexibleDirection(wxBOTH);
    fgSizer1551111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1551111->SetMinSize(wxSize(-1, 50));

    fgSizer1551111->Add(60, 10, 0, 0, 5);

    chkCHx_PA_BYPASS0 = new wxCheckBox(pnlChannel, ID_CHx_PA_ILIN2X, wxT("PA_BYPASS"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1551111->Add(chkCHx_PA_BYPASS0, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1551111->Add(60, 95, 1, wxEXPAND, 5);

    chkCHx_PA_PD0 = new wxCheckBox(pnlChannel, ID_CHx_PA_ILIN2X, wxT("PA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1551111->Add(chkCHx_PA_PD0, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156->Add(fgSizer1551111, 1, wxEXPAND, 5);

    bsChannel->Add(fgSizer156, 1, wxEXPAND, 5);

    pnlChannel->SetSizer(bsChannel);
    pnlChannel->Layout();
    bsChannel->Fit(pnlChannel);
    fgSizer85->Add(pnlChannel, 1, wxEXPAND | wxALL, 5);

    sbSizer115->Add(fgSizer85, 1, wxEXPAND, 5);

    fgSizer88->Add(sbSizer115, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer118;
    sbSizer118 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P0, wxID_ANY, wxT("LNA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer851;
    fgSizer851 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer851->SetFlexibleDirection(wxBOTH);
    fgSizer851->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText431111 = new wxStaticText(ID_PANEL_CH_P0, wxID_ANY, wxT("LIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111->Wrap(-1);
    fgSizer851->Add(m_staticText431111, 0, wxALL, 0);

    cmbCHx_LNA_ICT_LIN0 = new wxSpinCtrl(
        ID_PANEL_CH_P0, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer851->Add(cmbCHx_LNA_ICT_LIN0, 0, wxALL, 0);

    m_staticText4311111 = new wxStaticText(ID_PANEL_CH_P0, wxID_ANY, wxT("MAIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311111->Wrap(-1);
    fgSizer851->Add(m_staticText4311111, 0, wxALL, 0);

    cmbCHx_LNA_ICT_MAIN0 = new wxSpinCtrl(
        ID_PANEL_CH_P0, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer851->Add(cmbCHx_LNA_ICT_MAIN0, 0, wxALL, 0);

    m_staticText43111111 = new wxStaticText(ID_PANEL_CH_P0, wxID_ANY, wxT("Cgs additional"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111111->Wrap(-1);
    fgSizer851->Add(m_staticText43111111, 0, wxALL, 0);

    cmbCHx_LNA_CGSCTRL0 =
        new wxSpinCtrl(ID_PANEL_CH_P0, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 4, 2);
    fgSizer851->Add(cmbCHx_LNA_CGSCTRL0, 0, wxALL, 0);

    m_staticText431111111 = new wxStaticText(ID_PANEL_CH_P0, wxID_ANY, wxT("Gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111111->Wrap(-1);
    fgSizer851->Add(m_staticText431111111, 0, wxALL, 0);

    cmbCHx_LNA_GCTRL0 = new wxSpinCtrl(
        ID_PANEL_CH_P0, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 8);
    fgSizer851->Add(cmbCHx_LNA_GCTRL0, 0, wxALL, 0);

    sbSizer118->Add(fgSizer851, 1, wxEXPAND, 5);

    fgSizer88->Add(sbSizer118, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer119;
    sbSizer119 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P0, wxID_ANY, wxT("PA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8511;
    fgSizer8511 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer8511->SetFlexibleDirection(wxBOTH);
    fgSizer8511->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText4311111111 =
        new wxStaticText(ID_PANEL_CH_P0, wxID_ANY, wxT("LIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311111111->Wrap(-1);
    fgSizer8511->Add(m_staticText4311111111, 0, wxALL, 0);

    cmbCHx_PA_LIN_LOSS0 = new wxSpinCtrl(
        ID_PANEL_CH_P0, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 0);
    fgSizer8511->Add(cmbCHx_PA_LIN_LOSS0, 0, wxALL, 0);

    m_staticText43111111111 =
        new wxStaticText(ID_PANEL_CH_P0, wxID_ANY, wxT("MAIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111111111->Wrap(-1);
    fgSizer8511->Add(m_staticText43111111111, 0, wxALL, 0);

    cmbCHx_PA_MAIN_LOSS0 = new wxSpinCtrl(
        ID_PANEL_CH_P0, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 0);
    fgSizer8511->Add(cmbCHx_PA_MAIN_LOSS0, 0, wxALL, 0);

    sbSizer119->Add(fgSizer8511, 1, wxEXPAND, 5);

    fgSizer88->Add(sbSizer119, 1, wxEXPAND, 5);

    ID_PANEL_CH_P0->SetSizer(fgSizer88);
    ID_PANEL_CH_P0->Layout();
    fgSizer88->Fit(ID_PANEL_CH_P0);
    ID_NOTEBOOK_CH_PROGRAM->AddPage(ID_PANEL_CH_P0, wxT("Program 0"), true);
    ID_PANEL_CH_P1 = new wxPanel(ID_NOTEBOOK_CH_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer881;
    fgSizer881 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer881->SetFlexibleDirection(wxBOTH);
    fgSizer881->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer1151;
    sbSizer1151 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P1, wxID_ANY, wxT("Power-down")), wxVERTICAL);

    wxFlexGridSizer* fgSizer852;
    fgSizer852 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer852->SetFlexibleDirection(wxBOTH);
    fgSizer852->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    pnlChannel1 = new wxPanel(ID_PANEL_CH_P1, wxID_ANY, wxDefaultPosition, wxSize(-1, -1), wxRAISED_BORDER | wxTAB_TRAVERSAL);
    wxBoxSizer* bsChannel1;
    bsChannel1 = new wxBoxSizer(wxVERTICAL);

    bsChannel1->SetMinSize(wxSize(400, 187));
    wxFlexGridSizer* fgSizer1561;
    fgSizer1561 = new wxFlexGridSizer(0, 6, 0, 0);
    fgSizer1561->SetFlexibleDirection(wxBOTH);
    fgSizer1561->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1561->Add(30, 0, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer1552;
    fgSizer1552 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1552->SetFlexibleDirection(wxBOTH);
    fgSizer1552->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1552->SetMinSize(wxSize(-1, 50));

    fgSizer1552->Add(0, 120, 0, 0, 5);

    chkCHx_LNA_PD1 = new wxCheckBox(pnlChannel1, ID_CHx_PA_ILIN2X, wxT("LNA_PD"), wxPoint(50, -1), wxDefaultSize, 0);
    fgSizer1552->Add(chkCHx_LNA_PD1, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561->Add(fgSizer1552, 0, 0, 5);

    wxFlexGridSizer* fgSizer15512;
    fgSizer15512 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer15512->SetFlexibleDirection(wxBOTH);
    fgSizer15512->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer15512->SetMinSize(wxSize(-1, 50));

    fgSizer15512->Add(0, 10, 0, 0, 5);

    chkCHx_MIXB_LOBUFF_PD1 = new wxCheckBox(pnlChannel1, ID_CHx_PA_ILIN2X, wxT("MIXB_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15512->Add(chkCHx_MIXB_LOBUFF_PD1, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561->Add(fgSizer15512, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer155112;
    fgSizer155112 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer155112->SetFlexibleDirection(wxBOTH);
    fgSizer155112->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer155112->SetMinSize(wxSize(-1, 50));

    fgSizer155112->Add(0, 120, 0, 0, 5);

    chkCHx_MIXA_LOBUFF_PD1 = new wxCheckBox(pnlChannel1, ID_CHx_PA_ILIN2X, wxT("MIXA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer155112->Add(chkCHx_MIXA_LOBUFF_PD1, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561->Add(fgSizer155112, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer1551112;
    fgSizer1551112 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1551112->SetFlexibleDirection(wxBOTH);
    fgSizer1551112->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1551112->SetMinSize(wxSize(-1, 50));

    fgSizer1551112->Add(60, 165, 0, 0, 5);

    chkCHx_PA_R50_EN1 = new wxCheckBox(pnlChannel1, ID_CHx_PA_ILIN2X, wxT("PA_R50"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1551112->Add(chkCHx_PA_R50_EN1, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561->Add(fgSizer1551112, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer15511111;
    fgSizer15511111 = new wxFlexGridSizer(4, 0, 0, 0);
    fgSizer15511111->SetFlexibleDirection(wxBOTH);
    fgSizer15511111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer15511111->SetMinSize(wxSize(-1, 50));

    fgSizer15511111->Add(60, 10, 0, 0, 5);

    chkCHx_PA_BYPASS1 = new wxCheckBox(pnlChannel1, ID_CHx_PA_ILIN2X, wxT("PA_BYPASS"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15511111->Add(chkCHx_PA_BYPASS1, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer15511111->Add(60, 95, 1, wxEXPAND, 5);

    chkCHx_PA_PD1 = new wxCheckBox(pnlChannel1, ID_CHx_PA_ILIN2X, wxT("PA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15511111->Add(chkCHx_PA_PD1, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561->Add(fgSizer15511111, 1, wxEXPAND, 5);

    bsChannel1->Add(fgSizer1561, 1, wxEXPAND, 5);

    pnlChannel1->SetSizer(bsChannel1);
    pnlChannel1->Layout();
    bsChannel1->Fit(pnlChannel1);
    fgSizer852->Add(pnlChannel1, 1, wxEXPAND | wxALL, 5);

    sbSizer1151->Add(fgSizer852, 1, wxEXPAND, 5);

    fgSizer881->Add(sbSizer1151, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer1181;
    sbSizer1181 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P1, wxID_ANY, wxT("LNA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8512;
    fgSizer8512 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer8512->SetFlexibleDirection(wxBOTH);
    fgSizer8512->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText4311112 = new wxStaticText(ID_PANEL_CH_P1, wxID_ANY, wxT("LIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311112->Wrap(-1);
    fgSizer8512->Add(m_staticText4311112, 0, wxALL, 0);

    cmbCHx_LNA_ICT_LIN1 = new wxSpinCtrl(
        ID_PANEL_CH_P1, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer8512->Add(cmbCHx_LNA_ICT_LIN1, 0, wxALL, 0);

    m_staticText43111112 =
        new wxStaticText(ID_PANEL_CH_P1, wxID_ANY, wxT("MAIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111112->Wrap(-1);
    fgSizer8512->Add(m_staticText43111112, 0, wxALL, 0);

    cmbCHx_LNA_ICT_MAIN1 = new wxSpinCtrl(
        ID_PANEL_CH_P1, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer8512->Add(cmbCHx_LNA_ICT_MAIN1, 0, wxALL, 0);

    m_staticText431111112 = new wxStaticText(ID_PANEL_CH_P1, wxID_ANY, wxT("Cgs additional"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111112->Wrap(-1);
    fgSizer8512->Add(m_staticText431111112, 0, wxALL, 0);

    cmbCHx_LNA_CGSCTRL1 =
        new wxSpinCtrl(ID_PANEL_CH_P1, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 4, 2);
    fgSizer8512->Add(cmbCHx_LNA_CGSCTRL1, 0, wxALL, 0);

    m_staticText4311111112 = new wxStaticText(ID_PANEL_CH_P1, wxID_ANY, wxT("Gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311111112->Wrap(-1);
    fgSizer8512->Add(m_staticText4311111112, 0, wxALL, 0);

    cmbCHx_LNA_GCTRL1 = new wxSpinCtrl(
        ID_PANEL_CH_P1, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 8);
    fgSizer8512->Add(cmbCHx_LNA_GCTRL1, 0, wxALL, 0);

    sbSizer1181->Add(fgSizer8512, 1, wxEXPAND, 5);

    fgSizer881->Add(sbSizer1181, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer1191;
    sbSizer1191 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P1, wxID_ANY, wxT("PA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer85111;
    fgSizer85111 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer85111->SetFlexibleDirection(wxBOTH);
    fgSizer85111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText43111111112 =
        new wxStaticText(ID_PANEL_CH_P1, wxID_ANY, wxT("LIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111111112->Wrap(-1);
    fgSizer85111->Add(m_staticText43111111112, 0, wxALL, 0);

    cmbCHx_PA_LIN_LOSS1 = new wxSpinCtrl(
        ID_PANEL_CH_P1, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 0);
    fgSizer85111->Add(cmbCHx_PA_LIN_LOSS1, 0, wxALL, 0);

    m_staticText431111111111 =
        new wxStaticText(ID_PANEL_CH_P1, wxID_ANY, wxT("MAIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111111111->Wrap(-1);
    fgSizer85111->Add(m_staticText431111111111, 0, wxALL, 0);

    cmbCHx_PA_MAIN_LOSS1 = new wxSpinCtrl(
        ID_PANEL_CH_P1, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 0);
    fgSizer85111->Add(cmbCHx_PA_MAIN_LOSS1, 0, wxALL, 0);

    sbSizer1191->Add(fgSizer85111, 1, wxEXPAND, 5);

    fgSizer881->Add(sbSizer1191, 1, wxEXPAND, 5);

    ID_PANEL_CH_P1->SetSizer(fgSizer881);
    ID_PANEL_CH_P1->Layout();
    fgSizer881->Fit(ID_PANEL_CH_P1);
    ID_NOTEBOOK_CH_PROGRAM->AddPage(ID_PANEL_CH_P1, wxT("Program 1"), false);
    ID_PANEL_CH_P2 = new wxPanel(ID_NOTEBOOK_CH_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer8811;
    fgSizer8811 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer8811->SetFlexibleDirection(wxBOTH);
    fgSizer8811->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer11511;
    sbSizer11511 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P2, wxID_ANY, wxT("Power-down")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8521;
    fgSizer8521 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer8521->SetFlexibleDirection(wxBOTH);
    fgSizer8521->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    pnlChannel11 = new wxPanel(ID_PANEL_CH_P2, wxID_ANY, wxDefaultPosition, wxSize(-1, -1), wxRAISED_BORDER | wxTAB_TRAVERSAL);
    wxBoxSizer* bsChannel11;
    bsChannel11 = new wxBoxSizer(wxVERTICAL);

    bsChannel11->SetMinSize(wxSize(400, 187));
    wxFlexGridSizer* fgSizer15611;
    fgSizer15611 = new wxFlexGridSizer(0, 6, 0, 0);
    fgSizer15611->SetFlexibleDirection(wxBOTH);
    fgSizer15611->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer15611->Add(30, 0, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer15521;
    fgSizer15521 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer15521->SetFlexibleDirection(wxBOTH);
    fgSizer15521->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer15521->SetMinSize(wxSize(-1, 50));

    fgSizer15521->Add(0, 120, 0, 0, 5);

    chkCHx_LNA_PD2 = new wxCheckBox(pnlChannel11, ID_CHx_PA_ILIN2X, wxT("LNA_PD"), wxPoint(50, -1), wxDefaultSize, 0);
    fgSizer15521->Add(chkCHx_LNA_PD2, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer15611->Add(fgSizer15521, 0, 0, 5);

    wxFlexGridSizer* fgSizer155121;
    fgSizer155121 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer155121->SetFlexibleDirection(wxBOTH);
    fgSizer155121->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer155121->SetMinSize(wxSize(-1, 50));

    fgSizer155121->Add(0, 10, 0, 0, 5);

    chkCHx_MIXB_LOBUFF_PD2 = new wxCheckBox(pnlChannel11, ID_CHx_PA_ILIN2X, wxT("MIXB_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer155121->Add(chkCHx_MIXB_LOBUFF_PD2, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer15611->Add(fgSizer155121, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer1551121;
    fgSizer1551121 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1551121->SetFlexibleDirection(wxBOTH);
    fgSizer1551121->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1551121->SetMinSize(wxSize(-1, 50));

    fgSizer1551121->Add(0, 120, 0, 0, 5);

    chkCHx_MIXA_LOBUFF_PD2 = new wxCheckBox(pnlChannel11, ID_CHx_PA_ILIN2X, wxT("MIXA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1551121->Add(chkCHx_MIXA_LOBUFF_PD2, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer15611->Add(fgSizer1551121, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer15511121;
    fgSizer15511121 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer15511121->SetFlexibleDirection(wxBOTH);
    fgSizer15511121->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer15511121->SetMinSize(wxSize(-1, 50));

    fgSizer15511121->Add(60, 165, 0, 0, 5);

    chkCHx_PA_R50_EN2 = new wxCheckBox(pnlChannel11, ID_CHx_PA_ILIN2X, wxT("PA_R50"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15511121->Add(chkCHx_PA_R50_EN2, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer15611->Add(fgSizer15511121, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer155111111;
    fgSizer155111111 = new wxFlexGridSizer(4, 0, 0, 0);
    fgSizer155111111->SetFlexibleDirection(wxBOTH);
    fgSizer155111111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer155111111->SetMinSize(wxSize(-1, 50));

    fgSizer155111111->Add(60, 10, 0, 0, 5);

    chkCHx_PA_BYPASS2 = new wxCheckBox(pnlChannel11, ID_CHx_PA_ILIN2X, wxT("PA_BYPASS"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer155111111->Add(chkCHx_PA_BYPASS2, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer155111111->Add(60, 95, 1, wxEXPAND, 5);

    chkCHx_PA_PD2 = new wxCheckBox(pnlChannel11, ID_CHx_PA_ILIN2X, wxT("PA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer155111111->Add(chkCHx_PA_PD2, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer15611->Add(fgSizer155111111, 1, wxEXPAND, 5);

    bsChannel11->Add(fgSizer15611, 1, wxEXPAND, 5);

    pnlChannel11->SetSizer(bsChannel11);
    pnlChannel11->Layout();
    bsChannel11->Fit(pnlChannel11);
    fgSizer8521->Add(pnlChannel11, 1, wxEXPAND | wxALL, 5);

    sbSizer11511->Add(fgSizer8521, 1, wxEXPAND, 5);

    fgSizer8811->Add(sbSizer11511, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer11811;
    sbSizer11811 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P2, wxID_ANY, wxT("LNA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer85121;
    fgSizer85121 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer85121->SetFlexibleDirection(wxBOTH);
    fgSizer85121->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText43111121 = new wxStaticText(ID_PANEL_CH_P2, wxID_ANY, wxT("LIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111121->Wrap(-1);
    fgSizer85121->Add(m_staticText43111121, 0, wxALL, 0);

    cmbCHx_LNA_ICT_LIN2 = new wxSpinCtrl(
        ID_PANEL_CH_P2, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer85121->Add(cmbCHx_LNA_ICT_LIN2, 0, wxALL, 0);

    m_staticText431111121 =
        new wxStaticText(ID_PANEL_CH_P2, wxID_ANY, wxT("MAIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111121->Wrap(-1);
    fgSizer85121->Add(m_staticText431111121, 0, wxALL, 0);

    cmbCHx_LNA_ICT_MAIN2 = new wxSpinCtrl(
        ID_PANEL_CH_P2, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer85121->Add(cmbCHx_LNA_ICT_MAIN2, 0, wxALL, 0);

    m_staticText4311111121 = new wxStaticText(ID_PANEL_CH_P2, wxID_ANY, wxT("Cgs additional"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311111121->Wrap(-1);
    fgSizer85121->Add(m_staticText4311111121, 0, wxALL, 0);

    cmbCHx_LNA_CGSCTRL2 =
        new wxSpinCtrl(ID_PANEL_CH_P2, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 4, 2);
    fgSizer85121->Add(cmbCHx_LNA_CGSCTRL2, 0, wxALL, 0);

    m_staticText43111111121 = new wxStaticText(ID_PANEL_CH_P2, wxID_ANY, wxT("Gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111111121->Wrap(-1);
    fgSizer85121->Add(m_staticText43111111121, 0, wxALL, 0);

    cmbCHx_LNA_GCTRL2 = new wxSpinCtrl(
        ID_PANEL_CH_P2, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 8);
    fgSizer85121->Add(cmbCHx_LNA_GCTRL2, 0, wxALL, 0);

    sbSizer11811->Add(fgSizer85121, 1, wxEXPAND, 5);

    fgSizer8811->Add(sbSizer11811, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer11911;
    sbSizer11911 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P2, wxID_ANY, wxT("PA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer851111;
    fgSizer851111 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer851111->SetFlexibleDirection(wxBOTH);
    fgSizer851111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText431111111121 =
        new wxStaticText(ID_PANEL_CH_P2, wxID_ANY, wxT("LIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111111121->Wrap(-1);
    fgSizer851111->Add(m_staticText431111111121, 0, wxALL, 0);

    cmbCHx_PA_LIN_LOSS2 = new wxSpinCtrl(
        ID_PANEL_CH_P2, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 0);
    fgSizer851111->Add(cmbCHx_PA_LIN_LOSS2, 0, wxALL, 0);

    m_staticText4311111111111 =
        new wxStaticText(ID_PANEL_CH_P2, wxID_ANY, wxT("MAIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311111111111->Wrap(-1);
    fgSizer851111->Add(m_staticText4311111111111, 0, wxALL, 0);

    cmbCHx_PA_MAIN_LOSS2 = new wxSpinCtrl(
        ID_PANEL_CH_P2, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 0);
    fgSizer851111->Add(cmbCHx_PA_MAIN_LOSS2, 0, wxALL, 0);

    sbSizer11911->Add(fgSizer851111, 1, wxEXPAND, 5);

    fgSizer8811->Add(sbSizer11911, 1, wxEXPAND, 5);

    ID_PANEL_CH_P2->SetSizer(fgSizer8811);
    ID_PANEL_CH_P2->Layout();
    fgSizer8811->Fit(ID_PANEL_CH_P2);
    ID_NOTEBOOK_CH_PROGRAM->AddPage(ID_PANEL_CH_P2, wxT("Program 2"), false);
    ID_PANEL_CH_P3 = new wxPanel(ID_NOTEBOOK_CH_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer88111;
    fgSizer88111 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer88111->SetFlexibleDirection(wxBOTH);
    fgSizer88111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer115111;
    sbSizer115111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P3, wxID_ANY, wxT("Power-down")), wxVERTICAL);

    wxFlexGridSizer* fgSizer85211;
    fgSizer85211 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer85211->SetFlexibleDirection(wxBOTH);
    fgSizer85211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    pnlChannel111 = new wxPanel(ID_PANEL_CH_P3, wxID_ANY, wxDefaultPosition, wxSize(-1, -1), wxRAISED_BORDER | wxTAB_TRAVERSAL);
    wxBoxSizer* bsChannel111;
    bsChannel111 = new wxBoxSizer(wxVERTICAL);

    bsChannel111->SetMinSize(wxSize(400, 187));
    wxFlexGridSizer* fgSizer156111;
    fgSizer156111 = new wxFlexGridSizer(0, 6, 0, 0);
    fgSizer156111->SetFlexibleDirection(wxBOTH);
    fgSizer156111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer156111->Add(30, 0, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer155211;
    fgSizer155211 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer155211->SetFlexibleDirection(wxBOTH);
    fgSizer155211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer155211->SetMinSize(wxSize(-1, 50));

    fgSizer155211->Add(0, 120, 0, 0, 5);

    chkCHx_LNA_PD3 = new wxCheckBox(pnlChannel111, ID_CHx_PA_ILIN2X, wxT("LNA_PD"), wxPoint(50, -1), wxDefaultSize, 0);
    fgSizer155211->Add(chkCHx_LNA_PD3, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156111->Add(fgSizer155211, 0, 0, 5);

    wxFlexGridSizer* fgSizer1551211;
    fgSizer1551211 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1551211->SetFlexibleDirection(wxBOTH);
    fgSizer1551211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1551211->SetMinSize(wxSize(-1, 50));

    fgSizer1551211->Add(0, 10, 0, 0, 5);

    chkCHx_MIXB_LOBUFF_PD3 = new wxCheckBox(pnlChannel111, ID_CHx_PA_ILIN2X, wxT("MIXB_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1551211->Add(chkCHx_MIXB_LOBUFF_PD3, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156111->Add(fgSizer1551211, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer15511211;
    fgSizer15511211 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer15511211->SetFlexibleDirection(wxBOTH);
    fgSizer15511211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer15511211->SetMinSize(wxSize(-1, 50));

    fgSizer15511211->Add(0, 120, 0, 0, 5);

    chkCHx_MIXA_LOBUFF_PD3 = new wxCheckBox(pnlChannel111, ID_CHx_PA_ILIN2X, wxT("MIXA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15511211->Add(chkCHx_MIXA_LOBUFF_PD3, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156111->Add(fgSizer15511211, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer155111211;
    fgSizer155111211 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer155111211->SetFlexibleDirection(wxBOTH);
    fgSizer155111211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer155111211->SetMinSize(wxSize(-1, 50));

    fgSizer155111211->Add(60, 165, 0, 0, 5);

    chkCHx_PA_R50_EN3 = new wxCheckBox(pnlChannel111, ID_CHx_PA_ILIN2X, wxT("PA_R50"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer155111211->Add(chkCHx_PA_R50_EN3, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156111->Add(fgSizer155111211, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer1551111111;
    fgSizer1551111111 = new wxFlexGridSizer(4, 0, 0, 0);
    fgSizer1551111111->SetFlexibleDirection(wxBOTH);
    fgSizer1551111111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1551111111->SetMinSize(wxSize(-1, 50));

    fgSizer1551111111->Add(60, 10, 0, 0, 5);

    chkCHx_PA_BYPASS3 = new wxCheckBox(pnlChannel111, ID_CHx_PA_ILIN2X, wxT("PA_BYPASS"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1551111111->Add(chkCHx_PA_BYPASS3, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1551111111->Add(60, 95, 1, wxEXPAND, 5);

    chkCHx_PA_PD3 = new wxCheckBox(pnlChannel111, ID_CHx_PA_ILIN2X, wxT("PA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1551111111->Add(chkCHx_PA_PD3, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer156111->Add(fgSizer1551111111, 1, wxEXPAND, 5);

    bsChannel111->Add(fgSizer156111, 1, wxEXPAND, 5);

    pnlChannel111->SetSizer(bsChannel111);
    pnlChannel111->Layout();
    bsChannel111->Fit(pnlChannel111);
    fgSizer85211->Add(pnlChannel111, 1, wxEXPAND | wxALL, 5);

    sbSizer115111->Add(fgSizer85211, 1, wxEXPAND, 5);

    fgSizer88111->Add(sbSizer115111, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer118111;
    sbSizer118111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P3, wxID_ANY, wxT("LNA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer851211;
    fgSizer851211 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer851211->SetFlexibleDirection(wxBOTH);
    fgSizer851211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText431111211 =
        new wxStaticText(ID_PANEL_CH_P3, wxID_ANY, wxT("LIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111211->Wrap(-1);
    fgSizer851211->Add(m_staticText431111211, 0, wxALL, 0);

    cmbCHx_LNA_ICT_LIN3 = new wxSpinCtrl(
        ID_PANEL_CH_P3, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer851211->Add(cmbCHx_LNA_ICT_LIN3, 0, wxALL, 0);

    m_staticText4311111211 =
        new wxStaticText(ID_PANEL_CH_P3, wxID_ANY, wxT("MAIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311111211->Wrap(-1);
    fgSizer851211->Add(m_staticText4311111211, 0, wxALL, 0);

    cmbCHx_LNA_ICT_MAIN3 = new wxSpinCtrl(
        ID_PANEL_CH_P3, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    fgSizer851211->Add(cmbCHx_LNA_ICT_MAIN3, 0, wxALL, 0);

    m_staticText43111111211 =
        new wxStaticText(ID_PANEL_CH_P3, wxID_ANY, wxT("Cgs additional"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111111211->Wrap(-1);
    fgSizer851211->Add(m_staticText43111111211, 0, wxALL, 0);

    cmbCHx_LNA_CGSCTRL3 =
        new wxSpinCtrl(ID_PANEL_CH_P3, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 4, 2);
    fgSizer851211->Add(cmbCHx_LNA_CGSCTRL3, 0, wxALL, 0);

    m_staticText431111111211 = new wxStaticText(ID_PANEL_CH_P3, wxID_ANY, wxT("Gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111111211->Wrap(-1);
    fgSizer851211->Add(m_staticText431111111211, 0, wxALL, 0);

    cmbCHx_LNA_GCTRL3 = new wxSpinCtrl(
        ID_PANEL_CH_P3, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 8);
    fgSizer851211->Add(cmbCHx_LNA_GCTRL3, 0, wxALL, 0);

    sbSizer118111->Add(fgSizer851211, 1, wxEXPAND, 5);

    fgSizer88111->Add(sbSizer118111, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer119111;
    sbSizer119111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_P3, wxID_ANY, wxT("PA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8511111;
    fgSizer8511111 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer8511111->SetFlexibleDirection(wxBOTH);
    fgSizer8511111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText4311111111211 =
        new wxStaticText(ID_PANEL_CH_P3, wxID_ANY, wxT("LIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311111111211->Wrap(-1);
    fgSizer8511111->Add(m_staticText4311111111211, 0, wxALL, 0);

    cmbCHx_PA_LIN_LOSS3 = new wxSpinCtrl(
        ID_PANEL_CH_P3, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 0);
    fgSizer8511111->Add(cmbCHx_PA_LIN_LOSS3, 0, wxALL, 0);

    m_staticText43111111111111 =
        new wxStaticText(ID_PANEL_CH_P3, wxID_ANY, wxT("MAIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111111111111->Wrap(-1);
    fgSizer8511111->Add(m_staticText43111111111111, 0, wxALL, 0);

    cmbCHx_PA_MAIN_LOSS3 = new wxSpinCtrl(
        ID_PANEL_CH_P3, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 0);
    fgSizer8511111->Add(cmbCHx_PA_MAIN_LOSS3, 0, wxALL, 0);

    sbSizer119111->Add(fgSizer8511111, 1, wxEXPAND, 5);

    fgSizer88111->Add(sbSizer119111, 1, wxEXPAND, 5);

    ID_PANEL_CH_P3->SetSizer(fgSizer88111);
    ID_PANEL_CH_P3->Layout();
    fgSizer88111->Fit(ID_PANEL_CH_P3);
    ID_NOTEBOOK_CH_PROGRAM->AddPage(ID_PANEL_CH_P3, wxT("Program 3"), false);
    ID_PANEL_CH_RB = new wxPanel(ID_NOTEBOOK_CH_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer881111;
    fgSizer881111 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer881111->SetFlexibleDirection(wxBOTH);
    fgSizer881111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer1151111;
    sbSizer1151111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_RB, wxID_ANY, wxT("Power-down")), wxVERTICAL);

    wxFlexGridSizer* fgSizer852111;
    fgSizer852111 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer852111->SetFlexibleDirection(wxBOTH);
    fgSizer852111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    pnlChannel1111 = new wxPanel(ID_PANEL_CH_RB, wxID_ANY, wxDefaultPosition, wxSize(-1, -1), wxRAISED_BORDER | wxTAB_TRAVERSAL);
    wxBoxSizer* bsChannel1111;
    bsChannel1111 = new wxBoxSizer(wxVERTICAL);

    bsChannel1111->SetMinSize(wxSize(400, 187));
    wxFlexGridSizer* fgSizer1561111;
    fgSizer1561111 = new wxFlexGridSizer(0, 6, 0, 0);
    fgSizer1561111->SetFlexibleDirection(wxBOTH);
    fgSizer1561111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1561111->Add(30, 0, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer1552111;
    fgSizer1552111 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1552111->SetFlexibleDirection(wxBOTH);
    fgSizer1552111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1552111->SetMinSize(wxSize(-1, 50));

    fgSizer1552111->Add(0, 120, 0, 0, 5);

    chkCHx_LNA_PD_RB = new wxCheckBox(pnlChannel1111, ID_CHx_PA_ILIN2X, wxT("LNA_PD"), wxPoint(50, -1), wxDefaultSize, 0);
    chkCHx_LNA_PD_RB->Enable(false);

    fgSizer1552111->Add(chkCHx_LNA_PD_RB, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561111->Add(fgSizer1552111, 0, 0, 5);

    wxFlexGridSizer* fgSizer15512111;
    fgSizer15512111 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer15512111->SetFlexibleDirection(wxBOTH);
    fgSizer15512111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer15512111->SetMinSize(wxSize(-1, 50));

    fgSizer15512111->Add(0, 10, 0, 0, 5);

    chkCHx_MIXB_LOBUFF_PD_RB =
        new wxCheckBox(pnlChannel1111, ID_CHx_PA_ILIN2X, wxT("MIXB_PD"), wxDefaultPosition, wxDefaultSize, 0);
    chkCHx_MIXB_LOBUFF_PD_RB->Enable(false);

    fgSizer15512111->Add(chkCHx_MIXB_LOBUFF_PD_RB, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561111->Add(fgSizer15512111, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer155112111;
    fgSizer155112111 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer155112111->SetFlexibleDirection(wxBOTH);
    fgSizer155112111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer155112111->SetMinSize(wxSize(-1, 50));

    fgSizer155112111->Add(0, 120, 0, 0, 5);

    chkCHx_MIXA_LOBUFF_PD_RB =
        new wxCheckBox(pnlChannel1111, ID_CHx_PA_ILIN2X, wxT("MIXA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    chkCHx_MIXA_LOBUFF_PD_RB->Enable(false);

    fgSizer155112111->Add(chkCHx_MIXA_LOBUFF_PD_RB, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561111->Add(fgSizer155112111, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer1551112111;
    fgSizer1551112111 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1551112111->SetFlexibleDirection(wxBOTH);
    fgSizer1551112111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer1551112111->SetMinSize(wxSize(-1, 50));

    fgSizer1551112111->Add(60, 165, 0, 0, 5);

    chkCHx_PA_R50_EN_RB = new wxCheckBox(pnlChannel1111, ID_CHx_PA_ILIN2X, wxT("PA_R50"), wxDefaultPosition, wxDefaultSize, 0);
    chkCHx_PA_R50_EN_RB->Enable(false);

    fgSizer1551112111->Add(chkCHx_PA_R50_EN_RB, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561111->Add(fgSizer1551112111, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer15511111111;
    fgSizer15511111111 = new wxFlexGridSizer(4, 0, 0, 0);
    fgSizer15511111111->SetFlexibleDirection(wxBOTH);
    fgSizer15511111111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    fgSizer15511111111->SetMinSize(wxSize(-1, 50));

    fgSizer15511111111->Add(60, 10, 0, 0, 5);

    chkCHx_PA_BYPASS_RB = new wxCheckBox(pnlChannel1111, ID_CHx_PA_ILIN2X, wxT("PA_BYPASS"), wxDefaultPosition, wxDefaultSize, 0);
    chkCHx_PA_BYPASS_RB->Enable(false);

    fgSizer15511111111->Add(chkCHx_PA_BYPASS_RB, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer15511111111->Add(60, 95, 1, wxEXPAND, 5);

    chkCHx_PA_PD_RB = new wxCheckBox(pnlChannel1111, ID_CHx_PA_ILIN2X, wxT("PA_PD"), wxDefaultPosition, wxDefaultSize, 0);
    chkCHx_PA_PD_RB->Enable(false);

    fgSizer15511111111->Add(chkCHx_PA_PD_RB, 0, wxALIGN_CENTER | wxALL, 0);

    fgSizer1561111->Add(fgSizer15511111111, 1, wxEXPAND, 5);

    bsChannel1111->Add(fgSizer1561111, 1, wxEXPAND, 5);

    pnlChannel1111->SetSizer(bsChannel1111);
    pnlChannel1111->Layout();
    bsChannel1111->Fit(pnlChannel1111);
    fgSizer852111->Add(pnlChannel1111, 1, wxEXPAND | wxALL, 5);

    sbSizer1151111->Add(fgSizer852111, 1, wxEXPAND, 5);

    fgSizer881111->Add(sbSizer1151111, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer1181111;
    sbSizer1181111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_RB, wxID_ANY, wxT("LNA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8512111;
    fgSizer8512111 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer8512111->SetFlexibleDirection(wxBOTH);
    fgSizer8512111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText4311112111 =
        new wxStaticText(ID_PANEL_CH_RB, wxID_ANY, wxT("LIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311112111->Wrap(-1);
    fgSizer8512111->Add(m_staticText4311112111, 0, wxALL, 0);

    cmbCHx_LNA_ICT_LIN_RB = new wxSpinCtrl(
        ID_PANEL_CH_RB, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    cmbCHx_LNA_ICT_LIN_RB->Enable(false);

    fgSizer8512111->Add(cmbCHx_LNA_ICT_LIN_RB, 0, wxALL, 0);

    m_staticText43111112111 =
        new wxStaticText(ID_PANEL_CH_RB, wxID_ANY, wxT("MAIN ref. current"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111112111->Wrap(-1);
    fgSizer8512111->Add(m_staticText43111112111, 0, wxALL, 0);

    cmbCHx_LNA_ICT_MAIN_RB = new wxSpinCtrl(
        ID_PANEL_CH_RB, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    cmbCHx_LNA_ICT_MAIN_RB->Enable(false);

    fgSizer8512111->Add(cmbCHx_LNA_ICT_MAIN_RB, 0, wxALL, 0);

    m_staticText431111112111 =
        new wxStaticText(ID_PANEL_CH_RB, wxID_ANY, wxT("Cgs additional"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111112111->Wrap(-1);
    fgSizer8512111->Add(m_staticText431111112111, 0, wxALL, 0);

    cmbCHx_LNA_CGSCTRL_RB =
        new wxSpinCtrl(ID_PANEL_CH_RB, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 4, 2);
    cmbCHx_LNA_CGSCTRL_RB->Enable(false);

    fgSizer8512111->Add(cmbCHx_LNA_CGSCTRL_RB, 0, wxALL, 0);

    m_staticText4311111112111 =
        new wxStaticText(ID_PANEL_CH_RB, wxID_ANY, wxT("Gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText4311111112111->Wrap(-1);
    fgSizer8512111->Add(m_staticText4311111112111, 0, wxALL, 0);

    cmbCHx_LNA_GCTRL_RB = new wxSpinCtrl(
        ID_PANEL_CH_RB, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 8);
    cmbCHx_LNA_GCTRL_RB->Enable(false);

    fgSizer8512111->Add(cmbCHx_LNA_GCTRL_RB, 0, wxALL, 0);

    sbSizer1181111->Add(fgSizer8512111, 1, wxEXPAND, 5);

    fgSizer881111->Add(sbSizer1181111, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer1191111;
    sbSizer1191111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CH_RB, wxID_ANY, wxT("PA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer85111111;
    fgSizer85111111 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer85111111->SetFlexibleDirection(wxBOTH);
    fgSizer85111111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText43111111112111 =
        new wxStaticText(ID_PANEL_CH_RB, wxID_ANY, wxT("LIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText43111111112111->Wrap(-1);
    fgSizer85111111->Add(m_staticText43111111112111, 0, wxALL, 0);

    cmbCHx_PA_LIN_LOSS_RB = new wxSpinCtrl(
        ID_PANEL_CH_RB, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 0);
    cmbCHx_PA_LIN_LOSS_RB->Enable(false);

    fgSizer85111111->Add(cmbCHx_PA_LIN_LOSS_RB, 0, wxALL, 0);

    m_staticText431111111111111 =
        new wxStaticText(ID_PANEL_CH_RB, wxID_ANY, wxT("MAIN gain control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText431111111111111->Wrap(-1);
    fgSizer85111111->Add(m_staticText431111111111111, 0, wxALL, 0);

    cmbCHx_PA_MAIN_LOSS_RB = new wxSpinCtrl(
        ID_PANEL_CH_RB, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 0);
    cmbCHx_PA_MAIN_LOSS_RB->Enable(false);

    fgSizer85111111->Add(cmbCHx_PA_MAIN_LOSS_RB, 0, wxALL, 0);

    sbSizer1191111->Add(fgSizer85111111, 1, wxEXPAND, 5);

    fgSizer881111->Add(sbSizer1191111, 1, wxEXPAND, 5);

    ID_PANEL_CH_RB->SetSizer(fgSizer881111);
    ID_PANEL_CH_RB->Layout();
    fgSizer881111->Fit(ID_PANEL_CH_RB);
    ID_NOTEBOOK_CH_PROGRAM->AddPage(ID_PANEL_CH_RB, wxT("Readback"), false);

    fgSizer125->Add(ID_NOTEBOOK_CH_PROGRAM, 1, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxEXPAND, 5);

    fgSizer69->Add(fgSizer125, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer126;
    fgSizer126 = new wxFlexGridSizer(1, 0, 0, 0);
    fgSizer126->SetFlexibleDirection(wxBOTH);
    fgSizer126->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer143;
    sbSizer143 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CHANNEL, wxID_ANY, wxT("MUX Control")), wxVERTICAL);

    wxFlexGridSizer* fgSizer161;
    fgSizer161 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer161->SetFlexibleDirection(wxBOTH);
    fgSizer161->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer171;
    sbSizer171 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CHANNEL, wxID_ANY, wxT("Power-down")), wxVERTICAL);

    wxFlexGridSizer* fgSizer369;
    fgSizer369 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer369->SetFlexibleDirection(wxBOTH);
    fgSizer369->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_NOTEBOOK_MUX_CONTROL_PD = new wxNotebook(ID_PANEL_CHANNEL, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_MUX_PD_SEL0 = new wxPanel(ID_NOTEBOOK_MUX_CONTROL_PD, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer882;
    fgSizer882 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer882->SetFlexibleDirection(wxBOTH);
    fgSizer882->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer160;
    fgSizer160 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer160->SetFlexibleDirection(wxBOTH);
    fgSizer160->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer168;
    sbSizer168 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer159;
    fgSizer159 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer159->SetFlexibleDirection(wxBOTH);
    fgSizer159->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_PD_SEL0_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer159->Add(chkCHx_PD_SEL0_INTERNAL, 0, wxALL, 0);

    chkCHx_PD_SEL0_INVERT =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer159->Add(chkCHx_PD_SEL0_INVERT, 0, wxALL, 0);

    sbSizer168->Add(fgSizer159, 1, wxEXPAND, 5);

    fgSizer160->Add(sbSizer168, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer1152;
    sbSizer1152 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer853;
    fgSizer853 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer853->SetFlexibleDirection(wxBOTH);
    fgSizer853->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText265 = new wxStaticText(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265->Wrap(-1);
    fgSizer853->Add(m_staticText265, 0, wxALL, 5);

    m_staticText2651 = new wxStaticText(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2651->Wrap(-1);
    fgSizer853->Add(m_staticText2651, 0, wxALL, 5);

    m_staticText2652 = new wxStaticText(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2652->Wrap(-1);
    fgSizer853->Add(m_staticText2652, 0, wxALL, 5);

    m_staticText2653 = new wxStaticText(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2653->Wrap(-1);
    fgSizer853->Add(m_staticText2653, 0, wxALL, 5);

    m_staticText2654 = new wxStaticText(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2654->Wrap(-1);
    fgSizer853->Add(m_staticText2654, 0, wxALL, 5);

    m_staticText2655 = new wxStaticText(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2655->Wrap(-1);
    fgSizer853->Add(m_staticText2655, 0, wxALL, 5);

    m_staticText2656 = new wxStaticText(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2656->Wrap(-1);
    fgSizer853->Add(m_staticText2656, 0, wxALL, 5);

    m_staticText2657 = new wxStaticText(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2657->Wrap(-1);
    fgSizer853->Add(m_staticText2657, 0, wxALL, 5);

    m_staticText2658 = new wxStaticText(ID_PANEL_MUX_PD_SEL0, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2658->Wrap(-1);
    fgSizer853->Add(m_staticText2658, 0, wxALL, 5);

    chkCHx_PD_SEL0_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkCHx_PD_SEL0_MASK0, 0, wxALL, 0);

    chkCHx_PD_SEL0_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkCHx_PD_SEL0_MASK1, 0, wxALL, 0);

    chkCHx_PD_SEL0_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkCHx_PD_SEL0_MASK2, 0, wxALL, 0);

    chkCHx_PD_SEL0_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkCHx_PD_SEL0_MASK3, 0, wxALL, 0);

    chkCHx_PD_SEL0_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkCHx_PD_SEL0_MASK4, 0, wxALL, 0);

    chkCHx_PD_SEL0_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkCHx_PD_SEL0_MASK5, 0, wxALL, 0);

    chkCHx_PD_SEL0_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkCHx_PD_SEL0_MASK6, 0, wxALL, 0);

    chkCHx_PD_SEL0_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkCHx_PD_SEL0_MASK7, 0, wxALL, 0);

    chkCHx_PD_SEL0_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkCHx_PD_SEL0_MASK8, 0, wxALL, 0);

    sbSizer1152->Add(fgSizer853, 1, wxEXPAND, 5);

    fgSizer160->Add(sbSizer1152, 1, wxEXPAND, 5);

    fgSizer882->Add(fgSizer160, 1, wxEXPAND, 5);

    ID_PANEL_MUX_PD_SEL0->SetSizer(fgSizer882);
    ID_PANEL_MUX_PD_SEL0->Layout();
    fgSizer882->Fit(ID_PANEL_MUX_PD_SEL0);
    ID_NOTEBOOK_MUX_CONTROL_PD->AddPage(ID_PANEL_MUX_PD_SEL0, wxT("Bit 0"), true);
    ID_PANEL_MUX_PD_SEL01 = new wxPanel(ID_NOTEBOOK_MUX_CONTROL_PD, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer8821;
    fgSizer8821 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer8821->SetFlexibleDirection(wxBOTH);
    fgSizer8821->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer1601;
    fgSizer1601 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1601->SetFlexibleDirection(wxBOTH);
    fgSizer1601->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer1681;
    sbSizer1681 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer1591;
    fgSizer1591 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer1591->SetFlexibleDirection(wxBOTH);
    fgSizer1591->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_PD_SEL1_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1591->Add(chkCHx_PD_SEL1_INTERNAL, 0, wxALL, 0);

    chkCHx_PD_SEL1_INVERT =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1591->Add(chkCHx_PD_SEL1_INVERT, 0, wxALL, 0);

    sbSizer1681->Add(fgSizer1591, 1, wxEXPAND, 5);

    fgSizer1601->Add(sbSizer1681, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer11521;
    sbSizer11521 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8531;
    fgSizer8531 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer8531->SetFlexibleDirection(wxBOTH);
    fgSizer8531->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText2659 = new wxStaticText(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2659->Wrap(-1);
    fgSizer8531->Add(m_staticText2659, 0, wxALL, 5);

    m_staticText26511 = new wxStaticText(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26511->Wrap(-1);
    fgSizer8531->Add(m_staticText26511, 0, wxALL, 5);

    m_staticText26521 = new wxStaticText(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26521->Wrap(-1);
    fgSizer8531->Add(m_staticText26521, 0, wxALL, 5);

    m_staticText26531 = new wxStaticText(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26531->Wrap(-1);
    fgSizer8531->Add(m_staticText26531, 0, wxALL, 5);

    m_staticText26541 = new wxStaticText(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26541->Wrap(-1);
    fgSizer8531->Add(m_staticText26541, 0, wxALL, 5);

    m_staticText26551 = new wxStaticText(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26551->Wrap(-1);
    fgSizer8531->Add(m_staticText26551, 0, wxALL, 5);

    m_staticText26561 = new wxStaticText(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26561->Wrap(-1);
    fgSizer8531->Add(m_staticText26561, 0, wxALL, 5);

    m_staticText26571 = new wxStaticText(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26571->Wrap(-1);
    fgSizer8531->Add(m_staticText26571, 0, wxALL, 5);

    m_staticText26581 = new wxStaticText(ID_PANEL_MUX_PD_SEL01, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26581->Wrap(-1);
    fgSizer8531->Add(m_staticText26581, 0, wxALL, 5);

    chkCHx_PD_SEL1_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkCHx_PD_SEL1_MASK0, 0, wxALL, 0);

    chkCHx_PD_SEL1_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkCHx_PD_SEL1_MASK1, 0, wxALL, 0);

    chkCHx_PD_SEL1_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkCHx_PD_SEL1_MASK2, 0, wxALL, 0);

    chkCHx_PD_SEL1_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkCHx_PD_SEL1_MASK3, 0, wxALL, 0);

    chkCHx_PD_SEL1_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkCHx_PD_SEL1_MASK4, 0, wxALL, 0);

    chkCHx_PD_SEL1_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkCHx_PD_SEL1_MASK5, 0, wxALL, 0);

    chkCHx_PD_SEL1_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkCHx_PD_SEL1_MASK6, 0, wxALL, 0);

    chkCHx_PD_SEL1_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkCHx_PD_SEL1_MASK7, 0, wxALL, 0);

    chkCHx_PD_SEL1_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL01, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkCHx_PD_SEL1_MASK8, 0, wxALL, 0);

    sbSizer11521->Add(fgSizer8531, 1, wxEXPAND, 5);

    fgSizer1601->Add(sbSizer11521, 1, wxEXPAND, 5);

    fgSizer8821->Add(fgSizer1601, 1, wxEXPAND, 5);

    ID_PANEL_MUX_PD_SEL01->SetSizer(fgSizer8821);
    ID_PANEL_MUX_PD_SEL01->Layout();
    fgSizer8821->Fit(ID_PANEL_MUX_PD_SEL01);
    ID_NOTEBOOK_MUX_CONTROL_PD->AddPage(ID_PANEL_MUX_PD_SEL01, wxT("Bit 1"), false);

    fgSizer369->Add(ID_NOTEBOOK_MUX_CONTROL_PD, 1, wxEXPAND | wxALL, 5);

    wxStaticBoxSizer* sbSizer304;
    sbSizer304 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CHANNEL, wxID_ANY, wxT("Internal")), wxVERTICAL);

    wxFlexGridSizer* fgSizer370;
    fgSizer370 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer370->SetFlexibleDirection(wxBOTH);
    fgSizer370->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_PD_INT_SEL0 = new wxCheckBox(ID_PANEL_CHANNEL, ID_CHx_PA_ILIN2X, wxT("Bit 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer370->Add(chkCHx_PD_INT_SEL0, 0, wxALL, 5);

    chkCHx_PD_INT_SEL1 = new wxCheckBox(ID_PANEL_CHANNEL, ID_CHx_PA_ILIN2X, wxT("Bit 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer370->Add(chkCHx_PD_INT_SEL1, 0, wxALL, 5);

    sbSizer304->Add(fgSizer370, 1, wxEXPAND, 5);

    fgSizer369->Add(sbSizer304, 1, wxEXPAND, 5);

    sbSizer171->Add(fgSizer369, 1, wxEXPAND, 5);

    fgSizer161->Add(sbSizer171, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer1711;
    sbSizer1711 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CHANNEL, wxID_ANY, wxT("LNA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer372;
    fgSizer372 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer372->SetFlexibleDirection(wxBOTH);
    fgSizer372->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_NOTEBOOK_MUX_CONTROL_LNA = new wxNotebook(ID_PANEL_CHANNEL, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_MUX_PD_SEL02 = new wxPanel(ID_NOTEBOOK_MUX_CONTROL_LNA, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer8822;
    fgSizer8822 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer8822->SetFlexibleDirection(wxBOTH);
    fgSizer8822->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer1602;
    fgSizer1602 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1602->SetFlexibleDirection(wxBOTH);
    fgSizer1602->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer1682;
    sbSizer1682 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer1592;
    fgSizer1592 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer1592->SetFlexibleDirection(wxBOTH);
    fgSizer1592->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_LNA_SEL0_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1592->Add(chkCHx_LNA_SEL0_INTERNAL, 0, wxALL, 0);

    chkCHx_LNA_SEL0_INVERT =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1592->Add(chkCHx_LNA_SEL0_INVERT, 0, wxALL, 0);

    sbSizer1682->Add(fgSizer1592, 1, wxEXPAND, 5);

    fgSizer1602->Add(sbSizer1682, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer11522;
    sbSizer11522 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8532;
    fgSizer8532 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer8532->SetFlexibleDirection(wxBOTH);
    fgSizer8532->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText26510 = new wxStaticText(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26510->Wrap(-1);
    fgSizer8532->Add(m_staticText26510, 0, wxALL, 5);

    m_staticText26512 = new wxStaticText(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26512->Wrap(-1);
    fgSizer8532->Add(m_staticText26512, 0, wxALL, 5);

    m_staticText26522 = new wxStaticText(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26522->Wrap(-1);
    fgSizer8532->Add(m_staticText26522, 0, wxALL, 5);

    m_staticText26532 = new wxStaticText(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26532->Wrap(-1);
    fgSizer8532->Add(m_staticText26532, 0, wxALL, 5);

    m_staticText26542 = new wxStaticText(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26542->Wrap(-1);
    fgSizer8532->Add(m_staticText26542, 0, wxALL, 5);

    m_staticText26552 = new wxStaticText(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26552->Wrap(-1);
    fgSizer8532->Add(m_staticText26552, 0, wxALL, 5);

    m_staticText26562 = new wxStaticText(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26562->Wrap(-1);
    fgSizer8532->Add(m_staticText26562, 0, wxALL, 5);

    m_staticText26572 = new wxStaticText(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26572->Wrap(-1);
    fgSizer8532->Add(m_staticText26572, 0, wxALL, 5);

    m_staticText26582 = new wxStaticText(ID_PANEL_MUX_PD_SEL02, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26582->Wrap(-1);
    fgSizer8532->Add(m_staticText26582, 0, wxALL, 5);

    chkCHx_LNA_SEL0_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkCHx_LNA_SEL0_MASK0, 0, wxALL, 0);

    chkCHx_LNA_SEL0_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkCHx_LNA_SEL0_MASK1, 0, wxALL, 0);

    chkCHx_LNA_SEL0_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkCHx_LNA_SEL0_MASK2, 0, wxALL, 0);

    chkCHx_LNA_SEL0_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkCHx_LNA_SEL0_MASK3, 0, wxALL, 0);

    chkCHx_LNA_SEL0_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkCHx_LNA_SEL0_MASK4, 0, wxALL, 0);

    chkCHx_LNA_SEL0_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkCHx_LNA_SEL0_MASK5, 0, wxALL, 0);

    chkCHx_LNA_SEL0_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkCHx_LNA_SEL0_MASK6, 0, wxALL, 0);

    chkCHx_LNA_SEL0_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkCHx_LNA_SEL0_MASK7, 0, wxALL, 0);

    chkCHx_LNA_SEL0_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL02, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkCHx_LNA_SEL0_MASK8, 0, wxALL, 0);

    sbSizer11522->Add(fgSizer8532, 1, wxEXPAND, 5);

    fgSizer1602->Add(sbSizer11522, 1, wxEXPAND, 5);

    fgSizer8822->Add(fgSizer1602, 1, wxEXPAND, 5);

    ID_PANEL_MUX_PD_SEL02->SetSizer(fgSizer8822);
    ID_PANEL_MUX_PD_SEL02->Layout();
    fgSizer8822->Fit(ID_PANEL_MUX_PD_SEL02);
    ID_NOTEBOOK_MUX_CONTROL_LNA->AddPage(ID_PANEL_MUX_PD_SEL02, wxT("Bit 0"), true);
    ID_PANEL_MUX_PD_SEL011 = new wxPanel(ID_NOTEBOOK_MUX_CONTROL_LNA, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer88211;
    fgSizer88211 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer88211->SetFlexibleDirection(wxBOTH);
    fgSizer88211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer16011;
    fgSizer16011 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer16011->SetFlexibleDirection(wxBOTH);
    fgSizer16011->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer16811;
    sbSizer16811 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer15911;
    fgSizer15911 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer15911->SetFlexibleDirection(wxBOTH);
    fgSizer15911->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_LNA_SEL1_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15911->Add(chkCHx_LNA_SEL1_INTERNAL, 0, wxALL, 0);

    chkCHx_LNA_SEL1_INVERT =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15911->Add(chkCHx_LNA_SEL1_INVERT, 0, wxALL, 0);

    sbSizer16811->Add(fgSizer15911, 1, wxEXPAND, 5);

    fgSizer16011->Add(sbSizer16811, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer115211;
    sbSizer115211 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer85311;
    fgSizer85311 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer85311->SetFlexibleDirection(wxBOTH);
    fgSizer85311->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText26591 = new wxStaticText(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26591->Wrap(-1);
    fgSizer85311->Add(m_staticText26591, 0, wxALL, 5);

    m_staticText265111 = new wxStaticText(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265111->Wrap(-1);
    fgSizer85311->Add(m_staticText265111, 0, wxALL, 5);

    m_staticText265211 = new wxStaticText(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265211->Wrap(-1);
    fgSizer85311->Add(m_staticText265211, 0, wxALL, 5);

    m_staticText265311 = new wxStaticText(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265311->Wrap(-1);
    fgSizer85311->Add(m_staticText265311, 0, wxALL, 5);

    m_staticText265411 = new wxStaticText(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265411->Wrap(-1);
    fgSizer85311->Add(m_staticText265411, 0, wxALL, 5);

    m_staticText265511 = new wxStaticText(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265511->Wrap(-1);
    fgSizer85311->Add(m_staticText265511, 0, wxALL, 5);

    m_staticText265611 = new wxStaticText(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265611->Wrap(-1);
    fgSizer85311->Add(m_staticText265611, 0, wxALL, 5);

    m_staticText265711 = new wxStaticText(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265711->Wrap(-1);
    fgSizer85311->Add(m_staticText265711, 0, wxALL, 5);

    m_staticText265811 = new wxStaticText(ID_PANEL_MUX_PD_SEL011, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265811->Wrap(-1);
    fgSizer85311->Add(m_staticText265811, 0, wxALL, 5);

    chkCHx_LNA_SEL1_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkCHx_LNA_SEL1_MASK0, 0, wxALL, 0);

    chkCHx_LNA_SEL1_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkCHx_LNA_SEL1_MASK1, 0, wxALL, 0);

    chkCHx_LNA_SEL1_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkCHx_LNA_SEL1_MASK2, 0, wxALL, 0);

    chkCHx_LNA_SEL1_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkCHx_LNA_SEL1_MASK3, 0, wxALL, 0);

    chkCHx_LNA_SEL1_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkCHx_LNA_SEL1_MASK4, 0, wxALL, 0);

    chkCHx_LNA_SEL1_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkCHx_LNA_SEL1_MASK5, 0, wxALL, 0);

    chkCHx_LNA_SEL1_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkCHx_LNA_SEL1_MASK6, 0, wxALL, 0);

    chkCHx_LNA_SEL1_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkCHx_LNA_SEL1_MASK7, 0, wxALL, 0);

    chkCHx_LNA_SEL1_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL011, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkCHx_LNA_SEL1_MASK8, 0, wxALL, 0);

    sbSizer115211->Add(fgSizer85311, 1, wxEXPAND, 5);

    fgSizer16011->Add(sbSizer115211, 1, wxEXPAND, 5);

    fgSizer88211->Add(fgSizer16011, 1, wxEXPAND, 5);

    ID_PANEL_MUX_PD_SEL011->SetSizer(fgSizer88211);
    ID_PANEL_MUX_PD_SEL011->Layout();
    fgSizer88211->Fit(ID_PANEL_MUX_PD_SEL011);
    ID_NOTEBOOK_MUX_CONTROL_LNA->AddPage(ID_PANEL_MUX_PD_SEL011, wxT("Bit 1"), false);

    fgSizer372->Add(ID_NOTEBOOK_MUX_CONTROL_LNA, 1, wxEXPAND | wxALL, 5);

    wxStaticBoxSizer* sbSizer3041;
    sbSizer3041 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CHANNEL, wxID_ANY, wxT("Internal")), wxVERTICAL);

    wxFlexGridSizer* fgSizer3701;
    fgSizer3701 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer3701->SetFlexibleDirection(wxBOTH);
    fgSizer3701->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_LNA_INT_SEL0 = new wxCheckBox(ID_PANEL_CHANNEL, ID_CHx_PA_ILIN2X, wxT("Bit 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer3701->Add(chkCHx_LNA_INT_SEL0, 0, wxALL, 5);

    chkCHx_LNA_INT_SEL1 = new wxCheckBox(ID_PANEL_CHANNEL, ID_CHx_PA_ILIN2X, wxT("Bit 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer3701->Add(chkCHx_LNA_INT_SEL1, 0, wxALL, 5);

    sbSizer3041->Add(fgSizer3701, 1, wxEXPAND, 5);

    fgSizer372->Add(sbSizer3041, 1, wxEXPAND, 5);

    sbSizer1711->Add(fgSizer372, 1, wxEXPAND, 5);

    fgSizer161->Add(sbSizer1711, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer1712;
    sbSizer1712 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CHANNEL, wxID_ANY, wxT("PA")), wxVERTICAL);

    wxFlexGridSizer* fgSizer375;
    fgSizer375 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer375->SetFlexibleDirection(wxBOTH);
    fgSizer375->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_NOTEBOOK_MUX_CONTROL_PA = new wxNotebook(ID_PANEL_CHANNEL, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_MUX_PD_SEL03 = new wxPanel(ID_NOTEBOOK_MUX_CONTROL_PA, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer8823;
    fgSizer8823 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer8823->SetFlexibleDirection(wxBOTH);
    fgSizer8823->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer1603;
    fgSizer1603 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1603->SetFlexibleDirection(wxBOTH);
    fgSizer1603->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer1683;
    sbSizer1683 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer1593;
    fgSizer1593 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer1593->SetFlexibleDirection(wxBOTH);
    fgSizer1593->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_PA_SEL0_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1593->Add(chkCHx_PA_SEL0_INTERNAL, 0, wxALL, 0);

    chkCHx_PA_SEL0_INVERT =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1593->Add(chkCHx_PA_SEL0_INVERT, 0, wxALL, 0);

    sbSizer1683->Add(fgSizer1593, 1, wxEXPAND, 5);

    fgSizer1603->Add(sbSizer1683, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer11523;
    sbSizer11523 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8533;
    fgSizer8533 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer8533->SetFlexibleDirection(wxBOTH);
    fgSizer8533->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText26513 = new wxStaticText(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26513->Wrap(-1);
    fgSizer8533->Add(m_staticText26513, 0, wxALL, 5);

    m_staticText26514 = new wxStaticText(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26514->Wrap(-1);
    fgSizer8533->Add(m_staticText26514, 0, wxALL, 5);

    m_staticText26523 = new wxStaticText(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26523->Wrap(-1);
    fgSizer8533->Add(m_staticText26523, 0, wxALL, 5);

    m_staticText26533 = new wxStaticText(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26533->Wrap(-1);
    fgSizer8533->Add(m_staticText26533, 0, wxALL, 5);

    m_staticText26543 = new wxStaticText(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26543->Wrap(-1);
    fgSizer8533->Add(m_staticText26543, 0, wxALL, 5);

    m_staticText26553 = new wxStaticText(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26553->Wrap(-1);
    fgSizer8533->Add(m_staticText26553, 0, wxALL, 5);

    m_staticText26563 = new wxStaticText(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26563->Wrap(-1);
    fgSizer8533->Add(m_staticText26563, 0, wxALL, 5);

    m_staticText26573 = new wxStaticText(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26573->Wrap(-1);
    fgSizer8533->Add(m_staticText26573, 0, wxALL, 5);

    m_staticText26583 = new wxStaticText(ID_PANEL_MUX_PD_SEL03, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26583->Wrap(-1);
    fgSizer8533->Add(m_staticText26583, 0, wxALL, 5);

    chkCHx_PA_SEL0_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8533->Add(chkCHx_PA_SEL0_MASK0, 0, wxALL, 0);

    chkCHx_PA_SEL0_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8533->Add(chkCHx_PA_SEL0_MASK1, 0, wxALL, 0);

    chkCHx_PA_SEL0_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8533->Add(chkCHx_PA_SEL0_MASK2, 0, wxALL, 0);

    chkCHx_PA_SEL0_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8533->Add(chkCHx_PA_SEL0_MASK3, 0, wxALL, 0);

    chkCHx_PA_SEL0_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8533->Add(chkCHx_PA_SEL0_MASK4, 0, wxALL, 0);

    chkCHx_PA_SEL0_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8533->Add(chkCHx_PA_SEL0_MASK5, 0, wxALL, 0);

    chkCHx_PA_SEL0_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8533->Add(chkCHx_PA_SEL0_MASK6, 0, wxALL, 0);

    chkCHx_PA_SEL0_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8533->Add(chkCHx_PA_SEL0_MASK7, 0, wxALL, 0);

    chkCHx_PA_SEL0_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL03, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8533->Add(chkCHx_PA_SEL0_MASK8, 0, wxALL, 0);

    sbSizer11523->Add(fgSizer8533, 1, wxEXPAND, 5);

    fgSizer1603->Add(sbSizer11523, 1, wxEXPAND, 5);

    fgSizer8823->Add(fgSizer1603, 1, wxEXPAND, 5);

    ID_PANEL_MUX_PD_SEL03->SetSizer(fgSizer8823);
    ID_PANEL_MUX_PD_SEL03->Layout();
    fgSizer8823->Fit(ID_PANEL_MUX_PD_SEL03);
    ID_NOTEBOOK_MUX_CONTROL_PA->AddPage(ID_PANEL_MUX_PD_SEL03, wxT("Bit 0"), true);
    ID_PANEL_MUX_PD_SEL012 = new wxPanel(ID_NOTEBOOK_MUX_CONTROL_PA, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer88212;
    fgSizer88212 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer88212->SetFlexibleDirection(wxBOTH);
    fgSizer88212->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer16012;
    fgSizer16012 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer16012->SetFlexibleDirection(wxBOTH);
    fgSizer16012->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer16812;
    sbSizer16812 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer15912;
    fgSizer15912 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer15912->SetFlexibleDirection(wxBOTH);
    fgSizer15912->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_PA_SEL1_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15912->Add(chkCHx_PA_SEL1_INTERNAL, 0, wxALL, 0);

    chkCHx_PA_SEL1_INVERT =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15912->Add(chkCHx_PA_SEL1_INVERT, 0, wxALL, 0);

    sbSizer16812->Add(fgSizer15912, 1, wxEXPAND, 5);

    fgSizer16012->Add(sbSizer16812, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer115212;
    sbSizer115212 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer85312;
    fgSizer85312 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer85312->SetFlexibleDirection(wxBOTH);
    fgSizer85312->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText26592 = new wxStaticText(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26592->Wrap(-1);
    fgSizer85312->Add(m_staticText26592, 0, wxALL, 5);

    m_staticText265112 = new wxStaticText(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265112->Wrap(-1);
    fgSizer85312->Add(m_staticText265112, 0, wxALL, 5);

    m_staticText265212 = new wxStaticText(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265212->Wrap(-1);
    fgSizer85312->Add(m_staticText265212, 0, wxALL, 5);

    m_staticText265312 = new wxStaticText(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265312->Wrap(-1);
    fgSizer85312->Add(m_staticText265312, 0, wxALL, 5);

    m_staticText265412 = new wxStaticText(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265412->Wrap(-1);
    fgSizer85312->Add(m_staticText265412, 0, wxALL, 5);

    m_staticText265512 = new wxStaticText(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265512->Wrap(-1);
    fgSizer85312->Add(m_staticText265512, 0, wxALL, 5);

    m_staticText265612 = new wxStaticText(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265612->Wrap(-1);
    fgSizer85312->Add(m_staticText265612, 0, wxALL, 5);

    m_staticText265712 = new wxStaticText(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265712->Wrap(-1);
    fgSizer85312->Add(m_staticText265712, 0, wxALL, 5);

    m_staticText265812 = new wxStaticText(ID_PANEL_MUX_PD_SEL012, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265812->Wrap(-1);
    fgSizer85312->Add(m_staticText265812, 0, wxALL, 5);

    chkCHx_PA_SEL1_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85312->Add(chkCHx_PA_SEL1_MASK0, 0, wxALL, 0);

    chkCHx_PA_SEL1_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85312->Add(chkCHx_PA_SEL1_MASK1, 0, wxALL, 0);

    chkCHx_PA_SEL1_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85312->Add(chkCHx_PA_SEL1_MASK2, 0, wxALL, 0);

    chkCHx_PA_SEL1_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85312->Add(chkCHx_PA_SEL1_MASK3, 0, wxALL, 0);

    chkCHx_PA_SEL1_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85312->Add(chkCHx_PA_SEL1_MASK4, 0, wxALL, 0);

    chkCHx_PA_SEL1_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85312->Add(chkCHx_PA_SEL1_MASK5, 0, wxALL, 0);

    chkCHx_PA_SEL1_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85312->Add(chkCHx_PA_SEL1_MASK6, 0, wxALL, 0);

    chkCHx_PA_SEL1_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85312->Add(chkCHx_PA_SEL1_MASK7, 0, wxALL, 0);

    chkCHx_PA_SEL1_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_PD_SEL012, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85312->Add(chkCHx_PA_SEL1_MASK8, 0, wxALL, 0);

    sbSizer115212->Add(fgSizer85312, 1, wxEXPAND, 5);

    fgSizer16012->Add(sbSizer115212, 1, wxEXPAND, 5);

    fgSizer88212->Add(fgSizer16012, 1, wxEXPAND, 5);

    ID_PANEL_MUX_PD_SEL012->SetSizer(fgSizer88212);
    ID_PANEL_MUX_PD_SEL012->Layout();
    fgSizer88212->Fit(ID_PANEL_MUX_PD_SEL012);
    ID_NOTEBOOK_MUX_CONTROL_PA->AddPage(ID_PANEL_MUX_PD_SEL012, wxT("Bit 1"), false);

    fgSizer375->Add(ID_NOTEBOOK_MUX_CONTROL_PA, 1, wxEXPAND | wxALL, 5);

    wxStaticBoxSizer* sbSizer30411;
    sbSizer30411 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CHANNEL, wxID_ANY, wxT("Internal")), wxVERTICAL);

    wxFlexGridSizer* fgSizer37011;
    fgSizer37011 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer37011->SetFlexibleDirection(wxBOTH);
    fgSizer37011->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkCHx_PA_INT_SEL0 = new wxCheckBox(ID_PANEL_CHANNEL, ID_CHx_PA_ILIN2X, wxT("Bit 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer37011->Add(chkCHx_PA_INT_SEL0, 0, wxALL, 5);

    chkCHx_PA_INT_SEL1 = new wxCheckBox(ID_PANEL_CHANNEL, ID_CHx_PA_ILIN2X, wxT("Bit 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer37011->Add(chkCHx_PA_INT_SEL1, 0, wxALL, 5);

    sbSizer30411->Add(fgSizer37011, 1, wxEXPAND, 5);

    fgSizer375->Add(sbSizer30411, 1, wxEXPAND, 5);

    sbSizer1712->Add(fgSizer375, 1, wxEXPAND, 5);

    fgSizer161->Add(sbSizer1712, 1, wxEXPAND, 5);

    sbSizer143->Add(fgSizer161, 1, wxEXPAND, 5);

    fgSizer126->Add(sbSizer143, 1, wxALL | wxEXPAND, 5);

    fgSizer69->Add(fgSizer126, 1, wxEXPAND, 5);

    ID_PANEL_CHANNEL->SetSizer(fgSizer69);
    ID_PANEL_CHANNEL->Layout();
    fgSizer69->Fit(ID_PANEL_CHANNEL);
    ID_NOTEBOOK_CHANNEL->AddPage(ID_PANEL_CHANNEL, wxT("Channel"), false);

    fgSizer68->Add(ID_NOTEBOOK_CHANNEL, 1, wxEXPAND | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    this->SetSizer(fgSizer68);
    this->Layout();
    fgSizer68->Fit(this);

    // Connect Events
    chkCHx_PA_ILIN2X->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_ICT_LIN->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_ICT_LIN->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_ICT_LIN->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_ICT_MAIN->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_ICT_MAIN->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_ICT_MAIN->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    //pnlChannel->Connect( wxEVT_ERASE_BACKGROUND, wxEraseEventHandler( lms8001_pnlChannel_view::OnEraseBackground_pnlChannel ), NULL, this );
    chkCHx_LNA_PD0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXB_LOBUFF_PD0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXA_LOBUFF_PD0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_R50_EN0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_BYPASS0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_BYPASS0), NULL, this);
    chkCHx_PA_PD0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_PD0), NULL, this);
    cmbCHx_LNA_ICT_LIN0->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN0->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN0->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN0->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN0->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN0->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL0->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL0->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL0->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL0->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL0->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL0->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS0->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS0->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS0->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS0->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS0->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS0->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    // pnlChannel1->Connect( wxEVT_ERASE_BACKGROUND, wxEraseEventHandler( lms8001_pnlChannel_view::OnEraseBackground_pnlChannel ), NULL, this );
    chkCHx_LNA_PD1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXB_LOBUFF_PD1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXA_LOBUFF_PD1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_R50_EN1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_BYPASS1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_BYPASS0), NULL, this);
    chkCHx_PA_PD1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_PD0), NULL, this);
    cmbCHx_LNA_ICT_LIN1->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN1->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN1->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN1->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN1->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN1->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL1->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL1->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL1->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL1->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL1->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL1->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS1->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS1->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS1->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS1->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS1->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS1->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    // pnlChannel11->Connect( wxEVT_ERASE_BACKGROUND, wxEraseEventHandler( lms8001_pnlChannel_view::OnEraseBackground_pnlChannel ), NULL, this );
    chkCHx_LNA_PD2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXB_LOBUFF_PD2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXA_LOBUFF_PD2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_R50_EN2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_BYPASS2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_BYPASS0), NULL, this);
    chkCHx_PA_PD2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_PD0), NULL, this);
    cmbCHx_LNA_ICT_LIN2->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN2->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN2->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN2->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN2->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN2->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL2->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL2->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL2->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL2->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL2->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL2->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS2->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS2->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS2->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS2->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS2->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS2->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    // pnlChannel111->Connect( wxEVT_ERASE_BACKGROUND, wxEraseEventHandler( lms8001_pnlChannel_view::OnEraseBackground_pnlChannel ), NULL, this );
    chkCHx_LNA_PD3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXB_LOBUFF_PD3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXA_LOBUFF_PD3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_R50_EN3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_BYPASS3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_BYPASS0), NULL, this);
    chkCHx_PA_PD3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_PD0), NULL, this);
    cmbCHx_LNA_ICT_LIN3->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN3->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN3->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN3->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN3->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN3->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL3->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL3->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL3->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL3->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL3->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL3->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS3->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS3->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS3->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS3->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS3->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS3->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    // pnlChannel1111->Connect( wxEVT_ERASE_BACKGROUND, wxEraseEventHandler( lms8001_pnlChannel_view::OnEraseBackground_pnlChannel ), NULL, this );
    chkCHx_LNA_PD_RB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXB_LOBUFF_PD_RB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_MIXA_LOBUFF_PD_RB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_R50_EN_RB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_BYPASS_RB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_BYPASS0), NULL, this);
    chkCHx_PA_PD_RB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::OnClick_chkCHx_PA_PD0), NULL, this);
    cmbCHx_LNA_ICT_LIN_RB->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN_RB->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_LIN_RB->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN_RB->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN_RB->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_ICT_MAIN_RB->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL_RB->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL_RB->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_CGSCTRL_RB->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL_RB->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL_RB->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_LNA_GCTRL_RB->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS_RB->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS_RB->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_LIN_LOSS_RB->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS_RB->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS_RB->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    cmbCHx_PA_MAIN_LOSS_RB->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL0_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_SEL1_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_INT_SEL0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PD_INT_SEL1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL0_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_SEL1_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_INT_SEL0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_LNA_INT_SEL1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL0_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_SEL1_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_INT_SEL0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);
    chkCHx_PA_INT_SEL1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlChannel_view::ParameterChangeHandler), NULL, this);

    //	wndId2Enum[cmbCHx_MIXA_ICT] = CHx_MIXA_ICT;
    //	wndId2Enum[cmbCHx_MIXB_ICT] = CHx_MIXB_ICT;

    wndId2Enum[chkCHx_PA_ILIN2X] = CHx_PA_ILIN2X;
    wndId2Enum[cmbCHx_PA_ICT_LIN] = CHx_PA_ICT_LIN;
    wndId2Enum[cmbCHx_PA_ICT_MAIN] = CHx_PA_ICT_MAIN;

    wndId2Enum[chkCHx_PA_R50_EN0] = CHx_PA_R50_EN0;
    wndId2Enum[chkCHx_PA_BYPASS0] = CHx_PA_BYPASS0;
    wndId2Enum[chkCHx_PA_PD0] = CHx_PA_PD0;
    wndId2Enum[chkCHx_MIXB_LOBUFF_PD0] = CHx_MIXB_LOBUFF_PD0;
    //	wndId2Enum[chkCHx_MIXB_BIAS_PD0] = CHx_MIXB_BIAS_PD0;
    wndId2Enum[chkCHx_MIXA_LOBUFF_PD0] = CHx_MIXA_LOBUFF_PD0;
    //	wndId2Enum[chkCHx_MIXA_BIAS_PD0] = CHx_MIXA_BIAS_PD0;
    wndId2Enum[chkCHx_LNA_PD0] = CHx_LNA_PD0;

    wndId2Enum[cmbCHx_LNA_ICT_LIN0] = CHx_LNA_ICT_LIN0;
    wndId2Enum[cmbCHx_LNA_ICT_MAIN0] = CHx_LNA_ICT_MAIN0;
    wndId2Enum[cmbCHx_LNA_CGSCTRL0] = CHx_LNA_CGSCTRL0;
    wndId2Enum[cmbCHx_LNA_GCTRL0] = CHx_LNA_GCTRL0;

    wndId2Enum[cmbCHx_PA_LIN_LOSS0] = CHx_PA_LIN_LOSS0;
    wndId2Enum[cmbCHx_PA_MAIN_LOSS0] = CHx_PA_MAIN_LOSS0;

    wndId2Enum[chkCHx_PA_R50_EN1] = CHx_PA_R50_EN1;
    wndId2Enum[chkCHx_PA_BYPASS1] = CHx_PA_BYPASS1;
    wndId2Enum[chkCHx_PA_PD1] = CHx_PA_PD1;
    wndId2Enum[chkCHx_MIXB_LOBUFF_PD1] = CHx_MIXB_LOBUFF_PD1;
    //	wndId2Enum[chkCHx_MIXB_BIAS_PD1] = CHx_MIXB_BIAS_PD1;
    wndId2Enum[chkCHx_MIXA_LOBUFF_PD1] = CHx_MIXA_LOBUFF_PD1;
    //	wndId2Enum[chkCHx_MIXA_BIAS_PD1] = CHx_MIXA_BIAS_PD1;
    wndId2Enum[chkCHx_LNA_PD1] = CHx_LNA_PD1;

    wndId2Enum[cmbCHx_LNA_ICT_LIN1] = CHx_LNA_ICT_LIN1;
    wndId2Enum[cmbCHx_LNA_ICT_MAIN1] = CHx_LNA_ICT_MAIN1;
    wndId2Enum[cmbCHx_LNA_CGSCTRL1] = CHx_LNA_CGSCTRL1;
    wndId2Enum[cmbCHx_LNA_GCTRL1] = CHx_LNA_GCTRL1;

    wndId2Enum[cmbCHx_PA_LIN_LOSS1] = CHx_PA_LIN_LOSS1;
    wndId2Enum[cmbCHx_PA_MAIN_LOSS1] = CHx_PA_MAIN_LOSS1;

    wndId2Enum[chkCHx_PA_R50_EN2] = CHx_PA_R50_EN2;
    wndId2Enum[chkCHx_PA_BYPASS2] = CHx_PA_BYPASS2;
    wndId2Enum[chkCHx_PA_PD2] = CHx_PA_PD2;
    wndId2Enum[chkCHx_MIXB_LOBUFF_PD2] = CHx_MIXB_LOBUFF_PD2;
    //	wndId2Enum[chkCHx_MIXB_BIAS_PD2] = CHx_MIXB_BIAS_PD2;
    wndId2Enum[chkCHx_MIXA_LOBUFF_PD2] = CHx_MIXA_LOBUFF_PD2;
    //	wndId2Enum[chkCHx_MIXA_BIAS_PD2] = CHx_MIXA_BIAS_PD2;
    wndId2Enum[chkCHx_LNA_PD2] = CHx_LNA_PD2;

    wndId2Enum[cmbCHx_LNA_ICT_LIN2] = CHx_LNA_ICT_LIN2;
    wndId2Enum[cmbCHx_LNA_ICT_MAIN2] = CHx_LNA_ICT_MAIN2;
    wndId2Enum[cmbCHx_LNA_CGSCTRL2] = CHx_LNA_CGSCTRL2;
    wndId2Enum[cmbCHx_LNA_GCTRL2] = CHx_LNA_GCTRL2;

    wndId2Enum[cmbCHx_PA_LIN_LOSS2] = CHx_PA_LIN_LOSS2;
    wndId2Enum[cmbCHx_PA_MAIN_LOSS2] = CHx_PA_MAIN_LOSS2;

    wndId2Enum[chkCHx_PA_R50_EN3] = CHx_PA_R50_EN3;
    wndId2Enum[chkCHx_PA_BYPASS3] = CHx_PA_BYPASS3;
    wndId2Enum[chkCHx_PA_PD3] = CHx_PA_PD3;
    wndId2Enum[chkCHx_MIXB_LOBUFF_PD3] = CHx_MIXB_LOBUFF_PD3;
    //	wndId2Enum[chkCHx_MIXB_BIAS_PD3] = CHx_MIXB_BIAS_PD3;
    wndId2Enum[chkCHx_MIXA_LOBUFF_PD3] = CHx_MIXA_LOBUFF_PD3;
    //	wndId2Enum[chkCHx_MIXA_BIAS_PD3] = CHx_MIXA_BIAS_PD3;
    wndId2Enum[chkCHx_LNA_PD3] = CHx_LNA_PD3;

    wndId2Enum[cmbCHx_LNA_ICT_LIN3] = CHx_LNA_ICT_LIN3;
    wndId2Enum[cmbCHx_LNA_ICT_MAIN3] = CHx_LNA_ICT_MAIN3;
    wndId2Enum[cmbCHx_LNA_CGSCTRL3] = CHx_LNA_CGSCTRL3;
    wndId2Enum[cmbCHx_LNA_GCTRL3] = CHx_LNA_GCTRL3;

    wndId2Enum[cmbCHx_PA_LIN_LOSS3] = CHx_PA_LIN_LOSS3;
    wndId2Enum[cmbCHx_PA_MAIN_LOSS3] = CHx_PA_MAIN_LOSS3;

    wndId2Enum[chkCHx_PA_R50_EN_RB] = CHx_PA_R50_EN_RB;
    wndId2Enum[chkCHx_PA_BYPASS_RB] = CHx_PA_BYPASS_RB;
    wndId2Enum[chkCHx_PA_PD_RB] = CHx_PA_PD_RB;
    wndId2Enum[chkCHx_MIXB_LOBUFF_PD_RB] = CHx_MIXB_LOBUFF_PD_RB;
    //	wndId2Enum[chkCHx_MIXB_BIAS_PD_RB] = CHx_MIXB_BIAS_PD_RB;
    wndId2Enum[chkCHx_MIXA_LOBUFF_PD_RB] = CHx_MIXA_LOBUFF_PD_RB;
    //	wndId2Enum[chkCHx_MIXA_BIAS_PD_RB] = CHx_MIXA_BIAS_PD_RB;
    wndId2Enum[chkCHx_LNA_PD_RB] = CHx_LNA_PD_RB;

    wndId2Enum[cmbCHx_LNA_ICT_LIN_RB] = CHx_LNA_ICT_LIN_RB;
    wndId2Enum[cmbCHx_LNA_ICT_MAIN_RB] = CHx_LNA_ICT_MAIN_RB;
    wndId2Enum[cmbCHx_LNA_CGSCTRL_RB] = CHx_LNA_CGSCTRL_RB;
    wndId2Enum[cmbCHx_LNA_GCTRL_RB] = CHx_LNA_GCTRL_RB;

    wndId2Enum[cmbCHx_PA_LIN_LOSS_RB] = CHx_PA_LIN_LOSS_RB;
    wndId2Enum[cmbCHx_PA_MAIN_LOSS_RB] = CHx_PA_MAIN_LOSS_RB;

    wndId2Enum[chkCHx_PD_SEL0_INTERNAL] = CHx_PD_SEL0_INTERNAL;
    wndId2Enum[chkCHx_PD_SEL0_INVERT] = CHx_PD_SEL0_INVERT;
    wndId2Enum[chkCHx_PD_SEL0_MASK0] = CHx_PD_SEL0_MASK0;
    wndId2Enum[chkCHx_PD_SEL0_MASK1] = CHx_PD_SEL0_MASK1;
    wndId2Enum[chkCHx_PD_SEL0_MASK2] = CHx_PD_SEL0_MASK2;
    wndId2Enum[chkCHx_PD_SEL0_MASK3] = CHx_PD_SEL0_MASK3;
    wndId2Enum[chkCHx_PD_SEL0_MASK4] = CHx_PD_SEL0_MASK4;
    wndId2Enum[chkCHx_PD_SEL0_MASK5] = CHx_PD_SEL0_MASK5;
    wndId2Enum[chkCHx_PD_SEL0_MASK6] = CHx_PD_SEL0_MASK6;
    wndId2Enum[chkCHx_PD_SEL0_MASK7] = CHx_PD_SEL0_MASK7;
    wndId2Enum[chkCHx_PD_SEL0_MASK8] = CHx_PD_SEL0_MASK8;

    wndId2Enum[chkCHx_PD_SEL1_INTERNAL] = CHx_PD_SEL1_INTERNAL;
    wndId2Enum[chkCHx_PD_SEL1_INVERT] = CHx_PD_SEL1_INVERT;
    wndId2Enum[chkCHx_PD_SEL1_MASK0] = CHx_PD_SEL1_MASK0;
    wndId2Enum[chkCHx_PD_SEL1_MASK1] = CHx_PD_SEL1_MASK1;
    wndId2Enum[chkCHx_PD_SEL1_MASK2] = CHx_PD_SEL1_MASK2;
    wndId2Enum[chkCHx_PD_SEL1_MASK3] = CHx_PD_SEL1_MASK3;
    wndId2Enum[chkCHx_PD_SEL1_MASK4] = CHx_PD_SEL1_MASK4;
    wndId2Enum[chkCHx_PD_SEL1_MASK5] = CHx_PD_SEL1_MASK5;
    wndId2Enum[chkCHx_PD_SEL1_MASK6] = CHx_PD_SEL1_MASK6;
    wndId2Enum[chkCHx_PD_SEL1_MASK7] = CHx_PD_SEL1_MASK7;
    wndId2Enum[chkCHx_PD_SEL1_MASK8] = CHx_PD_SEL1_MASK8;

    wndId2Enum[chkCHx_LNA_SEL0_INTERNAL] = CHx_LNA_SEL0_INTERNAL;
    wndId2Enum[chkCHx_LNA_SEL0_INVERT] = CHx_LNA_SEL0_INVERT;
    wndId2Enum[chkCHx_LNA_SEL0_MASK0] = CHx_LNA_SEL0_MASK0;
    wndId2Enum[chkCHx_LNA_SEL0_MASK1] = CHx_LNA_SEL0_MASK1;
    wndId2Enum[chkCHx_LNA_SEL0_MASK2] = CHx_LNA_SEL0_MASK2;
    wndId2Enum[chkCHx_LNA_SEL0_MASK3] = CHx_LNA_SEL0_MASK3;
    wndId2Enum[chkCHx_LNA_SEL0_MASK4] = CHx_LNA_SEL0_MASK4;
    wndId2Enum[chkCHx_LNA_SEL0_MASK5] = CHx_LNA_SEL0_MASK5;
    wndId2Enum[chkCHx_LNA_SEL0_MASK6] = CHx_LNA_SEL0_MASK6;
    wndId2Enum[chkCHx_LNA_SEL0_MASK7] = CHx_LNA_SEL0_MASK7;
    wndId2Enum[chkCHx_LNA_SEL0_MASK8] = CHx_LNA_SEL0_MASK8;

    wndId2Enum[chkCHx_LNA_SEL1_INTERNAL] = CHx_LNA_SEL1_INTERNAL;
    wndId2Enum[chkCHx_LNA_SEL1_INVERT] = CHx_LNA_SEL1_INVERT;
    wndId2Enum[chkCHx_LNA_SEL1_MASK0] = CHx_LNA_SEL1_MASK0;
    wndId2Enum[chkCHx_LNA_SEL1_MASK1] = CHx_LNA_SEL1_MASK1;
    wndId2Enum[chkCHx_LNA_SEL1_MASK2] = CHx_LNA_SEL1_MASK2;
    wndId2Enum[chkCHx_LNA_SEL1_MASK3] = CHx_LNA_SEL1_MASK3;
    wndId2Enum[chkCHx_LNA_SEL1_MASK4] = CHx_LNA_SEL1_MASK4;
    wndId2Enum[chkCHx_LNA_SEL1_MASK5] = CHx_LNA_SEL1_MASK5;
    wndId2Enum[chkCHx_LNA_SEL1_MASK6] = CHx_LNA_SEL1_MASK6;
    wndId2Enum[chkCHx_LNA_SEL1_MASK7] = CHx_LNA_SEL1_MASK7;
    wndId2Enum[chkCHx_LNA_SEL1_MASK8] = CHx_LNA_SEL1_MASK8;

    wndId2Enum[chkCHx_PA_SEL0_INTERNAL] = CHx_PA_SEL0_INTERNAL;
    wndId2Enum[chkCHx_PA_SEL0_INVERT] = CHx_PA_SEL0_INVERT;
    wndId2Enum[chkCHx_PA_SEL0_MASK0] = CHx_PA_SEL0_MASK0;
    wndId2Enum[chkCHx_PA_SEL0_MASK1] = CHx_PA_SEL0_MASK1;
    wndId2Enum[chkCHx_PA_SEL0_MASK2] = CHx_PA_SEL0_MASK2;
    wndId2Enum[chkCHx_PA_SEL0_MASK3] = CHx_PA_SEL0_MASK3;
    wndId2Enum[chkCHx_PA_SEL0_MASK4] = CHx_PA_SEL0_MASK4;
    wndId2Enum[chkCHx_PA_SEL0_MASK5] = CHx_PA_SEL0_MASK5;
    wndId2Enum[chkCHx_PA_SEL0_MASK6] = CHx_PA_SEL0_MASK6;
    wndId2Enum[chkCHx_PA_SEL0_MASK7] = CHx_PA_SEL0_MASK7;
    wndId2Enum[chkCHx_PA_SEL0_MASK8] = CHx_PA_SEL0_MASK8;

    wndId2Enum[chkCHx_PA_SEL1_INTERNAL] = CHx_PA_SEL1_INTERNAL;
    wndId2Enum[chkCHx_PA_SEL1_INVERT] = CHx_PA_SEL1_INVERT;
    wndId2Enum[chkCHx_PA_SEL1_MASK0] = CHx_PA_SEL1_MASK0;
    wndId2Enum[chkCHx_PA_SEL1_MASK1] = CHx_PA_SEL1_MASK1;
    wndId2Enum[chkCHx_PA_SEL1_MASK2] = CHx_PA_SEL1_MASK2;
    wndId2Enum[chkCHx_PA_SEL1_MASK3] = CHx_PA_SEL1_MASK3;
    wndId2Enum[chkCHx_PA_SEL1_MASK4] = CHx_PA_SEL1_MASK4;
    wndId2Enum[chkCHx_PA_SEL1_MASK5] = CHx_PA_SEL1_MASK5;
    wndId2Enum[chkCHx_PA_SEL1_MASK6] = CHx_PA_SEL1_MASK6;
    wndId2Enum[chkCHx_PA_SEL1_MASK7] = CHx_PA_SEL1_MASK7;
    wndId2Enum[chkCHx_PA_SEL1_MASK8] = CHx_PA_SEL1_MASK8;

    wndId2Enum[chkCHx_PD_INT_SEL0] = CHx_PD_INT_SEL0;
    wndId2Enum[chkCHx_PD_INT_SEL1] = CHx_PD_INT_SEL1;
    wndId2Enum[chkCHx_LNA_INT_SEL0] = CHx_LNA_INT_SEL0;
    wndId2Enum[chkCHx_LNA_INT_SEL1] = CHx_LNA_INT_SEL1;
    wndId2Enum[chkCHx_PA_INT_SEL0] = CHx_PA_INT_SEL0;
    wndId2Enum[chkCHx_PA_INT_SEL1] = CHx_PA_INT_SEL1;

    LMS8001_WXGUI::UpdateTooltips(wndId2Enum, true);
}

void lms8001_pnlChannel_view::OnClick_chkCHx_PA_PD0(wxCommandEvent& event)
{
    bool pa_pd = chkCHx_PA_PD0->GetValue();
    bool pa_bypass = chkCHx_PA_BYPASS0->GetValue();

    if (!pa_pd && pa_bypass)
    {
        wxString str;
        str = wxString::Format("It is not allowed to have Power Amplifier enabled and bypassed at the same time!");
        wxMessageBox(str, "ERROR!");
        chkCHx_PA_PD0->SetValue(true);
    }
    else
        ParameterChangeHandler(event);
}

void lms8001_pnlChannel_view::OnClick_chkCHx_PA_BYPASS0(wxCommandEvent& event)
{
    bool pa_pd = chkCHx_PA_PD0->GetValue();
    bool pa_bypass = chkCHx_PA_BYPASS0->GetValue();

    if (!pa_pd && pa_bypass)
    {
        wxString str;
        str = wxString::Format("It is not allowed to have Power Amplifier enabled and bypassed at the same time!");
        wxMessageBox(str, "ERROR!");
        chkCHx_PA_BYPASS0->SetValue(false);
    }
    else
        ParameterChangeHandler(event);
}
