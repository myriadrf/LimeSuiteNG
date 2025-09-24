#include "lms8001_pnlHLMIX_view.h"
#include <map>
#include "lms8001_gui_utilities.h"

#include "chips/LMS8001/LMS8001.h"
#include "chips/LMS8001/LMS8001_parameters.h"

using namespace lime;

lms8001_pnlHLMIX_view::lms8001_pnlHLMIX_view(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ILMS8001Tab(parent, id, pos, size, style)
{
    wxFlexGridSizer* fgSizer68;
    fgSizer68 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer68->SetFlexibleDirection(wxBOTH);
    fgSizer68->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_NOTEBOOK_HLMIX = new wxNotebook(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_HLMIX = new wxPanel(ID_NOTEBOOK_HLMIX, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer69;
    fgSizer69 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer69->SetFlexibleDirection(wxBOTH);
    fgSizer69->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_NOTEBOOK_HLMIX_PROGRAM = new wxNotebook(ID_PANEL_HLMIX, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_HLMIX_P0 = new wxPanel(ID_NOTEBOOK_HLMIX_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer378;
    fgSizer378 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer378->SetFlexibleDirection(wxBOTH);
    fgSizer378->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer312;
    sbSizer312 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_P0, wxID_ANY, wxT("Configuration")), wxVERTICAL);

    m_staticText6493 = new wxStaticText(ID_PANEL_HLMIX_P0, wxID_ANY, wxT("LO bias voltage"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText6493->Wrap(-1);
    sbSizer312->Add(m_staticText6493, 0, wxALL, 0);

    cmbHLMIXx_VGCAS0 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P0, ID_HLMIXx_VGCAS0, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 127, 64);
    sbSizer312->Add(cmbHLMIXx_VGCAS0, 0, wxALL, 0);

    m_staticText649 =
        new wxStaticText(ID_PANEL_HLMIX_P0, wxID_ANY, wxT("Bias current control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText649->Wrap(-1);
    sbSizer312->Add(m_staticText649, 0, wxALL, 0);

    cmbHLMIXx_ICT_BIAS0 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P0, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    sbSizer312->Add(cmbHLMIXx_ICT_BIAS0, 0, wxALL, 0);

    chkHLMIXx_BIAS_PD0 = new wxCheckBox(ID_PANEL_HLMIX_P0, ID_CHx_PA_ILIN2X, wxT("BIAS_PD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer312->Add(chkHLMIXx_BIAS_PD0, 0, wxALL, 0);

    chkHLMIXx_LOBUF_PD0 = new wxCheckBox(ID_PANEL_HLMIX_P0, ID_CHx_PA_ILIN2X, wxT("LOBUF_PD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer312->Add(chkHLMIXx_LOBUF_PD0, 0, wxALL, 0);

    fgSizer378->Add(sbSizer312, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer3121;
    sbSizer3121 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_P0, wxID_ANY, wxT("Loss")), wxVERTICAL);

    m_staticText6491 = new wxStaticText(ID_PANEL_HLMIX_P0, wxID_ANY, wxT("Loss control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText6491->Wrap(-1);
    sbSizer3121->Add(m_staticText6491, 0, wxALL, 0);

    cmbHLMIXx_MIXLOSS0 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P0, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 0);
    sbSizer3121->Add(cmbHLMIXx_MIXLOSS0, 0, wxALL, 0);

    fgSizer378->Add(sbSizer3121, 1, wxEXPAND, 5);

    ID_PANEL_HLMIX_P0->SetSizer(fgSizer378);
    ID_PANEL_HLMIX_P0->Layout();
    fgSizer378->Fit(ID_PANEL_HLMIX_P0);
    ID_NOTEBOOK_HLMIX_PROGRAM->AddPage(ID_PANEL_HLMIX_P0, wxT("Prog. 0"), true);
    ID_PANEL_HLMIX_P1 = new wxPanel(ID_NOTEBOOK_HLMIX_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer3781;
    fgSizer3781 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer3781->SetFlexibleDirection(wxBOTH);
    fgSizer3781->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer3122;
    sbSizer3122 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_P1, wxID_ANY, wxT("Config")), wxVERTICAL);

    m_staticText64922 = new wxStaticText(ID_PANEL_HLMIX_P1, wxID_ANY, wxT("LO bias voltage"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText64922->Wrap(-1);
    sbSizer3122->Add(m_staticText64922, 0, wxALL, 0);

    cmbHLMIXx_VGCAS1 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P1, ID_HLMIXx_VGCAS1, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 127, 64);
    sbSizer3122->Add(cmbHLMIXx_VGCAS1, 0, wxALL, 0);

    m_staticText6492 =
        new wxStaticText(ID_PANEL_HLMIX_P1, wxID_ANY, wxT("Bias current control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText6492->Wrap(-1);
    sbSizer3122->Add(m_staticText6492, 0, wxALL, 0);

    cmbHLMIXx_ICT_BIAS1 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P1, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    sbSizer3122->Add(cmbHLMIXx_ICT_BIAS1, 0, wxALL, 0);

    chkHLMIXx_BIAS_PD1 = new wxCheckBox(ID_PANEL_HLMIX_P1, ID_CHx_PA_ILIN2X, wxT("BIAS_PD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer3122->Add(chkHLMIXx_BIAS_PD1, 0, wxALL, 0);

    chkHLMIXx_LOBUF_PD1 = new wxCheckBox(ID_PANEL_HLMIX_P1, ID_CHx_PA_ILIN2X, wxT("LOBUF_PD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer3122->Add(chkHLMIXx_LOBUF_PD1, 0, wxALL, 0);

    fgSizer3781->Add(sbSizer3122, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer31211;
    sbSizer31211 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_P1, wxID_ANY, wxT("Loss")), wxVERTICAL);

    m_staticText64912 = new wxStaticText(ID_PANEL_HLMIX_P1, wxID_ANY, wxT("Loss control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText64912->Wrap(-1);
    sbSizer31211->Add(m_staticText64912, 0, wxALL, 0);

    cmbHLMIXx_MIXLOSS1 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P1, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 0);
    sbSizer31211->Add(cmbHLMIXx_MIXLOSS1, 0, wxALL, 0);

    fgSizer3781->Add(sbSizer31211, 1, wxEXPAND, 5);

    ID_PANEL_HLMIX_P1->SetSizer(fgSizer3781);
    ID_PANEL_HLMIX_P1->Layout();
    fgSizer3781->Fit(ID_PANEL_HLMIX_P1);
    ID_NOTEBOOK_HLMIX_PROGRAM->AddPage(ID_PANEL_HLMIX_P1, wxT("Prog. 1"), false);
    ID_PANEL_HLMIX_P2 = new wxPanel(ID_NOTEBOOK_HLMIX_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer37811;
    fgSizer37811 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer37811->SetFlexibleDirection(wxBOTH);
    fgSizer37811->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer31221;
    sbSizer31221 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_P2, wxID_ANY, wxT("Config")), wxVERTICAL);

    m_staticText649212 = new wxStaticText(ID_PANEL_HLMIX_P2, wxID_ANY, wxT("LO bias voltage"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText649212->Wrap(-1);
    sbSizer31221->Add(m_staticText649212, 0, wxALL, 0);

    cmbHLMIXx_VGCAS2 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P2, ID_HLMIXx_VGCAS2, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 127, 64);
    sbSizer31221->Add(cmbHLMIXx_VGCAS2, 0, wxALL, 0);

    m_staticText64921 =
        new wxStaticText(ID_PANEL_HLMIX_P2, wxID_ANY, wxT("Bias current control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText64921->Wrap(-1);
    sbSizer31221->Add(m_staticText64921, 0, wxALL, 0);

    cmbHLMIXx_ICT_BIAS2 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P2, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    sbSizer31221->Add(cmbHLMIXx_ICT_BIAS2, 0, wxALL, 0);

    chkHLMIXx_BIAS_PD2 = new wxCheckBox(ID_PANEL_HLMIX_P2, ID_CHx_PA_ILIN2X, wxT("BIAS_PD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer31221->Add(chkHLMIXx_BIAS_PD2, 0, wxALL, 0);

    chkHLMIXx_LOBUF_PD2 = new wxCheckBox(ID_PANEL_HLMIX_P2, ID_CHx_PA_ILIN2X, wxT("LOBUF_PD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer31221->Add(chkHLMIXx_LOBUF_PD2, 0, wxALL, 0);

    fgSizer37811->Add(sbSizer31221, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer312111;
    sbSizer312111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_P2, wxID_ANY, wxT("Loss")), wxVERTICAL);

    m_staticText649121 = new wxStaticText(ID_PANEL_HLMIX_P2, wxID_ANY, wxT("Loss control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText649121->Wrap(-1);
    sbSizer312111->Add(m_staticText649121, 0, wxALL, 0);

    cmbHLMIXx_MIXLOSS2 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P2, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 0);
    sbSizer312111->Add(cmbHLMIXx_MIXLOSS2, 0, wxALL, 0);

    fgSizer37811->Add(sbSizer312111, 1, wxEXPAND, 5);

    ID_PANEL_HLMIX_P2->SetSizer(fgSizer37811);
    ID_PANEL_HLMIX_P2->Layout();
    fgSizer37811->Fit(ID_PANEL_HLMIX_P2);
    ID_NOTEBOOK_HLMIX_PROGRAM->AddPage(ID_PANEL_HLMIX_P2, wxT("Prog. 2"), false);
    ID_PANEL_HLMIX_P3 = new wxPanel(ID_NOTEBOOK_HLMIX_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer378111;
    fgSizer378111 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer378111->SetFlexibleDirection(wxBOTH);
    fgSizer378111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer312211;
    sbSizer312211 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_P3, wxID_ANY, wxT("Config")), wxVERTICAL);

    m_staticText6492121 =
        new wxStaticText(ID_PANEL_HLMIX_P3, wxID_ANY, wxT("LO bias voltage"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText6492121->Wrap(-1);
    sbSizer312211->Add(m_staticText6492121, 0, wxALL, 0);

    cmbHLMIXx_VGCAS3 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P3, ID_HLMIXx_VGCAS3, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 127, 64);
    sbSizer312211->Add(cmbHLMIXx_VGCAS3, 0, wxALL, 0);

    m_staticText649211 =
        new wxStaticText(ID_PANEL_HLMIX_P3, wxID_ANY, wxT("Bias current control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText649211->Wrap(-1);
    sbSizer312211->Add(m_staticText649211, 0, wxALL, 0);

    cmbHLMIXx_ICT_BIAS3 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P3, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    sbSizer312211->Add(cmbHLMIXx_ICT_BIAS3, 0, wxALL, 0);

    chkHLMIXx_BIAS_PD3 = new wxCheckBox(ID_PANEL_HLMIX_P3, ID_CHx_PA_ILIN2X, wxT("BIAS_PD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer312211->Add(chkHLMIXx_BIAS_PD3, 0, wxALL, 0);

    chkHLMIXx_LOBUF_PD3 = new wxCheckBox(ID_PANEL_HLMIX_P3, ID_CHx_PA_ILIN2X, wxT("LOBUF_PD"), wxDefaultPosition, wxDefaultSize, 0);
    sbSizer312211->Add(chkHLMIXx_LOBUF_PD3, 0, wxALL, 0);

    fgSizer378111->Add(sbSizer312211, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer3121111;
    sbSizer3121111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_P3, wxID_ANY, wxT("Loss")), wxVERTICAL);

    m_staticText6491211 = new wxStaticText(ID_PANEL_HLMIX_P3, wxID_ANY, wxT("Loss control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText6491211->Wrap(-1);
    sbSizer3121111->Add(m_staticText6491211, 0, wxALL, 0);

    cmbHLMIXx_MIXLOSS3 = new wxSpinCtrl(
        ID_PANEL_HLMIX_P3, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 0);
    sbSizer3121111->Add(cmbHLMIXx_MIXLOSS3, 0, wxALL, 0);

    fgSizer378111->Add(sbSizer3121111, 1, wxEXPAND, 5);

    ID_PANEL_HLMIX_P3->SetSizer(fgSizer378111);
    ID_PANEL_HLMIX_P3->Layout();
    fgSizer378111->Fit(ID_PANEL_HLMIX_P3);
    ID_NOTEBOOK_HLMIX_PROGRAM->AddPage(ID_PANEL_HLMIX_P3, wxT("Prog. 3"), false);
    ID_PANEL_HLMIX_RB = new wxPanel(ID_NOTEBOOK_HLMIX_PROGRAM, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer3781111;
    fgSizer3781111 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer3781111->SetFlexibleDirection(wxBOTH);
    fgSizer3781111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer3122111;
    sbSizer3122111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_RB, wxID_ANY, wxT("Config")), wxVERTICAL);

    m_staticText64921211 =
        new wxStaticText(ID_PANEL_HLMIX_RB, wxID_ANY, wxT("LO bias voltage"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText64921211->Wrap(-1);
    sbSizer3122111->Add(m_staticText64921211, 0, wxALL, 0);

    cmbHLMIXx_VGCAS_RB = new wxSpinCtrl(
        ID_PANEL_HLMIX_RB, ID_HLMIXx_VGCAS3, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 127, 64);
    cmbHLMIXx_VGCAS_RB->Enable(false);

    sbSizer3122111->Add(cmbHLMIXx_VGCAS_RB, 0, wxALL, 0);

    m_staticText6492111 =
        new wxStaticText(ID_PANEL_HLMIX_RB, wxID_ANY, wxT("Bias current control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText6492111->Wrap(-1);
    sbSizer3122111->Add(m_staticText6492111, 0, wxALL, 0);

    cmbHLMIXx_ICT_BIAS_RB = new wxSpinCtrl(
        ID_PANEL_HLMIX_RB, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 31, 16);
    cmbHLMIXx_ICT_BIAS_RB->Enable(false);

    sbSizer3122111->Add(cmbHLMIXx_ICT_BIAS_RB, 0, wxALL, 0);

    chkHLMIXx_BIAS_PD_RB = new wxCheckBox(ID_PANEL_HLMIX_RB, ID_CHx_PA_ILIN2X, wxT("BIAS_PD"), wxDefaultPosition, wxDefaultSize, 0);
    chkHLMIXx_BIAS_PD_RB->Enable(false);

    sbSizer3122111->Add(chkHLMIXx_BIAS_PD_RB, 0, wxALL, 0);

    chkHLMIXx_LOBUF_PD_RB =
        new wxCheckBox(ID_PANEL_HLMIX_RB, ID_CHx_PA_ILIN2X, wxT("LOBUF_PD"), wxDefaultPosition, wxDefaultSize, 0);
    chkHLMIXx_LOBUF_PD_RB->Enable(false);

    sbSizer3122111->Add(chkHLMIXx_LOBUF_PD_RB, 0, wxALL, 0);

    fgSizer3781111->Add(sbSizer3122111, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer31211111;
    sbSizer31211111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX_RB, wxID_ANY, wxT("Loss")), wxVERTICAL);

    m_staticText64912111 = new wxStaticText(ID_PANEL_HLMIX_RB, wxID_ANY, wxT("Loss control"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText64912111->Wrap(-1);
    sbSizer31211111->Add(m_staticText64912111, 0, wxALL, 0);

    cmbHLMIXx_MIXLOSS_RB = new wxSpinCtrl(
        ID_PANEL_HLMIX_RB, ID_RP_CALIB_BIAS, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 15, 0);
    cmbHLMIXx_MIXLOSS_RB->Enable(false);

    sbSizer31211111->Add(cmbHLMIXx_MIXLOSS_RB, 0, wxALL, 0);

    fgSizer3781111->Add(sbSizer31211111, 1, wxEXPAND, 5);

    ID_PANEL_HLMIX_RB->SetSizer(fgSizer3781111);
    ID_PANEL_HLMIX_RB->Layout();
    fgSizer3781111->Fit(ID_PANEL_HLMIX_RB);
    ID_NOTEBOOK_HLMIX_PROGRAM->AddPage(ID_PANEL_HLMIX_RB, wxT("Readback"), false);

    fgSizer69->Add(ID_NOTEBOOK_HLMIX_PROGRAM, 1, wxEXPAND | wxALL, 5);

    wxFlexGridSizer* fgSizer126;
    fgSizer126 = new wxFlexGridSizer(1, 0, 0, 0);
    fgSizer126->SetFlexibleDirection(wxBOTH);
    fgSizer126->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer171;
    sbSizer171 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX, wxID_ANY, wxT("MUX Control")), wxVERTICAL);

    wxFlexGridSizer* fgSizer369;
    fgSizer369 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer369->SetFlexibleDirection(wxBOTH);
    fgSizer369->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer3941;
    sbSizer3941 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX, wxID_ANY, wxT("Configuration")), wxVERTICAL);

    ID_NOTEBOOK_MUX_CONTROL_HLMIX_CONFIG = new wxNotebook(ID_PANEL_HLMIX, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_MUX_HLMIX_CONFIG_SEL0 =
        new wxPanel(ID_NOTEBOOK_MUX_CONTROL_HLMIX_CONFIG, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer8822;
    fgSizer8822 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer8822->SetFlexibleDirection(wxBOTH);
    fgSizer8822->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer1602;
    fgSizer1602 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1602->SetFlexibleDirection(wxBOTH);
    fgSizer1602->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer1682;
    sbSizer1682 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer1592;
    fgSizer1592 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer1592->SetFlexibleDirection(wxBOTH);
    fgSizer1592->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkHLMIXx_CONFIG_SEL0_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1592->Add(chkHLMIXx_CONFIG_SEL0_INTERNAL, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL0_INVERT =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1592->Add(chkHLMIXx_CONFIG_SEL0_INVERT, 0, wxALL, 0);

    sbSizer1682->Add(fgSizer1592, 1, wxEXPAND, 5);

    fgSizer1602->Add(sbSizer1682, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer11522;
    sbSizer11522 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8532;
    fgSizer8532 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer8532->SetFlexibleDirection(wxBOTH);
    fgSizer8532->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText26510 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26510->Wrap(-1);
    fgSizer8532->Add(m_staticText26510, 0, wxALL, 5);

    m_staticText26512 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26512->Wrap(-1);
    fgSizer8532->Add(m_staticText26512, 0, wxALL, 5);

    m_staticText26522 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26522->Wrap(-1);
    fgSizer8532->Add(m_staticText26522, 0, wxALL, 5);

    m_staticText26532 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26532->Wrap(-1);
    fgSizer8532->Add(m_staticText26532, 0, wxALL, 5);

    m_staticText26542 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26542->Wrap(-1);
    fgSizer8532->Add(m_staticText26542, 0, wxALL, 5);

    m_staticText26552 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26552->Wrap(-1);
    fgSizer8532->Add(m_staticText26552, 0, wxALL, 5);

    m_staticText26562 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26562->Wrap(-1);
    fgSizer8532->Add(m_staticText26562, 0, wxALL, 5);

    m_staticText26572 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26572->Wrap(-1);
    fgSizer8532->Add(m_staticText26572, 0, wxALL, 5);

    m_staticText26582 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26582->Wrap(-1);
    fgSizer8532->Add(m_staticText26582, 0, wxALL, 5);

    chkHLMIXx_CONFIG_SEL0_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkHLMIXx_CONFIG_SEL0_MASK0, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL0_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkHLMIXx_CONFIG_SEL0_MASK1, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL0_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkHLMIXx_CONFIG_SEL0_MASK2, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL0_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkHLMIXx_CONFIG_SEL0_MASK3, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL0_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkHLMIXx_CONFIG_SEL0_MASK4, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL0_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkHLMIXx_CONFIG_SEL0_MASK5, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL0_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkHLMIXx_CONFIG_SEL0_MASK6, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL0_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkHLMIXx_CONFIG_SEL0_MASK7, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL0_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8532->Add(chkHLMIXx_CONFIG_SEL0_MASK8, 0, wxALL, 0);

    sbSizer11522->Add(fgSizer8532, 1, wxEXPAND, 5);

    fgSizer1602->Add(sbSizer11522, 1, wxEXPAND, 5);

    fgSizer8822->Add(fgSizer1602, 1, wxEXPAND, 5);

    ID_PANEL_MUX_HLMIX_CONFIG_SEL0->SetSizer(fgSizer8822);
    ID_PANEL_MUX_HLMIX_CONFIG_SEL0->Layout();
    fgSizer8822->Fit(ID_PANEL_MUX_HLMIX_CONFIG_SEL0);
    ID_NOTEBOOK_MUX_CONTROL_HLMIX_CONFIG->AddPage(ID_PANEL_MUX_HLMIX_CONFIG_SEL0, wxT("Bit 0"), true);
    ID_PANEL_MUX_HLMIX_CONFIG_SEL1 =
        new wxPanel(ID_NOTEBOOK_MUX_CONTROL_HLMIX_CONFIG, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer88211;
    fgSizer88211 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer88211->SetFlexibleDirection(wxBOTH);
    fgSizer88211->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer16011;
    fgSizer16011 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer16011->SetFlexibleDirection(wxBOTH);
    fgSizer16011->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer16811;
    sbSizer16811 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer15911;
    fgSizer15911 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer15911->SetFlexibleDirection(wxBOTH);
    fgSizer15911->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkHLMIXx_CONFIG_SEL1_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15911->Add(chkHLMIXx_CONFIG_SEL1_INTERNAL, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL1_INVERT =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer15911->Add(chkHLMIXx_CONFIG_SEL1_INVERT, 0, wxALL, 0);

    sbSizer16811->Add(fgSizer15911, 1, wxEXPAND, 5);

    fgSizer16011->Add(sbSizer16811, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer115211;
    sbSizer115211 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer85311;
    fgSizer85311 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer85311->SetFlexibleDirection(wxBOTH);
    fgSizer85311->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText26591 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26591->Wrap(-1);
    fgSizer85311->Add(m_staticText26591, 0, wxALL, 5);

    m_staticText265111 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265111->Wrap(-1);
    fgSizer85311->Add(m_staticText265111, 0, wxALL, 5);

    m_staticText265211 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265211->Wrap(-1);
    fgSizer85311->Add(m_staticText265211, 0, wxALL, 5);

    m_staticText265311 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265311->Wrap(-1);
    fgSizer85311->Add(m_staticText265311, 0, wxALL, 5);

    m_staticText265411 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265411->Wrap(-1);
    fgSizer85311->Add(m_staticText265411, 0, wxALL, 5);

    m_staticText265511 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265511->Wrap(-1);
    fgSizer85311->Add(m_staticText265511, 0, wxALL, 5);

    m_staticText265611 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265611->Wrap(-1);
    fgSizer85311->Add(m_staticText265611, 0, wxALL, 5);

    m_staticText265711 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265711->Wrap(-1);
    fgSizer85311->Add(m_staticText265711, 0, wxALL, 5);

    m_staticText265811 = new wxStaticText(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265811->Wrap(-1);
    fgSizer85311->Add(m_staticText265811, 0, wxALL, 5);

    chkHLMIXx_CONFIG_SEL1_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkHLMIXx_CONFIG_SEL1_MASK0, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL1_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkHLMIXx_CONFIG_SEL1_MASK1, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL1_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkHLMIXx_CONFIG_SEL1_MASK2, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL1_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkHLMIXx_CONFIG_SEL1_MASK3, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL1_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkHLMIXx_CONFIG_SEL1_MASK4, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL1_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkHLMIXx_CONFIG_SEL1_MASK5, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL1_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkHLMIXx_CONFIG_SEL1_MASK6, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL1_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkHLMIXx_CONFIG_SEL1_MASK7, 0, wxALL, 0);

    chkHLMIXx_CONFIG_SEL1_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer85311->Add(chkHLMIXx_CONFIG_SEL1_MASK8, 0, wxALL, 0);

    sbSizer115211->Add(fgSizer85311, 1, wxEXPAND, 5);

    fgSizer16011->Add(sbSizer115211, 1, wxEXPAND, 5);

    fgSizer88211->Add(fgSizer16011, 1, wxEXPAND, 5);

    ID_PANEL_MUX_HLMIX_CONFIG_SEL1->SetSizer(fgSizer88211);
    ID_PANEL_MUX_HLMIX_CONFIG_SEL1->Layout();
    fgSizer88211->Fit(ID_PANEL_MUX_HLMIX_CONFIG_SEL1);
    ID_NOTEBOOK_MUX_CONTROL_HLMIX_CONFIG->AddPage(ID_PANEL_MUX_HLMIX_CONFIG_SEL1, wxT("Bit 1"), false);

    sbSizer3941->Add(ID_NOTEBOOK_MUX_CONTROL_HLMIX_CONFIG, 1, wxEXPAND | wxALL, 5);

    wxStaticBoxSizer* sbSizer3041;
    sbSizer3041 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX, wxID_ANY, wxT("Internal")), wxVERTICAL);

    wxFlexGridSizer* fgSizer3701;
    fgSizer3701 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer3701->SetFlexibleDirection(wxBOTH);
    fgSizer3701->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkHLMIXx_CONFIG_INT_SEL0 = new wxCheckBox(ID_PANEL_HLMIX, ID_CHx_PA_ILIN2X, wxT("Bit 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer3701->Add(chkHLMIXx_CONFIG_INT_SEL0, 0, wxALL, 5);

    chkHLMIXx_CONFIG_INT_SEL1 = new wxCheckBox(ID_PANEL_HLMIX, ID_CHx_PA_ILIN2X, wxT("Bit 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer3701->Add(chkHLMIXx_CONFIG_INT_SEL1, 0, wxALL, 5);

    sbSizer3041->Add(fgSizer3701, 1, wxEXPAND, 5);

    sbSizer3941->Add(sbSizer3041, 0, wxEXPAND, 5);

    fgSizer369->Add(sbSizer3941, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer394;
    sbSizer394 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX, wxID_ANY, wxT("Loss")), wxVERTICAL);

    ID_NOTEBOOK_MUX_CONTROL_HLMIX = new wxNotebook(ID_PANEL_HLMIX, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_MUX_HLMIX_SEL0 =
        new wxPanel(ID_NOTEBOOK_MUX_CONTROL_HLMIX, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer882;
    fgSizer882 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer882->SetFlexibleDirection(wxBOTH);
    fgSizer882->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer160;
    fgSizer160 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer160->SetFlexibleDirection(wxBOTH);
    fgSizer160->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer168;
    sbSizer168 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer159;
    fgSizer159 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer159->SetFlexibleDirection(wxBOTH);
    fgSizer159->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkHLMIXx_LOSS_SEL0_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer159->Add(chkHLMIXx_LOSS_SEL0_INTERNAL, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL0_INVERT =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer159->Add(chkHLMIXx_LOSS_SEL0_INVERT, 0, wxALL, 0);

    sbSizer168->Add(fgSizer159, 1, wxEXPAND, 5);

    fgSizer160->Add(sbSizer168, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer1152;
    sbSizer1152 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer853;
    fgSizer853 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer853->SetFlexibleDirection(wxBOTH);
    fgSizer853->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText265 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText265->Wrap(-1);
    fgSizer853->Add(m_staticText265, 0, wxALL, 5);

    m_staticText2651 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2651->Wrap(-1);
    fgSizer853->Add(m_staticText2651, 0, wxALL, 5);

    m_staticText2652 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2652->Wrap(-1);
    fgSizer853->Add(m_staticText2652, 0, wxALL, 5);

    m_staticText2653 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2653->Wrap(-1);
    fgSizer853->Add(m_staticText2653, 0, wxALL, 5);

    m_staticText2654 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2654->Wrap(-1);
    fgSizer853->Add(m_staticText2654, 0, wxALL, 5);

    m_staticText2655 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2655->Wrap(-1);
    fgSizer853->Add(m_staticText2655, 0, wxALL, 5);

    m_staticText2656 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2656->Wrap(-1);
    fgSizer853->Add(m_staticText2656, 0, wxALL, 5);

    m_staticText2657 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2657->Wrap(-1);
    fgSizer853->Add(m_staticText2657, 0, wxALL, 5);

    m_staticText2658 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL0, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2658->Wrap(-1);
    fgSizer853->Add(m_staticText2658, 0, wxALL, 5);

    chkHLMIXx_LOSS_SEL0_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkHLMIXx_LOSS_SEL0_MASK0, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL0_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkHLMIXx_LOSS_SEL0_MASK1, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL0_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkHLMIXx_LOSS_SEL0_MASK2, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL0_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkHLMIXx_LOSS_SEL0_MASK3, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL0_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkHLMIXx_LOSS_SEL0_MASK4, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL0_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkHLMIXx_LOSS_SEL0_MASK5, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL0_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkHLMIXx_LOSS_SEL0_MASK6, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL0_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkHLMIXx_LOSS_SEL0_MASK7, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL0_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL0, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer853->Add(chkHLMIXx_LOSS_SEL0_MASK8, 0, wxALL, 0);

    sbSizer1152->Add(fgSizer853, 1, wxEXPAND, 5);

    fgSizer160->Add(sbSizer1152, 1, wxEXPAND, 5);

    fgSizer882->Add(fgSizer160, 1, wxEXPAND, 5);

    ID_PANEL_MUX_HLMIX_SEL0->SetSizer(fgSizer882);
    ID_PANEL_MUX_HLMIX_SEL0->Layout();
    fgSizer882->Fit(ID_PANEL_MUX_HLMIX_SEL0);
    ID_NOTEBOOK_MUX_CONTROL_HLMIX->AddPage(ID_PANEL_MUX_HLMIX_SEL0, wxT("Bit 0"), true);
    ID_PANEL_MUX_HLMIX_SEL1 =
        new wxPanel(ID_NOTEBOOK_MUX_CONTROL_HLMIX, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer8821;
    fgSizer8821 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer8821->SetFlexibleDirection(wxBOTH);
    fgSizer8821->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer1601;
    fgSizer1601 = new wxFlexGridSizer(2, 0, 0, 0);
    fgSizer1601->SetFlexibleDirection(wxBOTH);
    fgSizer1601->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer1681;
    sbSizer1681 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxEmptyString), wxVERTICAL);

    wxFlexGridSizer* fgSizer1591;
    fgSizer1591 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer1591->SetFlexibleDirection(wxBOTH);
    fgSizer1591->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkHLMIXx_LOSS_SEL1_INTERNAL =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxT("INTERNAL"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1591->Add(chkHLMIXx_LOSS_SEL1_INTERNAL, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL1_INVERT =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxT("INVERT"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer1591->Add(chkHLMIXx_LOSS_SEL1_INVERT, 0, wxALL, 0);

    sbSizer1681->Add(fgSizer1591, 1, wxEXPAND, 5);

    fgSizer1601->Add(sbSizer1681, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer11521;
    sbSizer11521 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("GPIO Mask")), wxVERTICAL);

    wxFlexGridSizer* fgSizer8531;
    fgSizer8531 = new wxFlexGridSizer(2, 9, 0, 0);
    fgSizer8531->SetFlexibleDirection(wxBOTH);
    fgSizer8531->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText2659 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2659->Wrap(-1);
    fgSizer8531->Add(m_staticText2659, 0, wxALL, 5);

    m_staticText26511 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26511->Wrap(-1);
    fgSizer8531->Add(m_staticText26511, 0, wxALL, 5);

    m_staticText26521 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26521->Wrap(-1);
    fgSizer8531->Add(m_staticText26521, 0, wxALL, 5);

    m_staticText26531 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26531->Wrap(-1);
    fgSizer8531->Add(m_staticText26531, 0, wxALL, 5);

    m_staticText26541 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26541->Wrap(-1);
    fgSizer8531->Add(m_staticText26541, 0, wxALL, 5);

    m_staticText26551 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26551->Wrap(-1);
    fgSizer8531->Add(m_staticText26551, 0, wxALL, 5);

    m_staticText26561 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26561->Wrap(-1);
    fgSizer8531->Add(m_staticText26561, 0, wxALL, 5);

    m_staticText26571 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26571->Wrap(-1);
    fgSizer8531->Add(m_staticText26571, 0, wxALL, 5);

    m_staticText26581 = new wxStaticText(ID_PANEL_MUX_HLMIX_SEL1, wxID_ANY, wxT("8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText26581->Wrap(-1);
    fgSizer8531->Add(m_staticText26581, 0, wxALL, 5);

    chkHLMIXx_LOSS_SEL1_MASK0 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkHLMIXx_LOSS_SEL1_MASK0, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL1_MASK1 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkHLMIXx_LOSS_SEL1_MASK1, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL1_MASK2 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkHLMIXx_LOSS_SEL1_MASK2, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL1_MASK3 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkHLMIXx_LOSS_SEL1_MASK3, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL1_MASK4 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkHLMIXx_LOSS_SEL1_MASK4, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL1_MASK5 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkHLMIXx_LOSS_SEL1_MASK5, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL1_MASK6 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkHLMIXx_LOSS_SEL1_MASK6, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL1_MASK7 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkHLMIXx_LOSS_SEL1_MASK7, 0, wxALL, 0);

    chkHLMIXx_LOSS_SEL1_MASK8 =
        new wxCheckBox(ID_PANEL_MUX_HLMIX_SEL1, ID_CHx_PA_ILIN2X, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    fgSizer8531->Add(chkHLMIXx_LOSS_SEL1_MASK8, 0, wxALL, 0);

    sbSizer11521->Add(fgSizer8531, 1, wxEXPAND, 5);

    fgSizer1601->Add(sbSizer11521, 1, wxEXPAND, 5);

    fgSizer8821->Add(fgSizer1601, 1, wxEXPAND, 5);

    ID_PANEL_MUX_HLMIX_SEL1->SetSizer(fgSizer8821);
    ID_PANEL_MUX_HLMIX_SEL1->Layout();
    fgSizer8821->Fit(ID_PANEL_MUX_HLMIX_SEL1);
    ID_NOTEBOOK_MUX_CONTROL_HLMIX->AddPage(ID_PANEL_MUX_HLMIX_SEL1, wxT("Bit 1"), false);

    sbSizer394->Add(ID_NOTEBOOK_MUX_CONTROL_HLMIX, 1, wxEXPAND | wxALL, 5);

    wxStaticBoxSizer* sbSizer304;
    sbSizer304 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_HLMIX, wxID_ANY, wxT("Internal")), wxVERTICAL);

    wxFlexGridSizer* fgSizer370;
    fgSizer370 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer370->SetFlexibleDirection(wxBOTH);
    fgSizer370->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkHLMIXx_LOSS_INT_SEL0 = new wxCheckBox(ID_PANEL_HLMIX, ID_CHx_PA_ILIN2X, wxT("Bit 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer370->Add(chkHLMIXx_LOSS_INT_SEL0, 0, wxALL, 5);

    chkHLMIXx_LOSS_INT_SEL1 = new wxCheckBox(ID_PANEL_HLMIX, ID_CHx_PA_ILIN2X, wxT("Bit 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer370->Add(chkHLMIXx_LOSS_INT_SEL1, 0, wxALL, 5);

    sbSizer304->Add(fgSizer370, 1, wxEXPAND, 5);

    sbSizer394->Add(sbSizer304, 0, wxEXPAND, 5);

    fgSizer369->Add(sbSizer394, 1, wxEXPAND, 5);

    sbSizer171->Add(fgSizer369, 1, wxEXPAND, 5);

    fgSizer126->Add(sbSizer171, 1, wxEXPAND, 5);

    fgSizer69->Add(fgSizer126, 1, wxEXPAND, 5);

    ID_PANEL_HLMIX->SetSizer(fgSizer69);
    ID_PANEL_HLMIX->Layout();
    fgSizer69->Fit(ID_PANEL_HLMIX);
    ID_NOTEBOOK_HLMIX->AddPage(ID_PANEL_HLMIX, wxT("HLMIX"), false);

    fgSizer68->Add(ID_NOTEBOOK_HLMIX, 1, wxEXPAND | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    this->SetSizer(fgSizer68);
    this->Layout();
    fgSizer68->Fit(this);

    // Connect Events
    cmbHLMIXx_VGCAS0->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS0->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS0->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS0->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS0->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS0->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_BIAS_PD0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOBUF_PD0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS0->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS0->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS0->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS1->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS1->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS1->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS1->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS1->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS1->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_BIAS_PD1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOBUF_PD1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS1->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS1->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS1->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS2->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS2->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS2->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS2->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS2->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS2->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_BIAS_PD2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOBUF_PD2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS2->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS2->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS2->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS3->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS3->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS3->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS3->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS3->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS3->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_BIAS_PD3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOBUF_PD3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS3->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS3->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS3->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS_RB->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS_RB->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_VGCAS_RB->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS_RB->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS_RB->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_ICT_BIAS_RB->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_BIAS_PD_RB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOBUF_PD_RB->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS_RB->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS_RB->Connect(
        wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    cmbHLMIXx_MIXLOSS_RB->Connect(
        wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL0_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_SEL1_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_INT_SEL0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_CONFIG_INT_SEL1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL0_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_INTERNAL->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_INVERT->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_MASK0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_MASK1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_MASK2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_MASK3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_MASK4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_MASK5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_MASK6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_MASK7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_SEL1_MASK8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_INT_SEL0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);
    chkHLMIXx_LOSS_INT_SEL1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlHLMIX_view::ParameterChangeHandler), NULL, this);

    wndId2Enum[cmbHLMIXx_VGCAS0] = HLMIXx_VGCAS0;
    wndId2Enum[cmbHLMIXx_ICT_BIAS0] = HLMIXx_ICT_BIAS0;
    wndId2Enum[chkHLMIXx_BIAS_PD0] = HLMIXx_BIAS_PD0;
    wndId2Enum[chkHLMIXx_LOBUF_PD0] = HLMIXx_LOBUF_PD0;
    wndId2Enum[cmbHLMIXx_MIXLOSS0] = HLMIXx_MIXLOSS0;

    wndId2Enum[cmbHLMIXx_VGCAS1] = HLMIXx_VGCAS1;
    wndId2Enum[cmbHLMIXx_ICT_BIAS1] = HLMIXx_ICT_BIAS1;
    wndId2Enum[chkHLMIXx_BIAS_PD1] = HLMIXx_BIAS_PD1;
    wndId2Enum[chkHLMIXx_LOBUF_PD1] = HLMIXx_LOBUF_PD1;
    wndId2Enum[cmbHLMIXx_MIXLOSS1] = HLMIXx_MIXLOSS1;

    wndId2Enum[cmbHLMIXx_VGCAS2] = HLMIXx_VGCAS2;
    wndId2Enum[cmbHLMIXx_ICT_BIAS2] = HLMIXx_ICT_BIAS2;
    wndId2Enum[chkHLMIXx_BIAS_PD2] = HLMIXx_BIAS_PD2;
    wndId2Enum[chkHLMIXx_LOBUF_PD2] = HLMIXx_LOBUF_PD2;
    wndId2Enum[cmbHLMIXx_MIXLOSS2] = HLMIXx_MIXLOSS2;

    wndId2Enum[cmbHLMIXx_VGCAS3] = HLMIXx_VGCAS3;
    wndId2Enum[cmbHLMIXx_ICT_BIAS3] = HLMIXx_ICT_BIAS3;
    wndId2Enum[chkHLMIXx_BIAS_PD3] = HLMIXx_BIAS_PD3;
    wndId2Enum[chkHLMIXx_LOBUF_PD3] = HLMIXx_LOBUF_PD3;
    wndId2Enum[cmbHLMIXx_MIXLOSS3] = HLMIXx_MIXLOSS3;

    wndId2Enum[cmbHLMIXx_VGCAS_RB] = HLMIXx_VGCAS_RB;
    wndId2Enum[cmbHLMIXx_ICT_BIAS_RB] = HLMIXx_ICT_BIAS_RB;
    wndId2Enum[chkHLMIXx_BIAS_PD_RB] = HLMIXx_BIAS_PD_RB;
    wndId2Enum[chkHLMIXx_LOBUF_PD_RB] = HLMIXx_LOBUF_PD_RB;
    wndId2Enum[cmbHLMIXx_MIXLOSS_RB] = HLMIXx_MIXLOSS_RB;

    wndId2Enum[chkHLMIXx_CONFIG_SEL0_INTERNAL] = HLMIXx_CONFIG_SEL0_INTERNAL;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_INVERT] = HLMIXx_CONFIG_SEL0_INVERT;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_MASK0] = HLMIXx_CONFIG_SEL0_MASK0;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_MASK1] = HLMIXx_CONFIG_SEL0_MASK1;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_MASK2] = HLMIXx_CONFIG_SEL0_MASK2;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_MASK3] = HLMIXx_CONFIG_SEL0_MASK3;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_MASK4] = HLMIXx_CONFIG_SEL0_MASK4;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_MASK5] = HLMIXx_CONFIG_SEL0_MASK5;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_MASK6] = HLMIXx_CONFIG_SEL0_MASK6;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_MASK7] = HLMIXx_CONFIG_SEL0_MASK7;
    wndId2Enum[chkHLMIXx_CONFIG_SEL0_MASK8] = HLMIXx_CONFIG_SEL0_MASK8;

    wndId2Enum[chkHLMIXx_CONFIG_SEL1_INTERNAL] = HLMIXx_CONFIG_SEL1_INTERNAL;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_INVERT] = HLMIXx_CONFIG_SEL1_INVERT;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_MASK0] = HLMIXx_CONFIG_SEL1_MASK0;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_MASK1] = HLMIXx_CONFIG_SEL1_MASK1;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_MASK2] = HLMIXx_CONFIG_SEL1_MASK2;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_MASK3] = HLMIXx_CONFIG_SEL1_MASK3;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_MASK4] = HLMIXx_CONFIG_SEL1_MASK4;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_MASK5] = HLMIXx_CONFIG_SEL1_MASK5;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_MASK6] = HLMIXx_CONFIG_SEL1_MASK6;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_MASK7] = HLMIXx_CONFIG_SEL1_MASK7;
    wndId2Enum[chkHLMIXx_CONFIG_SEL1_MASK8] = HLMIXx_CONFIG_SEL1_MASK8;

    wndId2Enum[chkHLMIXx_CONFIG_INT_SEL0] = HLMIXx_CONFIG_INT_SEL0;
    wndId2Enum[chkHLMIXx_CONFIG_INT_SEL1] = HLMIXx_CONFIG_INT_SEL1;

    wndId2Enum[chkHLMIXx_LOSS_SEL0_INTERNAL] = HLMIXx_LOSS_SEL0_INTERNAL;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_INVERT] = HLMIXx_LOSS_SEL0_INVERT;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_MASK0] = HLMIXx_LOSS_SEL0_MASK0;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_MASK1] = HLMIXx_LOSS_SEL0_MASK1;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_MASK2] = HLMIXx_LOSS_SEL0_MASK2;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_MASK3] = HLMIXx_LOSS_SEL0_MASK3;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_MASK4] = HLMIXx_LOSS_SEL0_MASK4;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_MASK5] = HLMIXx_LOSS_SEL0_MASK5;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_MASK6] = HLMIXx_LOSS_SEL0_MASK6;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_MASK7] = HLMIXx_LOSS_SEL0_MASK7;
    wndId2Enum[chkHLMIXx_LOSS_SEL0_MASK8] = HLMIXx_LOSS_SEL0_MASK8;

    wndId2Enum[chkHLMIXx_LOSS_SEL1_INTERNAL] = HLMIXx_LOSS_SEL1_INTERNAL;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_INVERT] = HLMIXx_LOSS_SEL1_INVERT;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_MASK0] = HLMIXx_LOSS_SEL1_MASK0;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_MASK1] = HLMIXx_LOSS_SEL1_MASK1;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_MASK2] = HLMIXx_LOSS_SEL1_MASK2;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_MASK3] = HLMIXx_LOSS_SEL1_MASK3;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_MASK4] = HLMIXx_LOSS_SEL1_MASK4;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_MASK5] = HLMIXx_LOSS_SEL1_MASK5;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_MASK6] = HLMIXx_LOSS_SEL1_MASK6;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_MASK7] = HLMIXx_LOSS_SEL1_MASK7;
    wndId2Enum[chkHLMIXx_LOSS_SEL1_MASK8] = HLMIXx_LOSS_SEL1_MASK8;

    wndId2Enum[chkHLMIXx_LOSS_INT_SEL0] = HLMIXx_LOSS_INT_SEL0;
    wndId2Enum[chkHLMIXx_LOSS_INT_SEL1] = HLMIXx_LOSS_INT_SEL1;

    LMS8001_WXGUI::UpdateTooltips(wndId2Enum, true);
}
