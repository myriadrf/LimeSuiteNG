
#include "lms8001_pnlConfig_view.h"
#include "lms8001_gui_utilities.h"
#include <iostream>
#include <wx/event.h>

#include <map>
#include <chrono>
#include <thread>

#include "chips/LMS8001/LMS8001.h"
#include "chips/LMS8001/LMS8001_parameters.h"

#include "dlgTempCalibrate.h"

using namespace lime;

lms8001_pnlConfig_view::lms8001_pnlConfig_view(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ILMS8001Tab(parent, id, pos, size, style)
{
    wxFlexGridSizer* fgSizer68;
    fgSizer68 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer68->SetFlexibleDirection(wxBOTH);
    fgSizer68->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    ID_NOTEBOOK_CONFIG = new wxNotebook(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);
    ID_PANEL_CONFIG = new wxPanel(ID_NOTEBOOK_CONFIG, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxFlexGridSizer* fgSizer22;
    fgSizer22 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer22->SetFlexibleDirection(wxBOTH);
    fgSizer22->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer20;
    sbSizer20 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO")), wxHORIZONTAL);

    wxStaticBoxSizer* sbSizer211;
    sbSizer211 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Value OUT")), wxVERTICAL);

    wxFlexGridSizer* fgSizer241;
    fgSizer241 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer241->SetFlexibleDirection(wxBOTH);
    fgSizer241->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkGPIO_OUT_SPI_0 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_OUT_SPI_0, wxT("GPIO 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241->Add(chkGPIO_OUT_SPI_0, 0, wxALL, 4);

    chkGPIO_OUT_SPI_1 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_OUT_SPI_1, wxT("GPIO 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241->Add(chkGPIO_OUT_SPI_1, 0, wxALL, 4);

    chkGPIO_OUT_SPI_2 = new wxCheckBox(ID_PANEL_CONFIG, IID_GPIO_OUT_SPI_2, wxT("GPIO 2"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241->Add(chkGPIO_OUT_SPI_2, 0, wxALL, 4);

    chkGPIO_OUT_SPI_3 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_OUT_SPI_3, wxT("GPIO 3"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241->Add(chkGPIO_OUT_SPI_3, 0, wxALL, 4);

    chkGPIO_OUT_SPI_4 = new wxCheckBox(ID_PANEL_CONFIG, IID_GPIO_OUT_SPI_4, wxT("GPIO 4"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241->Add(chkGPIO_OUT_SPI_4, 0, wxALL, 4);

    chkGPIO_OUT_SPI_5 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_OUT_SPI_5, wxT("GPIO 5"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241->Add(chkGPIO_OUT_SPI_5, 0, wxALL, 4);

    chkGPIO_OUT_SPI_6 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_OUT_SPI_6, wxT("GPIO 6"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241->Add(chkGPIO_OUT_SPI_6, 0, wxALL, 4);

    chkGPIO_OUT_SPI_7 = new wxCheckBox(ID_PANEL_CONFIG, IID_GPIO_OUT_SPI_7, wxT("GPIO 7"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241->Add(chkGPIO_OUT_SPI_7, 0, wxALL, 4);

    chkGPIO_OUT_SPI_8 = new wxCheckBox(ID_PANEL_CONFIG, IID_GPIO_OUT_SPI_8, wxT("GPIO 8"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241->Add(chkGPIO_OUT_SPI_8, 0, wxALL, 4);

    sbSizer211->Add(fgSizer241, 0, wxEXPAND, 0);

    sbSizer20->Add(sbSizer211, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer21;
    sbSizer21 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Source select")), wxVERTICAL);

    wxFlexGridSizer* fgSizer24;
    fgSizer24 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer24->SetFlexibleDirection(wxBOTH);
    fgSizer24->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText1 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO 0"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1->Wrap(-1);
    fgSizer24->Add(m_staticText1, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT, 0);

    cmbGPIO0_SEL = new wxComboBox(ID_PANEL_CONFIG, ID_GPIO0_SEL, wxT("from SPI"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbGPIO0_SEL->Append(wxT("from SPI"));
    cmbGPIO0_SEL->Append(wxT("PLL_LOCK"));
    cmbGPIO0_SEL->Append(wxT("VTUNE_LOW"));
    cmbGPIO0_SEL->Append(wxT("VTUNE_HIGH"));
    cmbGPIO0_SEL->Append(wxT("Fast lock active"));
    cmbGPIO0_SEL->SetSelection(0);
    fgSizer24->Add(cmbGPIO0_SEL, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, 0);

    m_staticText11 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO 1"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText11->Wrap(-1);
    fgSizer24->Add(m_staticText11, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 0);

    cmbGPIO1_SEL = new wxComboBox(ID_PANEL_CONFIG, ID_GPIO0_SEL, wxT("from SPI"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbGPIO1_SEL->Append(wxT("from SPI"));
    cmbGPIO1_SEL->Append(wxT("PLL_LOCK"));
    cmbGPIO1_SEL->Append(wxT("VTUNE_LOW"));
    cmbGPIO1_SEL->Append(wxT("VTUNE_HIGH"));
    cmbGPIO1_SEL->Append(wxT("Fast lock active"));
    cmbGPIO1_SEL->SetSelection(0);
    fgSizer24->Add(cmbGPIO1_SEL, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, 0);

    m_staticText111 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO 2"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText111->Wrap(-1);
    fgSizer24->Add(m_staticText111, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    cmbGPIO2_SEL = new wxComboBox(ID_PANEL_CONFIG, ID_GPIO0_SEL, wxT("from SPI"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbGPIO2_SEL->Append(wxT("from SPI"));
    cmbGPIO2_SEL->Append(wxT("PLL_LOCK"));
    cmbGPIO2_SEL->Append(wxT("VTUNE_LOW"));
    cmbGPIO2_SEL->Append(wxT("VTUNE_HIGH"));
    cmbGPIO2_SEL->Append(wxT("Fast lock active"));
    cmbGPIO2_SEL->SetSelection(0);
    fgSizer24->Add(cmbGPIO2_SEL, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, 0);

    m_staticText1111 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO 3"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1111->Wrap(-1);
    fgSizer24->Add(m_staticText1111, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    cmbGPIO3_SEL = new wxComboBox(ID_PANEL_CONFIG, ID_GPIO0_SEL, wxT("from SPI"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbGPIO3_SEL->Append(wxT("from SPI"));
    cmbGPIO3_SEL->Append(wxT("PLL_LOCK"));
    cmbGPIO3_SEL->Append(wxT("VTUNE_LOW"));
    cmbGPIO3_SEL->Append(wxT("VTUNE_HIGH"));
    cmbGPIO3_SEL->Append(wxT("Fast lock active"));
    cmbGPIO3_SEL->SetSelection(0);
    fgSizer24->Add(cmbGPIO3_SEL, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, 0);

    m_staticText11111 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO 4"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText11111->Wrap(-1);
    fgSizer24->Add(m_staticText11111, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    cmbGPIO4_SEL = new wxComboBox(ID_PANEL_CONFIG, ID_GPIO0_SEL, wxT("from SPI"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbGPIO4_SEL->Append(wxT("from SPI"));
    cmbGPIO4_SEL->Append(wxT("PLL_LOCK"));
    cmbGPIO4_SEL->Append(wxT("VTUNE_LOW"));
    cmbGPIO4_SEL->Append(wxT("VTUNE_HIGH"));
    cmbGPIO4_SEL->Append(wxT("Fast lock active"));
    cmbGPIO4_SEL->SetSelection(0);
    fgSizer24->Add(cmbGPIO4_SEL, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, 0);

    m_staticText111111 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO 5"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText111111->Wrap(-1);
    fgSizer24->Add(m_staticText111111, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    cmbGPIO5_SEL = new wxComboBox(ID_PANEL_CONFIG, ID_GPIO0_SEL, wxT("from SPI"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbGPIO5_SEL->Append(wxT("from SPI"));
    cmbGPIO5_SEL->Append(wxT("PLL_LOCK"));
    cmbGPIO5_SEL->Append(wxT("VTUNE_LOW"));
    cmbGPIO5_SEL->Append(wxT("VTUNE_HIGH"));
    cmbGPIO5_SEL->Append(wxT("Fast lock active"));
    cmbGPIO5_SEL->SetSelection(0);
    fgSizer24->Add(cmbGPIO5_SEL, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, 0);

    m_staticText1111111 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO 6"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText1111111->Wrap(-1);
    fgSizer24->Add(m_staticText1111111, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    cmbGPIO6_SEL = new wxComboBox(ID_PANEL_CONFIG, ID_GPIO0_SEL, wxT("from SPI"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbGPIO6_SEL->Append(wxT("from SPI"));
    cmbGPIO6_SEL->Append(wxT("PLL_LOCK"));
    cmbGPIO6_SEL->Append(wxT("VTUNE_LOW"));
    cmbGPIO6_SEL->Append(wxT("VTUNE_HIGH"));
    cmbGPIO6_SEL->Append(wxT("Fast lock active"));
    cmbGPIO6_SEL->SetSelection(0);
    fgSizer24->Add(cmbGPIO6_SEL, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, 0);

    m_staticText11111111 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO 7"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText11111111->Wrap(-1);
    fgSizer24->Add(m_staticText11111111, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    cmbGPIO7_SEL = new wxComboBox(ID_PANEL_CONFIG, ID_GPIO0_SEL, wxT("from SPI"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbGPIO7_SEL->Append(wxT("from SPI"));
    cmbGPIO7_SEL->Append(wxT("PLL_LOCK"));
    cmbGPIO7_SEL->Append(wxT("VTUNE_LOW"));
    cmbGPIO7_SEL->Append(wxT("VTUNE_HIGH"));
    cmbGPIO7_SEL->Append(wxT("Fast lock active"));
    cmbGPIO7_SEL->SetSelection(0);
    fgSizer24->Add(cmbGPIO7_SEL, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, 0);

    m_staticText111111111 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("GPIO 8"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText111111111->Wrap(-1);
    fgSizer24->Add(m_staticText111111111, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL | wxALL, 0);

    cmbGPIO8_SEL = new wxComboBox(ID_PANEL_CONFIG, ID_GPIO0_SEL, wxT("from SPI"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0);
    cmbGPIO8_SEL->Append(wxT("from SPI"));
    cmbGPIO8_SEL->Append(wxT("PLL_LOCK"));
    cmbGPIO8_SEL->Append(wxT("VTUNE_LOW"));
    cmbGPIO8_SEL->Append(wxT("VTUNE_HIGH"));
    cmbGPIO8_SEL->Append(wxT("Fast lock active"));
    cmbGPIO8_SEL->SetSelection(0);
    fgSizer24->Add(cmbGPIO8_SEL, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, 0);

    sbSizer21->Add(fgSizer24, 0, wxEXPAND, 0);

    sbSizer20->Add(sbSizer21, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL | wxEXPAND, 0);

    wxStaticBoxSizer* sbSizer2111;
    sbSizer2111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Value IN")), wxVERTICAL);

    wxFlexGridSizer* fgSizer2411;
    fgSizer2411 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer2411->SetFlexibleDirection(wxBOTH);
    fgSizer2411->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkGPIO_IN_0 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_IN_0, wxT("GPIO 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411->Add(chkGPIO_IN_0, 0, wxALL, 4);

    chkGPIO_IN_1 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_IN_1, wxT("GPIO 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411->Add(chkGPIO_IN_1, 0, wxALL, 4);

    chkGPIO_IN_2 = new wxCheckBox(ID_PANEL_CONFIG, IID_GPIO_IN_2, wxT("GPIO 2"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411->Add(chkGPIO_IN_2, 0, wxALL, 4);

    chkGPIO_IN_3 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_IN_3, wxT("GPIO 3"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411->Add(chkGPIO_IN_3, 0, wxALL, 4);

    chkGPIO_IN_4 = new wxCheckBox(ID_PANEL_CONFIG, IID_GPIO_IN_4, wxT("GPIO 4"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411->Add(chkGPIO_IN_4, 0, wxALL, 4);

    chkGPIO_IN_5 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_IN_5, wxT("GPIO 5"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411->Add(chkGPIO_IN_5, 0, wxALL, 4);

    chkGPIO_IN_6 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_IN_6, wxT("GPIO 6"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411->Add(chkGPIO_IN_6, 0, wxALL, 4);

    chkGPIO_IN_7 = new wxCheckBox(ID_PANEL_CONFIG, IID_GPIO_IN_7, wxT("GPIO 7"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411->Add(chkGPIO_IN_7, 0, wxALL, 4);

    chkGPIO_IN_8 = new wxCheckBox(ID_PANEL_CONFIG, IID_GPIO_IN_8, wxT("GPIO 8"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411->Add(chkGPIO_IN_8, 0, wxALL, 4);

    sbSizer2111->Add(fgSizer2411, 0, wxEXPAND, 0);

    sbSizer20->Add(sbSizer2111, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer21111;
    sbSizer21111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Pull-up")), wxVERTICAL);

    wxFlexGridSizer* fgSizer24111;
    fgSizer24111 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer24111->SetFlexibleDirection(wxBOTH);
    fgSizer24111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkGPIO_PE_0 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_PE_0, wxT("GPIO 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer24111->Add(chkGPIO_PE_0, 0, wxALL, 4);

    chkGPIO_PE_1 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_PE_1, wxT("GPIO 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer24111->Add(chkGPIO_PE_1, 0, wxALL, 4);

    chkGPIO_PE_2 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_PE_2, wxT("GPIO 2"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer24111->Add(chkGPIO_PE_2, 0, wxALL, 4);

    chkGPIO_PE_3 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_PE_3, wxT("GPIO 3"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer24111->Add(chkGPIO_PE_3, 0, wxALL, 4);

    chkGPIO_PE_4 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_PE_4, wxT("GPIO 4"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer24111->Add(chkGPIO_PE_4, 0, wxALL, 4);

    chkGPIO_PE_5 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_PE_5, wxT("GPIO 5"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer24111->Add(chkGPIO_PE_5, 0, wxALL, 4);

    chkGPIO_PE_6 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_PE_6, wxT("GPIO 6"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer24111->Add(chkGPIO_PE_6, 0, wxALL, 4);

    chkGPIO_PE_7 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_PE_7, wxT("GPIO 7"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer24111->Add(chkGPIO_PE_7, 0, wxALL, 4);

    chkGPIO_PE_8 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_PE_8, wxT("GPIO 8"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer24111->Add(chkGPIO_PE_8, 0, wxALL, 4);

    sbSizer21111->Add(fgSizer24111, 0, wxEXPAND, 0);

    sbSizer20->Add(sbSizer21111, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer211111;
    sbSizer211111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Driver strength")), wxVERTICAL);

    wxFlexGridSizer* fgSizer241111;
    fgSizer241111 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer241111->SetFlexibleDirection(wxBOTH);
    fgSizer241111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkGPIO_DS_0 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_DS_0, wxT("GPIO 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241111->Add(chkGPIO_DS_0, 0, wxALL, 4);

    chkGPIO_DS_1 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_DS_1, wxT("GPIO 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241111->Add(chkGPIO_DS_1, 0, wxALL, 4);

    chkGPIO_DS_2 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_DS_2, wxT("GPIO 2"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241111->Add(chkGPIO_DS_2, 0, wxALL, 4);

    chkGPIO_DS_3 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_DS_3, wxT("GPIO 3"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241111->Add(chkGPIO_DS_3, 0, wxALL, 4);

    chkGPIO_DS_4 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_DS_4, wxT("GPIO 4"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241111->Add(chkGPIO_DS_4, 0, wxALL, 4);

    chkGPIO_DS_5 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_DS_5, wxT("GPIO 5"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241111->Add(chkGPIO_DS_5, 0, wxALL, 4);

    chkGPIO_DS_6 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_DS_6, wxT("GPIO 6"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241111->Add(chkGPIO_DS_6, 0, wxALL, 4);

    chkGPIO_DS_7 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_DS_7, wxT("GPIO 7"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241111->Add(chkGPIO_DS_7, 0, wxALL, 4);

    chkGPIO_DS_8 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_DS_8, wxT("GPIO 8"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer241111->Add(chkGPIO_DS_8, 0, wxALL, 4);

    sbSizer211111->Add(fgSizer241111, 0, wxEXPAND, 0);

    m_staticText37 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("0 - 4 mA"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText37->Wrap(-1);
    sbSizer211111->Add(m_staticText37, 0, wxALL, 4);

    m_staticText372 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("1 - 8 mA"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText372->Wrap(-1);
    sbSizer211111->Add(m_staticText372, 0, wxALL, 5);

    sbSizer20->Add(sbSizer211111, 1, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer2111111;
    sbSizer2111111 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Input/Output")), wxVERTICAL);

    wxFlexGridSizer* fgSizer2411111;
    fgSizer2411111 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer2411111->SetFlexibleDirection(wxBOTH);
    fgSizer2411111->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkGPIO_InO_0 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_InO_0, wxT("GPIO 0"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411111->Add(chkGPIO_InO_0, 0, wxALL, 4);

    chkGPIO_InO_1 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_InO_1, wxT("GPIO 1"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411111->Add(chkGPIO_InO_1, 0, wxALL, 4);

    chkGPIO_InO_2 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_InO_2, wxT("GPIO 2"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411111->Add(chkGPIO_InO_2, 0, wxALL, 4);

    chkGPIO_InO_3 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_InO_3, wxT("GPIO 3"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411111->Add(chkGPIO_InO_3, 0, wxALL, 4);

    chkGPIO_InO_4 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_InO_4, wxT("GPIO 4"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411111->Add(chkGPIO_InO_4, 0, wxALL, 4);

    chkGPIO_InO_5 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_InO_5, wxT("GPIO 5"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411111->Add(chkGPIO_InO_5, 0, wxALL, 4);

    chkGPIO_InO_6 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_InO_6, wxT("GPIO 6"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411111->Add(chkGPIO_InO_6, 0, wxALL, 4);

    chkGPIO_InO_7 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_InO_7, wxT("GPIO 7"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411111->Add(chkGPIO_InO_7, 0, wxALL, 4);

    chkGPIO_InO_8 = new wxCheckBox(ID_PANEL_CONFIG, ID_GPIO_InO_8, wxT("GPIO 8"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer2411111->Add(chkGPIO_InO_8, 0, wxALL, 4);

    sbSizer2111111->Add(fgSizer2411111, 0, wxEXPAND, 0);

    m_staticText371 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("0 - Out"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText371->Wrap(-1);
    sbSizer2111111->Add(m_staticText371, 0, wxALL, 4);

    m_staticText3711 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("1 - In"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText3711->Wrap(-1);
    sbSizer2111111->Add(m_staticText3711, 0, wxALL, 5);

    sbSizer20->Add(sbSizer2111111, 1, wxEXPAND, 5);

    fgSizer22->Add(sbSizer20, 0, wxEXPAND, 0);

    wxFlexGridSizer* fgSizer27;
    fgSizer27 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer27->AddGrowableCol(2);
    fgSizer27->SetFlexibleDirection(wxBOTH);
    fgSizer27->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer11;
    sbSizer11 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("SPI")), wxVERTICAL);

    wxFlexGridSizer* fgSizer143;
    fgSizer143 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer143->SetFlexibleDirection(wxBOTH);
    fgSizer143->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizer7;
    sbSizer7 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Driver strength")), wxVERTICAL);

    wxFlexGridSizer* fgSizer69;
    fgSizer69 = new wxFlexGridSizer(0, 4, 0, 0);
    fgSizer69->SetFlexibleDirection(wxBOTH);
    fgSizer69->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxString rgrSPI_SDIO_DSChoices[] = { wxT("4 mA"), wxT("8 mA") };
    int rgrSPI_SDIO_DSNChoices = sizeof(rgrSPI_SDIO_DSChoices) / sizeof(wxString);
    rgrSPI_SDIO_DS = new wxRadioBox(ID_PANEL_CONFIG,
        ID_SPI_SDIO_DS,
        wxT("SDIO pad"),
        wxDefaultPosition,
        wxDefaultSize,
        rgrSPI_SDIO_DSNChoices,
        rgrSPI_SDIO_DSChoices,
        1,
        wxRA_SPECIFY_COLS);
    rgrSPI_SDIO_DS->SetSelection(0);
    fgSizer69->Add(rgrSPI_SDIO_DS, 1, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL | wxEXPAND, 0);

    wxString rgrSPI_SDO_DSChoices[] = { wxT("4 mA"), wxT("8 mA") };
    int rgrSPI_SDO_DSNChoices = sizeof(rgrSPI_SDO_DSChoices) / sizeof(wxString);
    rgrSPI_SDO_DS = new wxRadioBox(ID_PANEL_CONFIG,
        ID_SPI_SDO_DS,
        wxT("SDO pad"),
        wxPoint(-1, -1),
        wxDefaultSize,
        rgrSPI_SDO_DSNChoices,
        rgrSPI_SDO_DSChoices,
        1,
        wxRA_SPECIFY_COLS);
    rgrSPI_SDO_DS->SetSelection(0);
    fgSizer69->Add(rgrSPI_SDO_DS, 1, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL | wxEXPAND, 0);

    sbSizer7->Add(fgSizer69, 1, wxEXPAND, 5);

    fgSizer143->Add(sbSizer7, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer144;
    fgSizer144 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer144->SetFlexibleDirection(wxBOTH);
    fgSizer144->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* sbSizerEngagePullUp;
    sbSizerEngagePullUp = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Pull-up")), wxHORIZONTAL);

    wxFlexGridSizer* fgSizer691;
    fgSizer691 = new wxFlexGridSizer(3, 0, 0, 0);
    fgSizer691->SetFlexibleDirection(wxBOTH);
    fgSizer691->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkSPI_SDIO_PE = new wxCheckBox(ID_PANEL_CONFIG, ID_SPI_SDIO_PE, wxT("SDIO pad"), wxDefaultPosition, wxDefaultSize, 0);
    chkSPI_SDIO_PE->SetValue(true);
    fgSizer691->Add(chkSPI_SDIO_PE, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL | wxEXPAND, 0);

    chkSPI_SDO_PE = new wxCheckBox(ID_PANEL_CONFIG, ID_SPI_SDO_PE, wxT("SDO pad"), wxDefaultPosition, wxDefaultSize, 0);
    chkSPI_SDO_PE->SetValue(true);
    fgSizer691->Add(chkSPI_SDO_PE, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL | wxEXPAND, 0);

    chkSPI_SCLK_PE = new wxCheckBox(ID_PANEL_CONFIG, ID_SPI_SDO_PE, wxT("SCLK pad"), wxDefaultPosition, wxDefaultSize, 0);
    chkSPI_SCLK_PE->SetValue(true);
    fgSizer691->Add(chkSPI_SCLK_PE, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL | wxEXPAND, 0);

    chkSPI_SEN_PE = new wxCheckBox(ID_PANEL_CONFIG, ID_SPI_SEN_PE, wxT("SEN pad"), wxDefaultPosition, wxDefaultSize, 0);
    chkSPI_SEN_PE->SetValue(true);
    fgSizer691->Add(chkSPI_SEN_PE, 0, wxALL, 0);

    sbSizerEngagePullUp->Add(fgSizer691, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL | wxEXPAND, 0);

    fgSizer144->Add(sbSizerEngagePullUp, 0, wxALIGN_LEFT | wxALIGN_TOP | wxEXPAND, 5);

    wxFlexGridSizer* fgSizer692;
    fgSizer692 = new wxFlexGridSizer(0, 0, 0, 0);
    fgSizer692->AddGrowableCol(0);
    fgSizer692->SetFlexibleDirection(wxBOTH);
    fgSizer692->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxString rgrSPIMODEChoices[] = { wxT("3 wire"), wxT("4 wire") };
    int rgrSPIMODENChoices = sizeof(rgrSPIMODEChoices) / sizeof(wxString);
    rgrSPIMODE = new wxRadioBox(ID_PANEL_CONFIG,
        ID_SPI_SDIO_DS,
        wxT("SPI Mode"),
        wxDefaultPosition,
        wxDefaultSize,
        rgrSPIMODENChoices,
        rgrSPIMODEChoices,
        2,
        wxRA_SPECIFY_COLS);
    rgrSPIMODE->SetSelection(1);
    fgSizer692->Add(rgrSPIMODE, 0, wxALIGN_LEFT | wxEXPAND, 0);

    fgSizer144->Add(fgSizer692, 0, wxEXPAND, 0);

    fgSizer143->Add(fgSizer144, 1, wxEXPAND, 5);

    sbSizer11->Add(fgSizer143, 0, wxEXPAND, 5);

    fgSizer27->Add(sbSizer11, 0, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer28;
    sbSizer28 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Temp. Sensor")), wxVERTICAL);

    wxStaticBoxSizer* sbSizer29;
    sbSizer29 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Control")), wxVERTICAL);

    wxFlexGridSizer* fgSizer26;
    fgSizer26 = new wxFlexGridSizer(3, 0, 0, 0);
    fgSizer26->SetFlexibleDirection(wxBOTH);
    fgSizer26->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    chkTEMP_SENS_EN = new wxCheckBox(ID_PANEL_CONFIG, ID_TEMP_SENS_EN, wxT("Enable"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer26->Add(chkTEMP_SENS_EN, 0, wxALL, 0);

    chkTEMP_SENS_CLKEN =
        new wxCheckBox(ID_PANEL_CONFIG, ID_TEMP_SENS_CLKEN, wxT("Clock enable"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer26->Add(chkTEMP_SENS_CLKEN, 0, wxALL, 0);

    chkTEMP_START_CONV = new wxCheckBox(ID_PANEL_CONFIG, ID_TEMP_START_CONV, wxT("Start"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer26->Add(chkTEMP_START_CONV, 0, wxALL, 0);

    sbSizer29->Add(fgSizer26, 1, wxALIGN_TOP | wxEXPAND, 5);

    sbSizer28->Add(sbSizer29, 0, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer154;
    fgSizer154 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer154->SetFlexibleDirection(wxBOTH);
    fgSizer154->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer153;
    fgSizer153 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer153->SetFlexibleDirection(wxBOTH);
    fgSizer153->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText280 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("Value:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText280->Wrap(-1);
    fgSizer153->Add(m_staticText280, 0, wxALL, 5);

    sttxtTEMP_READ = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("000"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtTEMP_READ->Wrap(-1);
    fgSizer153->Add(sttxtTEMP_READ, 0, wxLEFT | wxTOP, 5);

    m_staticText2801 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("T [°C]:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText2801->Wrap(-1);
    fgSizer153->Add(m_staticText2801, 0, wxALIGN_TOP | wxLEFT, 5);

    sttxtTemp = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("000.0"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtTemp->Wrap(-1);
    fgSizer153->Add(sttxtTemp, 0, wxALIGN_TOP | wxLEFT, 5);

    fgSizer154->Add(fgSizer153, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer155;
    fgSizer155 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer155->SetFlexibleDirection(wxBOTH);
    fgSizer155->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    btnTempRead = new wxButton(ID_PANEL_CONFIG, wxID_ANY, wxT("Measure"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer155->Add(btnTempRead, 0, wxLEFT, 5);

    btnTempCalibrate = new wxButton(ID_PANEL_CONFIG, wxID_ANY, wxT("Calibrate"), wxDefaultPosition, wxDefaultSize, 0);
    fgSizer155->Add(btnTempCalibrate, 0, wxALL, 5);

    fgSizer154->Add(fgSizer155, 1, wxEXPAND, 5);

    sbSizer28->Add(fgSizer154, 1, wxEXPAND, 5);

    fgSizer27->Add(sbSizer28, 0, wxEXPAND, 5);

    wxStaticBoxSizer* sbSizer31;
    sbSizer31 = new wxStaticBoxSizer(new wxStaticBox(ID_PANEL_CONFIG, wxID_ANY, wxT("Chip Info")), wxVERTICAL);

    wxFlexGridSizer* fgSizer28;
    fgSizer28 = new wxFlexGridSizer(0, 2, 0, 0);
    fgSizer28->AddGrowableCol(0);
    fgSizer28->SetFlexibleDirection(wxBOTH);
    fgSizer28->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    m_staticText40 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("Version:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText40->Wrap(-1);
    fgSizer28->Add(m_staticText40, 0, wxALL, 5);

    sttxtVER = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("?"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtVER->Wrap(-1);
    fgSizer28->Add(sttxtVER, 0, wxALL, 5);

    m_staticText41 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("Revision:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText41->Wrap(-1);
    fgSizer28->Add(m_staticText41, 0, wxALL, 5);

    sttxtREV = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("?"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtREV->Wrap(-1);
    fgSizer28->Add(sttxtREV, 0, wxALL, 5);

    m_staticText42 = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("Mask:"), wxDefaultPosition, wxDefaultSize, 0);
    m_staticText42->Wrap(-1);
    fgSizer28->Add(m_staticText42, 0, wxALL, 5);

    sttxtMASK = new wxStaticText(ID_PANEL_CONFIG, wxID_ANY, wxT("?"), wxDefaultPosition, wxDefaultSize, 0);
    sttxtMASK->Wrap(-1);
    fgSizer28->Add(sttxtMASK, 0, wxALL, 5);

    sbSizer31->Add(fgSizer28, 0, wxEXPAND, 5);

    fgSizer27->Add(sbSizer31, 1, wxEXPAND, 5);

    fgSizer22->Add(fgSizer27, 0, wxEXPAND, 5);

    ID_PANEL_CONFIG->SetSizer(fgSizer22);
    ID_PANEL_CONFIG->Layout();
    fgSizer22->Fit(ID_PANEL_CONFIG);
    ID_NOTEBOOK_CONFIG->AddPage(ID_PANEL_CONFIG, wxT("Config"), false);

    fgSizer68->Add(ID_NOTEBOOK_CONFIG, 1, wxEXPAND | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    this->SetSizer(fgSizer68);
    this->Layout();
    fgSizer68->Fit(this);

    // Connect Events
    chkGPIO_OUT_SPI_0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_OUT_SPI_1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_OUT_SPI_2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_OUT_SPI_3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_OUT_SPI_4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_OUT_SPI_5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_OUT_SPI_6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_OUT_SPI_7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_OUT_SPI_8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    cmbGPIO0_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    cmbGPIO1_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    cmbGPIO2_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    cmbGPIO3_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    cmbGPIO4_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    cmbGPIO5_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    cmbGPIO6_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    cmbGPIO7_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    cmbGPIO8_SEL->Connect(
        wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_IN_0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_IN_1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_IN_2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_IN_3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_IN_4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_IN_5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_IN_6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_IN_7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_IN_8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_PE_0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_PE_1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_PE_2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_PE_3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_PE_4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_PE_5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_PE_6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_PE_7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_PE_8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_DS_0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_DS_1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_DS_2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_DS_3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_DS_4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_DS_5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_DS_6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_DS_7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_DS_8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_InO_0->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_InO_1->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_InO_2->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_InO_3->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_InO_4->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_InO_5->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_InO_6->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_InO_7->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkGPIO_InO_8->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    rgrSPI_SDIO_DS->Connect(
        wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    rgrSPI_SDO_DS->Connect(
        wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkSPI_SDIO_PE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkSPI_SDO_PE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkSPI_SCLK_PE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkSPI_SEN_PE->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    rgrSPIMODE->Connect(
        wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkTEMP_SENS_EN->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkTEMP_SENS_CLKEN->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    chkTEMP_START_CONV->Connect(
        wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::ParameterChangeHandler), NULL, this);
    sttxtTemp->Connect(wxEVT_UPDATE_UI, wxUpdateUIEventHandler(lms8001_pnlConfig_view::OnUpdateUI_sttxtTemp), NULL, this);
    btnTempRead->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::OnClick_btnTempRead), NULL, this);
    btnTempCalibrate->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms8001_pnlConfig_view::OnClick_btnTempCalibrate), NULL, this);

    tempCalibrate = nullptr;

    //ids for updating from chip
    wndId2Enum[rgrSPI_SDIO_DS] = SPI_SDIO_DS;
    wndId2Enum[rgrSPI_SDO_DS] = SPI_SDO_DS;
    wndId2Enum[chkSPI_SDIO_PE] = SPI_SDIO_PE;
    wndId2Enum[chkSPI_SDO_PE] = SPI_SDO_PE;
    wndId2Enum[chkSPI_SCLK_PE] = SPI_SCLK_PE;
    wndId2Enum[chkSPI_SEN_PE] = SPI_SEN_PE;
    wndId2Enum[rgrSPIMODE] = SPIMODE;
    wndId2Enum[chkGPIO_OUT_SPI_0] = GPIO_OUT_SPI_0;
    wndId2Enum[chkGPIO_OUT_SPI_1] = GPIO_OUT_SPI_1;
    wndId2Enum[chkGPIO_OUT_SPI_2] = GPIO_OUT_SPI_2;
    wndId2Enum[chkGPIO_OUT_SPI_3] = GPIO_OUT_SPI_3;
    wndId2Enum[chkGPIO_OUT_SPI_4] = GPIO_OUT_SPI_4;
    wndId2Enum[chkGPIO_OUT_SPI_5] = GPIO_OUT_SPI_5;
    wndId2Enum[chkGPIO_OUT_SPI_6] = GPIO_OUT_SPI_6;
    wndId2Enum[chkGPIO_OUT_SPI_7] = GPIO_OUT_SPI_7;
    wndId2Enum[chkGPIO_OUT_SPI_8] = GPIO_OUT_SPI_8;
    wndId2Enum[cmbGPIO0_SEL] = GPIO0_SEL;
    wndId2Enum[cmbGPIO1_SEL] = GPIO1_SEL;
    wndId2Enum[cmbGPIO2_SEL] = GPIO2_SEL;
    wndId2Enum[cmbGPIO3_SEL] = GPIO3_SEL;
    wndId2Enum[cmbGPIO4_SEL] = GPIO4_SEL;
    wndId2Enum[cmbGPIO5_SEL] = GPIO5_SEL;
    wndId2Enum[cmbGPIO6_SEL] = GPIO6_SEL;
    wndId2Enum[cmbGPIO7_SEL] = GPIO7_SEL;
    wndId2Enum[cmbGPIO8_SEL] = GPIO8_SEL;
    wndId2Enum[chkGPIO_IN_0] = GPIO_IN_0;
    wndId2Enum[chkGPIO_IN_1] = GPIO_IN_1;
    wndId2Enum[chkGPIO_IN_2] = GPIO_IN_2;
    wndId2Enum[chkGPIO_IN_3] = GPIO_IN_3;
    wndId2Enum[chkGPIO_IN_4] = GPIO_IN_4;
    wndId2Enum[chkGPIO_IN_5] = GPIO_IN_5;
    wndId2Enum[chkGPIO_IN_6] = GPIO_IN_6;
    wndId2Enum[chkGPIO_IN_7] = GPIO_IN_7;
    wndId2Enum[chkGPIO_IN_8] = GPIO_IN_8;
    wndId2Enum[chkGPIO_PE_0] = GPIO_PE_0;
    wndId2Enum[chkGPIO_PE_1] = GPIO_PE_1;
    wndId2Enum[chkGPIO_PE_2] = GPIO_PE_2;
    wndId2Enum[chkGPIO_PE_3] = GPIO_PE_3;
    wndId2Enum[chkGPIO_PE_4] = GPIO_PE_4;
    wndId2Enum[chkGPIO_PE_5] = GPIO_PE_5;
    wndId2Enum[chkGPIO_PE_6] = GPIO_PE_6;
    wndId2Enum[chkGPIO_PE_7] = GPIO_PE_7;
    wndId2Enum[chkGPIO_PE_8] = GPIO_PE_8;
    wndId2Enum[chkGPIO_DS_0] = GPIO_DS_0;
    wndId2Enum[chkGPIO_DS_1] = GPIO_DS_1;
    wndId2Enum[chkGPIO_DS_2] = GPIO_DS_2;
    wndId2Enum[chkGPIO_DS_3] = GPIO_DS_3;
    wndId2Enum[chkGPIO_DS_4] = GPIO_DS_4;
    wndId2Enum[chkGPIO_DS_5] = GPIO_DS_5;
    wndId2Enum[chkGPIO_DS_6] = GPIO_DS_6;
    wndId2Enum[chkGPIO_DS_7] = GPIO_DS_7;
    wndId2Enum[chkGPIO_DS_8] = GPIO_DS_8;
    wndId2Enum[chkGPIO_InO_0] = GPIO_InO_0;
    wndId2Enum[chkGPIO_InO_1] = GPIO_InO_1;
    wndId2Enum[chkGPIO_InO_2] = GPIO_InO_2;
    wndId2Enum[chkGPIO_InO_3] = GPIO_InO_3;
    wndId2Enum[chkGPIO_InO_4] = GPIO_InO_4;
    wndId2Enum[chkGPIO_InO_5] = GPIO_InO_5;
    wndId2Enum[chkGPIO_InO_6] = GPIO_InO_6;
    wndId2Enum[chkGPIO_InO_7] = GPIO_InO_7;
    wndId2Enum[chkGPIO_InO_8] = GPIO_InO_8;
    wndId2Enum[chkTEMP_SENS_EN] = TEMP_SENS_EN;
    wndId2Enum[chkTEMP_SENS_CLKEN] = TEMP_SENS_CLKEN;
    wndId2Enum[chkTEMP_START_CONV] = TEMP_START_CONV;
    wndId2Enum[sttxtTEMP_READ] = TEMP_READ;
    //	wndId2Enum[txtVER] = VER;
    //	wndId2Enum[txtREV] = REV;
    //	wndId2Enum[txtMASK] = MASK;

    wndId2Enum[sttxtVER] = VER;
    wndId2Enum[sttxtREV] = REV;
    wndId2Enum[sttxtMASK] = MASK;

    LMS8001_WXGUI::UpdateTooltips(wndId2Enum, true);
}

void lms8001_pnlConfig_view::OnClick_btnTempRead(wxCommandEvent& event)
{
    int enable, clock_enable, pll_xbuf_en;

    enable = chip->Get_SPI_Reg_bits(TEMP_SENS_EN, true);
    clock_enable = chip->Get_SPI_Reg_bits(TEMP_SENS_CLKEN, true);
    pll_xbuf_en = chip->Get_SPI_Reg_bits(PLL_XBUF_EN, true);

    if (!enable)
        chip->Modify_SPI_Reg_bits(TEMP_SENS_EN, 1);
    if (!clock_enable)
        chip->Modify_SPI_Reg_bits(TEMP_SENS_CLKEN, 1);
    if (!pll_xbuf_en)
        chip->Modify_SPI_Reg_bits(PLL_XBUF_EN, 1);

    chip->Modify_SPI_Reg_bits(TEMP_START_CONV, 1);

    //  Wait - 5ms?
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if (!enable)
        chip->Modify_SPI_Reg_bits(TEMP_SENS_EN, 0);
    if (!clock_enable)
        chip->Modify_SPI_Reg_bits(TEMP_SENS_CLKEN, 0);
    if (!pll_xbuf_en)
        chip->Modify_SPI_Reg_bits(PLL_XBUF_EN, 0);

    int value = chip->Get_SPI_Reg_bits(TEMP_READ, true);

    sttxtTEMP_READ->SetLabelText(wxString::Format(_("%d"), value));
}

void lms8001_pnlConfig_view::OnUpdateUI_sttxtTemp(wxUpdateUIEvent& event)
{
    long code;
    sttxtTEMP_READ->GetLabel().ToLong(&code);
    double temperature;
    double tempSens_T1 = TEMPSENS_T1;
    double tempSens_T2 = TEMPSENS_T2;

    temperature = chip->tempSens_T0 + tempSens_T1 * code + tempSens_T2 * pow(code, 2);

    wxString newValue = wxString::Format(_("%.1f"), temperature);
    wxString oldValue = sttxtTemp->GetLabelText();

    if (oldValue != newValue)
        sttxtTemp->SetLabelText(newValue);
}

void lms8001_pnlConfig_view::OnClick_btnTempCalibrate(wxCommandEvent& event)
{
    if (tempCalibrate) //it's already opened
        tempCalibrate->Show();
    else
    {
        tempCalibrate = new lms8_dlgTempCalibrate(
            this, wxNewId(), _("Temp. Calibrate"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE);
        tempCalibrate->Initialize(chip);
        tempCalibrate->Show();
    }
}
