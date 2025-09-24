#ifndef __lms8001_pnlConfig_view__
#define __lms8001_pnlConfig_view__

#include <map>

#include "ILMS8001Tab.h"
class lms8_dlgTempCalibrate;

class lms8001_pnlConfig_view : public ILMS8001Tab
{
  public:
    lms8001_pnlConfig_view(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);

    void OnClick_btnTempRead(wxCommandEvent& event);
    void OnUpdateUI_sttxtTemp(wxUpdateUIEvent& event);
    void OnClick_btnTempCalibrate(wxCommandEvent& event);

  protected:
    lms8_dlgTempCalibrate* tempCalibrate;

    enum {
        ID_GPIO_OUT_SPI_0 = 2048,
        ID_GPIO_OUT_SPI_1,
        IID_GPIO_OUT_SPI_2,
        ID_GPIO_OUT_SPI_3,
        IID_GPIO_OUT_SPI_4,
        ID_GPIO_OUT_SPI_5,
        ID_GPIO_OUT_SPI_6,
        IID_GPIO_OUT_SPI_7,
        IID_GPIO_OUT_SPI_8,
        ID_GPIO0_SEL,
        ID_GPIO_IN_0,
        ID_GPIO_IN_1,
        IID_GPIO_IN_2,
        ID_GPIO_IN_3,
        IID_GPIO_IN_4,
        ID_GPIO_IN_5,
        ID_GPIO_IN_6,
        IID_GPIO_IN_7,
        IID_GPIO_IN_8,
        ID_GPIO_PE_0,
        ID_GPIO_PE_1,
        ID_GPIO_PE_2,
        ID_GPIO_PE_3,
        ID_GPIO_PE_4,
        ID_GPIO_PE_5,
        ID_GPIO_PE_6,
        ID_GPIO_PE_7,
        ID_GPIO_PE_8,
        ID_GPIO_DS_0,
        ID_GPIO_DS_1,
        ID_GPIO_DS_2,
        ID_GPIO_DS_3,
        ID_GPIO_DS_4,
        ID_GPIO_DS_5,
        ID_GPIO_DS_6,
        ID_GPIO_DS_7,
        ID_GPIO_DS_8,
        ID_GPIO_InO_0,
        ID_GPIO_InO_1,
        ID_GPIO_InO_2,
        ID_GPIO_InO_3,
        ID_GPIO_InO_4,
        ID_GPIO_InO_5,
        ID_GPIO_InO_6,
        ID_GPIO_InO_7,
        ID_GPIO_InO_8,
        ID_SPI_SDIO_DS,
        ID_SPI_SDO_DS,
        ID_SPI_SDIO_PE,
        ID_SPI_SDO_PE,
        ID_SPI_SEN_PE,
        ID_TEMP_SENS_EN,
        ID_TEMP_SENS_CLKEN,
        ID_TEMP_START_CONV
    };

    wxNotebook* ID_NOTEBOOK_CONFIG;
    wxPanel* ID_PANEL_CONFIG;
    wxCheckBox* chkGPIO_OUT_SPI_0;
    wxCheckBox* chkGPIO_OUT_SPI_1;
    wxCheckBox* chkGPIO_OUT_SPI_2;
    wxCheckBox* chkGPIO_OUT_SPI_3;
    wxCheckBox* chkGPIO_OUT_SPI_4;
    wxCheckBox* chkGPIO_OUT_SPI_5;
    wxCheckBox* chkGPIO_OUT_SPI_6;
    wxCheckBox* chkGPIO_OUT_SPI_7;
    wxCheckBox* chkGPIO_OUT_SPI_8;
    wxStaticText* m_staticText1;
    wxComboBox* cmbGPIO0_SEL;
    wxStaticText* m_staticText11;
    wxComboBox* cmbGPIO1_SEL;
    wxStaticText* m_staticText111;
    wxComboBox* cmbGPIO2_SEL;
    wxStaticText* m_staticText1111;
    wxComboBox* cmbGPIO3_SEL;
    wxStaticText* m_staticText11111;
    wxComboBox* cmbGPIO4_SEL;
    wxStaticText* m_staticText111111;
    wxComboBox* cmbGPIO5_SEL;
    wxStaticText* m_staticText1111111;
    wxComboBox* cmbGPIO6_SEL;
    wxStaticText* m_staticText11111111;
    wxComboBox* cmbGPIO7_SEL;
    wxStaticText* m_staticText111111111;
    wxComboBox* cmbGPIO8_SEL;
    wxCheckBox* chkGPIO_IN_0;
    wxCheckBox* chkGPIO_IN_1;
    wxCheckBox* chkGPIO_IN_2;
    wxCheckBox* chkGPIO_IN_3;
    wxCheckBox* chkGPIO_IN_4;
    wxCheckBox* chkGPIO_IN_5;
    wxCheckBox* chkGPIO_IN_6;
    wxCheckBox* chkGPIO_IN_7;
    wxCheckBox* chkGPIO_IN_8;
    wxCheckBox* chkGPIO_PE_0;
    wxCheckBox* chkGPIO_PE_1;
    wxCheckBox* chkGPIO_PE_2;
    wxCheckBox* chkGPIO_PE_3;
    wxCheckBox* chkGPIO_PE_4;
    wxCheckBox* chkGPIO_PE_5;
    wxCheckBox* chkGPIO_PE_6;
    wxCheckBox* chkGPIO_PE_7;
    wxCheckBox* chkGPIO_PE_8;
    wxCheckBox* chkGPIO_DS_0;
    wxCheckBox* chkGPIO_DS_1;
    wxCheckBox* chkGPIO_DS_2;
    wxCheckBox* chkGPIO_DS_3;
    wxCheckBox* chkGPIO_DS_4;
    wxCheckBox* chkGPIO_DS_5;
    wxCheckBox* chkGPIO_DS_6;
    wxCheckBox* chkGPIO_DS_7;
    wxCheckBox* chkGPIO_DS_8;
    wxStaticText* m_staticText37;
    wxStaticText* m_staticText372;
    wxCheckBox* chkGPIO_InO_0;
    wxCheckBox* chkGPIO_InO_1;
    wxCheckBox* chkGPIO_InO_2;
    wxCheckBox* chkGPIO_InO_3;
    wxCheckBox* chkGPIO_InO_4;
    wxCheckBox* chkGPIO_InO_5;
    wxCheckBox* chkGPIO_InO_6;
    wxCheckBox* chkGPIO_InO_7;
    wxCheckBox* chkGPIO_InO_8;
    wxStaticText* m_staticText371;
    wxStaticText* m_staticText3711;
    wxRadioBox* rgrSPI_SDIO_DS;
    wxRadioBox* rgrSPI_SDO_DS;
    wxCheckBox* chkSPI_SDIO_PE;
    wxCheckBox* chkSPI_SDO_PE;
    wxCheckBox* chkSPI_SCLK_PE;
    wxCheckBox* chkSPI_SEN_PE;
    wxRadioBox* rgrSPIMODE;
    wxCheckBox* chkTEMP_SENS_EN;
    wxCheckBox* chkTEMP_SENS_CLKEN;
    wxCheckBox* chkTEMP_START_CONV;
    wxStaticText* m_staticText280;
    wxStaticText* sttxtTEMP_READ;
    wxStaticText* m_staticText2801;
    wxStaticText* sttxtTemp;
    wxButton* btnTempRead;
    wxButton* btnTempCalibrate;
    wxStaticText* m_staticText40;
    wxStaticText* sttxtVER;
    wxStaticText* m_staticText41;
    wxStaticText* sttxtREV;
    wxStaticText* m_staticText42;
    wxStaticText* sttxtMASK;
};

#endif // __lms8001_pnlConfig_view__
