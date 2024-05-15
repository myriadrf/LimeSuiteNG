#ifndef __lms7002_pnlCalibrations_view__
#define __lms7002_pnlCalibrations_view__

#include "ILMS7002MTab.h"

class NumericSlider;

class lms7002_pnlCalibrations_view : public ILMS7002MTab
{
  protected:
    void ParameterChangeHandler(wxCommandEvent& event) override;
    void ParameterChangeHandler(wxSpinEvent& event);
    void OnbtnCalibrateRx(wxCommandEvent& event);
    void OnbtnCalibrateTx(wxCommandEvent& event);
    void OnbtnCalibrateAll(wxCommandEvent& event);
    void OnCalibrationMethodChange(wxCommandEvent& event);

  public:
    lms7002_pnlCalibrations_view(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);
    void Initialize(lime::LMS7002M* pControl) override;
    void UpdateGUI() override;

  protected:
    enum {
        ID_GCORRI_RXTSP = 2048,
        ID_GCORRQ_RXTSP,
        ID_IQCORR_RXTSP,
        ID_DCOFFI_RFE,
        ID_DCOFFQ_RFE,
        ID_EN_DCOFF_RXFE_RFE,
        ID_DCMODE,
        ID_GCORRI_TXTSP,
        ID_GCORRQ_TXTSP,
        ID_IQCORR_TXTSP,
        ID_DCCORRI_TXTSP,
        ID_DCCORRQ_TXTSP
    };

    NumericSlider* cmbGCORRI_RXTSP;
    NumericSlider* cmbGCORRQ_RXTSP;
    NumericSlider* cmbIQCORR_RXTSP;
    wxStaticText* txtRxPhaseAlpha;
    NumericSlider* cmbDCOFFI_RFE;
    NumericSlider* cmbDCOFFQ_RFE;
    wxCheckBox* chkEN_DCOFF_RXFE_RFE;
    wxCheckBox* chkDCMODE;
    wxButton* btnCalibrateRx;
    NumericSlider* cmbGCORRI_TXTSP;
    NumericSlider* cmbGCORRQ_TXTSP;
    NumericSlider* cmbIQCORR_TXTSP;
    wxStaticText* txtPhaseAlpha;
    NumericSlider* cmbDCCORRI_TXTSP;
    NumericSlider* cmbDCCORRQ_TXTSP;
    wxButton* btnCalibrateTx;
    wxButton* btnCalibrateAll;
    wxStaticText* lblCGENrefClk;
    wxTextCtrl* txtCalibrationBW;
    wxRadioBox* rgrCalibrationMethod;
    wxStaticText* lblCalibrationNote;
};

#endif // __lms7002_pnlCalibrations_view__
