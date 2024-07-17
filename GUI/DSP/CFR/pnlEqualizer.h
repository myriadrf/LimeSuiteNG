#pragma once

#include <wx/panel.h>
#include "commonWxForwardDeclarations.h"
#include <map>

#include "ISOCPanel.h"
#include "limesuiteng/types.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/Register.h"
#include "limesuiteng/OpStatus.h"
#include "limesuiteng/CSRegister.h"
#include "DSP/CFR/CrestFactorReduction.h"

class wxSpinCtrlDouble;

class EqualiserTest : public ISOCPanel
{
  public:
    EqualiserTest(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        int style = 0);
    void Initialize(lime::CrestFactorReduction* pControl);
    virtual ~EqualiserTest();

    wxCheckBox* chkEN_TXTSP;
    wxCheckBox* chkEN_RXTSP;
    wxCheckBox* chkRX_DCCORR_BYP;
    wxCheckBox* chkRX_PHCORR_BYP;
    wxCheckBox* chkRX_GCORR_BYP;
    wxCheckBox* chkRX_DCLOOP_BYP;
    wxCheckBox* chkRX_EQU_BYP;
    wxCheckBox* chkTX_DCCORR_BYP;
    wxCheckBox* chkTX_PHCORR_BYP;
    wxCheckBox* chkTX_GCORR_BYP;
    wxCheckBox* chkTX_ISINC_BYP;
    wxCheckBox* chkTX_EQU_BYP;
    wxCheckBox* chkTX_HB_BYP;
    wxCheckBox* chkTX_HB_DEL;
    wxSpinCtrl* spinTX_DCCORRI;
    wxSpinCtrl* spinTX_DCCORRQ;
    wxSpinCtrl* spinTX_GCORRQ;
    wxSpinCtrl* spinTX_GCORRI;
    wxSpinCtrl* spinTX_PHCORR;
    wxSpinCtrl* spinRX_GCORRQ;
    wxSpinCtrl* spinRX_GCORRI;
    wxSpinCtrl* spinRX_PHCORR;
    wxChoice* cmbInsel;
    wxRadioButton* rbChannelA;
    wxRadioButton* rbChannelB;
    wxChoice* cmbPAsrc;
    wxTextCtrl* txtNcoFreq;
    wxSpinCtrl* spinRX_DCLOOP_AVG;

    static const long ID_BUTTON_UPDATEALL;
    static const long ID_VCXOCV;

    void OnbtnUpdateAll(wxCommandEvent& event);
    void OnNcoFrequencyChanged(wxCommandEvent& event);

    wxCheckBox* chkSLEEP_CFR;
    wxCheckBox* chkBYPASS_CFR;
    wxCheckBox* chkODD_CFR;
    wxCheckBox* chkBYPASSGAIN_CFR;
    wxCheckBox* chkSLEEP_FIR;
    wxCheckBox* chkBYPASS_FIR;
    wxCheckBox* chkODD_FIR;
    wxCheckBox* chkTX_INVERTQ;

    wxSpinCtrlDouble* thresholdSpin;
    wxSpinCtrlDouble* thresholdGain;
    wxSpinCtrl* spinCFR_ORDER;
    wxButton* setFIR1;

  protected:
    bool ChA;
    // void OnConfigurePLL(wxCommandEvent &event);
    void OnReadAll(wxCommandEvent& event);
    void OnWriteAll(wxCommandEvent& event);
    void OnSwitchToChannelA(wxCommandEvent& event);
    void OnSwitchToChannelB(wxCommandEvent& event);

    wxButton* btnEqualizerSettings;
    wxButton* btnResetEqualizer;
    wxButton* btnSaveEqualizerSettings;

    void LoadEqualizerSettings(wxCommandEvent& event);
    void ResetEqualizer(wxCommandEvent& event);
    void SaveEqualizerSettings(wxCommandEvent& event);

    void OnOrderChanged(wxCommandEvent& event);
    void OnThresholdChanged(wxCommandEvent& event);
    void OnGainChanged(wxCommandEvent& event);
    void onbtnFIRCoef();

    std::map<wxObject*, lime::CSRegister> controlsPtr2Registers;
    void RegisterParameterChangeHandler(wxCommandEvent& event);
    void SetRegValue(const lime::Register& reg, uint16_t newValue);

    lime::OpStatus FPGAWriteRegister(uint32_t addr, uint32_t val);
    lime::OpStatus FPGAReadRegister(uint32_t addr, uint16_t* value);

  protected:
    lime::CrestFactorReduction* equalizer;
    DECLARE_EVENT_TABLE()
};
