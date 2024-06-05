/**
@file 	lms7002_gui_utilities.cpp
@author Lime Microsystems (www.limemicro.com)
@brief 	Implementation of common functions used by all panels
*/
#include "lms7002_gui_utilities.h"
#include "commonWxHeaders.h"
#include <wx/defs.h>
#include <wx/combobox.h>
#include <wx/checkbox.h>
#include <wx/radiobox.h>
#include <wx/radiobut.h>
#include <wx/stattext.h>
#include <wx/msgdlg.h>
#include "numericSlider.h"
#include <wx/spinctrl.h>
#include <wx/object.h>
#include <wx/tooltip.h>
#include "limesuiteng/LMS7002M.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"

using namespace lime;

void LMS7002_WXGUI::UpdateControlsByMap(
    wxPanel* panel, LMS7002M* lmsControl, const std::map<wxWindow*, LMS7002MCSR>& wndId2param, const uint8_t channel)
{
    if (panel == nullptr || lmsControl == nullptr)
        return;
    panel->Freeze();

    wxObject* wnd;
    uint16_t value = 0;
    wxClassInfo* wndClass;
    wxClassInfo* cmbInfo = wxClassInfo::FindClass(_("wxComboBox"));
    wxClassInfo* chkInfo = wxClassInfo::FindClass(_("wxCheckBox"));
    wxClassInfo* rgrInfo = wxClassInfo::FindClass(_("wxRadioBox"));
    wxClassInfo* numericSliderInfo = wxClassInfo::FindClass(_("NumericSlider"));
    wxClassInfo* spinCtrlInfo = wxClassInfo::FindClass(_("wxSpinCtrl"));
    wxClassInfo* labelInfo = wxClassInfo::FindClass(_("wxStaticText"));
    wxClassInfo* radioBtnInfo = wxClassInfo::FindClass(_("wxRadioButton"));

    for (auto idParam : wndId2param)
    {
        wnd = idParam.first;
        if (wnd == nullptr)
            continue;
        wndClass = wnd->GetClassInfo();

        value = lmsControl->Get_SPI_Reg_bits(idParam.second, true);
        //cast window to specific control, to set value, or set selection
        if (wndClass->IsKindOf(cmbInfo))
        {
            wxComboBox* box = wxStaticCast(wnd, wxComboBox);
            if (box->GetCount() <= value)
            {
                wxString str;
                str = wxString::Format(_("combobox value(%i) is out of range [0-%u]"), value, box->GetCount() - 1);
                //wxMessageBox(str, "WARNING!");
                value = 0;
            }
            box->SetSelection(value);
        }
        else if (wndClass->IsKindOf(chkInfo))
        {
            wxStaticCast(wnd, wxCheckBox)->SetValue(value);
        }
        else if (wndClass->IsKindOf(rgrInfo))
        {
            wxRadioBox* box = wxStaticCast(wnd, wxRadioBox);
            if (box->GetCount() <= value)
            {
                wxString str;
                str = wxString::Format(_("radiogroup value(%i) is out of range [0-%u]"), value, box->GetCount() - 1);
                //wxMessageBox(str, "WARNING!");
                continue;
            }
            box->SetSelection(value);
        }
        else if (wndClass->IsKindOf(labelInfo))
        {
            wxStaticCast(wnd, wxStaticText)->SetLabel(std::to_string(value));
        }
        else if (wndClass->IsKindOf(numericSliderInfo))
        {
            wxStaticCast(wnd, NumericSlider)->SetValue(value);
        }
        else if (wndClass->IsKindOf(spinCtrlInfo))
        {
            wxStaticCast(wnd, wxSpinCtrl)->SetValue(value);
        }
        else if (wndClass->IsKindOf(radioBtnInfo))
        {
            wxStaticCast(wnd, wxRadioButton)->SetValue(value);
        }
        else
        {
            wxString str;
            // #ifndef NDEBUG
            //             str = wxString::Format(_("Unhandled control class type. className=%s, was assigned address %04X"),
            //                 wndClass->GetClassName(),
            //                 idParam.second.address);
            //             wxMessageBox(str, "ERROR!");
            // #endif
        }
    }
    panel->Thaw();
}

int LMS7002_WXGUI::index2value(int index, const indexValueMap& pairs)
{
    for (size_t i = 0; i < pairs.size(); ++i)
        if (index == pairs[i].first)
            return pairs[i].second;
    return 0;
}

int LMS7002_WXGUI::value2index(int value, const indexValueMap& pairs)
{
    for (size_t i = 0; i < pairs.size(); ++i)
        if (value == pairs[i].second)
            return pairs[i].first;
    return 0;
}

/** @brief Changes given wxWidget controls tooltips to parameter descriptions
    @param wndId2param wxWidgets controls and LMS parameters pairs
    @param replace Replace all tooltips with new ones, or keep old ones and just add missing ones
*/
void LMS7002_WXGUI::UpdateTooltips(const std::map<wxWindow*, LMS7002MCSR>& wndId2param, bool replace)
{
#if wxUSE_TOOLTIPS
    wxString sttip = _("");
    std::map<wxWindow*, LMS7002MCSR>::const_iterator iter;
    for (iter = wndId2param.begin(); iter != wndId2param.end(); ++iter)
    {
        const lime::LMS7002MCSR_Data::CSRegister& reg = GetRegister(iter->second);
        wxToolTip* ttip = iter->first->GetToolTip();
        if (ttip)
            sttip = ttip->GetTip();
        else
            sttip = _("");

        if (replace || sttip.length() == 0)
            sttip = wxString::From8BitData(reg.tooltip);

        if (sttip.length() != 0)
            sttip += _("\n");

        int bitCount = reg.msb - reg.lsb + 1;
        if (bitCount == 1)
            sttip += wxString::Format(_("0x%.4X[%i]"), reg.address, reg.lsb);
        else
            sttip += wxString::Format(_("0x%.4X[%i:%i]"), reg.address, reg.msb, reg.lsb);
        if (iter->first->IsKindOf(wxClassInfo::FindClass(_("NumericSlider")))) //set tooltip is not virtual method, need to cast
            (reinterpret_cast<NumericSlider*>(iter->first))->SetToolTip(sttip + wxString::From8BitData(reg.name));
        else
            iter->first->SetToolTip(sttip + wxString::From8BitData(reg.name));
    }
#endif
}
