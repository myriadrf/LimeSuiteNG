#include "lms7002_dlgGFIR_Coefficients.h"
#include <wx/filedlg.h>
#include <wx/msgdlg.h>
#include "parsers/CoefficientFileParser.h"
#include "limesuiteng/types.h"
#include "limesuiteng/LMS7002M.h"

using namespace lime;

lms7002_dlgGFIR_Coefficients::lms7002_dlgGFIR_Coefficients(wxWindow* parent, wxWindowID id, const wxString& title)
    : wxDialog(parent, id, title, wxDefaultPosition, wxDefaultSize, 0)
{
    SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxFlexGridSizer* fgSizer235;
    fgSizer235 = new wxFlexGridSizer(0, 1, 0, 0);
    fgSizer235->SetFlexibleDirection(wxBOTH);
    fgSizer235->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxFlexGridSizer* fgSizer237;
    fgSizer237 = new wxFlexGridSizer(0, 5, 0, 0);
    fgSizer237->SetFlexibleDirection(wxBOTH);
    fgSizer237->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    btnLoadFromFile = new wxButton(this, wxID_ANY, wxT("Load from file"), wxDefaultPosition, wxDefaultSize, 0);
    btnLoadFromFile->SetDefault();
    fgSizer237->Add(btnLoadFromFile, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    btnSaveToFile = new wxButton(this, wxID_ANY, wxT("Save to file"), wxDefaultPosition, wxDefaultSize, 0);
    btnSaveToFile->SetDefault();
    fgSizer237->Add(btnSaveToFile, 1, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    btnClearTable = new wxButton(this, wxID_ANY, wxT("Clear table"), wxDefaultPosition, wxDefaultSize, 0);
    btnClearTable->SetDefault();
    fgSizer237->Add(btnClearTable, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    ID_STATICTEXT3 = new wxStaticText(this, wxID_ANY, wxT("Coefficients count:"), wxDefaultPosition, wxDefaultSize, 0);
    ID_STATICTEXT3->Wrap(-1);
    fgSizer237->Add(ID_STATICTEXT3, 1, wxLEFT | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    spinCoefCount = new wxSpinCtrl(this, wxID_ANY, wxT("40"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 120, 40);
    fgSizer237->Add(spinCoefCount, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    fgSizer235->Add(fgSizer237, 1, wxALIGN_LEFT | wxALIGN_TOP, 5);

    wxFlexGridSizer* fgSizer238;
    fgSizer238 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer238->AddGrowableCol(0);
    fgSizer238->AddGrowableRow(0);
    fgSizer238->SetFlexibleDirection(wxBOTH);
    fgSizer238->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    gridCoef = new wxGrid(this, wxID_ANY, wxDefaultPosition, wxSize(-1, -1), 0);

    // Grid
    gridCoef->CreateGrid(5, 1);
    gridCoef->EnableEditing(true);
    gridCoef->EnableGridLines(true);
    gridCoef->EnableDragGridSize(false);
    gridCoef->SetMargins(0, 0);

    // Columns
    gridCoef->EnableDragColMove(false);
    gridCoef->EnableDragColSize(true);
    gridCoef->SetColLabelSize(30);
    gridCoef->SetColLabelAlignment(wxALIGN_CENTRE, wxALIGN_CENTRE);

    // Rows
    gridCoef->EnableDragRowSize(true);
    gridCoef->SetRowLabelSize(80);
    gridCoef->SetRowLabelAlignment(wxALIGN_CENTRE, wxALIGN_CENTRE);

    // Label Appearance

    // Cell Defaults
    gridCoef->SetDefaultCellAlignment(wxALIGN_LEFT, wxALIGN_TOP);
    gridCoef->SetMinSize(wxSize(100, 200));

    fgSizer238->Add(gridCoef, 1, wxEXPAND, 5);

    fgSizer235->Add(fgSizer238, 1, wxEXPAND, 5);

    wxFlexGridSizer* fgSizer239;
    fgSizer239 = new wxFlexGridSizer(0, 3, 0, 0);
    fgSizer239->SetFlexibleDirection(wxBOTH);
    fgSizer239->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    btnOk = new wxButton(this, wxID_ANY, wxT("Ok"), wxDefaultPosition, wxDefaultSize, 0);
    btnOk->SetDefault();
    fgSizer239->Add(btnOk, 1, wxRIGHT | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    btnCancel = new wxButton(this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0);
    btnCancel->SetDefault();
    fgSizer239->Add(btnCancel, 1, wxLEFT | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    fgSizer235->Add(fgSizer239, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

    SetSizer(fgSizer235);
    Layout();
    fgSizer235->Fit(this);

    // Connect Events
    btnLoadFromFile->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_dlgGFIR_Coefficients::OnLoadFromFile), NULL, this);
    btnSaveToFile->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_dlgGFIR_Coefficients::OnSaveToFile), NULL, this);
    btnClearTable->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_dlgGFIR_Coefficients::OnClearTable), NULL, this);
    spinCoefCount->Connect(
        wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler(lms7002_dlgGFIR_Coefficients::OnspinCoefCountChange), NULL, this);
    btnOk->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_dlgGFIR_Coefficients::OnBtnOkClick), NULL, this);
    btnCancel->Connect(
        wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(lms7002_dlgGFIR_Coefficients::OnBtnCancelClick), NULL, this);
}

void lms7002_dlgGFIR_Coefficients::OnLoadFromFile(wxCommandEvent& event)
{
    wxFileDialog dlg(this, _("Open coefficients file"), "", "", "FIR Coeffs (*.fir)|*.fir", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() == wxID_CANCEL)
        return;

    std::vector<double> coefficients(200, 0);
    auto parser = CoefficientFileParser(dlg.GetPath().ToStdString());
    int iVal = parser.getCoefficients(coefficients, 200);

    switch (iVal)
    {
    case -2:
        wxMessageDialog(this, "syntax error within the file", "Warning");
        break;
    case -3:
        wxMessageDialog(this, "filename is empty string", "Warning");
        break;
    case -4:
        wxMessageDialog(this, "can not open the file", "Warning");
        break;
    case -5:
        wxMessageDialog(this, "too many coefficients in the file", "Warning");
        break;
    }
    if (iVal < 0)
        return;

    spinCoefCount->SetValue(iVal);
    if (gridCoef->GetTable()->GetRowsCount() > 0)
        gridCoef->GetTable()->DeleteRows(0, gridCoef->GetTable()->GetRowsCount());
    gridCoef->GetTable()->AppendRows(spinCoefCount->GetValue());
    for (int i = 0; i < iVal; ++i)
    {
        gridCoef->SetCellValue(i, 0, std::to_string(coefficients[i]));
    }
}

void lms7002_dlgGFIR_Coefficients::OnSaveToFile(wxCommandEvent& event)
{
    wxFileDialog dlg(this, _("Save coefficients file"), "", "", "FIR Coeffs (*.fir)|*.fir", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() == wxID_CANCEL)
    {
        return;
    }

    const int coefficientCount = spinCoefCount->GetValue();
    std::vector<double> coefficients(coefficientCount, 0);

    for (int i = 0; i < coefficientCount; ++i)
    {
        gridCoef->GetCellValue(i, 0).ToDouble(&coefficients[i]);
    }

    auto parser = CoefficientFileParser(dlg.GetPath().ToStdString());
    parser.saveToFile(coefficients);
}

void lms7002_dlgGFIR_Coefficients::OnClearTable(wxCommandEvent& event)
{
    if (gridCoef->GetTable()->GetRowsCount() > 0)
        gridCoef->GetTable()->DeleteRows(0, gridCoef->GetTable()->GetRowsCount());
    gridCoef->GetTable()->AppendRows(spinCoefCount->GetValue());
    for (int i = 0; i < spinCoefCount->GetValue(); ++i)
    {
        gridCoef->SetCellValue(i, 0, "0");
    }
}

void lms7002_dlgGFIR_Coefficients::OnspinCoefCountChange(wxSpinEvent& event)
{
    if (spinCoefCount->GetValue() < gridCoef->GetTable()->GetRowsCount())
        gridCoef->GetTable()->DeleteRows(
            spinCoefCount->GetValue(), gridCoef->GetTable()->GetRowsCount() - spinCoefCount->GetValue());
    else
        gridCoef->GetTable()->AppendRows(spinCoefCount->GetValue() - gridCoef->GetTable()->GetRowsCount());
}

void lms7002_dlgGFIR_Coefficients::SetCoefficients(const std::vector<double>& coefficients)
{
    spinCoefCount->SetValue(coefficients.size());
    if (gridCoef->GetTable()->GetRowsCount() > 0)
        gridCoef->GetTable()->DeleteRows(0, gridCoef->GetTable()->GetRowsCount());
    gridCoef->GetTable()->AppendRows(coefficients.size());
    for (unsigned i = 0; i < coefficients.size(); ++i)
        gridCoef->SetCellValue(i, 0, wxString::Format("%.6f", coefficients[i]));
}

std::vector<double> lms7002_dlgGFIR_Coefficients::GetCoefficients()
{
    std::vector<double> coefficients;
    coefficients.resize(spinCoefCount->GetValue(), 0);
    for (int i = 0; i < spinCoefCount->GetValue(); ++i)
    {
        double dtemp = 0;
        gridCoef->GetCellValue(i, 0).ToDouble(&dtemp);
        coefficients[i] = dtemp;
    }
    return coefficients;
}

void lms7002_dlgGFIR_Coefficients::OnBtnOkClick(wxCommandEvent& event)
{
    EndModal(wxID_OK);
}

void lms7002_dlgGFIR_Coefficients::OnBtnCancelClick(wxCommandEvent& event)
{
    EndModal(wxID_CANCEL);
}

int lms7002_dlgGFIR_Coefficients::ReadCoefficients(lime::TRXDir direction, uint8_t gfirIndex, lime::LMS7002M* lmsControl)
{
    if (gfirIndex > 2)
    {
        gfirIndex = 2;
    }

    std::vector<double> coefficients;
    const int maxCoefCount = gfirIndex == 2 ? 120 : 40;
    coefficients.resize(maxCoefCount, 0);

    OpStatus status = lmsControl->GetGFIRCoefficients(direction, gfirIndex, &coefficients[0], coefficients.size());
    if (status != OpStatus::Success)
    {
        wxMessageBox(_("Error reading GFIR coefficients"), _("ERROR"), wxICON_ERROR | wxOK);
        return -1;
    }

    SetCoefficients(coefficients);
    return 0;
}

void lms7002_dlgGFIR_Coefficients::WriteCoefficients(lime::TRXDir direction, uint8_t gfirIndex, lime::LMS7002M* lmsControl)
{
    std::vector<double> coefficients = GetCoefficients();
    OpStatus status = lmsControl->SetGFIRCoefficients(direction, gfirIndex, &coefficients[0], coefficients.size());

    if (status != OpStatus::Success)
    {
        wxMessageBox(_("Error writing GFIR coefficients"), _("ERROR"), wxICON_ERROR | wxOK);
    }
}
