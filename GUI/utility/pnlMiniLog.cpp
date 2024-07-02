#include "pnlMiniLog.h"
#include "dlgFullMessageLog.h"

using namespace lime;

static wxTextAttr mDefaultStyle;

pnlMiniLog::pnlMiniLog(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : wxPanel(parent, id, pos, size, style)
    , log_level(3)
{
    wxFlexGridSizer* mainSizer;
    mainSizer = new wxFlexGridSizer(2, 0, 0);
    mainSizer->AddGrowableCol(0);
    mainSizer->AddGrowableRow(0);
    mainSizer->SetFlexibleDirection(wxBOTH);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    txtMessageField = new wxTextCtrl(
        this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxTE_READONLY | wxTE_RICH2);
#ifdef __WXGTK__
    if (!txtMessageField->HasFlag(wxTE_MULTILINE))
        txtMessageField->SetMaxLength(100);
#else
    txtMessageField->SetMaxLength(100);
#endif
    txtMessageField->SetMinSize(wxSize(300, 48));

    mainSizer->Add(txtMessageField, 0, wxEXPAND, 5);

    wxFlexGridSizer* szControls = new wxFlexGridSizer(1, 0, 0);
    //szControls->SetFlexibleDirection( wxVERTICAL );

    btnClear = new wxButton(this, wxID_ANY, wxT("Clear"));
    szControls->Add(btnClear, 0, wxEXPAND, 5);

    btnFullLog = new wxButton(this, wxID_ANY, wxT("Show Log"));
    szControls->Add(btnFullLog, 0, wxEXPAND, 5);

    wxStaticText* m_staticText52 = new wxStaticText(this, wxID_ANY, wxT("Log level:"), wxDefaultPosition, wxDefaultSize, 0);
    szControls->Add(m_staticText52, 0, wxTOP, 5);

    wxString choiceLogLvlChoices[] = { wxT("Error"), wxT("Warning"), wxT("Info"), wxT("Verbose"), wxT("Debug") };
    int choiceLogLvlNChoices = sizeof(choiceLogLvlChoices) / sizeof(wxString);
    choiceLogLvl = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, choiceLogLvlNChoices, choiceLogLvlChoices, 0);
    choiceLogLvl->SetSelection(2);
    szControls->Add(choiceLogLvl, 0, 0, 1);

    mainSizer->Add(szControls, 0, 0, 5);

    SetSizerAndFit(mainSizer);
    //Layout();
    //mainSizer->Fit( this );

    // Connect Events
    Connect(wxEVT_UPDATE_UI, wxUpdateUIEventHandler(pnlMiniLog::OnUpdateGUI));
    btnClear->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(pnlMiniLog::OnBtnClearClicked), NULL, this);
    btnFullLog->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(pnlMiniLog::OnShowFullLog), NULL, this);
    choiceLogLvl->Connect(wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler(pnlMiniLog::onLogLvlChange), NULL, this);

    mDefaultStyle = txtMessageField->GetDefaultStyle();
    wxUpdateUIEvent::SetUpdateInterval(100);
}

static const char* logLevelToName(const LogLevel level)
{
    switch (level)
    {
    case LogLevel::Critical:
        return "CRITICAL";
    case LogLevel::Error:
        return "ERROR";
    case LogLevel::Warning:
        return "WARNING";
    case LogLevel::Info:
        return "INFO";
    case LogLevel::Verbose:
        return "VERBOSE";
    case LogLevel::Debug:
        return "DEBUG";
    }
    return "";
}

void pnlMiniLog::HandleMessage(wxCommandEvent& event)
{
    auto level = lime::LogLevel(event.GetInt());
    if (static_cast<int>(level) > log_level)
        return;
    time_t rawtime;
    struct tm* timeinfo;
    char buffer[80];
    //add time stamp
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%H:%M:%S", timeinfo);
    wxString line(wxString::Format("[%s] %s: %s", buffer, logLevelToName(level), event.GetString()));

    mAllMessages.push_back(line);
    const int allMessageLimit = 3000;
    if (mAllMessages.size() > allMessageLimit)
        mAllMessages.pop_front();

    const int miniLogMessageLimit = 800;
    wxTextAttr style = mDefaultStyle;
    switch (level)
    {
    case lime::LogLevel::Critical:
    case lime::LogLevel::Error:
        style.SetTextColour(*wxRED);
        break;
    case lime::LogLevel::Warning:
        style.SetBackgroundColour(*wxYELLOW);
        style.SetTextColour(*wxBLACK);
        break;
    default:
        break;
    }
    txtMessageField->SetDefaultStyle(style);
    txtMessageField->AppendText(line);
    txtMessageField->SetDefaultStyle(mDefaultStyle);
    txtMessageField->AppendText(_("\n"));
    if (mAllMessages.size() > miniLogMessageLimit)
        txtMessageField->Remove(0, txtMessageField->GetLineLength(0) + 1);
}

void pnlMiniLog::OnUpdateGUI(wxUpdateUIEvent& event)
{
}

void pnlMiniLog::OnBtnClearClicked(wxCommandEvent& event)
{
    mMessageList.clear();
    txtMessageField->Clear();
    mAllMessages.clear();
}

void pnlMiniLog::OnShowFullLog(wxCommandEvent& event)
{
    dlgFullMessageLog* dlg = new dlgFullMessageLog(this);
    dlg->AddMessages(mAllMessages);
    dlg->ShowModal();
}

void pnlMiniLog::onLogLvlChange(wxCommandEvent& event)
{
    log_level = 1 + event.GetInt();
}
