#ifdef WX_PRECOMP
    #include "wx_pch.h"
#endif

#ifdef __BORLANDC__
    #pragma hdrstop
#endif //__BORLANDC__

#include "limeGUI.h"
#include "limeGUIFrame.h"
#include <wx/splash.h>
#include <wx/icon.h>
#if !defined(NDEBUG) & defined(__unix__)
IMPLEMENT_APP_CONSOLE(limeGUI)
#else
IMPLEMENT_APP(limeGUI)
#endif

#include "resources/splash.h"
#include "resources/LMS_ICO.xpm"

using namespace std::literals::string_literals;

bool limeGUI::OnInit()
{
    if (!wxApp::OnInit())
        return false;

    wxInitAllImageHandlers();
    wxBitmap splashBitmap = wxBITMAP_PNG_FROM_DATA(splash);
    wxSplashScreen* splash = new wxSplashScreen(
        splashBitmap, wxSPLASH_CENTRE_ON_SCREEN, 6000, NULL, -1, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER | wxSTAY_ON_TOP);
    wxYield(); //linux needs this to load splash image
    limeGUIFrame* frame = new limeGUIFrame(0L, appArgs);
    frame->SetIcon(wxICON(LMS_ICO));
    splash->Destroy();
    frame->Show();
    return true;
}

static const wxCmdLineEntryDesc cmdDescriptions[] = {
    { wxCMD_LINE_SWITCH, "h", "help", "displays help on the command line parameters", wxCMD_LINE_VAL_NONE, wxCMD_LINE_OPTION_HELP },
    { wxCMD_LINE_OPTION, "d", "device", "select device on launch", wxCMD_LINE_VAL_STRING, wxCMD_LINE_NONE },
    { wxCMD_LINE_OPTION, "s", "show-tree", "select initial tree window", wxCMD_LINE_VAL_STRING, wxCMD_LINE_NONE },

    { wxCMD_LINE_NONE }
};

void limeGUI::OnInitCmdLine(wxCmdLineParser& parser)
{
    parser.SetDesc(cmdDescriptions);
    parser.SetSwitchChars("-");
}

bool limeGUI::OnCmdLineParsed(wxCmdLineParser& parser)
{
    appArgs = {};
    if (parser.Found("d", &(appArgs.device)))
        parser.Found("s", &(appArgs.searchTree));

    return true;
}
