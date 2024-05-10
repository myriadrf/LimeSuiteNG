/***************************************************************
 * Name:      lms7projectApp.cpp
 * Purpose:   Code for Application Class
 * Author:    Lime Microsystems ()
 * Created:   2015-03-05
 * Copyright: Lime Microsystems (limemicro.com)
 * License:
 **************************************************************/

#ifdef WX_PRECOMP
    #include "wx_pch.h"
#endif

#ifdef __BORLANDC__
    #pragma hdrstop
#endif //__BORLANDC__

#include "lms7suiteApp.h"
#include "lms7suiteAppFrame.h"
#include <wx/time.h>
#include <wx/splash.h>
#include <wx/icon.h>
#include <iostream>
#if !defined(NDEBUG) & defined(__unix__)
IMPLEMENT_APP_CONSOLE(lms7suiteApp)
#else
IMPLEMENT_APP(lms7suiteApp)
#endif

#include "limesuiteng/Logger.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "resources/splash.h"
#include "resources/LMS_ICO.xpm"
#include "events.h"

using namespace std::literals::string_literals;

bool lms7suiteApp::OnInit()
{
    if (!wxApp::OnInit())
        return false;

    wxInitAllImageHandlers();
    wxBitmap splashBitmap = wxBITMAP_PNG_FROM_DATA(splash);
    wxSplashScreen* splash = new wxSplashScreen(
        splashBitmap, wxSPLASH_CENTRE_ON_SCREEN, 6000, NULL, -1, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER | wxSTAY_ON_TOP);
    wxYield(); //linux needs this to load splash image
    LMS7SuiteAppFrame* frame = new LMS7SuiteAppFrame(0L, appArgs);
    frame->SetIcon(wxICON(LMS_ICO));
#ifndef NDEBUG
    wxLongLong t1 = wxGetUTCTimeMillis();
    lime::debug("Create time "s + (wxGetUTCTimeMillis() - t1).ToString() + " ms"s);
#endif
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

void lms7suiteApp::OnInitCmdLine(wxCmdLineParser& parser)
{
    parser.SetDesc(cmdDescriptions);
    parser.SetSwitchChars("-");
}

bool lms7suiteApp::OnCmdLineParsed(wxCmdLineParser& parser)
{
    appArgs = {};
    if (parser.Found("d", &(appArgs.device)))
        parser.Found("s", &(appArgs.searchTree));

    return true;
}
