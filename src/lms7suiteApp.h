/***************************************************************
 * Name:      lms7projectApp.h
 * Purpose:   Defines Application Class
 * Author:    Lime Microsystems ()
 * Created:   2015-03-05
 * Copyright: Lime Microsystems (limemicro.com)
 * License:
 **************************************************************/

#ifndef LMS7PROJECTAPP_H
#define LMS7PROJECTAPP_H

#include <wx/app.h>
#include <wx/cmdline.h>

struct AppArgs {
    wxString device{};
    wxString searchTree{};
};

class lms7suiteApp : public wxApp
{
    AppArgs appArgs;

  public:
    virtual bool OnInit();
    virtual void OnInitCmdLine(wxCmdLineParser& parser);
    virtual bool OnCmdLineParsed(wxCmdLineParser& parser);
};

#endif // LMS7PROJECTAPP_H
