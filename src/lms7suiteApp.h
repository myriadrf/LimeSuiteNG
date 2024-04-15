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

class lms7suiteApp : public wxApp
{
  public:
    virtual bool OnInit();
    virtual void OnInitCmdLine(wxCmdLineParser& parser);
    virtual bool OnCmdLineParsed(wxCmdLineParser& parser);
};

static const wxCmdLineEntryDesc cmdDescriptions[] = {
    { wxCMD_LINE_SWITCH, "h", "help", "displays help on the command line parameters", wxCMD_LINE_VAL_NONE, wxCMD_LINE_OPTION_HELP },
    { wxCMD_LINE_OPTION, "d", "device", "select device on launch", wxCMD_LINE_VAL_STRING, wxCMD_LINE_NONE },
    { wxCMD_LINE_OPTION, "s", "show-tree", "select initial tree window", wxCMD_LINE_VAL_STRING, wxCMD_LINE_NONE },

    { wxCMD_LINE_NONE }
};

#endif // LMS7PROJECTAPP_H
