#ifndef LIME_GUI_H
#define LIME_GUI_H

#include <wx/app.h>
#include <wx/cmdline.h>

struct AppArgs {
    wxString device{};
    wxString searchTree{};
};

class limeGUI : public wxApp
{
    AppArgs appArgs;

  public:
    virtual bool OnInit();
    virtual void OnInitCmdLine(wxCmdLineParser& parser);
    virtual bool OnCmdLineParsed(wxCmdLineParser& parser);
};

#endif // LIME_GUI_H
