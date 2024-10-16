#ifndef __dlgFullMessageLog__
#define __dlgFullMessageLog__

/**
@file
Subclass of dlgFullMessageLog_view, which is generated by wxFormBuilder.
*/

#include "utilities_gui.h"

//// end generated include
#include <deque>
/** Implementing dlgFullMessageLog_view */
class dlgFullMessageLog : public dlgFullMessageLog_view
{
  protected:
    // Handlers for dlgFullMessageLog_view events.
  public:
    /** Constructor */
    dlgFullMessageLog(wxWindow* parent);
    void AddMessages(const std::deque<wxString>& messages);
    //// end generated class members
};

#endif // __dlgFullMessageLog__
