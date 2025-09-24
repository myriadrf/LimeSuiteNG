/**
@file 	gui_utilities.h
@author Lime Microsystems (www.limemicro.com)
@brief 	Common functions used by all panels
*/
#include <map>
#include <vector>
class wxPanel;
class wxWindow;
class LMS8001;

namespace lime {
class LMS8001;
struct LMS8Parameter;
} // namespace lime

namespace LMS8001_WXGUI {

void UpdateControlsByMap(wxPanel* panel, lime::LMS8001* lmsControl, const std::map<wxWindow*, lime::LMS8Parameter>& wndId2param);

typedef std::pair<int, int> indexValuePair;
typedef std::vector<indexValuePair> indexValueMap;

int index2value(int index, const indexValueMap& pairs);
int value2index(int value, const indexValueMap& pairs);

void UpdateTooltips(const std::map<wxWindow*, lime::LMS8Parameter>& wndId2param, bool replace);

} // namespace LMS8001_WXGUI
