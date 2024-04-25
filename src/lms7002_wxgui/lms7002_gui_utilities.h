/**
@file 	lms7002_gui_utilities.h
@author Lime Microsystems (www.limemicro.com)
@brief 	Common functions used by all panels
*/
#include <map>
#include <vector>
#include <cstdint>

#include "limesuiteng/LMS7002MCSR.h"

class wxPanel;
class wxWindow;

namespace lime {
class LMS7002M;
}

namespace LMS7002_WXGUI {

void UpdateControlsByMap(
    wxPanel* panel, lime::LMS7002M* lmsControl, const std::map<wxWindow*, lime::LMS7002MCSR>& wndId2param, uint8_t channel);

typedef std::pair<int, int> indexValuePair;
typedef std::vector<indexValuePair> indexValueMap;

int index2value(int index, const indexValueMap& pairs);
int value2index(int value, const indexValueMap& pairs);

void UpdateTooltips(const std::map<wxWindow*, lime::LMS7002MCSR>& wndId2param, bool replace);

} // namespace LMS7002_WXGUI
