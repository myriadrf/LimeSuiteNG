#pragma once

#include <wx/event.h>

wxDECLARE_EVENT(limeEVT_SDR_HANDLE_SELECTED, wxCommandEvent);
wxDECLARE_EVENT(CGEN_FREQUENCY_CHANGED, wxCommandEvent);
wxDECLARE_EVENT(LOG_MESSAGE, wxCommandEvent);
wxDECLARE_EVENT(READ_ALL_VALUES, wxCommandEvent);
wxDECLARE_EVENT(WRITE_ALL_VALUES, wxCommandEvent);
