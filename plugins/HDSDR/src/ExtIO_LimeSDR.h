/*
The MIT License

Copyright(c) 2018 Lime Microsystems Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE
*/
#ifdef EXTIO_EXPORTS
#define EXTIO_API __declspec(dllexport) __stdcall
#else
#define EXTIO_API __declspec(dllimport)
#endif

#include "LC_ExtIO_Types.h"

#define ExtIO_LimeSDR_VERSION @ExtIO_LimeSDR_VERSION @

extern "C" bool EXTIO_API InitHW(char* name, char* model, int& type);
extern "C" int EXTIO_API StartHW64(int64_t freq);
extern "C" bool EXTIO_API OpenHW(void);
extern "C" int EXTIO_API StartHW(long freq);
extern "C" void EXTIO_API StopHW(void);
extern "C" void EXTIO_API CloseHW(void);

extern "C" int EXTIO_API SetHWLO(long LOfreq);
extern "C" int64_t EXTIO_API SetHWLO64(int64_t LOfreq);
extern "C" long EXTIO_API GetHWLO(void);
extern "C" int64_t EXTIO_API GetHWLO64(void);
extern "C" long EXTIO_API GetHWSR(void);

extern "C" int EXTIO_API GetStatus(void);
extern "C" void EXTIO_API SetCallback(pfnExtIOCallback funcptr);

extern "C" void EXTIO_API VersionInfo(const char* progname, int ver_major, int ver_minor);
extern "C" int EXTIO_API GetAttenuators(int idx, float* attenuation); // fill in attenuation
    // use positive attenuation levels if signal is amplified (LNA)
    // use negative attenuation levels if signal is attenuated
    // sort by attenuation: use idx 0 for highest attenuation / most damping
    // this functions is called with incrementing idx
    //    - until this functions return != 0 for no more attenuator setting
extern "C" int EXTIO_API GetActualAttIdx(void); // returns -1 on error
extern "C" int EXTIO_API SetAttenuator(int idx); // returns != 0 on error

extern "C" int EXTIO_API ExtIoGetSrates(int idx, double* samplerate); // fill in possible sample rates
extern "C" int EXTIO_API ExtIoGetActualSrateIdx(void); // returns -1 on error
extern "C" int EXTIO_API ExtIoSetSrate(int idx); // returns != 0 on error
extern "C" int EXTIO_API ExtIoGetSetting(
    int idx, char* description, char* value); // will be called (at least) before exiting application
extern "C" void EXTIO_API ExtIoSetSetting(int idx, const char* value); // before calling InitHW() !!!

extern "C" void EXTIO_API ShowGUI();
extern "C" void EXTIO_API HideGUI();
extern "C" void EXTIO_API SwitchGUI();
