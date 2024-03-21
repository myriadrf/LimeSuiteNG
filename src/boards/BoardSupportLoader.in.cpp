
#cmakedefine ENABLE_LIMESDR_USB
#cmakedefine ENABLE_LIMESDR_X3
#cmakedefine ENABLE_LIMESDR_XTRX
#cmakedefine ENABLE_LIMESDR_MMX8
#cmakedefine ENABLE_LIMESDR_MINI

#cmakedefine ENABLE_LITE_PCIE

void __loadLimeSDR();
void __loadLimeSDR_Mini();
void __loadDeviceFactoryPCIe();

void __loadBoardSupport()
{
#ifdef ENABLE_LIMESDR_USB
    __loadLimeSDR();
#endif

#ifdef ENABLE_LIMESDR_MINI
    __loadLimeSDR_Mini();
#endif

#ifdef ENABLE_LITE_PCIE
    __loadDeviceFactoryPCIe();
#endif
}
