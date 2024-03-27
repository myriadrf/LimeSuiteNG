
#cmakedefine ENABLE_USB_FX3
#cmakedefine ENABLE_LIMESDR_X3
#cmakedefine ENABLE_LIMESDR_XTRX
#cmakedefine ENABLE_LIMESDR_MMX8
#cmakedefine ENABLE_USB_FTDI

#cmakedefine ENABLE_LITE_PCIE

void __loadFX3();
void __loadFTDI();
void __loadDeviceFactoryPCIe();

void __loadBoardSupport()
{
#ifdef ENABLE_USB_FX3
    __loadFX3();
#endif

#ifdef ENABLE_USB_FTDI
    __loadFTDI();
#endif

#ifdef ENABLE_LITE_PCIE
    __loadDeviceFactoryPCIe();
#endif
}
