
#cmakedefine01 LIMESUITENG_USB_FX3
#cmakedefine01 LIMESUITENG_USB_FTDI
#cmakedefine01 LIMESUITENG_PCIE
#cmakedefine01 LIMESUITENG_XILLYBUS

void __loadFX3();
void __loadFTDI();
void __loadDeviceFactoryPCIe();
void __loadDeviceFactoryXillybus();

void __loadBoardSupport()
{
#if LIMESUITENG_USB_FX3
    __loadFX3();
#endif

#if LIMESUITENG_USB_FTDI
    __loadFTDI();
#endif

#if LIMESUITENG_PCIE
    __loadDeviceFactoryPCIe();
#endif

#if LIMESUITENG_XILLYBUS
    __loadDeviceFactoryXillybus();
#endif
}
