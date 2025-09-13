
#cmakedefine01 LIMESUITENG_USB_FX3
#cmakedefine01 LIMESUITENG_USB_FTDI
#cmakedefine01 LIMESUITENG_PCIE
#cmakedefine01 LIMESUITENG_shiva

void __loadFX3();
void __loadFTDI();
void __loadDeviceFactoryPCIe();
void __loadDeviceFactoryLA9310();
void __loadDeviceFactoryPCIe_shiva();

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
    __loadDeviceFactoryLA9310();
#endif
}
