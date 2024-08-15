Device drivers
==============

Use ``limeDevice`` command to see detected devices information. For example:

.. code-block:: bash

	user@computer:~$ limeDevice
	Found 4 device(s) :
	0: LimeSDR Mini, media=USB 3.0, addr=0403:601f, serial=1D9EFA3E84B944
	1: LimeSDR-USB, media=USB 3.0, addr=1d50:6108, serial=00090706024F2403
	2: LimeX30, media=PCIe, addr=/dev/LimeX30_control, serial=0000000000000000
	3: LimeXTRX0, media=PCIe, addr=/dev/LimeXTRX0_control, serial=0000000000000000

If your expected device is not in the list follow the troubleshooting steps for respective interfaces.

USB
---

LimeSuiteNG uses ``libusb-1.0`` library to communicate with the USB devices. It comes preinstalled with most Linux distributions, so no extra steps should be needed to use them.


PCIe
----

LimeSuiteNG contains "limepcie" linux kernel module for communicating with PCIe based devices.
If the PCIe devices are not detected by ``limeDevice``, inspect the system log for "limepcie" errors.
For successfully detected device the output should look like:

.. code-block:: bash

	user@computer:~$ sudo dmesg | grep limepcie
	[    3.907729] limepcie 0000:04:00.0: [Probing device]
	[    3.907766] limepcie 0000:04:00.0: enabling device (0000 -> 0002)
	[    3.907795] limepcie 0000:04:00.0: BAR0 address=0x000000008d46aa6e
	[    3.928825] limepcie 0000:04:00.0: 1 MSI IRQs allocated.
	[    3.938803] limepcie 0000:04:00.0: [device info] LimeXTRX FW:1 HW:0 PROTOCOL:1 S/N:0x0000000000000000
	[    3.938808] limepcie 0000:04:00.0: DMA channels: 1, buffer size: 8192, buffers count: 256
	[    3.938812] limepcie 0000:04:00.0: Creating /dev/LimeXTRX0_trx0
	[    3.938877] limepcie 0000:04:00.0: Creating /dev/LimeXTRX0_control

If the system log does not contain errors or does not show any messages from the "limepcie" module, check if the kernel module is actually loaded:

.. code-block:: bash

	user@computer:~$ lsmod | grep limepcie
	limepcie               45056  0

If the module is not in the loaded modules list, it can be loaded manually by:

.. code-block:: bash

	user@computer:~$ sudo modprobe limepcie
