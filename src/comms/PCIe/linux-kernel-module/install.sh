#!/bin/bash
KERNEL_DRIVER_VERSION="0.1.0"

# Check root access
if [ "$EUID" -ne 0 ]; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
fi

if [ -f uninstall-legacy-module.sh ]; then
    ./uninstall-legacy-module.sh
fi

cp -r . "/usr/src/limepcie-$KERNEL_DRIVER_VERSION"
dkms add -m limepcie/$KERNEL_DRIVER_VERSION
dkms install -m limepcie/$KERNEL_DRIVER_VERSION

exit 0
