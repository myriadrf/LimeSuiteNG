#!/bin/bash
KERNEL_DRIVER_VERSION="$(cat version)"

# Check root access
if [ "$EUID" -ne 0 ]; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
fi

dkms uninstall -m limepcie/$KERNEL_DRIVER_VERSION
dkms remove -m limepcie/$KERNEL_DRIVER_VERSION
rm -rf "/usr/src/limepcie-$KERNEL_DRIVER_VERSION"

exit 0
