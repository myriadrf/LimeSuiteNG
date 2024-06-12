#!/bin/bash
KERNEL_DRIVER_VERSION="$(cat version)"

# Check root access
if [ "$EUID" -ne 0 ]; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
fi

if [ -f uninstall-legacy-module.sh ]; then
    ./uninstall-legacy-module.sh
fi

SOURCE_FILES="limepcie.c limepcie.h boards.h bsp/"
cp -r $SOURCE_FILES "/usr/src/limepcie-$KERNEL_DRIVER_VERSION"
sed -e "s/@DRIVER_VERSION@/$KERNEL_DRIVER_VERSION/" dkms.conf.in > "/usr/src/limepcie-$KERNEL_DRIVER_VERSION/dkms.conf"

dkms add -m limepcie/$KERNEL_DRIVER_VERSION
dkms install -m limepcie/$KERNEL_DRIVER_VERSION

exit 0
