#!/bin/bash

function dkms_install {
    SOURCE_FILES="Makefile version.h limepcie.c limepcie.h boards.h bsp/"
    # KERNEL_DRIVER_VERSION="$(cat version)"
    KERNEL_DRIVER_VERSION="$(sed -nr 's/.*LIMEPCIE_VERSION \"([0-9]+\.[0-9]+\.[0-9]+)\"/\1/p' version.h)"

    mkdir -p "/usr/src/limepcie-$KERNEL_DRIVER_VERSION"
    cp -r $SOURCE_FILES "/usr/src/limepcie-$KERNEL_DRIVER_VERSION"
    sed -e "s/@DRIVER_VERSION@/$KERNEL_DRIVER_VERSION/" dkms.conf.in >"/usr/src/limepcie-$KERNEL_DRIVER_VERSION/dkms.conf"

    # If exists just reinstall
    MODULE_AND_VERSION=$(dkms status | grep "limepcie" | cut -d ',' -f 1)
    if [ "$MODULE_AND_VERSION" != "" ]; then
        dkms remove -m "$MODULE_AND_VERSION"
    else
        dkms add -m "limepcie/$KERNEL_DRIVER_VERSION"
    fi

    dkms install -m "limepcie/$KERNEL_DRIVER_VERSION"

    # dkms only installs, but does not load the driver, load it manually
    modprobe limepcie
}

function legacy_install {
    LOG_FILE=/var/tmp/$MODULE_NAME.log

    if ! lsmod | grep -Eq "^$MODULE_NAME"; then
        depmod
        modprobe -v "$MODULE_NAME" >>"$LOG_FILE"

        if lsmod | grep -Eq "^$MODULE_NAME"; then
            echo "Loaded kernel module: \"modprobe $MODULE_NAME\""
            echo "SUCCESS: $MODULE_NAME module loaded" >>"$LOG_FILE"
        else
            echo "Failed to load kernel module: \"modprobe $MODULE_NAME\""
            echo "FAILURE: $MODULE_NAME install failed" >>"$LOG_FILE"
            exit 1
        fi
    else
        echo "$MODULE_NAME module was loaded successfully" >>"$LOG_FILE"
    fi
}

# Check root access
if [ "$EUID" -ne 0 ]; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
fi

if [ -f uninstall-legacy-module.sh ]; then
    ./uninstall-legacy-module.sh
fi

# Default to using DKMS if available
if command -v dkms --version &>/dev/null; then
    MODE="dkms"
else
    MODE="legacy"
fi

# If first argument is an option
if [[ $1 == -* ]]; then
    if [[ $1 == "--dkms" && $MODE == "legacy" ]]; then
        echo "DKMS mode was specified but DKMS is not installed; aborting"
        exit 1
    elif [[ $1 == "--legacy" ]]; then
        MODE="legacy"
    fi
    shift
fi

if [[ $MODE == "legacy" ]]; then
    MODULE_NAME=$1
    if [ -z "$MODULE_NAME" ]; then
        echo "Kernel module name not specified"
        exit 1
    fi

    legacy_install
else # DKMS mode
    dkms_install
fi

exit 0
