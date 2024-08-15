#!/bin/bash
# This script unloads module

MODULE_NAME=$1
if [ -z "$MODULE_NAME" ]; then
    echo "Kernel module name not specified"
    exit 1
fi

if [ "$(id -u)" != 0 ]; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
fi

if lsmod | grep -Eq "^$MODULE_NAME"; then
    echo "Unloading module: \"rmmod $MODULE_NAME\""
    rmmod "$MODULE_NAME"
fi

if lsmod | grep -Eq "^litepcie"; then
    echo "Unloading module: \"rmmod litepcie\""

    if ! rmmod litepcie 2>/dev/null 1>&2; then
        echo -e "\033[33mCould not unload legacy driver. Reboot device to finalize installation.\033[0m"
    fi
fi

exit 0
