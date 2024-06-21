#!/bin/bash

function dkms_uninstall {
    MODULE_AND_VERSION=$(dkms status | grep "limepcie" | cut -d ',' -f 1)
    if [ "$MODULE_AND_VERSION" == "" ]; then
        echo "Module not found; doing nothing."
        exit 0
    fi

    printf "Uninstalling %s\n" "$MODULE_AND_VERSION"

    dkms uninstall -m "$MODULE_AND_VERSION"
    dkms remove -m "$MODULE_AND_VERSION"
    rm -rf "/usr/src/$MODULE_AND_VERSION"

    printf "Uninstalled %s\n" "$MODULE_AND_VERSION"
}

function legacy_uninstall {
    if ! "$(dirname "$0")/unload.sh" "$MODULE_NAME"; then
        exit 1
    fi

    # Disable auto loading of the module
    echo "Removing module $MODULE_NAME from /etc/modules"
    temp_file=$(mktemp)
    chown --reference=/etc/modules "$temp_file"
    chmod --reference=/etc/modules "$temp_file"
    grep -vx "^$MODULE_NAME" /etc/modules >"$temp_file"
    mv "$temp_file" /etc/modules
}

# Check root access
if [ "$EUID" -ne 0 ]; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
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

    legacy_uninstall
else
    dkms_uninstall
fi

exit 0
