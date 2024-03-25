#!/bin/bash
# This script unloads module

MODULE_NAME=$1
if [ -z $MODULE_NAME ]
then
    echo "Kernel module name not specified"
    exit 1
fi

if [ `id -u` != 0 ] ; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
fi

if lsmod | grep -Eq "^$MODULE_NAME"; then
    echo "Removing module $MODULE_NAME"
    rmmod $MODULE_NAME
fi

exit 0
