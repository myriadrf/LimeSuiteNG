#!/bin/bash

# Test if old legacy module was installed into system by comparing modinfo status code
# Workaround to check if module installed without any device or module loaded
if [ `modinfo litepcie 1> /dev/null 2> /dev/null; echo $?` -ne 0 ]
then
    # No legacy module found
    exit 0
fi

if [ `id -u` != 0 ] ; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
fi

if [ `  modinfo -F alias litepcie | \
        grep -i -e v0*10eed0*7022   \
                -e v0*10eed0*7023   \
                -e v0*1172d0*e001 | \
        wc -l` -eq 3 ]
then
    echo "Deleting legacy driver"
    sed -i '/^litepcie$/d' /etc/modules
    modinfo -F filename litepcie | xargs rm
fi