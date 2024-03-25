#!/bin/bash
# Check root access
SOURCE_LOC="/usr/src/litepcie"

if [ `id -u` != 0 ] ; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
fi

mkdir -p $SOURCE_LOC
cp -r . $SOURCE_LOC
chmod +x $SOURCE_LOC/install_litepcie.sh

#check if module is loaded
if lsmod | grep -Eq "^litepcie"; then
	rmmod litepcie
fi

sh $SOURCE_LOC/litepcie_loader.sh

exit 0

