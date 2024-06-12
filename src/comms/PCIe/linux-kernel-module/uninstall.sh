#!/bin/bash
# uninstalls specified module from system directory

MODULE_NAME=$1
if [ -z "$MODULE_NAME" ]
then
    echo "Kernel module name not specified"
    exit 1
fi

if [ "$EUID" -ne 0 ]; then
    echo -e "\033[33mWarning, script must be run with root permissions\033[0m"
    exit 1
fi


if ! output=$("$(dirname "$0")/unload.sh" "$MODULE_NAME"); then
    exit $output
fi

# Disable auto loading of the module
echo "Removing module $MODULE_NAME from /etc/modules"
temp_file=$(mktemp)
chown --reference=/etc/modules "$temp_file"
chmod --reference=/etc/modules "$temp_file"
grep -vx "^$MODULE_NAME" /etc/modules > "$temp_file"
mv "$temp_file" /etc/modules

exit 0