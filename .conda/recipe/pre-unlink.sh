#!/usr/bin/env bash

sudo rm /etc/udev/rules.d/65-limesuiteng-pcie.rules
sudo rm /etc/udev/rules.d/65-limesuiteng-usb.rules

sudo udevadm control --reload-rules
sudo udevadm trigger