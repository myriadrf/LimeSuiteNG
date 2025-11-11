#!/usr/bin/env bash

sudo mv ${PREFIX}/etc/udev/rules.d/65-limesuiteng-pcie.rules /etc/udev/rules.d
sudo mv ${PREFIX}/etc/udev/rules.d/65-limesuiteng-usb.rules /etc/udev/rules.d

sudo udevadm control --reload-rules
sudo udevadm trigger