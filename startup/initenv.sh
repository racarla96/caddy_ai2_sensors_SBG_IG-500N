#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="sbg"' > /etc/udev/rules.d/sbg.rules

service udev reload
sleep 2
service udev restart