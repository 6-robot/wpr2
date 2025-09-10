#!/bin/bash

echo "***************"
echo "delete the remap device serial port to ftdi"
sudo rm   /etc/udev/rules.d/ftdi.rules
echo "delete the remap device serial port to rplidar"
sudo rm   /etc/udev/rules.d/rplidar.rules
echo "delete the orbbec femto rules"
sudo rm   /etc/udev/rules.d/99-obsensor-ros1-libusb.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
