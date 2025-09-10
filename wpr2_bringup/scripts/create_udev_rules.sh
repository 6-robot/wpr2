#!/bin/bash

echo "***************"
echo "remap the device serial port(ttyUSBX) to ftdi"
echo "start copy ftdi.rules to  /etc/udev/rules.d/"
sudo cp ftdi.rules  /etc/udev/rules.d

echo "remap the device serial port(ttyUSBX) to rplidar"
echo "start copy rplidar.rules to  /etc/udev/rules.d/"
sudo cp rplidar.rules  /etc/udev/rules.d

echo "set orbbec femto rules"
echo "start copy 99-obsensor-ros1-libusb.rules to  /etc/udev/rules.d/"
sudo cp 99-obsensor-ros1-libusb.rules  /etc/udev/rules.d

echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
