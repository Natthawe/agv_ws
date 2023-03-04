#!bin/bash -x
echo zzzzzzzz | sudo -S modprobe usbserial vendor=0x0590 product=0x00ca
sudo chmod 666 /dev/ttyUSB0