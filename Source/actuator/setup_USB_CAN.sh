#!/bin/bash
str=".................Setting up USB_CAN ............."
echo $str
sudo slcand -o -c -f -s8 /dev/ttyUSB_CAN slcan0
sudo ifconfig slcan0 up
echo "................USB_CAN Connected..............."
echo $str
