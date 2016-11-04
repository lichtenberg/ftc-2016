#!/bin/sh
set -v

if [ "$1" == "" ] ; then
   PHONE=192.168.2.3
else
    PHONE=$1
fi

~/Library/Android/sdk/platform-tools/adb kill-server
sleep 3
~/Library/Android/sdk/platform-tools/adb tcpip 5555
sleep 3
~/Library/Android/sdk/platform-tools/adb connect $PHONE
sleep 1
~/Library/Android/sdk/platform-tools/adb devices
