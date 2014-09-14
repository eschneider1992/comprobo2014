#!/bin/bash


echo "Starting up adhoc wireless"
echo "Make sure to use sudo when executing"
echo ""

stop network-manager
ifconfig wlan0 down
sleep 2
iwconfig wlan0 mode ad-hoc
iwconfig wlan0 channel 1
iwconfig wlan0 key aaaaa11111
iwconfig wlan0 essid $1
ifconfig wlan0 10.0.0.100 netmask 255.255.255.0 up

ping 10.0.0.200
