#!/bin/sh
#if [ $1 == "up" ]
#then
ifconfig wlan0 up > /dev/null 
ifconfig wlan0 192.168.100.1 > /dev/null  
route add default gw 192.168.100.1 wlan0  

hostapd -B /netPrivate/hostapd.conf
if [ $? -ne 0 ] ; then
	cp /etc/hostapd.conf  /netPrivate/
#	MAC=$(cat /sys/class/net/wlan0/address|sed 's/://g')
#	hw=$(echo ${MAC:6})
	hw=`wifi_ssid_get`
	sed -i "s/^ssid=.*$/ssid=Sichuang_$hw/g" /netPrivate/hostapd.conf
	hostapd -B /netPrivate/hostapd.conf
	if [ $? -ne 0 ] ; then
		killall hostapd
		hostapd -B /netPrivate/hostapd.conf
	fi
fi

if [ ! -e "/netPrivate/config.ini" ];
then
	cp /img_app/bin/config.ini /netPrivate
fi

sync 
blockdev --flushbufs /dev/mtdblock4

udhcpd /etc/udhcpd.conf > /dev/null 
ftpd  -f /etc/ftpd.conf > /dev/null
#elif [ $1 == "down" ]
#then
#kill `pidof ftpd`
#kill `pidof udhcpd`
#kill `pidof hostapd`
#ifconfig wlan0 down

#elif [ $1 == "get_status" ]
#then
#ifconfig -a | grep "mon.wlan0" > /dev/null
#echo "$?"
#fi
