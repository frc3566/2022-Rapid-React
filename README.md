# 2022-Rapid-React

St. Mark's School Team Gone Fishin's code for 2022 season Rapid React. 

## Syncing system time and allowing package install on rpi:

> sudo chmod 1777 /tmp
> 
> sudo service ntp stop
> 
> sudo ntpdate 0.us.pool.ntp.org
> 
> sudo service ntp start

## Dependencies:
1. [wpilibpi rpi image] (https://github.com/wpilibsuite/WPILibPi/releases)
2. [OpenCV](https://github.com/JetsonHacksNano/buildOpenCV) [rpi installation guide] (https://qengineering.eu/install-opencv-4.5-on-raspberry-pi-4.html)
3. [Intel Realsense SDK (pyrealsense2)](https://github.com/JetsonHacksNano/installLibrealsense) [rpi installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_raspbian.md)
4. [pynetworktables](https://robotpy.readthedocs.io/en/stable/install/index.html)
5. [robotpy-cscore](https://robotpy.readthedocs.io/en/stable/install/cscore.html)
