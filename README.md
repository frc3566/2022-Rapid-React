# 2022-Rapid-React

St. Mark's School Team Gone Finin's code for 2022 season Rapid React. 

## Syncing system time and allowing package install on rpi:

> sudo chmod 1777 /tmp
> 
> sudo service ntp stop
> 
> sudo ntpdate 0.us.pool.ntp.org
> 
> sudo service ntp start

## Dependencies:

1. [OpenCV 4.2.0](https://github.com/JetsonHacksNano/buildOpenCV)
2. [Intel Realsense SDK (pyrealsense2)](https://github.com/JetsonHacksNano/installLibrealsense) [rpi installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_raspbian.md)

4. [pynetworktables](https://robotpy.readthedocs.io/en/stable/install/index.html)
5. [robotpy-cscore](https://robotpy.readthedocs.io/en/stable/install/cscore.html) (optional, for CameraServer video streaming)
