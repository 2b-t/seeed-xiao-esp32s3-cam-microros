# Seeed Studio Xiao ESP32-S3 Sense Cam with micro-ROS

Author: [Tobit Flatscher](https://github.com/2b-t) (2024)



## Overview

The following example **streams camera data** captured by a [Seeed Studio Xiao ESP32-S3 Sense](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) microcontroller to a computer running [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) over Wifi using [**micro-ROS**](https://micro.ros.org/).



## Testing without micro-ROS

For the basic set-up of the ESP32-S3 with ESP-IDF refer to [this article by Espressif](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) as well as [this guide for Linux](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/linux-macos-setup.html). [This tutorial](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/linux-macos-setup.html#get-started-linux-macos-first-steps) will walk you through a simple 'Hello World' example. In my case I first had to source `$ source /opt/esp/idf/export.sh`, then add my user to the dialout group with `$ sudo adduser $USER dialout` as described [here](https://askubuntu.com/a/112572), had to allow the user with `$ sudo chmod o+rw /dev/ttyACM0` to write to the port and finally monitor the output of the board with `$ idf.py -p /dev/ttyACM0 monitor`. Espressif furthermore offers a [ESP32 camera component](https://github.com/espressif/esp32-camera) that can be used to interface the camera as discussed [here](https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/). They also offer a [Docker image](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-docker-image.html) in case you do not want to install their toolchain directly on your development machine. For the ESP32-S3 I had to [change the pins](https://github.com/Seeed-Studio/SSCMA-Micro/blob/dev/porting/espressif/boards/seeed_xiao_esp32s3/board.h) and then with `.fb_location = CAMERA_FB_IN_DRAM` their example worked out of the box while for `CAMERA_FB_IN_PSRAM` I had to add `esp_psram` as a `PRIV_REQUIRES` inside the `CMakeLists.txt` as well as changing `Change Component config -> Camera configuration -> I2C peripheral to use for SCCB` to `I2C0` as described [here](https://github.com/espressif/esp32-camera/issues/450#issuecomment-1432009688).

## ROS 2 and micro-ROS

ROS 2 is build around the [ROS 2 C Client Library `rcl`](https://github.com/ros2/rcl). This library is wrapped and binded into other programming languages, e.g. [`rclc`](https://github.com/ros2/rclc) for C. The [micro-ROS project](https://micro.ros.org/) is an effort towards developing an micro-controller optimized client API based on `rcl` and `rclc` that supports all major ROS 2 concepts and this way allows to integrate them seamlessly into ROS 2. [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/) is used as a middleware in combination with a POSIX-based real-time operating system (RTOS for short; [FreeRTOS](https://www.freertos.org/), [Zephyr](https://www.zephyrproject.org/) or [NuttX](https://nuttx.apache.org/)). In the [list of supported micro-controllers](https://micro.ros.org/docs/overview/hardware/) you will also find Espressif's ESP32.

### micro-ROS Component for ESP-IDF

There are several ways of using micro-ROS but if you still want to leverage the advantages of other existing packages that are traditionally used in programming of ESP32 micro-controllers (so called [components](https://docs.espressif.com/projects/idf-component-manager/en/latest/), see [here](https://components.espressif.com/) for a list of components), you should use the corresponding **micro-ROS component for ESP-IDF** that can be found [here](https://github.com/micro-ROS/micro_ros_espidf_component). Again this project comes with a [Docker image](https://github.com/micro-ROS/micro_ros_espidf_component#build-with-docker-container) that extends the ESP-IDF Docker image. In order to be able to run the Docker image I had to change my locales settings by exporting `$ export LC_ALL=C` as described [here](https://stackoverflow.com/a/37112094) and had to delete the `.gitconfig` inside my Docker container with `$ /home/$USER/.gitconfig` as it caused problems with the supplied scripts (see [here](https://github.com/micro-ROS/micro_ros_espidf_component/issues/183#issuecomment-1837516081) for more details).

One can choose in between different transports, in particular UDP over Wifi or Ethernet or alternatively through UART. Most tutorials assume UDP but stress too little that the configuration of the Wifi connection with `$ idf.py menuconfig` is essential. After start-up the micro-controller will try to connect to the access point (if you monitor it, it outputs access point, password and the IP it has been assigned) and will then try to contact the micro-ROS agent running on the remote computer connected to the same network. In case it fails to connect to the micro-ROS agent it will output the error message [`Failed status on line 54: 1. Aborting.`](https://answers.ros.org/question/387035/esp32-micro-ros-failed-status-on-line-87-1-aborting/) where the line it refers to will contain `RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));`. In this case the micro-controller successfully connected to the Wifi but failed to contact the micro-ROS agent which suggests network issues or wrongly set IPs. First make sure that you can ping the micro-controller from the PC running the agent, then check the settings in `$ idf.py menuconfig`, making sure that in `micro-ROS Settings -> Wifi Configuration` you set the correct `WiFi SSID` and `WiFi Password` and that under **`micro-ROS Agent IP` you set the expected IP of the PC running the agent** (see [here](https://robofoundry.medium.com/esp32-micro-ros-actually-working-over-wifi-and-udp-transport-519a8ad52f65) for more details). Similar the micro-ROS agent should use the IP of the micro-controller as a launch argument. Another potential problem might be wrong settings to `rmw_microxrcedds` inside `colcon.meta` as described [here](https://github.com/micro-ROS/micro_ros_setup/issues/526).



## Running

This code can be run by opening the container and then running the following commands:

```bash
$ sudo chown -R $(whoami) /code
$ source /opt/esp/idf/export.sh
$ idf.py set-target esp32s3
$ idf.py menuconfig # Optional changes
$ idf.py build
$ sudo chmod o+rw /dev/ttyACM0
$ idf.py -p /dev/ttyACM0 flash
$ idf.py -p /dev/ttyACM0 monitor
# Press Ctrl + ] to exit
```





