cv_bridge:

```
sensor_msgs
cv_bridge
roscpp
std_msgs
image_transport
```

Create Package

```sh
$ catkin_create_pkg localization_with_artrack_cv roscpp usb_cam std_msgs geometry_msgs tf2 tf2_ros cv_bridge image_transport
```

Deps:

```sh
https://github.com/ros-perception/vision_opencv/tree/kinetic
$ sudo apt-get install ros-kinetic-vision-opencv
```

