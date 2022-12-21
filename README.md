# Elfin Touch X teleoperation

This repository contains code to teleoperate the Elfin using Touch X haptic device commands.

## Testing environment

This repository was tested in **Ubuntu20.04 LTS (Focal Fossa) 64-bit**.  This repository was specifically tested using the following elements:

- [3D Systems Touch X](https://www.3dsystems.com/haptics-devices/touch-x) device with [OpenHaptics v3.4](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US) and the **Touch Device Driver v2022** available in the same webpage.
- [Elfin3 robot arm](https://www.hansrobot.net/elfin) from Han's Robot. If you are using a different model make sure you change the DH parameters in [robot_kinematics.cpp](src/robot_kinematics.cpp) in the ElfinModel class.
- [Realsense D405 Depth Camera](https://www.intelrealsense.com/depth-camera-d405/) from Intel. The firmware of the camera was updated to the latest version (5.14) using the *realsense-viewer*.

## Requirements

This repository depends on different external libraries that need to be installed in your local pc.

### Eigen

You can download the latest version of Eigen [here](https://eigen.tuxfamily.org/index.php?title=Main_Page). In our *CMakeLists.txt* we look for Eigen in *3rdparty* folder inside the project folder. Just create the folder and put *Eigen* on that folder.

### OpenHaptics

To download and setup OpenHaptics follow the instructions on their [webpage](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US). If you followed the instructions properly OpenHaptics shoudl be at */opt/OpenHaptics*.

### Realsense 2.0

To use the libraries make sure to follow the instructions for *librealsense* in their [github](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md). In our case, we have built from source, but both installation methods worked for me.

### OpenCV

To install OpenCV use the following commands on the terminal.

```
$ sudo apt update
$ sudo apt install libopencv-dev python3-opencv
```

### STB

We use stb to save the images into the disk following the tutorial from librealsense [here](https://github.com/IntelRealSense/librealsense/tree/master/examples/save-to-disk). You can find STB in their [github repository](https://github.com/nothings/stb). Clone their repository in your home directory with the following command.

```
$ git clone https://github.com/nothings/stb.git
```

Change **line 43** in the *[CMakeLists.txt](CMakeLists.txt)* file to match with your directory to the STB library.