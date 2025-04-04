# Raspberry Pi + ROS2 + OpenCV Applications
Tested on a Raspberry Pi 4B (4GB) with Ubuntu 24.04 LTS.

## Requirements
After trying different methods to somehow use Picamera2, OpenCV, and ROS2 in a single environment,
the best solution was to build the Picamera2 and its dependencies in Ubuntu and install the rest
following the original instructions for Ubuntu.

## Reference
- Official instructions for building [libcamera](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera), [rpicam-apps](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-rpicam-apps), [kms++](https://github.com/tomba/kmsxx?tab=readme-ov-file#build-instructions), and installation steps for [Picamera2](https://github.com/raspberrypi/picamera2?tab=readme-ov-file#installation-using-pip).
- Well-organized full instruction is available at this [forum](https://github.com/raspberrypi/picamera2/issues/563#issuecomment-1981658308).
- This [answer](https://github.com/raspberrypi/rpicam-apps/issues/218#issuecomment-1020738780) solves sudo privilege issues accessing the Raspberry Pi camera modules in Ubuntu.

## Installation Steps

Here is a modified version if you install it in Ubuntu 24.04 LTS.

### Remove preinstalled packages
Check if there are packages related to older versions of libcamera:
```
dpkg -l | grep libcamera
```
Remove all the old packages.

### Resolve sudo privilege for Raspberry Pi Camera Modules in Ubuntu

First, create a rule with
```
sudo nano /etc/udev/rules.d/raspberrypi.rules
```
Then add the following line to the rule
```
SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"
```
Finally, reboot the Pi
```
sudo reboot
```

### Setup Python Virtual Environment (venv)
```
mkdir -p ~/pi_ros2_ws/src
python3 -m venv --system-site-packages ~/pi_ros2_ws/pi_ros2_venv
source ~/pi_ros2_ws/pi_ros2_venv/bin/activate
# For ROS2 builds
touch ~/pi_ros2_ws/pi_ros2_venv/COLCON_IGNORE
# The apt version is outdated. The latest version is required to be installed through pip.
pip install meson
```

### Build libcamera
```
sudo apt install -y libepoxy-dev libjpeg-dev libpng-dev python3 python3-dev
sudo apt install -y g++
sudo apt install -y python3-pip git python3-jinja2
sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev liblttng-ust-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
sudo apt install -y cmake ninja-build 
sudo apt install -y python3-yaml python3-ply python3-pip libyaml-dev
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev

cd
# Forked version of the official lib (Fixed some minor issues)
git clone https://github.com/koh43/libcamera.git
cd libcamera

meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled

ninja -C build # use -j 2 on Raspberry Pi 3 or earlier devices
sudo ninja -C build install
```

### Build rpicam-apps
```
sudo apt install -y libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install -y libavcodec-dev libavdevice-dev

cd 
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps

# For Desktop OS
meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled

# For lite/server OS
meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled

meson compile -C build # use -j1 on Raspberry Pi 3 or earlier devices
meson install -C build
```

### Build kms++
```
sudo apt install -y libfmt-dev libdrm-dev
cd
git clone https://github.com/tomba/kmsxx.git
cd kmsxx
git submodule update --init
meson build -Dpykms=enabled
ninja -C build
sudo ninja -C build install
```

### Install Picamera2
```
pip install rpi-libcamera -C setup-args="-Dversion=unknown" -C setup-args="-Drepository=https://github.com/koh43/libcamera.git" -C setup-args="-Drevision=main"
pip install rpi-kms picamera2
```


