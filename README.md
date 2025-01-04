# Raspberry Pi + ROS2 + OpenCV Applications

## Requirements
After trying different methods to somehow use Picamera2, OpenCV, and ROS2 in a single environment,
the best solution was to build the Picamera2 and its dependencies in Ubuntu and install the rest
following the original instructions for Ubuntu.

## Reference
- Official instructions for building [libcamera](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera), [rpicam-apps](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-rpicam-apps), [kms++](https://github.com/tomba/kmsxx?tab=readme-ov-file#build-instructions), and installation steps for [Picamera2](https://github.com/raspberrypi/picamera2?tab=readme-ov-file#installation-using-pip).
- Well-organized full instruction is available at this [forum](https://github.com/raspberrypi/picamera2/issues/563#issuecomment-1981658308).

## Installation Steps

Here is a modified version if you install it in Ubuntu 24.04 LTS.

### Setup Python Virtual Environment (venv)
```
mkdir -p venv
python3 -m venv --system-site-packages venv/my_venv
source venv/my_venv/bin/activate
# The apt version is outdated. The latest version is required to be installed through pip.
pip install meson
```

### Build libcamera
```
sudo apt install -y libcamera-dev libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev python3 python3-dev
sudo apt install -y libcamera-tools g++
sudo apt install -y python3-pip git python3-jinja2
sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev liblttng-ust-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
sudo apt install -y cmake ninja-build 
sudo apt install -y python3-yaml python3-ply python3-pip libyaml-dev
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev

cd 
git clone https://github.com/raspberrypi/libcamera.git
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
git clone https://github.com/raspberrypi/rpicam-apps.git --branch v1.4.3
cd rpicam-apps

# For Desktop OS
meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled

# For lite/server OS
meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled

meson compile -C build # use -j1 on Raspberry Pi 3 or earlier devices
meson install -C build
sudo ldconfig # This is only necessary on the first build
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
pip install rpi-libcamera
pip install rpi-kms picamera2
```


