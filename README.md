# 2024 SEA:ME Hackathon<br>

sudo sh -c 'echo 100 > /sys/devices/pwm-fan/target_pwm'

During this hackathon, your mission is to develop a autonomous driving system based on JetRacer Pro. We'll provide you a Nvidia Jetson nano embedded board. 

![image](https://github.com/AveesLab/sea-me-hackathon-2024/assets/117966644/038d2832-94b0-416d-af1e-464430ddb012)

https://www.waveshare.com/wiki/JetRacer_Pro_AI_Kit

## Step 0: Assemble your JetRacer Pro

[Jetracer_pro_Assembly_EN.pdf](https://github.com/user-attachments/files/16007743/Jetracer_pro_Assembly_EN.pdf)

## Step 1: Install Image
1. Download the JetRacer image and unzip it.

[https://drive.google.com/file/d/1ZBdqrwhW2n1uN8rughF7Puw98o76kUcH/view?usp=sharing]

2. Download Etcher.

https://github.com/balena-io/etcher/releases/download/v1.19.21/balenaEtcher-1.19.21.Setup.exe

3. Write the image using Etcher.

![image](https://github.com/AveesLab/sea-me-hackathon-2024/assets/117966644/27d577d6-3ba9-4511-930e-979d170aa077)

## Step 2: Initial setup
1. Insert SD card to SD card slot of Jetson Nano (the slot is on the back of Jetson Nano module)
2. Power on JetRacer carrier board
3. Connect WIFI
4. Run the following commands on the terminal
```bash 
cd ~/jetracer
git checkout ws/pro
sudo python3 setup.py install
sudo reboot
```
5. Configure Power Mode
>You need to launch a new Terminal and enter the following commands to select 5W power mode.
```bash
sudo nvpmodel -m1
```
6. Resize disk partition
```
sudo apt update
sudo apt install gparted
sudo gparted
```
>Resize partition to maxsize

![3](https://github.com/AveesLab/sea-me-hackathon-2024/assets/117966644/1a451278-ae0e-4fa4-a956-d33571ff38c5)

## Step 3. Fix the apt upgrade error
```
sudo apt update
sudo apt upgrade
```
>If you see this error

![apt_upgrade_error](https://github.com/SeungWoo3/jetracer/assets/78201406/5131c1a9-b7fd-421c-9ce1-b98dba039c8f)

>Execute the lines below

```
sudo mv /var/lib/dpkg/info/ /var/lib/dpkg/info_old/
sudo mkdir /var/lib/dpkg/info
sudo apt update
sudo apt -f install
sudo mv /var/lib/dpkg/info/* /var/lib/dpkg/info_old/
sudo rm -rf /var/lib/dpkg/info
sudo mv /var/lib/dpkg/info_old/ /var/lib/dpkg/info/
sudo apt update
sudo apt upgrade
```

## Step 4. Install ROS (melodic)

http://wiki.ros.org/melodic/Installation/Ubuntu  

1. Setup your sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
2. Setup your keys
```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
3. Installation
```
sudo apt update   
sudo apt install ros-melodic-desktop-full
```
4. Environment setup
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
```
5. Dependencies for building packages
>python 2.7
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```
>python3(essential)

```
sudo apt install python3-pip python3-all-dev python3-rospkg
sudo apt install ros-melodic-desktop-full --fix-missing
```

If you make a python ros package, you should add this line at the top of python script

```python
#!~/usr/bin/env python3
```
6. Create a ROS Workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```
>python3 (optional)
```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## alias command setup
```
sudo gedit ~/.bashrc
```
>Add the below

```
++ source ~/catkin_ws/devel/setup.bash
++ alias cw='cd ~/catkin_ws'
++ alias cs='cd ~/catkin_ws/src'
++ alias cm='cd ~/catkin_ws && catkin_make'
++ alias cb='source ~/catkin_ws/devel/setup.bash'
++ alias sb='source ~/.bashrc'
++ alias sd='cd ~/catkin_ws && source devel/setup.bash'
++ alias eb='sudo gedit ~/.bashrc'
```
```
source ~/.bashrc
```
>ROS packages build

```
cm
```

## Step 5. Install ROS Packages
### Joy controller
1. Check controller connection in https://hardwaretester.com/gamepad
>#Waiting for connection

![1](https://github.com/SeungWoo3/jetracer/assets/117966644/006ea758-bcf1-46fd-8e5c-c890bb4ee344)
>#Connection successful

![2](https://github.com/SeungWoo3/jetracer/assets/117966644/742b7db2-373e-4b68-85b1-b6b0e43f947b)

2. Install the package & Run
```
sudo apt install ros-melodic-teleop-twist-joy
rosrun joy joy_node
```
3. Check the ros topic
```
rostopic echo /joy
```
### Usb camera
```
sudo apt install ros-melodic-usb-cam
```
## Step 6. Edit OpenCV settings
```
sudo gedit /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake
```
>In line 96
```
-- set(_include_dirs "include;/usr/include;/usr/include/opencv")
++ set(_include_dirs "include;/usr/include;/usr/include/opencv4") 
```
>In line 119
```
-- set(libraries "cv_bridge;/usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0;/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0;/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0") 
++ set(libraries "cv_bridge;/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.1.1;/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.1.1;/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.1.1") 
```
>Check the settings
```
cd ~/catkin_ws/src
git clone https://github.com/AveesLab/sea-me-hackathon-2024.git
cd ../
catkin_make
```
```
roscore
rosrun opencv_test opencv_test
```
## Step 7. Use arduino ROS
1. install ros package
```
sudo apt install ros-melodic-rosserial ros-melodic-rosserial-arduino
```
2. download Arduino
   
>Install this file
- https://downloads.arduino.cc/arduino-1.8.19-linuxaarch64.tar.xz?_gl=1*cef7fs*_ga*ODQ1NTYxNzQ4LjE3MTgyNjE2NDc.*_ga_NEXN8H46L5*MTcxODI2MTY0Ni4xLjEuMTcxODI2MjkzMS4wLjAuMTcwNDA3OTI1MA
>Extract arduino-1.8.19-linuxaarch64.tar in Downloads directory
```
cd ~/Downloads
tar -xvf arduino-1.8.19-linuxaarch64.tar
cd ~/Downloads/arduino-1.8.19
sudo sh install.sh
```
>After connect arduino and Jetson nano, check connection
```
ls -l /dev/ttyACM0
```
>Authorize
```
sudo usermod -a -G dialout jetson
sudo chmod a+rw /dev/ttyACM0
```
>Install rosserial library
```
cd ~/catkin_ws/src/sea-me-hackathon-2024/ultrasonic_ros
arduino ultrasonic_ros.ino
```
![다운로드](https://github.com/AveesLab/sea-me-hackathon-2024/assets/117966644/ae8e9c9f-199f-4c23-8273-0b14bb9b94fd)
![4](https://github.com/AveesLab/sea-me-hackathon-2024/assets/117966644/04736dab-03c4-41ef-8e08-6f96812eacd4)
>Fix the code
```
cd ~/Arduino/libraries/Rosserial_Arduino_Library/src/ros
gedit msg.h
```
```
-- #include <cstring>
++ #include <string.h>
-- std::memcpy
++ memcpy
```
## Step 8. Verification
```
roscore
roslaunch usb_cam usb_cam-test.launch
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600
rosrun rosrun joy joy_node
rosrun control actuator.py
rosrun todo todo
```
>You can see this picture

![rqt_graph](https://github.com/AveesLab/sea-me-hackathon-2024/assets/78201406/a55781e3-d396-42e0-a811-7b0e72f5938b)


#################################################

## to use orbbec astro pro plus
check
https://github.com/orbbec/ros_astra_camera

if cmake error 
https://jstar0525.tistory.com/306

# Assuming you have sourced the ros environment, same below
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher  libusb-1.0-0-dev libeigen3-dev


git clone https://github.com/libuvc/libuvc.git

cd libuvc

mkdir build && cd build

cmake .. && make -j4

sudo make install

sudo ldconfig

cs


git clone https://github.com/orbbec/ros_astra_camera.git

cm

#there will be a error CMake Error at /opt/ros/melodic/share/image_geometry/cmake/image_geometryConfig.cmake:113 (message): Project 'image_geometry' specifies '/usr/include/opencv' as an include dir, which is not found. It does neither exist as an absolute directory nor in '${{prefix}}//usr/include/opencv'. Check the website 'http://www.ros.org/wiki/image_geometry' for information and consider reporting the problem.

pkg-config --cflags opencv

pkg-config --cflags opencv3
pkg-config --cflags opencv4

sudo gedit /opt/ros/melodic/share/image_geometry/cmake/image_geometryConfig.cmake

line 96 opencv -> opencv4

line 119     3.2.0 -> 4.5.0

pip3 install empy==3.3.2






cw
source ./devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo  udevadm trigger

source ./devel/setup.bash 
roslaunch astra_camera astra.launch






























## after seame

#check  https://whiteknight3672.tistory.com/316

# https://velog.io/@leedohyung28/Jetson-Nano-YOLOv5-OpenCV-%EC%84%A4%EC%B9%98%ED%95%98%EA%B8%B0

#https://sehooni.github.io/jetson/projects/Jetson_CUDA_Build/#4-opencv-450-with-cuda

sudo apt-get update

sudo apt-get upgrade

sudo apt-get install nano

sudo apt-get install dphys-swapfile

# dphys-swapfile을 설치

$ sudo apt-get install dphys-swapfile

## 두 Swap 파일의 값이 다음과 같도록 값을 추가하거나, 파일 내 주석 해제

# CONF_SWAPSIZE=4096

# CONF_SWAPFACTOR=2

# CONF_MAXSWAP=4096




# /sbin/dphys-swapfile를 엽니다.

$ sudo nano /sbin/dphys-swapfile
 
## 두 Swap 파일의 값이 다음과 같도록 값을 추가하거나, 파일 내 주석 해제

# CONF_SWAPSIZE=4096

# CONF_SWAPFACTOR=2

# CONF_MAXSWAP=4096


sudo reboot

free -m

wget https://github.com/Qengineering/Install-OpenCV-Jetson-Nano/raw/main/OpenCV-4-5-0.sh

sudo chmod 755 ./OpenCV-4-5-0.sh

./OpenCV-4-5-0.sh

sudo -H pip install -U jetson-stats

sudo reboot

## jetson_release





#####>>hanjulssig


# PyTorch 1.8.0 다운로드 및 dependencies 설치

wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O torch-1.8.0-cp36-cp36m-linux_aarch64.whl

sudo apt-get install python3-pip libopenblas-base libopenmpi-dev 

# Cython, numpy, pytorch 설치

pip3 install Cython

pip3 install numpy torch-1.8.0-cp36-cp36m-linux_aarch64.whl

# torchvision dependencies 설치

sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev

git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision

cd torchvision

export BUILD_VERSION=0.9.0

python3 setup.py install --user

cd ../  # attempting to load torchvision from build dir will result in import error


#######################################

# go github download zip. and change folder name to yolov5


# yolov5s.pt weight 다운로드

wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt




##########################################
# 다음 내용 requirements.txt에서 제거

numpy>=1.18.5

opencv-python>=4.1.2

torch>=1.7.0

torchvision>=0.8.1

tensorboard 2.4.1dlsrk

############################################

python3 -m pip install --upgrade pip

python3 -m pip install -r requirements.txt


#########################
#check

# 연결된 webcam을 통해 Inference 수행하기

python3 detect.py --source 0
