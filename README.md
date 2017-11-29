# sphand_ros

Packages which provide ROS tools for the Suction Pinching Hand after Ver.7.0.

## Installation

### Install Ubuntu 14.04 and ROS Indigo to UP Board

Please follow <https://01.org/developerjourney/installing-ubutnu-1404-lts-intel-realsense-robotic-development-kit>.

#### Caution

Please do `sudo apt-get -y autoremove --purge 'linux-.*generic'` again after rebooting for installing `linux-upboard`.

### Install Astra Camera

Please follow [jsk_recognition documentation](https://jsk-recognition.readthedocs.io/en/latest/install_astra_camera.html).

### Install C/C++ Library in libmraa

```bash
sudo add-apt-repository ppa:mraa/mraa
sudo apt-get update
sudo apt-get install libmraa1 libmraa-dev mraa-tools
```
Details are on [GitHub page](https://github.com/intel-iot-devkit/mraa).

### Install sphand_ros and its dependencies

```bash
mkdir -p ~/apc_ws/src
cd ~/apc_ws/src
git clone https://github.com/pazeshun/sphand_ros.git
ln -s sphand_ros/fc.rosinstall .rosinstall
wstool update
cd JSK_APC_WS
ln -s jsk_apc/.travis.rosinstall .rosinstall
wstool update
cd ~/apc_ws/src
rosdep install -y -r --from-paths . --ignore-src
sudo apt-get install python-catkin-tools ros-indigo-jsk-tools
catkin build
source ~/apc_ws/devel/setup.bash
```

### Setup DXHUB

```bash
rosrun sphand_driver create_udev_rules
```
