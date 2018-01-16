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

### Setup I<sup>2</sup>C, SPI and DXHUB

```bash
rosrun sphand_driver create_udev_rules
```

### Setup auto launching

```bash
sudo apt-get install supervisor
rosrun sphand_driver create_supervisor_conf
sudo service supervisor restart
```

### Setup dynamixel motors

1. Unite baudrate to 57600 by `rosrun dynamixel_driver set_servo_config.py`
2. Set unique ID by `rosrun dynamixel_driver change_id.py`
3. Disable Overload Error in Alarm LED and Alarm Shutdown of finger\_tendon\_winder by following
```
$ ipython

In [1]: import roslib

In [2]: roslib.load_manifest('dynamixel_driver')

In [3]: from dynamixel_driver import dynamixel_io

In [4]: dxl_io = dynamixel_io.DynamixelIO("/dev/dxhub", 57600)

In [5]: dxl_io.write(3, 17, (4,))  # Please check Alarm LED address is 17

In [6]: dxl_io.write(3, 18, (4,))  # Please check Alarm Shutdown address is 18
```

## Usage

### How to kill and restart roslaunch

```bash
# Kill
sudo service supervisor stop  # Or {sudo supervisorctl shutdown}
kill <roslaunch_pid>  # Get pid by {ps aux | grep roslaunch}
# Restart
sudo service supervisor start
```

### How to see stdout/stderr from roslaunch

```bash
cat /var/log/supervisor/LaunchLeftGripper.log
```
