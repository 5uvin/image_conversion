Install usb_cam, to install it from apt 
```
sudo apt-get install ros-<ros2-distro>-usb-cam
```
Follow their guide for more info: https://github.com/ros-drivers/usb_cam

Clone this repo into your workspace 
```
git clone https://github.com/5uvin/image_conversion.git
colcon build
```
Source the workspace
```
. install/setup.bash
```
Run the launch file 
```
ros2 launch image_conversion all.launch.py
```
Open RVIZ and visualize `/image_repub`, it should be in greyscale by default

To switch mode to color image
```
ros2 service call /change_mode std_srvs/srv/SetBool "{data: 1}"
```
You should see color image being published in the same topic.
