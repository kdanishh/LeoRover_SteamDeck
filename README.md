# Steam Deck - Leo Rover Integrations
# Introduction
This project uses the [Steam Deck LCD](https://store.steampowered.com/steamdeck) to control three modules on [Leo Rover](https://www.leorover.tech/). It primarily comprises:

|  Module|  Description|
|-----:|-----------|
|     1| Wheel Module|
|     2| Manipulator Module|
|     3| Pan-Tilt-Camera Module|

# Demonstration Video

[![Watch the video](https://img.youtube.com/vi/B2zQOKGyBeI/0.jpg)](https://www.youtube.com/watch?v=B2zQOKGyBeI)


# Manipulator
In this project, we have integrated [OpenMANIPULATIR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/) with the Leo Rover. The payload capacity of OpenMANIPULATIR-X is 500 g and it is composed of 5 [Dynamixel XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/) actuators. The base of the Manipulator is 3D printed for the Leo Rover and the CAD model is available [here]().

## Part List for OpenMANIPULATIR-X
|  #|  Item| Description| Quantity|
|-----|-----------|-----------|---------|
|    1| Open MANIPULATIR-X| Manipulator| 1|
|    2| U2D2 USB to Serial Adapter| USB communication converter| 1|
|    3| U2D2 PHB SET| Power Hub Board| 1|
|    4| DIN912 M5x10mm| Thread Cap Head Screws| 4|
|    5| Manipulator Base| 3D printed base for the OpenMANIPULATIR-X Manipulator| 1|


# Pan Tilt Camera
The Rover is equipped with the Pan Tilt [ArduCam B0202](https://www.arducam.com/product/arducam-1080p-low-light-wdr-ultra-wide-angle-usb-camera-module-for-computer-2mp-cmos-imx291-160-degree-fisheye-mini-uvc-usb2-0-spy-webcam-board-with-microphone-3-3ft-cable-for-windows-linux-mac-os/) for the improved field of view. The PT camera assembly comprises 2 [Dynamixel XL330-M288](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/) and some 3D printed parts. Please refer to the following list for all the required parts.
## Part List for Pan Tilt Camera
|  #|  Item| Description| Quantity|
|-----|-----------|-----------|---------|
|    1| Dynamixel XL330-M288| Smart servo| 2|
|    2| Arducam B0202| 2MP wide angle low light USB camera| 1|
|    3| FPX330-H10 Dynamixel| Frame for XL330| 2|
|    4| FPX330-S102 Dynamixel| Drame for XL330| 2|
|    5|	DIN912 M2.5x20|	M2.5x20 Socket head bolt|	4|
|    6|	DIN912 M2x5	|M2x5 socket head bolt|	4|
|    7|	Tappex 017M2.5|	M2.5 threaded insert|	4|
|    8|	Base mount|	3D Printed Base of the pan tilt device| 1|
|    9| Camera housing| 3D Printed Housing for the camera| 1|
|   10| Camera housing cover| 3D Printed	cover for the camera housing| 1|
|   11| Camera Lens cap| 3D Printed Lens cap for Arducam UC-684|	1|
|   12| Cable clip| 3D Printed Cable clip| 1|

# Power Distribution
The servo motors for the manipulator (Dynamixel XM430-W350) require 12V DC and the servo motors for the Pan Tilt Camera (Dynamixel XL330-M288) require 5V DC. We have used Leo Rover [PowerBox](https://www.leorover.tech/shop/powerbox) for the power distribution. The PowerBox module can be replace by the Leo Rover's right quarter for easy integration.The use of PowerBox is optional and the power can be distributed by using the step down power convertors.
Refer to the [U2D2 Power hub guide](https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/) for powering up the Dynamixel motors and connecting it to the Raspberry Pi.


# Installation

### STEP 1: Install Ubuntu 20.04 on Steam-Deck-LCD

As of the preparation of this document, it appears that the Steam Deck OLED version does not directly support native installation of Ubuntu LTS. To avoid the hassle of installing Ubuntu on the Steam Deck, we have chosen the Steam Deck LCD version for this project. To quickly install Ubuntu without disrupting the SteamOS environment, we prefer using Distrobox to run Ubuntu 20.04 as a subsystem from the terminal within SteamOS.

[Video Guide](https://www.youtube.com/watch?v=kkkyNA31KOA)

For Distrobox installation, first up, switch Steam Deck to Desktop mode, and run these installation scripts one after the other in the terminal app (i.e. Konsole on SteamOS)
```
curl -s https://raw.githubusercontent.com/89luca89/distrobox/main/install | sh -s -- --prefix ~/.loca
```
```
curl -s https://raw.githubusercontent.com/89luca89/distrobox/main/extras/install-podman | sh -s -- --prefix ~/.local
```
Next,  add the directories it uses into the .bashrc file, so we can run commands as normal in terminal.
```
nano ~/.bashrc
```
Add these three lines to the bottom of the .bashrc file:

```
export PATH=$HOME/.local/bin:$PATH
export PATH=$HOME/.local/podman/bin:$PATH
xhost +si:localuser:$USER
```
You can now install a Linux distribution using Distrobox. For this project, we are using Ubuntu 20.04, as the Leo Rover is equipped with a Raspberry Pi running ROS Noetic on Ubuntu 20.04. To maintain consistency with the ROS version, we also install Ubuntu 20.04 with Distrobox.

```
distrobox create -i ubuntu:20.04
```
Then once done, you can enter Ubuntu with distrobox:
```
distrobox enter ubuntu-20-04
```
This is where the fun begins. Now you can install or do whatever you like inside Ubuntu.

### STEP 2: Install ROS Noetic inside the Distrobox ubuntu-20-04

```
sudo apt install x11-xserver-utils
sudo apt install lsb-release
sudo apt install ros-noetic-desktop-full
```
After ROS installation, source the ROS environment:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Then install ROS dependencies and initialize rosdep
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### STEP 3: Build Openmanipulator Package

Create a catkin workspace and clone those packages from this repository to the source folder:
```
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone https://github.com/everskyrube/LeoRover_SteamDeck.git
```
You can now build the packages using catkin_make:
```
cd ~/catkin_ws && catkin_make
```

### STEP 4: Grant Permission to access the serial port ttyUSB(x)

Assume that you have correctly connected the Openmanipulator to the U2D2 Power Hub Board, to find the _use_port_ connected with the U2D2:
**_NOTE: The ttyUSB port number will change sometimes when you disconnect and re-connect the serial port_**
```
ls -l /dev/ttyUSB*
```
Grant permission to access the corresponding serial port ttyUSB(x), in our case, the openmanipulator is connected with ttyUSB0:

```
sudo chmod 666 /dev/ttyUSB0
```
### STEP 5: Switch joystick to Gamepad mode, and verify the input from Steam Deck joystick:

**_NOTE: For steamdeck, the default controls of the joystick is desktop mode, to switch to gamepad mode, hold the menu/start button to switch the mode of using steamdeck, (the 3 line button on the right side)_**
```
sudo apt install joystick jstest-gtk
jstest /dev/input/js0
```

### STEP 6: Source the environment and launch the package. Remember to update the _usb_port_ argument to match the connected port.
**_NOTE:Manually hold the manipulator hardware to a proper position (i.e. home position) when you are launching the ROS package_**
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=true usb_port:=/dev/ttyUSB0
```
If the manipulator_controlloer is successfully launched, you will see some ouput from the terminal like this:
```
##port_name and baud_rate are set to /dev/ttyUSB0, 1000000 
##Joint Dynamixel ID : 11, Model Name : XM430-W350
##Joint Dynamixel ID : 12, Model Name : XM430-W350
##Joint Dynamixel ID : 13, Model Name : XM430-W350
##Joint Dynamixel ID : 14, Model Name : XM430-W350
##Gripper Dynamixel ID : 15, Model Name :XM430-W350
```
### STEP 7: Open a separated terminal window, and launch the joystick teleop node.

```
distrobox enter ubuntu-20-04
cd ~/catkin_ws
source devel/setup.bash
roslaunch open_manipulator_teleop open_manipulator_teleop_joystick.launch 
```
You're in! Now, have fun playing with it!


### For the integration with other modules, stay tuned for further updates...
