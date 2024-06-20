# Steam Deck - Leo Rover Integrations
# Introduction
This project uses the [Steam Deck LCD](https://store.steampowered.com/steamdeck) to control three modules on [Leo Rover](https://www.leorover.tech/). It primarily comprises:

|  Module|  Description|
|-----:|-----------|
|     1| Wheel Module|
|     2| Manipulator Module|
|     3| Pan-Tilt-Camera Module|

However, the integration is not yet complete. This documentation includes only the first two modules and is prepared for temporary reference. 

# Demonstration Video

[![Watch the video](https://img.youtube.com/vi/B2zQOKGyBeI/0.jpg)](https://www.youtube.com/watch?v=B2zQOKGyBeI)

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
This is where the fun begines. Now you can install or do whatever you like inside Ubuntu.

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
**_NOTE: For steamdeck, the default controls of the joystick is desktop mode, to switch to gamepad mode, hold the menu/start button to switch the mode of using steamdeck, (the 3 line button on the right side)_**

To verify the input from Steam Deck joystick, you can use following commands:
```
sudo apt install joystick jstest-gtk
jstest /dev/input/js0
```

