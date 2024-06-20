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



