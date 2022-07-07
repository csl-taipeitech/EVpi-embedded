#!/bin/bash
# Apache License 2.0
# Copyright (c) 2020, ROBOTIS CO., LTD.

echo ""
echo "[Note] OS version  >>> Ubuntu 20.04 (Focal Fossa)"
echo "[Note] Target ROS version >>> ROS 2 Foxy Fitzroy"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set Locale]"
sudo apt update
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
apt-cache policy | grep universe
sudo apt install software-properties-common
sudo add-apt-repository universe
echo ""

echo "[Setup Sources]"
sudo rm -rf /var/lib/apt/lists/* && sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'
echo ""

echo "[Install ROS 2 packages]"
sudo apt update && sudo apt install -y ros-foxy-ros-base
echo ""

echo "[Install dependencies]"
sudo apt update && sudo apt install python3-rosdep2
rosdep update
sudo apt install python3-colcon-common-extensions
echo ""

echo "[Environment setup]"
source /opt/ros/foxy/setup.bash
sudo apt install -y python3-colcon-common-extensions git
echo ""

echo "[Set the ROS evironment]"
sh -c "echo \"source /opt/ros/foxy/setup.bash\" >> ~/.bashrc"

exec bash

echo "[Complete!!!]"
exit 0
