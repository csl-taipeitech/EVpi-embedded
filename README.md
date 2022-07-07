# EVpi-embedded

## Prerequirements
1. Install ROS2 foxy

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
```

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
apt-cache policy | grep universe
sudo apt update && sudo apt install curl gnupg2 lsb-release
```

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
```

```bash
sudo apt install python3-rosdep2
rosdep update
sudo apt install python3-colcon-common-extensions
```

```bash
sudo apt install ros-foxy-ros-base
```

```bash
source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> .bashrc
```


## Dependency

- [ROS2](https://docs.ros.org/en/foxy/Installation.html) (tested with Foxy)
  ```
  sudo apt-get install -y ros-foxy-ackermann-msgs
  sudo apt-get install -y ros-foxy-can-msgs
  sudo apt-get install -y ros-foxy-joy* ros-foxy-teleop*
  ```

- [can-util](https://github.com/linux-can/can-utils)
  ```
  sudo apt-get install can-utils
  ```

- other (optional)
    ```
    sudo apt install network-manager
    sudo apt install net-tools
    ```

## Install

Use the following commands to download and compile the package.
```sh
mkdir -p ~/evpi_ws/src && cd ~/evpi_ws/src
git clone https://github.com/csl-taipeitech/EVpi-embedded.git
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
. install/setup.bash
echo "source ~/evpi_ws/install/setup.bash" >> .bashrc
```

## Common used commands
1. Bringup can_manager
    ```bash
    ros2 launch evpi_bringup can_bringup.launch
    ```

