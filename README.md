# EVpi-embedded

## Prerequirements
1. Install ROS2 foxy

```bash
sudo apt update
./install_ros2_foxy_rpi.sh
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

