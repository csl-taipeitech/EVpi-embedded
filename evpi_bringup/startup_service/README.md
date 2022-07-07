# Set up service for automaic bringup

## Hardware

1. CANDO Module
![CANDO](https://i.imgur.com/enHvStl.jpg =400x200)

2. Connection

    CAN_H ←-→ Green_line

    CAN_L ←-→ Yello_line

---
## Setup

1. Prep CanDo connection
    ```sh
    cd  ~/evpi_ws/src/EVpi-embedded/evpi_bringup/startup_service
    ./setup.sh
    ```

2. Check the status whether it shows green lights
    ```sh
    systemctl status evpi.service
    ```
    ![](https://i.imgur.com/Xb25Zn5.png)


---
## SocketCAN (Linux only)

1. List CAN devices
    ```bash
    ifconfig -a
    ```

2. Set can bitrate to 500k bps
    ```bash
    sudo ip link set up can0 type can bitrate 500000
    ```

3. Enable can0 device
    ```bash
    sudo ip link set up can0
    ```

4. Disable can0 device
    ```bash
    sudo ip link set down can0
    ```

---
## can-utils
After SocketCAN setup is finished, you can use open source project “can-utils” to test
by “cansend” and “candump”.

1. Installation
    ```bash
    sudo apt-get install can-utils
    ```

2. See the raw traffic on the CAN bus.
    ```bash
    # for all devices
    candump can0 

    # for data from id 0x123
    candump vcan0,0x123:0x7FF  

    # for data from id 0x123 and 0x456
    candump vcan0,0x123:0x7FF,0x456:0x7FF 
    ```

3. Send a single CAN message on the bus. Pass the "CAN_ID # Data"
    ```bash
    cansend can0 120#0011223344
    ```

 - See changing CAN traffic. Very useful to identify which bytes of which message contain the value for a sensor like the accelerator, brake, steering wheel sensor.
    ```bash
    cansniffer can0 -cae
    ```

 - Generate random CAN messages so their effect can be investigated. See cangen -h for details on the arguments.
    ```bash
    cangen can0
    ```

 - Replay captured CAN log file from candump.
    ```bash
    canplayer -I can_trace.asc can0
    ```
