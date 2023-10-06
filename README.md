# ROS2 Off-board drone control with DDS and PX4

# Installation & Setup guide

## Ground control station
- Install [Qgroundcontrol](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu)

## Pixhawk 6X setup
- PX4 v1.14 for new Micro XRCE-DDS capabilities
- Build v1.14 instructions [here](https://docs.px4.io/main/en/dev_setup/building_px4.html)

        git clone https://github.com/PX4/PX4-Autopilot.git --recursive
        git checkout release/v1.14
        make px4_fmu-v6x_default upload # Plug in Pixhawk before running this

- TODO: Params to set in QGC

- If not using DDS, but serial and mavsdk then PX4 v1.13 -> This enables HITL
- TODO: Params to set in QGC

## Setup on drone control computer (Laptop for SITL or RPi 4 on drone)
- [Install ROS2 Humble](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2)
- [Setup Micro XRCE-DDS Agent](https://docs.px4.io/main/en/ros/ros2_comm.html#setup-the-agent)

        git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
        cd Micro-XRCE-DDS-Agent
        mkdir build
        cd build
        cmake ..
        make
        sudo make install
        sudo ldconfig /usr/local/lib/

## PX4 SITL Simulation with (Ignition) Gazebo setup (On Ubuntu Laptop)
- [Install PX4 development environment](https://docs.px4.io/main/en/ros/ros2_comm.html#install-px4)

        cd
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive
        bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
        !!! Relogin or reboot computer before proceeding !!!
        cd PX4-Autopilot/
        make px4_sitl

# SITL simulation:
- Start the simulation (On Ubuntu Laptop):

        make px4_sitl gz_x500

- Start Qgroundcontrol
- Start the agent on the drone control computer:

        MicroXRCEAgent udp4 -p 8888

### Simple ROS2 sensor reader package
- Instructions [here](https://docs.px4.io/main/en/ros/ros2_comm.html#build-ros-2-workspace)

# Off-board RPi control over serial (UART->Telem 2)
- Start the agent on the drone control computer:

        TODO: MicroXRCEAgent -----SOME SERIAL WAY------ 

- To see if DDS client is running on pixhawk
    - Open Qgroundcontrol
    - Open Mavlink Console
    - Run:

            uxrce_dds_client status




# TODO: HITL with PX4 v1.13 and Gazebo classic



# References
- [PX4 DDS topics](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml)