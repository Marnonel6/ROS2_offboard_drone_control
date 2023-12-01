# ROS2 Off-board drone control with DDS and PX4

High-level path planning and off-board control of a Pixhawk flight computer running PX4 with a 
Raspberry Pi 4B. The code is written in C++ and Python in a ROS2 Humble interface.

## Prerequisites:
- Ubuntu 22.04
- Python3
- C++
- ROS2 Humble

# Installation & Setup guide

## Ground control station
- Install [Qgroundcontrol](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu)

## Pixhawk 6X setup
- PX4 v1.14 for new Micro XRCE-DDS capabilities
- `NOTE` version 1.14 is now a stable version so do not have to build from source anymore you can
use QGC to install it
- Build v1.14 instructions [here](https://docs.px4.io/main/en/dev_setup/building_px4.html)

        git clone https://github.com/PX4/PX4-Autopilot.git --recursive
        git checkout release/v1.14
        make px4_fmu-v6x_default upload # Plug in Pixhawk before running this

- If not using DDS, but serial and mavsdk then PX4 v1.13 -> This enables HITL
- TODO: Params to set in QGC

        LIST PARAMS HER __________________________________________________________________________________________________________________________________

## Setup on drone control computer (Laptop for SITL or RPi 4 on drone)
- [Install ROS2 Humble](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2)
- [ROS2 Off-board drone control](https://github.com/Marnonel6/ROS2_offboard_drone_control)

        git clone git@github.com:Marnonel6/ROS2_offboard_drone_control.git

    - To clone the ROS2 control package an alias with the deployment key also exists on the Pi. Use:

            osprey_code_pull
- [Setup Micro XRCE-DDS Agent](https://docs.px4.io/main/en/ros/ros2_comm.html#setup-the-agent)

        git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
        cd Micro-XRCE-DDS-Agent
        mkdir build
        cd build
        cmake ..
        make
        sudo make install
        sudo ldconfig /usr/local/lib/
- [Fields2Cover Fork](https://github.com/Marnonel6/Fields2Cover) or [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover)

        sudo apt-get update
        sudo apt-get install --no-install-recommends software-properties-common
        sudo add-apt-repository ppa:ubuntugis/ppa
        sudo apt-get update
        sudo apt-get install --no-install-recommends build-essential ca-certificates cmake \
            doxygen g++ git libeigen3-dev libgdal-dev libpython3-dev python3 python3-pip \
            python3-matplotlib python3-tk lcov libgtest-dev libtbb-dev swig libgeos-dev
        python3 -m pip install gcovr

        git clone git@github.com:Marnonel6/Fields2Cover.git
        cd Fields2Cover
        mkdir -p build;
        cd build;
        cmake -DCMAKE_BUILD_TYPE=Release ..;
        make -j$(nproc);
        sudo make install;

    - On RPi I have a deployment key with an alias in the `.bashrc` file to clone the repo. Use:
            fields2cover_code_pull

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

- Start `Qgroundcontrol``
- Start the `DDS agent`` on the drone control computer:

        MicroXRCEAgent udp4 -p 8888
- Running `ros2 topic list` should now display all the flight computer topics
- Alternatively to running the agent you can run the off-board control ROS2 package with:

        ros2 launch drone_control offboard_control.launch.py 

- This will plan a path and then control the drone to follow the path.

# Steps to run off-board autonomous mission on drone:
- Start `Qgroundcontrol``
- Run the launch file to start the agent and all the nodes:

        ros2 launch drone_control offboard_control.launch.py 

- This will plan a path and then control the drone to follow the path.

# References
- [PX4 DDS topics](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml)
- Simple ROS2 sensor reader package [here](https://docs.px4.io/main/en/ros/ros2_comm.html#build-ros-2-workspace)