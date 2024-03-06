# QuadCopter-Autonomous-Pilot-Using-Pixhawk-Gazebo-MAVSDK-python

# Author: Fida'uddin bin Faizol Fadzli

This project is a project where an autonomous pilot quadcopter using the pixhawk V2 as the microcontroller is decveloped. The purpose of this project is to develop a drone that can monitor and identify pollution levels and moving automatically based on the readings of the gas sensor.

# Download and Setup for MAVSDK Library

Dowload Ubuntu 20.04


sudo pip install git
mkdir src
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh


git clone https://github.com/PX4/Firmware.git --recursive
cd Firmware
bash ./Tools/setup/ubuntu.sh
sudo reboot now

#test simulation
cd src
cd Firmware
make px4_sitl gazebo-classic


#download mavsdk
sudo pip install mavsdk
sudo pip install aioconsole

#new terminal to test python code
#basic.py letak dalam Firmware file
cd src 
cd Firmware
apython basic.py

#Dowload Qgroundcontrol
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y

#Logout and login again to enable the change to user permissions in ubuntu

Download QGroundControl.AppImage. #cari di qground control website #move file ni dari dl ke home.
#Install (and run) using the terminal commands:
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage

# Download and Setup for DroneKit Library
pip install dronekit
pip install dronekit-SITL

pip install dronekit-sitl -UI
dronekit-sitl copter

refer this link: https://dronekit-python.readthedocs.io/en/latest/develop/sitl_setup.html

Connecting vehicle to gazebo
python vehicle_state.py --connect 127.0.0.1:14550

