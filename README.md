# XIMU
10 axis localization module for Yonder Dynamics

![XIMU module](https://github.com/infinity1096/XIMU/blob/ROS-Filtering/images/XIMU%20assembled.jpg)


# Overview
The module can provide raw sensor information to ROS network through USB. The module integrated MPU9250, MS5611, and BN-800GPS(which includes a HMC5883L) 
and can provide imformation including acceleration, angular velocity, quaternion representation of rotation, magnetic field, air pressure, temperature, and GPS position. 

This module interface with the ROS network through USB with rosserial. 

# Output description
![XIMU information flow](https://github.com/infinity1096/XIMU/blob/ROS-Filtering/images/XIMU_information_flow.png)

# Frame definition
Color corrospondence: Red - X, Green - Y, Blue - Z
![IMU frame](https://github.com/infinity1096/XIMU/blob/ROS-Filtering/images/IMU%20frame.jpg)
(This is a old picture, the MCU is upgraded to STM32F407)

# PCB 
![XIMU PCB](https://github.com/infinity1096/XIMU/blob/ROS-Filtering/images/XIMUF407_PCB_3D.png)

# Status light code

## PWR
State | LED
------------ | ------------
XIMU powered | ON(solid)
XIMU not powered | OFF

## STAT
State | ON time | OFF time | period
------------ | ------------- | ------------- | -------------
STM32 initalizing | 50% | 50% | 0.5s
Self-check / sensor initialization fault | 50% | 50% | 0.25s
Reading sensor data | 87.5% | 12.5% | 1.0s

## ROS
State | LED
------------ | ------------
synchronized with ROS network | ON(solid)
not synchronized with ROS network | OFF
