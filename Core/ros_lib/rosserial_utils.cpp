/*
 * rosserial_utils.cpp
 *
 *  Created on: 2020年8月24日
 *      Author: yuche
 */

#include "rosserial_utils.h"
#include "stdint.h"

#include "ros.h"
#include "ros/time.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Temperature.h"


sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
sensor_msgs::NavSatFix fix_msg;
sensor_msgs::FluidPressure air_ps_msg;
sensor_msgs::Temperature temp_msg;


ros::NodeHandle nh;

ros::Publisher imu_pub("/imu",&imu_msg);
ros::Publisher mag_pub("/mag",&mag_msg);
ros::Publisher gps_pub("/fix",&fix_msg);
ros::Publisher pressure_pub("/air_pressure",&air_ps_msg);
ros::Publisher temp_pub("/temperature",&temp_msg);


extern "C" void ros_init(){
	nh.initNode();
	nh.advertise(imu_pub);
	nh.advertise(mag_pub);
	nh.advertise(gps_pub);
	nh.advertise(pressure_pub);
	nh.advertise(temp_pub);
	nh.spinOnce();
}

extern "C" void IMU_pub(double a[3], double w[3], double q[4]){

	imu_msg.header.stamp = nh.now();
	imu_msg.header.frame_id = "XIMU";

	imu_msg.linear_acceleration.x = a[0];
	imu_msg.linear_acceleration.y = a[1];
	imu_msg.linear_acceleration.z = a[2];

	imu_msg.angular_velocity.x = w[0];
	imu_msg.angular_velocity.y = w[1];
	imu_msg.angular_velocity.z = w[2];

	imu_msg.orientation.w = q[0];
	imu_msg.orientation.x = q[1];
	imu_msg.orientation.y = q[2];
	imu_msg.orientation.z = q[3];

	imu_pub.publish(&imu_msg);
}

extern "C" void MAG_pub(double m[3]){

	mag_msg.header.stamp = nh.now();
	mag_msg.header.frame_id = "XIMU";

	mag_msg.magnetic_field.x = m[0];
	mag_msg.magnetic_field.y = m[1];
	mag_msg.magnetic_field.z = m[2];

	mag_pub.publish(&mag_msg);
}

extern "C" void GPS_pub(double lla[3]){
	fix_msg.header.stamp = nh.now();

	fix_msg.latitude = lla[0];
	fix_msg.longitude = lla[1];
	fix_msg.altitude = lla[2];

	gps_pub.publish(&fix_msg);

}

extern "C" void PT_pub(double pt[2]){
	air_ps_msg.header.stamp = nh.now();
	air_ps_msg.fluid_pressure = pt[0];

	pressure_pub.publish(&air_ps_msg);

	temp_msg.header.stamp = nh.now();
	temp_msg.temperature = pt[1];

	temp_pub.publish(&temp_msg);
}

extern "C" void spinOnce(){
	nh.spinOnce();
}
