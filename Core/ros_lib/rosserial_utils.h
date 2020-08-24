/*
 * rosserial_utils.h
 *
 *  Created on: 2020年8月24日
 *      Author: yuche
 */

#ifndef ROS_LIB_ROSSERIAL_UTILS_H_
#define ROS_LIB_ROSSERIAL_UTILS_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

void ros_init();

void IMU_pub(double a[3], double w[3], double q[4]);
void MAG_pub(double m[3]);
void GPS_pub(double lla[3]);
void PT_pub(double pt[2]);

void spinOnce();

#ifdef __cplusplus
}
#endif

#endif /* ROS_LIB_ROSSERIAL_UTILS_H_ */


