/*
 * mpu9250.c
 *
 *  Created on: Jun 15, 2020
 *      Author: yuche
 */

#include "mpu9250.h"
#include "systick.h"
#include <math.h>

double 		ax = 0,		ay = 0,		az = 0,	//Acceleration xyz
			gx = 0,		gy = 0,		gz = 0,	//Gyration xyz
			mx = 0,		my = 0,		mz = 0,	//Magnetic field xyz
			temp = 0;						//Temperature

enum MPU9250_GYRO_FSR gyro_fsr = MPU9250_NUM_GYRO_FSR;
enum MPU9250_ACCEL_FSR accel_fsr = MPU9250_NUM_ACCEL_FSR;

void mpu_set_i2c(I2C_HandleTypeDef* i2cx){
	mpu_i2cx = i2cx;
}

uint8_t mpu_read_i2c(uint8_t register_address,uint8_t length,uint8_t* output){
	int state = I2C_read(mpu_i2cx,MPU9250_I2C_ADDR,register_address,length,output);
	if (state != HAL_OK){
		__NOP();
	}
	return state;
}

uint8_t mpu_write_i2c(uint8_t register_address,uint8_t length,uint8_t* input){
	int state = I2C_write(mpu_i2cx,MPU9250_I2C_ADDR,register_address,length,input);
	if (state != HAL_OK){
		__NOP();
	}
	return state;
}

// Functions implemented for this platform(STM32F103) for DMP

int i2c_write(unsigned char slave_addr, unsigned char reg_addr,
		unsigned char length, unsigned char const *data){
	return I2C_write(mpu_i2cx,slave_addr,reg_addr,length,data);
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
		unsigned char length, unsigned char *data){
	return I2C_read(mpu_i2cx,slave_addr,reg_addr,length,data);
}

void delay_ms(unsigned long num_ms){
	delay(num_ms);
}

unsigned long get_ms(unsigned long *count){
	*count = millis();
}

// Functions implemented for this platform(STM32F103) for DMP

/**
 * read slave sensor data through slave 0, since it can read multiple bytes at once.
 * after calling this function, mpu9250 will bulk read AK8963's data repetitively.
 *
 * make sure this function is called AFTER mpu_9250_init() !
 * if not, the transition won't begin since master mode defined in MPU9250_USER_CTRL is not enabled
 */
void mpu_AK8963_start_bulk_read_i2c(uint8_t register_address,uint8_t length){

	uint8_t buffer[1];

	buffer[0] = 0x00;//disable transition during configuration
	mpu_write_i2c(MPU9250_I2C_SLV0_CTRL,1,buffer);

	buffer[0] = MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_READ_FLAG;
	mpu_write_i2c(MPU9250_I2C_SLV0_ADDR,1,buffer); // send AK8963 address and read flag to mpu9250

	buffer[0] = register_address;
	mpu_write_i2c(MPU9250_I2C_SLV0_REG,1,buffer); // send position of AK8963 register we want to access to mpu9250

	buffer[0] = MPU9250_SLV_ENABLE | (length % 0x0F);
	mpu_write_i2c(MPU9250_I2C_SLV0_CTRL,1,buffer); // send start transition signal and length to mpu9250
}

/**
 * write a single byte to AK8963 through slave 4, which sends signal (flag register) when transition is complete.
 * This is best to be used to config AK8963
 *
 * make sure this function is called AFTER mpu_9250_init() !
 * if not, the transition won't begin since master mode defined in MPU9250_USER_CTRL is not enabled
 */
void mpu_AK8963_write_i2c(uint8_t register_address,uint8_t* input){
	uint8_t buffer[1];

	buffer[0] = MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_WRITE_FLAG;
	mpu_write_i2c(MPU9250_I2C_SLV4_ADDR,1,buffer);// send address of AK8963 and write flag to mpu9250

	buffer[0] = register_address;
	mpu_write_i2c(MPU9250_I2C_SLV4_REG,1,buffer);// send address of register we want to write in AK8963 to mpu9250

	mpu_write_i2c(MPU9250_I2C_SLV4_DO,1,input);// send data we want to write to AK8963 to mpu9250

	buffer[0] = MPU9250_SLV_ENABLE;
	mpu_write_i2c(MPU9250_I2C_SLV4_CTRL,1,buffer);// enable transition

	// check is the transition complete
	int is_transfer_complete = 0;
	uint8_t i2c_mst_stat[1] = {0x00};
	while(!is_transfer_complete){
		int state = mpu_read_i2c(MPU9250_I2C_MST_STATUS,1,i2c_mst_stat);
		is_transfer_complete = (i2c_mst_stat[0] & MPU9250_SLV4_TRANS_DONE) > 0;
		__NOP();
	}
}

/**
 * read a single byte from AK8963 through slave 4, which sends signal (flag register) when transition is complete.
 * This is best to be used to config AK8963
 *
 * make sure this function is called AFTER mpu_9250_init() !
 * if not, the transition won't begin since master mode defined in MPU9250_USER_CTRL is not enabled
 */
void mpu_AK8963_read_i2c(uint8_t register_address, uint8_t* output){

	uint8_t buffer[1];


	buffer[0] = MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_READ_FLAG;
	mpu_write_i2c(MPU9250_I2C_SLV4_ADDR,1,buffer);// send address of AK8963 and read flag to mpu9250

	buffer[0] = register_address;
	mpu_write_i2c(MPU9250_I2C_SLV4_REG,1,buffer);// send address of register we want to read in AK8963 to mpu9250

	buffer[0] = MPU9250_SLV_ENABLE;
	mpu_write_i2c(MPU9250_I2C_SLV4_CTRL,1,buffer);// enable transition

	// check is the transition complete
	int is_transfer_complete = 0;
	uint8_t i2c_mst_stat;
	while(!is_transfer_complete){
		mpu_read_i2c(MPU9250_I2C_MST_STATUS,1,&i2c_mst_stat);
		is_transfer_complete = (i2c_mst_stat & MPU9250_SLV4_TRANS_DONE) > 0;
	}

	mpu_read_i2c(MPU9250_I2C_SLV4_DI,1,output);// read data sent by AK8963 stored in mpu9250

}

void mpu_9250_init(){
	mpu_set_gyro_FSR(MPU9250_FSR_500DPS);
	mpu_set_accel_FSR(MPU9250_FSR_4G);

	if (mpu_get_gyro_FSR() != MPU9250_FSR_500DPS){
		__NOP();//error
	}

	if (mpu_get_accel_FSR() != MPU9250_FSR_4G){
		__NOP();//error
	}

	uint8_t buffer[1] = {MPU9250_I2C_MST_EN};
	int state = mpu_write_i2c(MPU9250_USER_CTRL,1,buffer);
	__NOP();
}

void mpu_AK8963_init(){
	uint8_t temp;

	temp = MPU9250_AK8963_POWER_DOWN;
	mpu_AK8963_write_i2c(MPU9250_AK8963_CNTL,&temp);//power down AK8963

	delay(1);//delay 1ms so that mpu9250 done communication with AK8963

	temp = MPU9250_AK8963_CONTINUOUS_MEASUREMENT;
	mpu_AK8963_write_i2c(MPU9250_AK8963_CNTL,&temp);//enter continuous measurement mode

	delay(1);

	mpu_AK8963_start_bulk_read_i2c(MPU9250_AK8963_ST1,8);//configure mpu9250 to read 8 bytes from AK8963 continuously.

	delay(1);

}


int mpu_get_gyro_FSR(){
	uint8_t buffer[1];
	mpu_read_i2c(MPU9250_GYRO_CONFIG,1,buffer);
	return buffer[0] >> 3 & 0x03;
}

int mpu_set_gyro_FSR(int gyro_fsr){
	uint8_t buffer[1];
	mpu_read_i2c(MPU9250_GYRO_CONFIG,1,buffer);//obtain old settings
	buffer[0] &= 0b11100111;
	buffer[0] |= gyro_fsr << 3;
	mpu_write_i2c(MPU9250_GYRO_CONFIG,1,buffer);
	return 0;
}

int mpu_get_accel_FSR(){
	uint8_t buffer[1];
	mpu_read_i2c(MPU9250_ACCEL_CONFIG,1,buffer);
	return buffer[0] >> 3 & 0x03;
}

int mpu_set_accel_FSR(int accel_fsr){
	uint8_t buffer[1];
	mpu_read_i2c(MPU9250_ACCEL_CONFIG,1,buffer);//obtain old settings
	buffer[0] &= 0b11100111;
	buffer[0] |= accel_fsr << 3;
	mpu_write_i2c(MPU9250_ACCEL_CONFIG,1,buffer);
	return 0;
}

/**
 *
 */
void mpu_update_6axis(){
	if (gyro_fsr == MPU9250_NUM_GYRO_FSR){//gyro fsr not initialized
		gyro_fsr = mpu_get_gyro_FSR();
	}

	if (accel_fsr == MPU9250_NUM_ACCEL_FSR){//accel fsr not initialized
		accel_fsr = mpu_get_accel_FSR();
	}

	uint8_t buffer[14];
	mpu_read_i2c(MPU9250_ACCEL_XOUT_H,14,buffer);

	ax = (int16_t)(buffer[0] << 8 | (buffer[1] & 0xFF)) / 32768.0 * ((2 << accel_fsr) * 9.80665 );
	ay = (int16_t)(buffer[2] << 8 | (buffer[3] & 0xFF)) / 32768.0 * ((2 << accel_fsr) * 9.80665 );
	az = (int16_t)(buffer[4] << 8 | (buffer[5] & 0xFF)) / 32768.0 * ((2 << accel_fsr) * 9.80665 );

	gx = (int16_t)(buffer[8] << 8 | (buffer[9] & 0xFF)) / 32768.0 * (250 << gyro_fsr);
	gy = (int16_t)(buffer[10] << 8 | (buffer[11] & 0xFF)) / 32768.0 * (250 << gyro_fsr);
	gz = (int16_t)(buffer[12] << 8 | (buffer[13] & 0xFF)) / 32768.0 * (250 << gyro_fsr);

	gx = M_PI * gx / 180;// deg/s to rad/s
	gy = M_PI * gy / 180;
	gz = M_PI * gz / 180;

}


void mpu_update_9axis(){
	if (gyro_fsr == MPU9250_NUM_GYRO_FSR){//gyro fsr not initialized
		gyro_fsr = mpu_get_gyro_FSR();
	}

	if (accel_fsr == MPU9250_NUM_ACCEL_FSR){//accel fsr not initialized
		accel_fsr = mpu_get_accel_FSR();
	}

	uint8_t buffer[22];
	mpu_read_i2c(MPU9250_ACCEL_XOUT_H,22,&buffer[0]);

	ax = (int16_t)(buffer[0] << 8 | (buffer[1] )) / 32768.0 * ((2 << accel_fsr) * 9.80665 );
	ay = (int16_t)(buffer[2] << 8 | (buffer[3] )) / 32768.0 * ((2 << accel_fsr) * 9.80665 );
	az = (int16_t)(buffer[4] << 8 | (buffer[5] )) / 32768.0 * ((2 << accel_fsr) * 9.80665 );

	gx = (int16_t)(buffer[8] << 8 | (buffer[9] & 0xFF)) / 32768.0 * (250 << gyro_fsr);
	gy = (int16_t)(buffer[10] << 8 | (buffer[11] & 0xFF)) / 32768.0 * (250 << gyro_fsr);
	gz = (int16_t)(buffer[12] << 8 | (buffer[13] & 0xFF)) / 32768.0 * (250 << gyro_fsr);

	gx = M_PI * gx / 180;// deg/s to rad/s
	gy = M_PI * gy / 180;
	gz = M_PI * gz / 180;

	//read magnetic sensor AK8963
	mx = (int16_t)(buffer[16] << 8 | (buffer[15] & 0xFF)) / 32760.0 * 4912;
	my = (int16_t)(buffer[18] << 8 | (buffer[17] & 0xFF)) / 32760.0 * 4912;
	mz = (int16_t)(buffer[20] << 8 | (buffer[19] & 0xFF)) / 32760.0 * 4912;

}


/**
 *
 */
void mpu_get_6axis(double* ax_, double* ay_, double* az_, double* gx_, double* gy_, double* gz_){
	*ax_ = ax;		*ay_ = ay;		*az_ = az;
	*gx_ = gx;		*gy_ = gy;		*gz_ = gz;
}

/**
 *
 */
void mpu_get_9axis(double* ax_, double* ay_, double* az_, double* gx_, double* gy_, double* gz_,double* mx_, double* my_, double* mz_){
	*ax_ = ax;		*ay_ = ay;		*az_ = az;
	*gx_ = gx;		*gy_ = gy;		*gz_ = gz;
	*mx_ = mx;		*my_ = my;		*mz_ = mz;
}
