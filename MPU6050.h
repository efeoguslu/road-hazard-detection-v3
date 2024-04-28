//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0


//-----------------------MODIFY THESE PARAMETERS-----------------------

#define GYRO_RANGE 0 //Select which gyroscope range to use (see the table below) - Default is 0
//	Gyroscope Range
//	0	+/- 250 degrees/second
//	1	+/- 500 degrees/second
//	2	+/- 1000 degrees/second
//	3	+/- 2000 degrees/second
//See the MPU6000 Register Map for more information


#define ACCEL_RANGE 0 //Select which accelerometer range to use (see the table below) - Default is 0
//	Accelerometer Range
//	0	+/- 2g
//	1	+/- 4g
//	2	+/- 8g
//	3	+/- 16g
//See the MPU6000 Register Map for more information


//Offsets - supply your own here (calculate offsets with getOffsets function)
//     Accelerometer
/*
#define A_OFF_X 19402
#define A_OFF_Y -2692
#define A_OFF_Z -8625
//    Gyroscope
#define G_OFF_X -733
#define G_OFF_Y 433
#define G_OFF_Z -75
*/
#define A_OFF_X (20077.7)
#define A_OFF_Y (-8989.97)
#define A_OFF_Z (-15068.1)
//    Gyroscope
#define G_OFF_X (293.735)
#define G_OFF_Y (78.6624)
#define G_OFF_Z (292.178)

/*
This could take a couple of minutes...Gyroscope R,P,Y: 290.715,87.3382,290.936
Accelerometer X,Y,Z: 20018,-8811.48,-15087.1

    This could take a couple of minutes...Gyroscope R,P,Y: 293.735,78.6624,292.178
Accelerometer X,Y,Z: 20077.7,-8989.97,-15068.1


*/

//-----------------------END MODIFY THESE PARAMETERS-----------------------

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
extern "C" {
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}
#include <cmath>
#include <thread>

#define _POSIX_C_SOURCE 200809L //Used for calculating time

#define TAU 0.05 //Complementary filter percentage
#define RAD_T_DEG 57.2957795

//Select the appropriate settings
#if GYRO_RANGE == 1
	#define GYRO_SENS 65.5
	#define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
	#define GYRO_SENS 32.8
	#define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
	#define GYRO_SENS 16.4
	#define GYRO_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define GYRO_SENS 131.0
	#define GYRO_CONFIG 0b00000000
#endif
#undef GYRO_RANGE


#if ACCEL_RANGE == 1
	#define ACCEL_SENS 8192.0
	#define ACCEL_CONFIG 0b00001000
#elif ACCEL_RANGE == 2
	#define ACCEL_SENS 4096.0
	#define ACCEL_CONFIG 0b00010000
#elif ACCEL_RANGE == 3
	#define ACCEL_SENS 2048.0
	#define ACCEL_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define ACCEL_SENS 16384.0
	#define ACCEL_CONFIG 0b00000000
#endif
#undef ACCEL_RANGE




class MPU6050 {
	private:
		
		/*
		void _update();

		double _accel_angle[3];
		double _gyro_angle[3];
		double _angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)

		// double ax, ay, az, gr, gp, gy; //Temporary storage variables used in _update()

		int MPU6050_addr;
		int f_dev; //Device file

		double dt; //Loop time (recalculated with each loop)

		struct timespec start,end; //Create a time structure

		bool _first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter
		
		*/
		

	public:

		void _update();

		double _accel_angle[3];
		double _gyro_angle[3];
		double _angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)

		// double ax, ay, az, gr, gp, gy; //Temporary storage variables used in _update()

		int MPU6050_addr;
		int f_dev; //Device file

		double dt; //Loop time (recalculated with each loop)

		struct timespec start,end; //Create a time structure

		bool _first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter



		//void _update();
		double ax, ay, az, gr, gp, gy; //Temporary storage variables used in _update()
		MPU6050(int8_t addr);
		MPU6050(int8_t addr, bool run_update_thread);
		void getAccelRaw(double *x, double *y, double *z);
		void getGyroRaw(double *roll, double *pitch, double *yaw);
		void getAccel(double *x, double *y, double *z);
		void getGyro(double *roll, double *pitch, double *yaw);
		void getOffsets(double *ax_off, double *ay_off, double *az_off, double *gr_off, double *gp_off, double *gy_off);
		int getAngle(int axis, double *result);
		bool calc_yaw;
};
