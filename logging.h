#ifndef LOGGING_H
#define LOGGING_H


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
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <vector>
#include <deque>
#include <numeric>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>


// void logAllSensorData(std::ofstream &logfile, float ax, float ay, float az, float gr, float gp, float gy);
// void logSingleSensorData(std::ofstream &logfile, float data);
// void logCompoundData(std::ofstream &logfile, float compoundAccelVector, float compoundGyroVector);
// void logBumpCount(std::ofstream &logfile, int count);
// void logAngles(std::ofstream &logfile, float roll, float pitch);

void logData(std::ofstream &logfile, float ax, float ay, float az, float gr, float gp, float gy);
void logData(std::ofstream &logfile, float data);
void logData(std::ofstream &logfile, float compoundAccelVector, float compoundGyroVector);
void logBumpCount(std::ofstream &logfile, int count);
void logAngles(std::ofstream &logfile, float roll, float pitch);



#endif // LOGGING_H

