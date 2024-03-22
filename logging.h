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

#include "queue.h"

void logData(std::ofstream &logfile, float ax, float ay, float az, float gr, float gp, float gy);
void logData(std::ofstream &logfile, float data);
void logData(std::ofstream &logfile, float compoundAccelVector, float compoundGyroVector);
void logBump(std::ofstream &logfile, queue* q);
void logAngles(std::ofstream &logfile, float roll, float pitch);


void extractSensorData(std::ifstream& inputFile, std::ofstream& outputFile);
bool createPlotScript(const std::string& directoryPath);



#endif // LOGGING_H

