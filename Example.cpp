#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <MPU6050.h>

MPU6050 device(0x68, false);

void logSensorData(std::ofstream &logfile, float ax, float ay, float az, float gr, float gp, float gy) {
    if (logfile.is_open()) {
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // Write timestamp and sensor data
        logfile << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X") << ", ";
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << ax << ", " << ay << ", " << az << ", ";
        logfile << gr << ", " << gp << ", " << gy << std::endl;
        
    }
}

void logCompoundData(std::ofstream &logfile, float compoundAccelVector, float compoundGyroVector) {
    if (logfile.is_open()) {
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // Write timestamp and sensor data
        logfile << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X") << ", ";
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << compoundAccelVector << "," << compoundGyroVector << std::endl;
    }
}


float compound_vector(float x, float y, float z){
  return std::sqrt(x*x + y*y + z*z);
}

int main() {

	float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values

	sleep(1); //Wait for the MPU6050 to stabilize

    std::ofstream sensorLogFile("sensor_data_log.txt");
    if (!sensorLogFile.is_open()) {
    	std::cerr << "Error: Unable to open sensor data log file." << std::endl;
    	return -1;
    }

	std::ofstream compoundVectorLogFile("compound_vector.txt");
    if (!compoundVectorLogFile.is_open()) {
    	std::cerr << "Error: Unable to open sensor data log file." << std::endl;
    	return -1;
    }

	//Read the current yaw angle
	device.calc_yaw = true;

	int bumpCounter = 0;

	while(true){
		//Get the current accelerometer values
		device.getAccel(&ax, &ay, &az);
		//Get the current gyroscope values
		device.getGyro(&gr, &gp, &gy);

		if(compound_vector(ax, ay, az) >= 3.0f && compound_vector(gr, gp, gy) >= 12.0f){
			bumpCounter++;
			std::cout << "Bump Count: " << bumpCounter << "\n";
		}

		std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << " Bump Counter: " << bumpCounter << "\n";
		logSensorData(sensorLogFile, ax, ay, az, gr, gp, gy);
		logCompoundData(compoundVectorLogFile, compound_vector(ax, ay, az), compound_vector(gr, gp, gy));
		usleep(10000);
	}

	return 0;
}


