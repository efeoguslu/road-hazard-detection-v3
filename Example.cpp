#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <vector>
#include <deque>
#include <numeric>
#include "MPU6050.h"

// ---------------------------------------------------------------------------------------------




MPU6050 device(0x68, false);

const int windowSize = 50;
std::deque<float> axBuffer;


// ----------------------------------------------------------------------------------------------

// Forward declaration of the log function
template<typename T, typename... Args>
void logData(std::ofstream& logfile, const T& data, const Args&... args);

// Base case for the variadic log function
template<typename T>
void logData(std::ofstream& logfile, const T& data) {
    if (logfile.is_open()) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        logfile << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X") << ", ";
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << data << std::endl;
    }
}

// Recursive case for the variadic log function
template<typename T, typename... Args>
void logData(std::ofstream& logfile, const T& data, const Args&... args) {
    if (logfile.is_open()) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        logfile << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X") << ", ";
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << data << ", ";
    }
    logData(logfile, args...); // Recur with the remaining arguments
}

// ----------------------------------------------------------------------------------------------

void updateAndLogMovingAverage(std::ofstream &axFilteredLogFile, float ax) {
    // Add the new value to the buffer
    axBuffer.push_back(ax);

    // If the buffer size exceeds the window size, remove the oldest value
    if (axBuffer.size() > windowSize) {
        axBuffer.pop_front();
    }

    // Calculate the moving average based on the current buffer contents
    double sum = std::accumulate(axBuffer.begin(), axBuffer.end(), 0.0);
    double movingAvg = sum / axBuffer.size();

    // Write the moving average to the output file
    axFilteredLogFile << movingAvg << "\n";
    axFilteredLogFile.flush(); // Flush the output buffer to ensure data is written immediately
}


bool toggleFlag() {
    std::ifstream flagFile("flag.txt");
    int flag = 0; // Default flag value
    if (flagFile.is_open()) {
        flagFile >> flag;
        flagFile.close();
    }
    // Toggle the flag
    flag = (flag == 0) ? 1 : 0;

    std::ofstream flagFileOut("flag.txt");
    if (flagFileOut.is_open()) {
        flagFileOut << flag;
        flagFileOut.close();
    } else {
        std::cerr << "Error: Unable to update flag file." << std::endl;
        return false; // Indicate failure to update the flag
    }

    return (flag == 1); // Return true if logging should proceed
}

// Function to get the current timestamp as a string
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

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


void logAcc(std::ofstream &logfile, float accel) {
    if (logfile.is_open()) {
        /*
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // Write timestamp and sensor data
        logfile << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X") << ", ";
        */
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << accel << std::endl;
        
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

void logBumpCount(std::ofstream &logfile, int count){
    if(logfile.is_open()){
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // Write timestamp and sensor data
        logfile << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X") << ", ";
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << count  << std::endl;
    }
}

float findMin(float a, float b, float c) {
    return (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
}

void logNormalizedAccelData(std::ofstream &logfile, float ax, float ay, float az) {
    if (logfile.is_open()) {
        // Find the minimum value among all three axes
        float min_value = findMin(ax, ay, az);

        // Find the range of each axis
        float range_x = std::max(ax, min_value) - min_value;
        float range_y = std::max(ay, min_value) - min_value;
        float range_z = std::max(az, min_value) - min_value;

        // Normalize each value using the provided formula
        ax = (ax - min_value) / range_x;
        ay = (ay - min_value) / range_y;
        az = (az - min_value) / range_z;

        // Write the normalized values to the file
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << ax << ", " << ay << ", " << az << std::endl;
    }
}

void logAngles(std::ofstream &logfile, float roll, float pitch) {
    if (logfile.is_open()) {
        // Write the normalized values to the file
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << roll << ", " << pitch << std::endl;
    }
}


float compoundVector(float x, float y, float z){
  return std::sqrt(x*x + y*y + z*z);
}

double movingAverage(const std::vector<double>& data, int windowSize){
    double sum = 0.0;
    for(int i = 0; i < windowSize; ++i){
        sum += data[i];
    }
    return sum/windowSize;
}


float roll_angle(float ax, float ay, float az){
    return atanf(ay/(std::sqrt(ax*ax + az*az)))*RAD_T_DEG;
}

float pitch_angle(float ax, float ay, float az){
    return atanf(-ax/(std::sqrt(ay*ay + az*az)))*RAD_T_DEG;
}

int main() {    

    if(!toggleFlag()){
        std::cout << "Logging has already occured. Exiting. (Flag == 0)" << std::endl;
        return 0;
    }

    float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values
    sleep(1); //Wait for the MPU6050 to stabilize

    auto timestamp = getCurrentTimestamp();

    // Input/Output Files:

    
    std::ofstream sensorLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_sensor_data_log.txt");
    std::ofstream compoundVectorLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_compound_vector.txt");
    std::ofstream bumpCountLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_bump_count_log.txt");

    std::ofstream axCalLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_ax_cal_log.txt");
    std::ofstream ayCalLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_ay_cal_log.txt");
    std::ofstream azCalLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_az_cal_log.txt");

    std::ofstream anglesLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_angles_log.txt");

    std::ifstream inputForFilter("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_ax_cal_log.txt");
    std::ofstream axFilteredLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_ax_filtered_log.txt");

    std::ofstream normalizedAccelLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "_normalized_accel_log.txt");

    if (!sensorLogFile.is_open() || !compoundVectorLogFile.is_open() || !bumpCountLogFile.is_open() || !axCalLogFile.is_open() || !inputForFilter.is_open() || !axFilteredLogFile.is_open() || !normalizedAccelLogFile.is_open() || !ayCalLogFile.is_open() || !azCalLogFile.is_open() || !anglesLogFile.is_open()) {
        std::cerr << "Error: Unable to open log files." << std::endl;
        return -1;
    }
    

    //Read the current yaw angle
    device.calc_yaw = true;
    //Get bump count
    auto bumpCounter = 0;

    // Instead of true, use buttons for start/stop
    while(true){

        //Get the current accelerometer values
        device.getAccel(&ax, &ay, &az);
        //Get the current gyroscope values
        device.getGyro(&gr, &gp, &gy);

        auto timestamp = getCurrentTimestamp();

        // Most of the algorithmic changes will be made here:
        
        
        if(compoundVector(ax, ay, az) >= 3.0f && compoundVector(gr, gp, gy) >= 12.0f){
            bumpCounter++;
            std::cout << "Bump Count: " << bumpCounter << "\n";
            // logBumpCount(bumpCountLogFile, bumpCounter);
            logData(bumpCountLogFile, bumpCounter);
        }
        

        //std::cout << "Time: " << timestamp << " Accel: " << ax << ", Y: " << ay << ", Z: " << az << " Bump Counter: " << bumpCounter << "\n"; 
        std::cout << std::fixed << std::setprecision(3) << std::setw(15);

        // Print accelerometer data
        std::cout << ax << " " << ay << " " << az << " " << roll_angle(ax, ay, az) << " " << pitch_angle(ax, ay, az) << std::endl;


        //logAcc(axCalLogFile, ax);
        //logAcc(ayCalLogFile, ay);
        //logAcc(azCalLogFile, az);

        logData(axCalLogFile, ax);
        logData(ayCalLogFile, ay);
        logData(azCalLogFile, az);

        //logAngles(anglesLogFile, roll_angle(ax, ay, az), pitch_angle(ax, ay, az));
        logData(anglesLogFile, roll_angle(ax, ay, az), pitch_angle(ax, ay, az));
        
        //logSensorData(sensorLogFile, ax, ay, az, gr, gp, gy);

        //logCompoundData(compoundVectorLogFile, compoundVector(ax, ay, az), compoundVector(gr, gp, gy));
        //logNormalizedAccelData(normalizedAccelLogFile, ax, ay, az);

        //updateAndLogMovingAverage(axFilteredLogFile, ax);
        

        usleep(1000000);
    }

	return 0;
}


