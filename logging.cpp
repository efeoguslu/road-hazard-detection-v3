#include "logging.h"

auto t1 = std::chrono::high_resolution_clock::now();

// Function to calculate elapsed time in hours:minutes:seconds format
std::string getElapsedTime() {
    // Calculate elapsed time since t1
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(now - t1).count();

    // Calculate hours, minutes, and remaining seconds
    int hours = elapsed_seconds / 3600;
    int minutes = (elapsed_seconds % 3600) / 60;
    int seconds = elapsed_seconds % 60;

    // Format elapsed time as hours:minutes:seconds
    std::ostringstream elapsed_time;
    elapsed_time << std::setfill('0') << std::setw(2) << hours << ":"; // Hours
    elapsed_time << std::setfill('0') << std::setw(2) << minutes << ":"; // Minutes
    elapsed_time << std::setfill('0') << std::setw(2) << seconds; // Seconds

    return elapsed_time.str();
}


void logData(std::ofstream &logfile, float ax, float ay, float az, float gr, float gp, float gy) {
    if (logfile.is_open()) {
        // Get elapsed time in hours:minutes:seconds format
        std::string elapsed_time = getElapsedTime();

        // Write elapsed time and sensor data
        logfile << elapsed_time << ", ";
        
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << ax << ", " << ay << ", " << az << ", ";
        logfile << gr << ", " << gp << ", " << gy << std::endl;
        
    }
}


void logData(std::ofstream &logfile, float data) {
    if (logfile.is_open()) {
        // Get elapsed time in hours:minutes:seconds format
        std::string elapsed_time = getElapsedTime();

        // Write elapsed time and sensor data
        logfile << elapsed_time << ", ";
        
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << data << std::endl;
        
    }
}

void logData(std::ofstream &logfile, float compoundAccelVector, float compoundGyroVector) {
    if (logfile.is_open()) {
        // Get elapsed time in hours:minutes:seconds format
        std::string elapsed_time = getElapsedTime();

        // Write elapsed time and sensor data
        logfile << elapsed_time << ", ";
        
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << compoundAccelVector << "," << compoundGyroVector << std::endl;
    }
}

void logBumpCount(std::ofstream &logfile, int count){
    if(logfile.is_open()){
        // Get elapsed time in hours:minutes:seconds format
        std::string elapsed_time = getElapsedTime();

        // Write elapsed time and sensor data
        logfile << elapsed_time << ", ";

        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << count  << std::endl;
    }
}


/*
void logAngles(std::ofstream &logfile, float roll, float pitch) {
    if (logfile.is_open()) {
        // Write the normalized values to the file
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << roll << ", " << pitch << std::endl;
    }
}

*/

void logAngles(std::ofstream &logfile, float roll, float pitch) {
    if (logfile.is_open()) {
        // Get elapsed time in hours:minutes:seconds format
        std::string elapsed_time = getElapsedTime();

        // Write elapsed time and sensor data
        logfile << elapsed_time << ", ";
        
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << roll << ", " << pitch << std::endl;
    }
}