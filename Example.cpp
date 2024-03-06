#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <vector>
#include <MPU6050.h>

MPU6050 device(0x68, false);

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


void logAx(std::ofstream &logfile, float ax) {
    if (logfile.is_open()) {
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // Write timestamp and sensor data
        logfile << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X") << ", ";
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << ax << std::endl;
        
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

int main() {

    if(!toggleFlag()){
        std::cout << "Logging has already occured. Exiting. (Flag == 0)" << std::endl;
        return 0;
    }

    float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values
    sleep(1); //Wait for the MPU6050 to stabilize

    auto timestamp = getCurrentTimestamp();

    // Output Files:

    std::ofstream sensorLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/" + timestamp + "_sensor_data_log.txt");
    if (!sensorLogFile.is_open()) {
        std::cerr << "Error: Unable to open sensor data log file." << std::endl;
        return -1;
    }
    std::ofstream compoundVectorLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/" + timestamp + "_compound_vector.txt");
    if (!compoundVectorLogFile.is_open()) {
        std::cerr << "Error: Unable to open sensor data log file." << std::endl;
        return -1;
    }
    std::ofstream bumpCountLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/" + timestamp + "_bump_count_log.txt");
    if (!bumpCountLogFile.is_open()) {
        std::cerr << "Error: Unable to open sensor data log file." << std::endl;
        return -1;
    }

    // ...
    

    std::ofstream axCalLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/" + timestamp + "_ax_cal_log.txt");
    if (!axCalLogFile.is_open()) {
        std::cerr << "Error: Unable to open sensor data log file." << std::endl;
        return -1;
    }

    std::ifstream inputForFilter("/home/efeoguslu/Desktop/road-hazard-detection-v3/" + timestamp + "_ax_cal_log.txt");
    if (!inputForFilter.is_open()) {
        std::cerr << "Error: Unable to open sensor data log file." << std::endl;
        return -1;
    }

    std::ofstream axFilteredLogFile("/home/efeoguslu/Desktop/road-hazard-detection-v3/" + timestamp + "_ax_filtered_log.txt");
    if (!axFilteredLogFile.is_open()) {
        std::cerr << "Error: Unable to open sensor data log file." << std::endl;
        return -1;
    }
    

    //Read the current yaw angle
    device.calc_yaw = true;
    //Get bump count
    auto bumpCounter = 0;
    //Window Size: adjust as needed
    // auto windowSize = 50;   

    /*
    // .... 

    std::vector<double> data;
    double value;

    while(inputForFilter >> value){
        data.push_back(value);
    }


    // ....
    */




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
            logBumpCount(bumpCountLogFile, bumpCounter);
        }

        std::cout << "Time: " << timestamp << " Accel: " << ax << ", Y: " << ay << ", Z: " << az << " Bump Counter: " << bumpCounter << "\n";
        // logAx(axLogFile, ax);
        logSensorData(sensorLogFile, ax, ay, az, gr, gp, gy);
        logCompoundData(compoundVectorLogFile, compoundVector(ax, ay, az), compoundVector(gr, gp, gy));

        // calculate moving average here...

        usleep(10000);
    }

	return 0;
}


