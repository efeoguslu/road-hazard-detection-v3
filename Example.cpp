#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <vector>
#include <deque>
#include <numeric>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "MPU6050.h"

// ---------------------------------------------------------------------------------------------


MPU6050 device(0x68, false);

const int windowSize{ 5 };

const int sampleRateHz{ 5 };                  // Sample rate in Hz
const int loopDurationMs{ 1000 / sampleRateHz }; // Duration of each loop iteration in milliseconds

const float gravity_mps2{ 9.80665 };
const float tau{ 0.98 };
const float radiansToDegrees{ 57.2957795 };


int time2Delay{ 0 };
float dt{ 0 };

// ----------------------------------------------------------------------------------------------

template<typename T>
class MovingAverage {
private:
    std::deque<T> buffer;
    size_t windowSize;
public:
    MovingAverage(size_t size) : windowSize(size) {}

    void updateAndLogMovingAverage(std::ofstream &logFile, T newValue) {
        // Add the new value to the buffer
        buffer.push_back(newValue);

        // If the buffer size exceeds the window size, remove the oldest value
        if (buffer.size() > windowSize) {
            buffer.pop_front();
        }

        // Calculate the moving average based on the current buffer contents
        double sum = std::accumulate(buffer.begin(), buffer.end(), 0.0);
        double movingAvg = sum / buffer.size();

        // Write the moving average to the output file
        logFile << movingAvg << "\n";
        logFile.flush(); // Flush the output buffer to ensure data is written immediately
    }
};



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


bool createDirectory(const std::string& path) {
    return mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0;
}


void logAllSensorData(std::ofstream &logfile, float ax, float ay, float az, float gr, float gp, float gy) {
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


void logSingleSensorData(std::ofstream &logfile, float data) {
    if (logfile.is_open()) {
        /*
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // Write timestamp and sensor data
        logfile << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X") << ", ";
        */
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << data << std::endl;
        
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

void writePlotScript(const std::string& directoryPath) {
    std::string plotFileName = directoryPath + "/plot.gp";
    
    // Open the file in write mode
    std::ofstream plotFile(plotFileName);
    
    // Check if the file was successfully opened
    if (!plotFile) {
        std::cerr << "Unable to open file for writing: " << plotFileName << std::endl;
        return;
    }

    /*
    # Set the title of the plot
set title "Sensor Data Comparison"

# Set labels for x and y axes
set xlabel "Time"
set ylabel "Value"

# Set the output file format and name
set terminal png
set output "sensor_data_comparison.png"

# Plot the data from the first .txt file
plot "axLogFile.txt" with lines title "Sensor 1", \
     "axFilteredLogFile.txt" with lines title "Sensor 2"
    */
    
    // Write the Gnuplot script
    plotFile << "set title 'Sensor Data Comparison'\n"; // Set the output format and size
    plotFile << "set xlabel 'Time'\n"; // Set the output file name
    plotFile << "set ylabel 'Value'\n"; // Set the plot title
    plotFile << "set terminal png\n"; // Set the x-axis label
    plotFile << "set output 'sensor_data_comparison.png'\n"; // Set the y-axis label
    plotFile << "plot 'axLogFile.txt' with lines title 'Sensor 1', \\n"; // Plot the data
    plotFile << "'axFilteredLogFile.txt' with lines title 'Sensor 2'\n"; // Plot the data
    
    // Close the file
    plotFile.close();
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

float getRollAngle(float ax, float ay, float az){
    return atan2f(ay, (std::sqrt(ax*ax + az*az)))*radiansToDegrees;
}
float getPitchAngle(float ax, float ay, float az){
    return atan2f(-ax, (std::sqrt(ay*ay + az*az)))*radiansToDegrees;
}

// -----


// -----


/*
void compFilter(float dt, float tau, float ax, float ay, float az, float gr, float gp, float gy) {

  // Complementary filter
  float accelPitch = atan2(ay, az) * radiansToDegrees;
  float accelRoll  = atan2(ax, az) * radiansToDegrees;

  attitude.roll = (tau)*(attitude.roll - imu_cal.gy*dt) + (1-tau)*(accelRoll);
  attitude.pitch = (tau)*(attitude.pitch + imu_cal.gx*dt) + (1-tau)*(accelPitch);
  attitude.yaw += imu_cal.gz*dt;
}
*/

// ***** spherical:

template <typename T> int sgn(T val){
    return (T(0) < val) - (val < T(0));
}

float inclination(float z, float r){
    return acosf(z/r);
}

float azimuth(float x, float y){
    return sgn(y)*acosf(x/std::sqrt(x*x + y*y));
}

void rotate(float pitchAngle, float ax, float ay, float az, float* ax_rotated, float* ay_rotated, float* az_rotated){
    *ax_rotated = ax*cosf(pitchAngle) - az*sinf(pitchAngle);
    *ay_rotated = ay;
    *az_rotated = ax*sinf(pitchAngle) + az*cosf(pitchAngle);
}

void rotateAll(float pitchAngle, float rollAngle, float ax, float ay, float az, float* ax_rotated, float* ay_rotated, float* az_rotated){
    *ax_rotated = ax*cosf(pitchAngle) - az*sinf(pitchAngle);
    *ay_rotated = ax*sinf(pitchAngle)*sinf(rollAngle) + ay*cosf(rollAngle) + az*sinf(rollAngle)*cosf(pitchAngle);
    *az_rotated = ax*sinf(pitchAngle)*cosf(rollAngle) - ay*sinf(rollAngle) + az*cosf(rollAngle)*cosf(pitchAngle);
}


// Time stabilization function
float timeSync(auto t1){
	// Find duration to sleep thread
	auto t2 = std::chrono::high_resolution_clock::now();
	time2Delay = std::chrono::duration<float, std::micro>(t2-t1).count();

	// Sleep thread
	std::this_thread::sleep_for(std::chrono::microseconds(loopDurationMs-time2Delay));

	// Calculate dt
	auto t3 = std::chrono::high_resolution_clock::now();
	dt = (std::chrono::duration<float, std::micro>(t3-t1).count()) * 1E-6;
	// std::cout << "\t" << 1/dt << std::endl;

	// Return dt and begin main loop again
	return dt;
}

// BURADASIN!!! Kontrol et.

void compFilter(float dt, float tau, float* pitchAngle, float* rollAngle, float ax, float ay, float az, float gr, float gp, float gy) {

  // Complementary filter
  float accelPitchAngle = atan2(ay, az) * radiansToDegrees;
  float accelRollAngle  = atan2(ax, az) * radiansToDegrees;

  *rollAngle = (tau)*(*rollAngle - gp*dt) + (1-tau)*(accelRollAngle);
  *pitchAngle = (tau)*(*pitchAngle + gr*dt) + (1-tau)*(accelPitchAngle);
}


int main() {    

    
    if(!toggleFlag()){
        std::cout << "Logging has already occured. Exiting. (Flag == 0)" << std::endl;
        return 0;
    }
    
    


    float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values
    float ax_rotated, ay_rotated, az_rotated; // Variables to store the rotated acceleration

    // !!!
    sleep(1); //Wait for the system clock to get ready


    auto timestamp = getCurrentTimestamp();
    auto directoryPath = "/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "/";
    
    /*
    if (!createDirectory(directoryPath)) {
        std::cerr << "Error: Unable to create directory." << std::endl;
        return -1;
    }

    // Input/Output Files:

    std::ofstream sensorLogFile(directoryPath + "sensorLogFile.txt");
    std::ofstream compoundVectorLogFile(directoryPath + "compoundVectorLogFile.txt");
    std::ofstream bumpCountLogFile(directoryPath + "bumpCountLogFile.txt");

    std::ofstream axLogFile(directoryPath + "axLogFile.txt");
    std::ofstream ayLogFile(directoryPath + "ayLogFile.txt");
    std::ofstream azLogFile(directoryPath + "azLogFile.txt");

    std::ofstream grLogFile(directoryPath + "grLogFile.txt");
    std::ofstream gpLogFile(directoryPath + "gpLogFile.txt");
    std::ofstream gyLogFile(directoryPath + "gyLogFile.txt");     
    
    std::ofstream anglesLogFile(directoryPath + "anglesLogFile.txt");
    std::ifstream inputForFilter(directoryPath +  "inputForFilter.txt");

    std::ofstream axFilteredLogFile(directoryPath +  "axFilteredLogFile.txt");
    std::ofstream ayFilteredLogFile(directoryPath +  "ayFilteredLogFile.txt");
    std::ofstream azFilteredLogFile(directoryPath +  "azFilteredLogFile.txt");

    std::ofstream compoundVectorFilteredLogFile(directoryPath +  "compoundVectorFilteredLogFile.txt");
    
    */

    // std::ofstream normalizedAccelLogFile(directoryPath + "normalizedAccel.txt");

    // writePlotScript(directoryPath);

    /*
    if (!sensorLogFile.is_open() || !compoundVectorLogFile.is_open() || !bumpCountLogFile.is_open() || !axLogFile.is_open() || !inputForFilter.is_open() || !axFilteredLogFile.is_open() || !normalizedAccelLogFile.is_open() || !ayLogFile.is_open() || !azLogFile.is_open() || !anglesLogFile.is_open()) {
        std::cerr << "Error: Unable to open log files." << std::endl;
        return -1;
    }
    */

    //Read the current yaw angle
    device.calc_yaw = true;
    //Get bump count
    auto bumpCounter{0};

    MovingAverage<float> axMovingAvg(windowSize);
    MovingAverage<float> ayMovingAvg(windowSize);
    MovingAverage<float> azMovingAvg(windowSize);
    
    MovingAverage<float> compoundAccelMovingAvg(windowSize);

    float thetaHat_rad{ 0 };
    float phiHat_rad{ 0 };

    float pitchAngleComp{ 0.0f };
    float rollAngleComp{ 0.0f };

    

    // Instead of true, use buttons for start/stop
    while(true){
        // Record loop time stamp
        auto startTime{std::chrono::high_resolution_clock::now()};

        //Get the current accelerometer values
        device.getAccel(&ax, &ay, &az);

        //Get the current gyroscope values
        device.getGyro(&gr, &gp, &gy);

        //Get the current roll and pitch angles
        float pitchAngleFormula = getPitchAngle(ax, ay, az);
        float rollAngleFormula = getRollAngle(ax, ay, az);

        

        //Get compound acceleration and gyro vector
        float compoundAccelerationVector = compoundVector(ax, ay, az);
        float compoundGyroVector = compoundVector(gr, gp, gy);
        
        compFilter(dt, tau, &pitchAngleComp, &rollAngleComp, ax, ay, az, gr, gp, gy);

        // rotateAll(pitchAngle, rollAngle, ax, ay, az, &ax_rotated, &ay_rotated, &az_rotated);




        // auto timestamp {getCurrentTimestamp()};



        // -----------------------------------------------------------------------------------------------

        /*
        // Calculating roll (phi) and pitch (theta) angle estimates using filtered accelerometer readings
        float phiHat_deg{ atanf(ay/az)*radiansToDegrees };
        float thetaHat_deg{ asinf(ax/gravity_mps2)*radiansToDegrees };

        float phiHat_acc_rad{ atanf(ay/az) };
        float thetaHat_acc_rad{ asinf(ax/gravity_mps2) };


        // Transforming body rates to Euler rates 
        float phiDot_rps{ gr + tanf(thetaHat_rad) * (sinf(phiHat_rad)*gp + cosf(phiHat_rad)*gy) }; 
        float thetaDot_rps{ cosf(phiHat_rad)*gp - sinf(phiHat_rad)*gy };

        // Integrate Euler rates to get estimate of roll and pitch angles
        // phiHat_rad = phiHat_rad + (loopDurationMs/1000.f)*phiDot_rps;
        // thetaHat_rad = thetaHat_rad + (loopDurationMs/1000.f)*thetaDot_rps;

        // Combine accelerometer estimates with integral of gyro readings

        phiHat_rad =         complementaryFilterAlpha  * phiHat_acc_rad
                   + (1.0f - complementaryFilterAlpha) * (phiHat_rad + (loopDurationMs/1000.f)*phiDot_rps);

        thetaHat_rad =       complementaryFilterAlpha  * thetaHat_acc_rad
                   + (1.0f - complementaryFilterAlpha) * (thetaHat_rad + (loopDurationMs/1000.f)*thetaDot_rps);


        std::cout << std::fixed << std::setprecision(3);
        std::cout << "phiHat: " << phiHat_rad*radiansToDegrees << " thetaHat: " << thetaHat_rad*radiansToDegrees << std::endl;        
        */
        



        

        // -----------------------------------------------------------------------------------------------


        /*
        
        // Most of the algorithmic changes will be made here:
        if(compoundVector(ax, ay, az) >= 3.0f && compoundVector(gr, gp, gy) >= 12.0f){
            bumpCounter++;
            std::cout << "Bump Count: " << bumpCounter << "\n";
            logBumpCount(bumpCountLogFile, bumpCounter);
            // logData(bumpCountLogFile, bumpCounter);
        }
        
        //std::cout << "Time: " << timestamp << " Accel: " << ax << ", Y: " << ay << ", Z: " << az << " Bump Counter: " << bumpCounter << "\n"; 

        */

        
        


        // std::cout << std::fixed << std::setprecision(3); // Set precision for floating point numbers

        /*
        std::cout 
            << "AccX (m/s^2): "         << std::setw(8) << ax
            << " AccY (m/s^2): "        << std::setw(8) << ay 
            << " AccZ (m/s^2): "        << std::setw(8) << az
            << " Roll Rate (deg/s): "   << std::setw(8) << gr
            << " Pitch Rate (deg/s): "  << std::setw(8) << gp
            << " Yaw Rate (deg/s): "    << std::setw(8) << gy << std::endl; 
        */

        // -------------------------------------------------------------------------------------------------------------------------

        /*
        std::cout << std::fixed << std::setprecision(3); // Set precision for floating point numbers

        std::cout 
            << "AccX (m/s^2): "         << std::setw(8) << ax
            << " AccY (m/s^2): "        << std::setw(8) << ay 
            << " AccZ (m/s^2): "        << std::setw(8) << az
            << " Roll Angle (deg): "    << std::setw(8) << rollAngle
            << " Pitch Angle (deg): "   << std::setw(8) << pitchAngle << std::endl;
        */
        
        
        

        // -------------------------------------------------------------------------------------------------------------------------

        /*
        std::cout 
            << "Roll Angle (deg): "     << std::setw(8) << roll_angle(ax, ay, az)
            << " Roll Rate (deg/s): "   << std::setw(8) << gr
            << " Pitch Angle (deg): "   << std::setw(8) << pitch_angle(ax, ay, az)
            << " Pitch Rate (deg/s): "  << std::setw(8) << gp
            << " Yaw Rate (deg/s): "    << std::setw(8) << gy << std::endl; 
        */

        //std::cout << "phiHat_deg: " << std::setw(8) << phiHat_deg(ay, az) << "thetaHat_deg: " << std::setw(8) << thetaHat_deg(ax) << std::endl;
        // -------------------------------------------------------------------------------------------------------------------------
        
        /*
        std::cout 
            << "radius (r): " << std::setw(8) << compoundVector(ax, ay, az) 
            << " inclination: " << std::setw(8) << inclination(az, compoundVector(ax, ay, az))*radiansToDegrees 
            << " azimuth: " << std::setw(8) << azimuth(ax, ay)*radiansToDegrees << std::endl;

        */

        //rotate(pitchAngle, ax, ay, az, &ax_rotated, &ay_rotated, &az_rotated);

        const int width = 5;
        
        /*
        std::cout << std::fixed << std::setprecision(2); // Set precision for floating point numbers
        std::cout 
            << "AccX (m/s^2): "         << std::setw(width) << ax << " | "
            << " AccY (m/s^2): "        << std::setw(width) << ay << " | "
            << " AccZ (m/s^2): "        << std::setw(width) << az << " | "
            << " AccX (rotated) (m/s^2): "         << std::setw(width) << ax_rotated << " | "
            << " AccY (rotated) (m/s^2): "        << std::setw(width) << ay_rotated << " | "
            << " AccZ (rotated)  (m/s^2): "        << std::setw(width) << az_rotated << " | "
            << " Pitch Angle (deg): "   << std::setw(width) << pitchAngleComp << " | "
            << " Roll Angle (deg): " << std::setw(width) << rollAngleComp << std::endl;
        
        */

        std::cout << std::fixed << std::setprecision(2); // Set precision for floating point numbers
        std::cout 
            << "AccX (m/s^2): "         << std::setw(width) << ax << " | "
            << " AccY (m/s^2): "        << std::setw(width) << ay << " | "
            << " AccZ (m/s^2): "        << std::setw(width) << az << " | "
            << " Pitch Angle Formula: " << std::setw(width) << pitchAngleFormula << " | "
            << " Pitch Angle Comp: "   << std::setw(width) << pitchAngleComp << " | "
            << " Roll Angle Formula: " << std::setw(width) << rollAngleFormula << " | "
            << " Roll Angle Comp: " << std::setw(width) << rollAngleComp << std::endl;
        
        

        

        /*
        std::vector<std::ofstream*> logFiles = {&axLogFile, &ayLogFile, &azLogFile, &grLogFile, &gpLogFile, &gyLogFile};
        std::vector<float> data = {ax, ay, az, gr, gp, gy}; 

        for (size_t i = 0; i < logFiles.size(); ++i) {
            logSingleSensorData(*logFiles[i], data[i]);
        }

        logAngles(anglesLogFile, rollAngle, pitchAngle);
        
        logAllSensorData(sensorLogFile, ax, ay, az, gr, gp, gy);

        logCompoundData(compoundVectorLogFile, compoundAccelerationVector, compoundGyroVector);

        axMovingAvg.updateAndLogMovingAverage(axFilteredLogFile, ax);
        ayMovingAvg.updateAndLogMovingAverage(ayFilteredLogFile, ay);
        azMovingAvg.updateAndLogMovingAverage(azFilteredLogFile, az);
        
        compoundAccelMovingAvg.updateAndLogMovingAverage(compoundVectorFilteredLogFile, compoundAccelerationVector);
        
        */


        /*

        auto endTime{std::chrono::high_resolution_clock::now()};
        auto duration{std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count()};

        auto delayMs{static_cast<long>(loopDurationMs - duration)};

        if (delayMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        }
        
        */
        
        

       dt = timeSync(startTime);
        
    }
        
    
	return 0;
}


