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
#include <algorithm>
#include <stack>
#include <mutex>

#include "MPU6050.h"
#include "filters.h"
#include "button.h"
#include "rgbled.h"
#include "sequence.h"

#include "common-structs.hpp"

MPU6050 device(0x68, false);

const int sampleRateHz{ 75 };                    // Sample rate in Hz
const int loopDurationMs{ 1000 / sampleRateHz }; // Duration of each loop iteration in milliseconds

const double gravity_mps2{ 9.80665 };
const double tau{ 0.05 };
const double radiansToDegrees{ 57.2957795 };
const double degreesToRadians{ 0.0174532925 };

double dt{ 0.0 };

const double filterAlpha{ 0.9 };

// Define the pin we are going to use
// const int ledPin{ 17 }; // Example: GPIO 17

// const int buttonPin{ 16 };

// const int onLedPin{ 15 };
// const int detectLedPin{ 26 };


static int activeBumpCount{ 0 };
static int activePotholeCount{ 0 };

// ----------------------------------------------------------------------------------------------


ThreeAxisIIR iirFiltAccel;
ThreeAxisIIR iirFiltGyro;

// ----------------------------------------------------------------------------------------------


/*
class SensitivityConfig{
public: 
    std::string configName;
    double threshold;

    SensitivityConfig(std::string name, double thres) : configName{name}, threshold{thres} { };
    
    void setConfig(SensitivityConfig inConf){
        this->configName = inConf.configName;
        this->threshold = inConf.threshold;
    }
    
    std::string getConfigStr()const{
        std::string ret = ",configname=" + this->configName + ",threshold=" + std::to_string(this->threshold);
        return ret;
    }

    std::string getConfigName()const{
        return this->configName;
    }
};
*/

class SensitivityConfig {
public:
    std::string configName;
    double bumpThreshold;
    double potholeThreshold;

    SensitivityConfig(std::string name, double bumpThres, double potholeThres) : configName{name}, bumpThreshold{bumpThres}, potholeThreshold{potholeThres} { }

    void setConfig(SensitivityConfig inConf) {
        this->configName = inConf.configName;
        this->bumpThreshold = inConf.bumpThreshold;
        this->potholeThreshold = inConf.potholeThreshold;
    }

    std::string getConfigStr() const {
        std::string ret = ",configname=" + this->configName +
                          ",bumpThreshold=" + std::to_string(this->bumpThreshold) +
                          ",potholeThreshold=" + std::to_string(this->potholeThreshold);
        return ret;
    }

    std::string getConfigName() const {
        return this->configName;
    }
};

// Pothole thresholds may be subject to change:

SensitivityConfig lowSensitivityConfig{  "low",  0.25, 0.15 }; 
SensitivityConfig midSensitivityConfig{  "mid",  0.12, 0.09 }; 
SensitivityConfig highSensitivityConfig{ "high", 0.07, 0.04 }; 
SensitivityConfig currentConfig{ "default", 0.12, 0.09 };

SensitivityConfig getNextConfig(const SensitivityConfig& current, RgbLed& led) {
    if (current.getConfigName() == "low") {
        led.configurationChanged(2);
        return midSensitivityConfig;

    } else if (current.getConfigName() == "mid") {
        led.configurationChanged(3);
        return highSensitivityConfig;

    } else if (current.getConfigName() == "high") {
        led.configurationChanged(1);
        return lowSensitivityConfig;

    } else {
        // Default case, return mid as the next config
        return midSensitivityConfig;
    }
}

// Function to get the current timestamp as a string
const std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

inline bool createDirectory(const std::string& path) {
    return mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0;
}

inline double compoundVector(double x, double y, double z){
    return std::sqrt(x*x + y*y + z*z);
}

void rotateAll(double rollAngle, double pitchAngle, double x, double y, double z, double* x_rotated, double* y_rotated, double* z_rotated){
    *x_rotated =  x*std::cos(pitchAngle)                                             + z*std::sin(pitchAngle);
    *y_rotated = -x*std::sin(pitchAngle)*std::sin(rollAngle) + y*std::cos(rollAngle) + z*std::cos(pitchAngle)*std::sin(rollAngle);
    *z_rotated = -x*std::sin(pitchAngle)*std::cos(rollAngle) - y*std::sin(rollAngle) + z*std::cos(rollAngle)*std::cos(pitchAngle);  
}

// Time stabilization function
double timeSync(std::chrono::_V2::system_clock::time_point t1){
    // Find duration to sleep thread
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    long int time2Delay = duration.count(); // Convert to int for sleep_for

    // Sleep thread
    std::this_thread::sleep_for(std::chrono::microseconds(loopDurationMs - time2Delay));

    // Calculate dt
    auto t3 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t1);
    dt = static_cast<double>(duration2.count()) * 1E-6; // Explicitly cast to float

    // Return dt and begin main loop again
    return dt;
}


void complementaryFilter(double ax, double ay, double az, double gr, double gp, double gy, double* rollAngle, double* pitchAngle){

	//X (roll) axis
	device._accel_angle[0] = std::atan2(az, ay) * radiansToDegrees - 90.0; //Calculate the angle with z and y convert to degrees and subtract 90 degrees to rotate
	device._gyro_angle[0] = device._angle[0] + gr*dt; //Use roll axis (X axis)
	//Y (pitch) axis
	device._accel_angle[1] = std::atan2(az, ax) * radiansToDegrees - 90.0; //Calculate the angle with z and x convert to degrees and subtract 90 degrees to rotate
	device._gyro_angle[1] = device._angle[1] + gp*dt; //Use pitch axis (Y axis)
	//Z (yaw) axis
	if (device.calc_yaw) {
		device._gyro_angle[2] = device._angle[2] + gy*dt; //Use yaw axis (Z axis)
	}
	if (device._first_run) { //Set the gyroscope angle reference point if this is the first function run
		for (int i = 0; i <= 1; i++) {
			device._gyro_angle[i] = device._accel_angle[i]; //Start off with angle from accelerometer (absolute angle since gyroscope is relative)
		}
		device._gyro_angle[2] = 0; //Set the yaw axis to zero (because the angle cannot be calculated with the accelerometer when vertical)
		device._first_run = 0;
	}
	double asum = std::fabs(ax) + std::fabs(ay) + std::fabs(az); //Calculate the sum of the accelerations
	double gsum = std::fabs(gr) + std::fabs(gp) + std::fabs(gy); //Calculate the sum of the gyro readings
	for (int i = 0; i <= 1; i++) { //Loop through roll and pitch axes
		if (std::fabs(device._gyro_angle[i] - device._accel_angle[i]) > 5) { //Correct for very large drift (or incorrect measurment of gyroscope by longer loop time)
			device._gyro_angle[i] = device._accel_angle[i];
		}
		//Create result from either complementary filter or directly from gyroscope or accelerometer depending on conditions
		if (asum > 0.1 && asum < 3 && gsum > 0.3) { //Check that th movement is not very high (therefore providing inacurate angles)
			device._angle[i] = (1.0f - tau)*(device._gyro_angle[i]) + (tau)*(device._accel_angle[i]); //Calculate the angle using a complementary filter
		}
		else if (gsum > 0.3) { //Use the gyroscope angle if the acceleration is high
			device._angle[i] = device._gyro_angle[i];
		}
		else if (gsum <= 0.3) { //Use accelerometer angle if not much movement
			device._angle[i] = device._accel_angle[i];
		}
	}
	//The yaw axis will not work with the accelerometer angle, so only use gyroscope angle
	if (device.calc_yaw) { //Only calculate the angle when we want it to prevent large drift
		device._angle[2] = device._gyro_angle[2];
	}
	else {
		device._angle[2] = 0;
		device._gyro_angle[2] = 0;
	}

    *rollAngle = device._angle[0];
    *pitchAngle = device._angle[1];

}

void AppendDeque(std::deque<double> &target, std::deque<double> source)
{
    for(long unsigned int i = 0; i < source.size(); i++)
    {
        target.push_back(source.at(i));
    }
}




bool detectHazard(const std::deque<double>& completedData, double bumpThreshold, double potholeThreshold, SequenceType& sequenceType){
    
    if (completedData.size() < 3){
        return false; // Not enough data points to detect a peak
    }

    // Find the maximum and minimum values in the deque
    auto maxEle = std::max_element(completedData.begin(), completedData.end());
    auto minEle = std::min_element(completedData.begin(), completedData.end());

    // Calculate the absolute difference between the max and min values
    double diff{ std::abs(*maxEle - *minEle) };

    bool isPeak{ false };
    bool isDip{ false };

    for (size_t i = 1; i < completedData.size() - 1; ++i) {

        // Check if the current data point is greater than both its adjacent points
        if ((completedData[i] > completedData[i - 1]) && (completedData[i] > completedData[i + 1])){
            isPeak = true;
        }

        // Check if the current data point is less than both its adjacent points (dip)
        else if ((completedData[i] < completedData[i - 1]) && (completedData[i] < completedData[i + 1])){
            isDip = true;
        }
    }

    // Determine the sequence type based on peak or dip detection
    if (isPeak && (diff > bumpThreshold)) {
        sequenceType = SequenceType::Rising;
    } else if (isDip && (diff > potholeThreshold)) {
        sequenceType = SequenceType::Falling;
    } else {
        sequenceType = SequenceType::Stable;
        return false;
    }

    if ((isPeak && (diff > bumpThreshold)) || (isDip && (diff > potholeThreshold))){
        return true; // Bump or Pothole detected
    }
    
    return false; // Return false if no bump or pothole is detected
}

bool detectBump(const std::deque<double>& completedData, double bumpThreshold){
    
    if (completedData.size() < 3){
        return false; // Not enough data points to detect a peak
    }

    // Find the maximum and minimum values in the deque
    auto maxEle = std::max_element(completedData.begin(), completedData.end());
    auto minEle = std::min_element(completedData.begin(), completedData.end());

    // Calculate the absolute difference between the max and min values
    double diff{ std::abs(*maxEle - *minEle) };

    bool isPeak{ false };

    for (size_t i = 1; i < completedData.size() - 1; ++i) {
        // Check if the current data point is greater than both its adjacent points
        if ((completedData[i] > completedData[i - 1]) && (completedData[i] > completedData[i + 1])){
            isPeak = true;
        }
    }

    // Check if the difference exceeds the threshold and if it is peak
    if ((diff > bumpThreshold) && isPeak){
        return true;
    }

    return false; // Return false if no bump is detected
}

int main(){
    // Initialize Active Filter
    ActiveFilter actFilter;

    // These parameters are subject to testing
    actFilter.setWindowParameters(50, 35);
    actFilter.setThreshold(0.25); // was 0.25 before

    ThreeAxisIIR_Init(&iirFiltAccel, filterAlpha);
    ThreeAxisIIR_Init(&iirFiltGyro, filterAlpha);

    // Initialize wiringPi and allow the use of BCM pin numbering
    wiringPiSetupGpio();

    RgbLed rgbLed{ redPinNumber, greenPinNumber, bluePinNumber };

    // Setup the pin as output
    //pinMode(ledPin, OUTPUT);
    // pinMode(detectLedPin, OUTPUT);
    //pinMode(onLedPin, OUTPUT);

    // Variables to store the accel, gyro and angle values
    double ax, ay, az;
    double gr, gp, gy;

    // Variables to store the filtered/rotated acceleration values
    double ax_filtered, ay_filtered, az_filtered;                         
    double ax_rotated,  ay_rotated,  az_rotated;

    // Variables to store the filtered/rotated gyroscope values

    double gr_filtered, gp_filtered, gy_filtered;
    double gr_rotated,  gp_rotated,  gy_rotated;

    // Initialize output deque
    std::deque<double> outData;
    outData.clear();

    sleep(1); // Wait for the system clock to get ready

    const std::string timestamp = getCurrentTimestamp();
    const std::string directoryPath = "/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "/";
    const std::string allSensorLogFile = "allSensorLogFile.txt";
    const std::string bumpCountLogFile = "bumpCountLogFile.txt";
    
    if (!createDirectory(directoryPath)) {
        std::cerr << "Error: Unable to create directory." << std::endl;
        return -1;
    }

    std::ofstream directoryPathFile("/home/efeoguslu/Desktop/directory_path.txt");

    if (directoryPathFile.is_open()) {
        directoryPathFile << directoryPath;
        directoryPathFile.close();
    } else {
        std::cout << "Unable to open file for writing." << std::endl;
    }
    
    //Do not read the current yaw angle
    device.calc_yaw = false;

    double pitchAngle{ 0.0 };
    double rollAngle{ 0.0 };

    double compoundAccelerationVector{ 0.0 };

    // double threshold{ 0.25 }; // Example threshold value: 0.5 before
    int sampleNumber{ 0 };

    // outData deque size is fixed value for now:
    const unsigned int wholeDequeSize{ 150 };

    // Initialization of number of samples to be removed:
    unsigned int removeSamples{ 0 };

    bool bumpDetected{ false };
    bool potholeDetected{ false };

    currentConfig.setConfig(midSensitivityConfig);

    Button bumpButton{ bumpPin };
    Button potholeButton{ potholePin };
    Button modeButton{ modePin };

    bool modeButtonPressed{ false };
    std::chrono::time_point<std::chrono::high_resolution_clock> modeButtonPressTime;

    SequenceType previousSequence = SequenceType::Stable;
    SequenceType currentSequence = SequenceType::Stable;

    while(true){
        // Record loop time stamp:
        auto startTime{ std::chrono::high_resolution_clock::now() };

        // Get the current accelerometer values:
        device.getAccel(&ax, &ay, &az);
        // Get the filtered accelerometer values:
        ThreeAxisIIR_Update(&iirFiltAccel, ax, ay, az, &ax_filtered, &ay_filtered, &az_filtered);

        // Get the current gyroscope values:
        device.getGyro(&gr, &gp, &gy);
        // Get the filtered gyroscope values:
        ThreeAxisIIR_Update(&iirFiltGyro, gr, gp, gy, &gr_filtered, &gp_filtered, &gy_filtered);

        // Get the current roll and pitch angles using complementary filter:
        complementaryFilter(ax_filtered, ay_filtered, az_filtered, gr_filtered, gp_filtered, gy_filtered, &rollAngle, &pitchAngle);

        // Rotation:
        rotateAll(rollAngle*degreesToRadians, pitchAngle*degreesToRadians, ax_filtered, ay_filtered, az_filtered, &ax_rotated, &ay_rotated, &az_rotated);
        rotateAll(rollAngle*degreesToRadians, pitchAngle*degreesToRadians, gr_filtered, gp_filtered, gy_filtered, &gr_rotated, &gp_rotated, &gy_rotated);

        // Calculate Rotated Compound Acceleration Vector:
        compoundAccelerationVector = compoundVector(ax_rotated, ay_rotated, az_rotated);
        
        // Apply Active Filter:
        actFilter.feedData(compoundAccelerationVector);

        if(actFilter.getCompletedDataSize() > 0){
            std::deque<double> completedData = actFilter.getCompletedData();
            AppendDeque(outData, completedData);        
        }

        // Increment the sample number for the next iteration
        sampleNumber++;

        // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        // previousSequence = currentSequence; // Save the previous sequence
        // SequenceType newSequence = SequenceType::Stable;

        if(outData.size() > wholeDequeSize){

            removeSamples = static_cast<unsigned int>(outData.size() - wholeDequeSize);
            outData.erase(outData.begin(), outData.begin() + removeSamples);

            if((sampleNumber % wholeDequeSize) == 0){
                bumpDetected = false;
                potholeDetected = false;
            }

            // Update previousSequence before checking for hazards
            previousSequence = currentSequence;

            if (!bumpDetected && !potholeDetected) {

                bool hazardFound = detectHazard(outData, currentConfig.bumpThreshold, currentConfig.potholeThreshold, currentSequence);

                if (hazardFound){

                    if(currentSequence != previousSequence){
                        previousSequence = currentSequence;

                        if(currentSequence == SequenceType::Rising){
                            bumpDetected = true;
                            rgbLed.bumpDetected();
                            ++activeBumpCount;
                            std::cout << "Bump detected at sample number: " << sampleNumber << std::endl;
                            std::string bumpLog = ",sample=" + std::to_string(sampleNumber) + ",bump_count=" + std::to_string(activeBumpCount);
                            TLogger::TLogInfo(directoryPath, bumpCountLogFile, bumpLog);
                        }

                        else if(currentSequence == SequenceType::Falling){
                            potholeDetected = true;
                            rgbLed.potholeDetected();
                            ++activePotholeCount;
                            std::cout << "Pothole detected at sample number: " << sampleNumber << std::endl;
                            std::string potholeLog = ",sample=" + std::to_string(sampleNumber) + ",pothole_count=" + std::to_string(activePotholeCount);
                            TLogger::TLogInfo(directoryPath, bumpCountLogFile, potholeLog);
                        }
                    }
                }
                else {
                    // No hazard found, explicitly set to Stable if not already
                    if (currentSequence != SequenceType::Stable) {
                        currentSequence = SequenceType::Stable;
                    }
                }
            }
        }
        // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        int modeButtonState{ modeButton.getButtonState() };

        // If the button is pressed and was not pressed in the previous iteration
        if (modeButtonState && !modeButtonPressed) {
            modeButtonPressed = true;
            modeButtonPressTime = std::chrono::high_resolution_clock::now(); // Record the time when the button is pressed
        }

        // If the button is not pressed and was pressed in the previous iteration
        else if (!modeButtonState && modeButtonPressed) {
            modeButtonPressed = false;
            auto buttonReleaseTime = std::chrono::high_resolution_clock::now();
            auto buttonPressDuration = std::chrono::duration_cast<std::chrono::milliseconds>(buttonReleaseTime - modeButtonPressTime).count();
            std::cout << "Button was pressed for: " << buttonPressDuration << " ms" << std::endl;

            if(buttonPressDuration > buttonPressDurationThresholdMs){
                currentConfig.setConfig(getNextConfig(currentConfig, rgbLed));
                std::cout << "Configuration changed to: " << currentConfig.getConfigName() << std::endl;
            }
        }
        
        // Update LED
        rgbLed.update();
        
        std::cout << sampleNumber << " " << currentConfig.getConfigStr() << "\n";

        // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        std::string logOut = 
        ",ax="+std::to_string(ax)+",ay="+std::to_string(ay)+",az="+std::to_string(az)+\
        ",ax_filtered="+std::to_string(ax_filtered)+",ay_filtered="+std::to_string(ay_filtered)+",az_filtered="+std::to_string(az_filtered)+\
        ",ax_rotated="+std::to_string(ax_rotated)+",ay_rotated="+std::to_string(ay_rotated)+",az_rotated="+std::to_string(az_rotated)+\
        ",gr="+std::to_string(gr)+",gp="+std::to_string(gp)+",gy="+std::to_string(gy)+\
        ",gr_filtered="+std::to_string(gr_filtered)+",gp_filtered="+std::to_string(gp_filtered)+",gy="+std::to_string(gy_filtered)+\
        ",gr_rotated="+std::to_string(gr_rotated)+",gp_rotated="+std::to_string(gp_rotated)+",gy_rotated="+std::to_string(gy_rotated)+\
        ",roll_angle="+std::to_string(rollAngle)+",pitch_angle="+std::to_string(pitchAngle)+\
        ",comp_vector="+std::to_string(compoundAccelerationVector)+\
        ",pothole_button_state="+potholeButton.getButtonStateStr()+",bump_button_state="+bumpButton.getButtonStateStr()+",mode_button_state="+modeButton.getButtonStateStr()+\
        currentConfig.getConfigStr();
        
        TLogger::TLogInfo(directoryPath, allSensorLogFile, logOut);

        std::cout << "prev: " << static_cast<int>(previousSequence) << " current: " << static_cast<int>(currentSequence) << "\n";

        // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        // Calculate Loop Duration
        auto endTime{ std::chrono::high_resolution_clock::now() };
        auto duration{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() };

        auto delayMs{ static_cast<long>(loopDurationMs - duration) } ;

        if (delayMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        }
    
        dt = timeSync(startTime);
    }
}

