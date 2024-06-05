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
#include <unordered_map>
#include <iterator>

#include "MPU6050.h"
#include "filters.h"
#include "button.h"
#include "rgbled.h"
#include "sequence.h"

#include "common-structs.hpp"


typedef unsigned int uint;
typedef std::deque<double>::iterator deque_iter_double;

class VectorStats {
public:
    VectorStats(deque_iter_double start_iter, deque_iter_double end_iter) {
        this->start = start_iter;
        this->end = end_iter;
        this->compute();
    }

    void compute() {
        double sum = std::accumulate(start, end, 0.0);
        uint slice_size = std::distance(start, end);
        double mean = sum / slice_size;
        std::deque<double> diff(slice_size);
        std::transform(start, end, diff.begin(), [mean](double x) { return x - mean; });
        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double std_dev = std::sqrt(sq_sum / slice_size);

        this->m1 = mean;
        this->m2 = std_dev;
    }

    double mean() {
        return m1;
    }

    double standard_deviation() {
        return m2;
    }

private:
    deque_iter_double start;
    deque_iter_double end;
    double m1;
    double m2;
};

std::deque<double> z_score_thresholding(std::deque<double> input, int lag, double threshold, double influence) {

    std::unordered_map<std::string, std::deque<double>> output;

    uint n = static_cast<uint>(input.size());
    std::deque<double> signals(n);
    std::deque<double> filtered_input(input);
    std::deque<double> filtered_mean(n);
    std::deque<double> filtered_stddev(n);

    VectorStats lag_subvector_stats(input.begin(), input.begin() + lag);
    filtered_mean[lag - 1] = lag_subvector_stats.mean();
    filtered_stddev[lag - 1] = lag_subvector_stats.standard_deviation();

    for (size_t i = lag; i < n; i++) {
        if (std::abs(input[i] - filtered_mean[i - 1]) > threshold * filtered_stddev[i - 1]) {
            signals[i] = (input[i] > filtered_mean[i - 1]) ? 1.0 : -1.0;
            filtered_input[i] = influence * input[i] + (1 - influence) * filtered_input[i - 1];
        } else {
            signals[i] = 0.0;
            filtered_input[i] = input[i];
        }
        VectorStats lag_subvector_stats(filtered_input.begin() + (i - lag), filtered_input.begin() + i);
        filtered_mean[i] = lag_subvector_stats.mean();
        filtered_stddev[i] = lag_subvector_stats.standard_deviation();
    }

    output["signals"] = signals;
    output["filtered_mean"] = filtered_mean;
    output["filtered_stddev"] = filtered_stddev;

    return output["signals"];
};



/*
typedef unsigned int uint;
using deque_iter_double = std::deque<double>::iterator;

class VectorStats {
public:
    VectorStats(deque_iter_double start_iterator, deque_iter_double end_iterator) {
        this->start = start_iterator;
        this->end = end_iterator;
        this->compute();
    }

    void compute() {
        double sum = std::accumulate(start, end, 0.0);
        uint slice_size = std::distance(start, end);
        double mean = sum / slice_size;
        std::vector<double> diff(slice_size);
        std::transform(start, end, diff.begin(), [mean](double x) { return x - mean; });
        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double std_dev = std::sqrt(sq_sum / slice_size);

        this->m1 = mean;
        this->m2 = std_dev;
    }

    double mean() const {
        return m1;
    }

    double standard_deviation() const {
        return m2;
    }

private:
    deque_iter_double start;
    deque_iter_double end;
    double m1;
    double m2;
};

std::deque<double> z_score_thresholding(std::deque<double> input, int lag, double threshold, double influence) {
    uint n = static_cast<uint>(input.size());
    std::deque<double> signals(n, 0.0);
    std::deque<double> filtered_input = input;
    std::deque<double> filtered_mean(n);
    std::deque<double> filtered_stddev(n);

    VectorStats lag_subvector_stats(input.begin(), input.begin() + lag);
    filtered_mean[lag - 1] = lag_subvector_stats.mean();
    filtered_stddev[lag - 1] = lag_subvector_stats.standard_deviation();

    double sum = std::accumulate(input.begin(), input.begin() + lag, 0.0);
    double sum_sq = std::inner_product(input.begin(), input.begin() + lag, input.begin(), 0.0);

    for (int i = lag; i < n; i++) {
        double mean = sum / lag;
        double variance = (sum_sq / lag) - (mean * mean);
        double std_dev = std::sqrt(variance);

        if (std::abs(input[i] - mean) > threshold * std_dev) {
            signals[i] = (input[i] > mean) ? 1.0 : -1.0;
            filtered_input[i] = influence * input[i] + (1 - influence) * filtered_input[i - 1];
        } else {
            signals[i] = 0.0;
            filtered_input[i] = input[i];
        }

        // Update the sum and sum of squares incrementally
        sum += filtered_input[i] - filtered_input[i - lag];
        sum_sq += filtered_input[i] * filtered_input[i] - filtered_input[i - lag] * filtered_input[i - lag];
    }

    return signals;
}
*/



const unsigned int lag{ 50 };
const double z_score_threshold{ 10.0 }; // ulaş abi kaydı 10.0 idi
const double influence{ 0.25 };


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
const int programOnLedPin{ 18 };

// Total counts for recorded bumps and potholes
static int activeBumpCount{ 0 };
static int activePotholeCount{ 0 };

// ----------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------------

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
// Pothole thresholds may be subject to change:

SensitivityConfig lowSensitivityConfig{  "low",  0.25, 0.15 }; 
SensitivityConfig midSensitivityConfig{  "mid",  0.12, 0.09 }; 
SensitivityConfig highSensitivityConfig{ "high", 0.07, 0.04 }; 
SensitivityConfig currentConfig{ "default",      0.12, 0.09 };

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

inline void rotateAll(double rollAngle, double pitchAngle, double x, double y, double z, double* x_rotated, double* y_rotated, double* z_rotated){
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



void complementaryFilter(double ax, double ay, double az, double gr, double gp, double gy, double* rollAngle, double* pitchAngle) {
    static double accel_angle[2] = {0.0, 0.0};
    static double gyro_angle[3] = {0.0, 0.0, 0.0};
    static double angle[3] = {0.0, 0.0, 0.0};
    static bool first_run = true;
    static bool calc_yaw = true;

    // X (roll) axis
    accel_angle[0] = std::atan2(az, ay) * radiansToDegrees - 90.0; // Calculate the angle with z and y, convert to degrees, and subtract 90 degrees to rotate
    gyro_angle[0] = angle[0] + gr * dt; // Use roll axis (X axis)

    // Y (pitch) axis
    accel_angle[1] = std::atan2(az, ax) * radiansToDegrees - 90.0; // Calculate the angle with z and x, convert to degrees, and subtract 90 degrees to rotate
    gyro_angle[1] = angle[1] + gp * dt; // Use pitch axis (Y axis)

    // Z (yaw) axis
    if (calc_yaw) {
        gyro_angle[2] = angle[2] + gy * dt; // Use yaw axis (Z axis)
    }

    if (first_run) { // Set the gyroscope angle reference point if this is the first function run
        for (int i = 0; i <= 1; i++) {
            gyro_angle[i] = accel_angle[i]; // Start off with angle from accelerometer (absolute angle since gyroscope is relative)
        }
        gyro_angle[2] = 0; // Set the yaw axis to zero (because the angle cannot be calculated with the accelerometer when vertical)
        first_run = false;
    }

    double asum = std::fabs(ax) + std::fabs(ay) + std::fabs(az); // Calculate the sum of the accelerations
    double gsum = std::fabs(gr) + std::fabs(gp) + std::fabs(gy); // Calculate the sum of the gyro readings

    for (int i = 0; i <= 1; i++) { // Loop through roll and pitch axes
        if (std::fabs(gyro_angle[i] - accel_angle[i]) > 5) { // Correct for very large drift (or incorrect measurement of gyroscope by longer loop time)
            gyro_angle[i] = accel_angle[i];
        }
        // Create result from either complementary filter or directly from gyroscope or accelerometer depending on conditions
        if (asum > 0.1 && asum < 3 && gsum > 0.3) { // Check that the movement is not very high (therefore providing inaccurate angles)
            angle[i] = (1.0 - tau) * (gyro_angle[i]) + (tau) * (accel_angle[i]); // Calculate the angle using a complementary filter
        }
        else if (gsum > 0.3) { // Use the gyroscope angle if the acceleration is high
            angle[i] = gyro_angle[i];
        }
        else if (gsum <= 0.3) { // Use accelerometer angle if not much movement
            angle[i] = accel_angle[i];
        }
    }

    // The yaw axis will not work with the accelerometer angle, so only use gyroscope angle
    if (calc_yaw) { // Only calculate the angle when we want it to prevent large drift
        angle[2] = gyro_angle[2];
    }
    else {
        angle[2] = 0;
        gyro_angle[2] = 0;
    }

    *rollAngle = angle[0];
    *pitchAngle = angle[1];
}



void AppendDeque(std::deque<double> &target, std::deque<double> source)
{
    for(long unsigned int i = 0; i < source.size(); i++)
    {
        target.push_back(source.at(i));
    }
}


SequenceType getStateChange(const std::deque<double>& states) {
    if (states.size() < 2) {
        return SequenceType::Stable;
    }

    for (size_t i = 1; i < states.size(); ++i) {
        if (states[i] == 0) {
            if (states[i-1] == 1) {
                return SequenceType::Rising;
            } else if (states[i-1] == -1) {
                return SequenceType::Falling;
            }
        }
    }

    return SequenceType::Stable;
}


EventType getStateChangeBump(const std::deque<double>& states) {
    if (states.size() < 2) {
        return EventType::Stable;
    }

    for (size_t i = 1; i < states.size(); ++i) {
        if (states[i] == 0) {
            if (states[i-1] == 1) {
                return EventType::Bump;
            }
        }
    }

    return EventType::Stable;
}

EventType getStateChangePothole(const std::deque<double>& states) {
    if (states.size() < 2) {
        return EventType::Stable;
    }

    for (size_t i = 1; i < states.size(); ++i) {
        if (states[i] == 0) {
            if (states[i-1] == -1) {
                return EventType::Pothole;
            }
        }
    }

    return EventType::Stable;
}

void endBlinkLed(int numBlinks) {
    for (int i = 0; i < numBlinks; ++i) {
        digitalWrite(programOnLedPin, HIGH); // Turn the LED on
        std::this_thread::sleep_for(std::chrono::milliseconds(endRecordingLedDelayMs)); 
        digitalWrite(programOnLedPin, LOW); // Turn the LED off
        std::this_thread::sleep_for(std::chrono::milliseconds(endRecordingLedDelayMs));
    }
}

void saveDequeToFile(const std::deque<double>& signal, const std::string& filename){
    std::ofstream outFile(filename, std::ios_base::app);

    for(size_t i = 0; i < signal.size(); ++i){
        outFile << signal[i] << " ";
    }

    outFile << "\n";
}


template<typename T>
void appendIfNotEmpty(const T& filterConfigured, std::deque<double>& output1, std::deque<double>& output2) {
    if (filterConfigured.getCompletedDataSize() > 0) {
        std::deque<double> completedData = filterConfigured.getCompletedData();
        AppendDeque(output1, completedData);
        AppendDeque(output2, completedData);
    }
}

void removeExcessSamples(std::deque<double>& outData, int size) {
    if (outData.size() > size) {
        unsigned int remove = static_cast<unsigned int>(outData.size() - size);
        outData.erase(outData.begin(), outData.begin() + remove);
    }
}

void applyZScoreThresholding(const std::deque<double>& outData, std::deque<double>& sequenceDeque, int size, unsigned int lag, double z_score_threshold, double influence){
    if (outData.size() == size) {
        sequenceDeque = z_score_thresholding(outData, lag, z_score_threshold, influence);
    }
}

void determineState(const std::deque<double>& sequenceDeque, std::deque<int>& stateDeque){
    if (getStateChange(sequenceDeque) == SequenceType::Rising) {
        stateDeque.push_back(1);
    }
    if (getStateChange(sequenceDeque) == SequenceType::Falling) {
        stateDeque.push_back(-1);
    }
    if (getStateChange(sequenceDeque) == SequenceType::Stable) {
        stateDeque.push_back(0);
    }
}



int main(){


   // Initialize Active Filters
    
    int activeFilterWindowSize{ 50 };
    int activeFilterOverlapSize{ 35 };

    double activeFilterThreshold{ 0.2 };
    double activeFilterPositiveCoefficient{ 1.6 };
    double activeFilterNegativeCoefficient{ 0.1 };

    /*
    ActiveFilter actFilterBumpConfigured;
    double bumpActiveFilterThreshold{ 0.19 };
    double bumpActiveFilterPositiveCoef{ 1.7 };
    double bumpActiveFilterNegativeCoef{ 0.7 }; 
    // Parameters for the Bump Active Filter
    actFilterBumpConfigured.setWindowParameters(activeFilterWindowSize, activeFilterOverlapSize);
    actFilterBumpConfigured.setThreshold(bumpActiveFilterThreshold);
    actFilterBumpConfigured.setCoefficients(bumpActiveFilterPositiveCoef, bumpActiveFilterNegativeCoef);


    ActiveFilter actFilterPotholeConfigured;
    double potholeActiveFilterThreshold{ 0.18 };
    double potholeActiveFilterPositiveCoef{ 1.9 };
    double potholeActiveFilterNegativeCoef{ 0.1 }; 
    // Parameters for the Pothole Active Filter
    actFilterPotholeConfigured.setWindowParameters(activeFilterWindowSize, activeFilterOverlapSize);
    actFilterPotholeConfigured.setThreshold(potholeActiveFilterThreshold);
    actFilterPotholeConfigured.setCoefficients(potholeActiveFilterPositiveCoef, potholeActiveFilterNegativeCoef);
    */



    // Initialize Active Filter
    
    ActiveFilter actFilter;

    // These parameters are subject to testing
    actFilter.setWindowParameters(activeFilterWindowSize, activeFilterOverlapSize);
    actFilter.setThreshold(activeFilterThreshold); // was 0.25 before
    actFilter.setCoefficients(activeFilterPositiveCoefficient, activeFilterNegativeCoefficient);

    
    // Initialize IIR Filter
    ThreeAxisIIR iirFiltAccel;
    ThreeAxisIIR iirFiltGyro;
    ThreeAxisIIR_Init(&iirFiltAccel, filterAlpha);
    ThreeAxisIIR_Init(&iirFiltGyro, filterAlpha);

    // Initialize wiringPi and allow the use of BCM pin numbering
    wiringPiSetupGpio();

    RgbLed rgbLed{ redPinNumber, greenPinNumber, bluePinNumber };

    // Setup the pin as output
    // pinMode(ledPin, OUTPUT);
    // pinMode(detectLedPin, OUTPUT);

    pinMode(programOnLedPin, OUTPUT);
    digitalWrite(programOnLedPin, HIGH);

    // Variables to store the accel, gyro and angle values
    double ax{0.0}, ay{0.0}, az{0.0};
    double gr{0.0}, gp{0.0}, gy{0.0};

    // Variables to store the filtered/rotated acceleration values
    double ax_filtered{0.0}, ay_filtered{0.0}, az_filtered{0.0};                         
    double ax_rotated{0.0},  ay_rotated{0.0},  az_rotated{0.0};

    // Variables to store the filtered/rotated gyroscope values

    double gr_filtered{0.0}, gp_filtered{0.0}, gy_filtered{0.0};
    double gr_rotated{0.0},  gp_rotated{0.0},  gy_rotated{0.0};

    // Initialize output deque

    
    std::deque<double> outData;
    outData.clear();

    std::deque<double> sequenceDeque;
    sequenceDeque.clear();

    std::deque<double> stateDeque;
    stateDeque.clear();
    
    /*
    std::deque<double> outBumpData;
    outBumpData.clear();
    std::deque<double> outPotholeData;
    outPotholeData.clear();

    std::deque<double> sequenceBumpDeque;
    sequenceBumpDeque.clear();
    std::deque<double> sequencePotholeDeque;
    sequencePotholeDeque.clear();
    */
    


    /*
    std::deque<int> stateBumpDeque;
    stateBumpDeque.clear();
    std::deque<int> statePotholeDeque;
    statePotholeDeque.clear();
    */
    

    
    sleep(1); // Wait for the system clock to get ready

    const std::string timestamp = getCurrentTimestamp();
    const std::string directoryPath = "/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "/";
    const std::string allSensorLogFile = "allSensorLogFile.txt";
    const std::string bumpCountLogFile = "bumpCountLogFile.txt";
    const std::string configurationFile = "configurationFile.txt";

    
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

    double rollAngle{ 0.0 };
    double pitchAngle{ 0.0 };

    double compoundAccelerationVector{ 0.0 };

    // double threshold{ 0.25 }; // Example threshold value: 0.5 before
    int sampleNumber{ 0 };

    // outData deque size is fixed value for now:
    const unsigned int wholeDequeSize{ 150 }; // was 150 before

    

    // currentConfig.setConfig(midSensitivityConfig);

    Button bumpButton{ bumpPin };
    Button potholeButton{ potholePin };
    Button modeButton{ modePin };

    Button endRecordingButton{ endRecordingPin };

    bool modeButtonPressed{ false };
    bool endRecordingButtonPressed{ false };

    std::chrono::time_point<std::chrono::high_resolution_clock> modeButtonPressTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> endRecordingButtonPressTime;

    /*
    std::deque<double> activeFilterBumpOutput;
    activeFilterBumpOutput.clear();

    std::deque<double> activeFilterPotholeOutput;
    activeFilterPotholeOutput.clear();
    */
    
    std::deque<double> activeFilterOutput;
    activeFilterOutput.clear();


    std::string configuration = 
        ",lag="+std::to_string(lag)+",z_score_threshold="+std::to_string(z_score_threshold)+",influence="+std::to_string(influence)+\
        ",activeFilterWindowSize="+std::to_string(activeFilterWindowSize)+",activeFilterOverlapSize="+std::to_string(activeFilterOverlapSize)+\
        ",activeFilterThreshold="+std::to_string(activeFilterThreshold)+",activeFilterPositiveCoef="+std::to_string(activeFilterPositiveCoefficient)+",activeFilterNegativeCoef="+std::to_string(activeFilterNegativeCoefficient);

    TLogger::TLogInfo(directoryPath, configurationFile, configuration);



    while(true){
        // Record loop time stamp:
        auto startTime{ std::chrono::high_resolution_clock::now() };

        // Get the current accelerometer values:
        device.getAccel(&ax, &ay, &az);
        // Get the current gyroscope values:
        device.getGyro(&gr, &gp, &gy);

        // Get the filtered accelerometer values:
        ThreeAxisIIR_Update(&iirFiltAccel, ax, ay, az, &ax_filtered, &ay_filtered, &az_filtered);        
        // Get the filtered gyroscope values:
        ThreeAxisIIR_Update(&iirFiltGyro, gr, gp, gy, &gr_filtered, &gp_filtered, &gy_filtered);

        // Get the current roll and pitch angles using complementary filter:
        complementaryFilter(ax_filtered, ay_filtered, az_filtered, gr_filtered, gp_filtered, gy_filtered, &rollAngle, &pitchAngle);

        // Rotation:
        rotateAll(rollAngle*degreesToRadians, pitchAngle*degreesToRadians, ax_filtered, ay_filtered, az_filtered, &ax_rotated, &ay_rotated, &az_rotated);
        //rotateAll(rollAngle*degreesToRadians, pitchAngle*degreesToRadians, gr_filtered, gp_filtered, gy_filtered, &gr_rotated, &gp_rotated, &gy_rotated);

        // Calculate Rotated Compound Acceleration Vector:
        compoundAccelerationVector = compoundVector(ax_rotated, ay_rotated, az_rotated);
        
        // Apply Active Filter:

        actFilter.feedData(compoundAccelerationVector);

        //actFilterBumpConfigured.feedData(compoundAccelerationVector);
        //actFilterPotholeConfigured.feedData(compoundAccelerationVector);

        appendIfNotEmpty(actFilter, outData, activeFilterOutput);

        //appendIfNotEmpty(actFilterBumpConfigured, outBumpData, activeFilterBumpOutput);
        //appendIfNotEmpty(actFilterPotholeConfigured, outPotholeData, activeFilterPotholeOutput);
        
        // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        removeExcessSamples(outData, wholeDequeSize);

        //removeExcessSamples(outBumpData, wholeDequeSize);
        //removeExcessSamples(outPotholeData, wholeDequeSize);
        
        // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        // std::cout << "sequence deque: " << sequenceDeque.size() << std::endl;

        applyZScoreThresholding(outData, sequenceDeque, wholeDequeSize, lag, z_score_threshold, influence);

        //applyZScoreThresholding(outBumpData, sequenceBumpDeque, wholeDequeSize, lag, z_score_threshold, influence);
        //applyZScoreThresholding(outPotholeData, sequencePotholeDeque, wholeDequeSize, lag, z_score_threshold, influence);



        // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        /*
        if (getStateChange(sequenceBumpDeque) == SequenceType::Rising) {
            rgbLed.bumpDetected();
            ++activeBumpCount;
            //std::cout << "Bump detected at sample number: " << sampleNumber << std::endl;
            std::string bumpLog = ",sample=" + std::to_string(sampleNumber) + ",bump_count=" + std::to_string(activeBumpCount);
            TLogger::TLogInfo(directoryPath, bumpCountLogFile, bumpLog);
        }

        if (getStateChange(sequencePotholeDeque) == SequenceType::Falling) {
            rgbLed.potholeDetected();
            ++activePotholeCount;
            //std::cout << "Pothole detected at sample number: " << sampleNumber << std::endl;
            std::string potholeLog = ",sample=" + std::to_string(sampleNumber) + ",pothole_count=" + std::to_string(activePotholeCount);
            TLogger::TLogInfo(directoryPath, bumpCountLogFile, potholeLog);
        }
        */

        if (getStateChangeBump(sequenceDeque) == EventType::Bump) {
            rgbLed.bumpDetected();
            ++activeBumpCount;
            //std::cout << "Bump detected at sample number: " << sampleNumber << std::endl;
            std::string bumpLog = ",sample=" + std::to_string(sampleNumber) + ",bump_count=" + std::to_string(activeBumpCount);
            TLogger::TLogInfo(directoryPath, bumpCountLogFile, bumpLog);
        }

        if (getStateChangePothole(sequenceDeque) == EventType::Pothole) {
            rgbLed.potholeDetected();
            ++activePotholeCount;
            //std::cout << "Pothole detected at sample number: " << sampleNumber << std::endl;
            std::string potholeLog = ",sample=" + std::to_string(sampleNumber) + ",pothole_count=" + std::to_string(activePotholeCount);
            TLogger::TLogInfo(directoryPath, bumpCountLogFile, potholeLog);
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
            // std::cout << "Button was pressed for: " << buttonPressDuration << " ms" << std::endl;

            if(buttonPressDuration > buttonPressDurationThresholdMs){
                currentConfig.setConfig(getNextConfig(currentConfig, rgbLed));
                std::cout << "Configuration changed to: " << currentConfig.getConfigName() << std::endl;
            }
        }

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

        //",state_bump="+std::to_string(static_cast<int>(getStateChange(sequenceBumpDeque)))+",state_pothole="+std::to_string(static_cast<int>(getStateChange(sequencePotholeDeque)))+

        ",state_bump="+std::to_string(static_cast<int>(getStateChangeBump(sequenceDeque))) + ",state_pothole="+std::to_string(static_cast<int>(getStateChangePothole(sequenceDeque)))+\
        ",pothole_button_state="+potholeButton.getButtonStateStr()+",bump_button_state="+bumpButton.getButtonStateStr()+",mode_button_state="+modeButton.getButtonStateStr();

        
        TLogger::TLogInfo(directoryPath, allSensorLogFile, logOut);
        
        

        // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        int endRecordingButtonState{ endRecordingButton.getButtonState() };

        // If the button is pressed and was not pressed in the previous iteration
        if (endRecordingButtonState && !endRecordingButtonPressed) {
            endRecordingButtonPressed = true;
            endRecordingButtonPressTime = std::chrono::high_resolution_clock::now(); // Record the time when the button is pressed
        }

        // If the button is not pressed and was pressed in the previous iteration
        else if (!endRecordingButtonState && endRecordingButtonPressed) {
            endRecordingButtonPressed = false;
            auto endButtonReleaseTime = std::chrono::high_resolution_clock::now();
            auto endButtonPressDuration = std::chrono::duration_cast<std::chrono::milliseconds>(endButtonReleaseTime - endRecordingButtonPressTime).count();
            // std::cout << "Button was pressed for: " << endButtonPressDuration << " ms" << std::endl;

            if(endButtonPressDuration > endButtonPressDurationThresholdMs){
                break;
            }
        }




        // Increment the sample number for the next iteration
        sampleNumber++;

        // Update LED
        rgbLed.update();
        
        // Calculate Loop Duration
        auto endTime{ std::chrono::high_resolution_clock::now() };
        auto duration{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() };

        auto delayMs{ static_cast<long>(loopDurationMs - duration) };
       
        if (delayMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        }
    
        dt = timeSync(startTime);
    }

    


    std::cout << "recording ended\n";
    endBlinkLed(5);
    return 0;
}

