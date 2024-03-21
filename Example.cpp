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

#include "logging.h"
#include "MPU6050.h"
#include "filters.h"
#include "queue.h"

MPU6050 device(0x68, false);

const int windowSize{ 5 };

const int sampleRateHz{ 75 };                  // Sample rate in Hz
const int loopDurationMs{ 1000 / sampleRateHz }; // Duration of each loop iteration in milliseconds

const float gravity_mps2{ 9.80665 };
const float tau{ 0.05 };
const float radiansToDegrees{ 57.2957795 };
const float degreesToRadians{ 0.0174532925 };


int time2Delay{ 0 };
float dt{ 0 };

float filterAlpha{ 0.5f };

// ----------------------------------------------------------------------------------------------

FirstOrderIIR filt;


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



void rotateRoll(float rollAngle, float ax, float ay, float az, float* ax_rotated, float* ay_rotated, float* az_rotated){
    *ax_rotated = ax;
    *ay_rotated = ay*cosf(rollAngle) + az*sinf(rollAngle);
    *az_rotated = ay*sinf(rollAngle) - az*cosf(rollAngle);
}

void rotatePitch(float pitchAngle, float ax, float ay, float az, float* ax_rotated, float* ay_rotated, float* az_rotated){
    *ax_rotated = ax*cosf(pitchAngle) + az*sinf(pitchAngle);
    *ay_rotated = ay;
    *az_rotated = -ax*sinf(pitchAngle) + az*cosf(pitchAngle);
}

// WINNER:

void rotateAll(float rollAngle, float pitchAngle, float ax, float ay, float az, float* ax_rotated, float* ay_rotated, float* az_rotated){

    *ax_rotated =  ax*cosf(pitchAngle)                                      + az*sinf(pitchAngle);
    *ay_rotated = -ax*sinf(pitchAngle)*sinf(rollAngle) + ay*cosf(rollAngle) + az*cosf(pitchAngle)*sinf(rollAngle);
    *az_rotated = -ax*sinf(pitchAngle)*cosf(rollAngle) - ay*sinf(rollAngle) + az*cosf(rollAngle)*cosf(pitchAngle);  
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

void complementaryFilter(float ax, float ay, float az, float gr, float gp, float gy, float* rollAngle, float* pitchAngle){

    clock_gettime(CLOCK_REALTIME, &device.start); //Read current time into start variable
	//X (roll) axis
	device._accel_angle[0] = atan2(az, ay) * RAD_T_DEG - 90.0; //Calculate the angle with z and y convert to degrees and subtract 90 degrees to rotate
	device._gyro_angle[0] = device._angle[0] + gr*dt; //Use roll axis (X axis)
	//Y (pitch) axis
	device._accel_angle[1] = atan2(az, ax) * RAD_T_DEG - 90.0; //Calculate the angle with z and x convert to degrees and subtract 90 degrees to rotate
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
	float asum = abs(ax) + abs(ay) + abs(az); //Calculate the sum of the accelerations
	float gsum = abs(gr) + abs(gp) + abs(gy); //Calculate the sum of the gyro readings
	for (int i = 0; i <= 1; i++) { //Loop through roll and pitch axes
		if (abs(device._gyro_angle[i] - device._accel_angle[i]) > 5) { //Correct for very large drift (or incorrect measurment of gyroscope by longer loop time)
			device._gyro_angle[i] = device._accel_angle[i];
		}
		//Create result from either complementary filter or directly from gyroscope or accelerometer depending on conditions
		if (asum > 0.1 && asum < 3 && gsum > 0.3) { //Check that th movement is not very high (therefore providing inacurate angles)
			device._angle[i] = (1 - TAU)*(device._gyro_angle[i]) + (TAU)*(device._accel_angle[i]); //Calculate the angle using a complementary filter
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

    clock_gettime(CLOCK_REALTIME, &device.end); //Save time to end clock
	dt = (device.end.tv_sec - device.start.tv_sec) + (device.end.tv_nsec - device.start.tv_nsec) / 1e9; //Calculate new dt
	clock_gettime(CLOCK_REALTIME, &device.start); //Save time to start clock

}


void logBumpsToFile(std::ofstream &logfile, int bumpCount) {
    // Get current timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // Write timestamp and sensor data
        logfile << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X") << ", ";
        logfile << std::setprecision(6) << std::fixed; // Set precision for all subsequent data
        logfile << bumpCount << std::endl;
}





int main() {     
    
    
    /*
    if(!toggleFlag()){
        std::cout << "Logging has already occured. Exiting. (Flag == 0)" << std::endl;
        return 0;
    }
    */
    
    FirstOrderIIR_Init(&filt, filterAlpha);
    

    float ax, ay, az, gr, gp, gy;             // Variables to store the accel, gyro and angle values
    float ax_rotated, ay_rotated, az_rotated; // Variables to store the rotated acceleration

    sleep(1); // Wait for the system clock to get ready


    auto timestamp = getCurrentTimestamp();
    auto directoryPath = "/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "/";
    
    
    if (!createDirectory(directoryPath)) {
        std::cerr << "Error: Unable to create directory." << std::endl;
        return -1;
    }

    // Input/Output Files:

    std::ofstream sensorLogFile(directoryPath + "sensorLogFile.txt");
    std::ofstream compoundVectorLogFile(directoryPath + "compoundVectorLogFile.txt");

    std::ofstream bumpCountNaiveLogFile(directoryPath + "bumpCountNaiveLogFile.txt");
    std::ofstream bumpCountNaiveSquaredLogFile(directoryPath + "bumpCountNaiveSquaredLogFile.txt");
    std::ofstream bumpCountCircularBufferLogFile(directoryPath + "bumpCountCircularBufferLogFile.txt");

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

    std::ofstream rotatedAzLogFile(directoryPath +  "rotatedAzLogFile.txt");
    std::ofstream rotatedAzFilteredLogFile(directoryPath +  "rotatedAzFilteredLogFile.txt");
    
    

    std::ofstream normalizedAccelLogFile(directoryPath + "normalizedAccel.txt");
    std::ofstream bumpCount(directoryPath + "bumpCount.txt");

    std::ofstream iirFilter(directoryPath + "iirFilter.txt");

    std::ofstream meanLogFile(directoryPath + "meanLogFile.txt");
    std::ofstream standartDeviationLogFile(directoryPath + "standartDeviationLogFile.txt");
    std::ofstream varianceLogFile(directoryPath + "varianceLogFile.txt");

    /*
    if (!sensorLogFile.is_open() || !compoundVectorLogFile.is_open() || !bumpCountLogFile.is_open() || !axLogFile.is_open() ||\
     !inputForFilter.is_open() || !axFilteredLogFile.is_open() || !normalizedAccelLogFile.is_open() || !ayLogFile.is_open() ||\
     !azLogFile.is_open() || !anglesLogFile.is_open()) {
        std::cerr << "Error: Unable to open log files." << std::endl;
        return -1;
    }
    */


    //Read the current yaw angle
    device.calc_yaw = false;


    //Get bump counts for different cases (Other than buffer)
    int bumpCounterNaive{ 0 };
    int bumpCounterNaiveSquared{ 0 };

    // TODO: add cases for mean, variance and std. dev.

    MovingAverage<float> axMovingAvg(windowSize);
    MovingAverage<float> ayMovingAvg(windowSize);
    MovingAverage<float> azMovingAvg(windowSize);
    
    MovingAverage<float> compoundAccelMovingAvg(windowSize);

    MovingAverage<float> rotatedAzMovingAvg(windowSize);

    float pitchAngleComp{ 0.0f };
    float rollAngleComp{ 0.0f };



    float iirFilterOutput{ 0 };


    queue q1;
    int bufferSize{ 30 };
    init_queue(&q1, bufferSize);

    float mean{ 0 };
    float std_dev{ 0 };
    float variance{ 0 };

    while(true){
        // Record loop time stamp
        auto startTime{std::chrono::high_resolution_clock::now()};

        //Get the current accelerometer values
        device.getAccel(&ax, &ay, &az);

        //Get the current gyroscope values
        device.getGyro(&gr, &gp, &gy);

        //Get the current roll and pitch angles using complementary filter
        complementaryFilter(ax, ay, az, gr, gp, gy, &rollAngleComp, &pitchAngleComp);

        // Rotation:
        rotateAll(rollAngleComp*degreesToRadians, pitchAngleComp*degreesToRadians, ax, ay, az, &ax_rotated, &ay_rotated, &az_rotated);

        // First Order IIR Implementation:
        iirFilterOutput = FirstOrderIIR_Update(&filt, az_rotated);

        enqueue(&q1, iirFilterOutput);

        if(queue_full(&q1)){
            mean = calculate_mean(&q1);
            std_dev = calculate_std_dev(&q1, mean);
            variance = calculate_variance(&q1, mean);


            std::cout << "Bump Count: " << q1.bump_counter << std::endl;

            logData(meanLogFile, mean);
            logData(standartDeviationLogFile, std_dev);
            logData(varianceLogFile, variance);

        }


        /*

        constexpr int width = 5;

        std::cout << std::fixed << std::setprecision(2); // Set precision for floating point numbers
        std::cout 
            << " AccZ (m/s^2): "                   << std::setw(width) << az << " | "
            << " AccZ (rotated)  (m/s^2): "        << std::setw(width) << az_rotated << " | "
            << " Filter Output: "               << std::setw(width) << iirFilterOutput << std::endl;
        */
        


        // auto timestamp {getCurrentTimestamp()};
        // -----------------------------------------------------------------------------------------------

        // Printing to Terminal:
        
        //std::cout << "Time: " << timestamp << " Accel: " << ax << ", Y: " << ay << ", Z: " << az << " Bump Counter: " << bumpCounter << "\n"; 



        /*

        std::cout << std::fixed << std::setprecision(3); // Set precision for floating point numbers


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
        
        constexpr int width = 5;

        std::cout << std::fixed << std::setprecision(2); // Set precision for floating point numbers
        std::cout 
            << "AccX (m/s^2): "                    << std::setw(width) << ax << " | "
            << " AccY (m/s^2): "                   << std::setw(width) << ay << " | "
            << " AccZ (m/s^2): "                   << std::setw(width) << az << " | "
            << " AccX (rotated) (m/s^2): "         << std::setw(width) << ax_rotated << " | "
            << " AccY (rotated) (m/s^2): "         << std::setw(width) << ay_rotated << " | "
            << " AccZ (rotated)  (m/s^2): "        << std::setw(width) << az_rotated << " | "
            << " Roll Angle (deg): "               << std::setw(width) << rollAngleComp << " | "
            << " Pitch Angle (deg): "              << std::setw(width) << pitchAngleComp << std::endl;

        */
        
        
        /*

        std::cout << std::fixed << std::setprecision(2); // Set precision for floating point numbers
        std::cout 
            << "AccX (m/s^2): "         << std::setw(width) << ax << " | "
            << " AccY (m/s^2): "        << std::setw(width) << ay << " | "
            << " AccZ (m/s^2): "        << std::setw(width) << az << " | "
            << " Pitch Angle Formula: " << std::setw(width) << pitchAngleFormula << " | "
            << " Pitch Angle Comp: "   << std::setw(width) << pitchAngleComp << " | "
            << " Roll Angle Formula: " << std::setw(width) << rollAngleFormula << " | "
            << " Roll Angle Comp: " << std::setw(width) << rollAngleComp << std::endl;


        */

        
        
        // Logging:
        
        // Most of the algorithmic changes will be made here:

        
        if(iirFilterOutput >= 1.4f){
            bumpCounterNaive++;
            std::cout << "Bump Count for Naive Case: " << bumpCounterNaive << "\n";
            logData(bumpCountNaiveLogFile, bumpCounterNaive);
        }

        if(iirFilterOutput*iirFilterOutput >= 1.8f){
            bumpCounterNaiveSquared++;
            std::cout << "Bump Count for Naive Squared Case: " << bumpCounterNaiveSquared << "\n";
            logData(bumpCountNaiveSquaredLogFile, bumpCounterNaiveSquared);
        }
        
        logData(bumpCountCircularBufferLogFile, q1.bump_counter);
        
        


        
        
        

        
        std::vector<std::ofstream*> logFiles = {&axLogFile, &ayLogFile, &azLogFile, &grLogFile, &gpLogFile, &gyLogFile};
        std::vector<float> data = {ax, ay, az, gr, gp, gy}; 

        for (size_t i = 0; i < logFiles.size(); ++i) {
            logData(*logFiles[i], data[i]);
        }
        
        logData(rotatedAzLogFile, az_rotated);
        logData(iirFilter, iirFilterOutput); // az_rotated is filtered with IIR 

        logAngles(anglesLogFile, rollAngleComp, pitchAngleComp);

        logData(sensorLogFile, ax, ay, az, gr, gp, gy);


        axMovingAvg.updateAndLogMovingAverage(axFilteredLogFile, ax);
        ayMovingAvg.updateAndLogMovingAverage(ayFilteredLogFile, ay);
        azMovingAvg.updateAndLogMovingAverage(azFilteredLogFile, az);
        
        
        rotatedAzMovingAvg.updateAndLogMovingAverage(rotatedAzFilteredLogFile, az_rotated);        
       
       




        // Calculate Loop Duration

        auto endTime{std::chrono::high_resolution_clock::now()};
        auto duration{std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count()};

        auto delayMs{static_cast<long>(loopDurationMs - duration)};

        if (delayMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        }
    
        dt = timeSync(startTime);
    }

	return 0;
}


