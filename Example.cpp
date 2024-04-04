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
#include "gpio.h"

#include "common-structs.hpp"

MPU6050 device(0x68, false);

const int windowSizeMovingAverage{ 5 };

const int sampleRateHz{ 75 };                  // Sample rate in Hz
const int loopDurationMs{ 1000 / sampleRateHz }; // Duration of each loop iteration in milliseconds

const float gravity_mps2{ 9.80665f };
const float tau{ 0.05f };
const float radiansToDegrees{ 57.2957795f };
const float degreesToRadians{ 0.0174532925f };


// int time2Delay{ 0 };
float dt{ 0.0f };

float filterAlpha{ 0.95f };

// Define the pin we are going to use
const int ledPin{ 17 }; // Example: GPIO 17
const int buttonPin{ 16 };

// ----------------------------------------------------------------------------------------------

FirstOrderIIR iirFilt;
// FIRFilter firFilt;

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

void rotateAll(float rollAngle, float pitchAngle, float ax, float ay, float az, float* ax_rotated, float* ay_rotated, float* az_rotated){

    *ax_rotated =  ax*cosf(pitchAngle)                                      + az*sinf(pitchAngle);
    *ay_rotated = -ax*sinf(pitchAngle)*sinf(rollAngle) + ay*cosf(rollAngle) + az*cosf(pitchAngle)*sinf(rollAngle);
    *az_rotated = -ax*sinf(pitchAngle)*cosf(rollAngle) - ay*sinf(rollAngle) + az*cosf(rollAngle)*cosf(pitchAngle);  
}

// Time stabilization function
float timeSync(auto t1){
    // Find duration to sleep thread
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    int time2Delay = duration.count(); // Convert to int for sleep_for

    // Sleep thread
    std::this_thread::sleep_for(std::chrono::microseconds(loopDurationMs - time2Delay));

    // Calculate dt
    auto t3 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t1);
    dt = static_cast<float>(duration2.count()) * 1E-6; // Explicitly cast to float

    // Return dt and begin main loop again
    return dt;
}


void complementaryFilter(float ax, float ay, float az, float gr, float gp, float gy, float* rollAngle, float* pitchAngle){

    // clock_gettime(CLOCK_REALTIME, &device.start); //Read current time into start variable


	//X (roll) axis
	device._accel_angle[0] = atan2f(az, ay) * radiansToDegrees - 90.0f; //Calculate the angle with z and y convert to degrees and subtract 90 degrees to rotate
	device._gyro_angle[0] = device._angle[0] + gr*dt; //Use roll axis (X axis)
	//Y (pitch) axis
	device._accel_angle[1] = atan2f(az, ax) * radiansToDegrees - 90.0f; //Calculate the angle with z and x convert to degrees and subtract 90 degrees to rotate
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
	float asum = std::fabs(ax) + std::fabs(ay) + std::fabs(az); //Calculate the sum of the accelerations
	float gsum = std::fabs(gr) + std::fabs(gp) + std::fabs(gy); //Calculate the sum of the gyro readings
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

    // clock_gettime(CLOCK_REALTIME, &device.end); //Save time to end clock
	// dt = (device.end.tv_sec - device.start.tv_sec) + (device.end.tv_nsec - device.start.tv_nsec) / 1e9; //Calculate new dt
	// clock_gettime(CLOCK_REALTIME, &device.start); //Save time to start clock

}
void lowThresholdUpdate(float* output, float x){
    if(x >= 0.8f && x <= 1.19f){
        *output = 0.0f;
    }
    else if(x < 0.8f){
        *output = x - 1.0f; // This will return a negative value for inputs smaller than 0.8
    }
    else{
        *output = x - 1.0f; // This will return the same output for inputs larger than 1.2
    }
}

typedef struct{
    float ax, ay, az, gr, gp, gy;
}sensorData;

int main() {     
    // Initialize wiringPi and allow the use of BCM pin numbering
    wiringPiSetupGpio();

    // Setup the pin as output
    pinMode(ledPin, OUTPUT);

    // Configure the pin as input
    pinMode(buttonPin, INPUT); 

    // Blink the LED 3 times with a delay of 0.1 second between each state change
    blink_led(ledPin, 3, 100);

    // Initialize Filters
    FirstOrderIIR_Init(&iirFilt, filterAlpha);
    // FIRFilter_Init(&firFilt);
    

    float ax, ay, az, gr, gp, gy;             // Variables to store the accel, gyro and angle values
    float ax_rotated, ay_rotated, az_rotated; // Variables to store the rotated acceleration

    float lowThresholdOutput;

    sleep(1); // Wait for the system clock to get ready


    auto timestamp = getCurrentTimestamp();
    auto directoryPath = "/home/efeoguslu/Desktop/road-hazard-detection-v3/logs/" + timestamp + "/";
    
    
    if (!createDirectory(directoryPath)) {
        std::cerr << "Error: Unable to create directory." << std::endl;
        return -1;
    }

    if (!createPlotScript(directoryPath)) {
        std::cerr << "Error: Unable to create plot script." << std::endl;
        return -1;
    }

    // Input/Output Files:

    std::ofstream sensorLogFile(directoryPath + "sensorLogFile.txt");
    std::ofstream compoundVectorLogFile(directoryPath + "compoundVectorLogFile.txt");

    std::ofstream bumpCountNaiveLogFile(directoryPath + "bumpCountNaiveLogFile.txt");
    std::ofstream bumpCountNaiveSquaredLogFile(directoryPath + "bumpCountNaiveSquaredLogFile.txt");
    std::ofstream bumpCountCircularBufferLogFile(directoryPath + "bumpCountCircularBufferLogFile.txt");
    std::ofstream bumpCountLowThresholdLogFile(directoryPath + "bumpCountLowThresholdLogFile.txt");

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

    std::ofstream iirFilterLogFile(directoryPath + "iirFilterLogFile.txt");
    std::ofstream firFilterLogFile(directoryPath + "firFilterLogFile.txt");

    std::ofstream meanLogFile(directoryPath + "meanLogFile.txt");
    std::ofstream standartDeviationLogFile(directoryPath + "standartDeviationLogFile.txt");
    std::ofstream varianceLogFile(directoryPath + "varianceLogFile.txt");

    std::ofstream userInputLogFile(directoryPath + "userInputLogFile.txt");
    
    std::ofstream lowThresholdLogFile(directoryPath + "lowThresholdLogFile.txt");

    std::ofstream allSensorDataLogFile(directoryPath + "allSensorDataLogFile.txt");


    /*
    if (!sensorLogFile.is_open() || !compoundVectorLogFile.is_open() || !bumpCountLogFile.is_open() || !axLogFile.is_open() ||\
     !inputForFilter.is_open() || !axFilteredLogFile.is_open() || !normalizedAccelLogFile.is_open() || !ayLogFile.is_open() ||\
     !azLogFile.is_open() || !anglesLogFile.is_open()) {
        std::cerr << "Error: Unable to open log files." << std::endl;
        return -1;
    }
    */


    //Do not read the current yaw angle
    device.calc_yaw = false;

    /*
    MovingAverage<float> axMovingAvg(windowSizeMovingAverage);
    MovingAverage<float> ayMovingAvg(windowSizeMovingAverage);
    MovingAverage<float> azMovingAvg(windowSizeMovingAverage);
    
    MovingAverage<float> compoundAccelMovingAvg(windowSizeMovingAverage);

    MovingAverage<float> rotatedAzMovingAvg(windowSizeMovingAverage);
    */

    

    float pitchAngleComp{ 0.0f };
    float rollAngleComp{ 0.0f };

    float iirFilterOutput{ 0.0f };

    // float firFilterOutput{ 0.0f };


    queue q1, q2;

    int circularBufferSize{ 10 };
    init_queue(&q1, circularBufferSize);
    init_queue(&q2, circularBufferSize);

    float mean{ 0.0f };
    float std_dev{ 0.0f };

    std::vector<int> verticalLineIndices;

    while(true){
        // Record loop time stamp
        auto startTime{std::chrono::high_resolution_clock::now()};

        // Convert high_resolution_clock::time_point to system_clock::time_point
        auto systemTimePoint = std::chrono::time_point_cast<std::chrono::system_clock::duration>(startTime - std::chrono::high_resolution_clock::now() + std::chrono::system_clock::now());

        // Convert system_clock::time_point to std::time_t
        std::time_t now = std::chrono::system_clock::to_time_t(systemTimePoint);

        // Print the time
        // std::cout << std::ctime(&now) << std::endl;


        //Get the current accelerometer values
        device.getAccel(&ax, &ay, &az);
        //Get the current gyroscope values
        device.getGyro(&gr, &gp, &gy);

        //Get the current roll and pitch angles using complementary filter
        complementaryFilter(ax, ay, az, gr, gp, gy, &rollAngleComp, &pitchAngleComp);

        // Rotation:
        rotateAll(rollAngleComp*degreesToRadians, pitchAngleComp*degreesToRadians, ax, ay, az, &ax_rotated, &ay_rotated, &az_rotated);

        // First Order IIR and FIR Implementation:
        iirFilterOutput = FirstOrderIIR_Update(&iirFilt, az_rotated);

        //firFilterOutput = FIRFilter_Update(&firFilt, az_rotated);

        // Zeroing Implementation
        lowThresholdUpdate(&lowThresholdOutput, az_rotated);
        logData(lowThresholdLogFile, lowThresholdOutput);

        // Change this for filter:
        enqueue(&q1, iirFilterOutput);

        if(queue_full(&q1)){
            mean = calculate_mean(&q1);
            std_dev = calculate_std_dev(&q1, mean);

            if(q1.bump_detected){
                std::cout << "Bump Detected at Sample: " << q1.samples_processed <<  " Count: " << q1.bump_counter << std::endl;
                // Add the sample index to the vector
                verticalLineIndices.push_back(q1.samples_processed);
                logBump(bumpCountCircularBufferLogFile, &q1);
                q1.bump_detected = false;
            }

            logData(meanLogFile, mean);
            logData(standartDeviationLogFile, std_dev);

        }

        enqueue(&q2, lowThresholdOutput);

        if(queue_full(&q2)){
            if(q2.bump_detected){
                std::cout << "Bump Detected at Sample: " << q2.samples_processed <<  " Count: " << q2.bump_counter << std::endl;
                logBump(bumpCountLowThresholdLogFile, &q2);
                q2.bump_detected = false;
            }
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

        // -------------------------------------------------------------------------------------------------------------------------
        
        
        /*
        if(iirFilterOutput >= 1.4f){
            bumpCounterNaive++;
            // std::cout << "Bump Count for Naive Case: " << bumpCounterNaive << "\n";
            logData(bumpCountNaiveLogFile, bumpCounterNaive);
        }

        if(iirFilterOutput*iirFilterOutput >= 1.8f){
            bumpCounterNaiveSquared++;
            // std::cout << "Bump Count for Naive Squared Case: " << bumpCounterNaiveSquared << "\n";
            logData(bumpCountNaiveSquaredLogFile, bumpCounterNaiveSquared);
        }
        */
        
        
        
        
        
        // -------------------------------------------------------------------------------------------------------------------------


        
        
        /*
        std::vector<std::ofstream*> logFiles = {&axLogFile, &ayLogFile, &azLogFile, &grLogFile, &gpLogFile, &gyLogFile};
        std::vector<float> data = {ax, ay, az, gr, gp, gy}; 

        for (size_t i = 0; i < logFiles.size(); ++i) {
            logData(*logFiles[i], data[i]);
        }
        */

        // -------------------------------------------------------------------------------------------------------------------------

        
        
        logData(rotatedAzLogFile, az_rotated);
        logData(iirFilterLogFile, iirFilterOutput); // az_rotated is filtered with IIR 
        
        // logData(firFilterLogFile, firFilterOutput); // az_rotated is filtered with FIR

        logAngles(anglesLogFile, rollAngleComp, pitchAngleComp);
        logData(sensorLogFile, ax, ay, az, gr, gp, gy);


        /*
        axMovingAvg.updateAndLogMovingAverage(axFilteredLogFile, ax);
        ayMovingAvg.updateAndLogMovingAverage(ayFilteredLogFile, ay);
        azMovingAvg.updateAndLogMovingAverage(azFilteredLogFile, az);
        rotatedAzMovingAvg.updateAndLogMovingAverage(rotatedAzFilteredLogFile, az_rotated); 
        */
               
       
       
        // Check the button state
        unsigned int state = digitalRead(buttonPin) == LOW ? 1 : 0;
        // Log the button state
        logUser(userInputLogFile, state);

        std::string logOut = "ax="+std::to_string(ax)+",ay="+std::to_string(ay)+",az="+std::to_string(az)+",gr="+std::to_string(gr)+\
                            ",gp="+std::to_string(gp)+",gy="+std::to_string(gy)+",roll_angle="+std::to_string(rollAngleComp)+",pitch_angle="+\
                            std::to_string(pitchAngleComp)+"ax_rotated="+std::to_string(ax_rotated)+"ay_rotated="+std::to_string(ay_rotated)+\
        "az_rotated="+std::to_string(az_rotated);

        // TO BE CONTINUED

        TLogger::TLogInfo(logOut);



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


