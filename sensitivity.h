#ifndef SENSITIVITY_H
#define SENSITIVITY_H

#include <string>
#include "rgbled.h"

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

SensitivityConfig lowSensitivityConfig{  "low",  0.25 }; 
SensitivityConfig midSensitivityConfig{  "mid",  0.12 }; 
SensitivityConfig highSensitivityConfig{ "high", 0.07 }; 
SensitivityConfig currentConfig{ "default", 0.12 }; 

SensitivityConfig getNextConfig(const SensitivityConfig& current, RgbLed& led) {
    if (current.getConfigName() == "low") {
        //blinkLED(ledPin, 2);
        //digitalWrite(ledPin, LOW);
        led.configurationChanged(1);
        return midSensitivityConfig;

    } else if (current.getConfigName() == "mid") {
        //blinkLED(ledPin, 3);
        //digitalWrite(ledPin, LOW);
        led.configurationChanged(2);
        return highSensitivityConfig;

    } else if (current.getConfigName() == "high") {
        //blinkLED(ledPin, 1);
        //digitalWrite(ledPin, LOW);
        led.configurationChanged(3);
        return lowSensitivityConfig;

    } else {
        // Default case, return mid as the next config
        return midSensitivityConfig;
    }
}

*/




#endif
