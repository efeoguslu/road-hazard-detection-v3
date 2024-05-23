#ifndef BUTTON_H
#define BUTTON_H

#include <iostream>
#include <wiringPi.h>
#include <chrono>
#include <thread>


const int bumpPin{ 19 };
const int potholePin{ 26 };
const int modePin{ 13 };

const int buttonPressDurationThresholdMs{ 1000 };


class Button {
public:
    Button(int pinNumber) : pin(pinNumber), pressed(false), pressTime(std::chrono::high_resolution_clock::now()){ pinMode(pinNumber, INPUT); }
    void checkButton();
    std::string getButtonStateStr();
    int getButtonState();

private:
    int pin;
    bool pressed;
    std::chrono::time_point<std::chrono::high_resolution_clock> pressTime;
};


#endif