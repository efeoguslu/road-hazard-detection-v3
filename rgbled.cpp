#include "rgbled.h"

RgbLed::RgbLed(int r, int g, int b) : redPin(r), greenPin(g), bluePin(b), lastEventTime(std::chrono::steady_clock::now()), currentEvent(Event::None), blinkCount(0), blinkState(0) {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    RgbLed::resetColor();
}

void RgbLed::setColor(int red, int green, int blue) {
    digitalWrite(redPin, red ? HIGH : LOW);
    digitalWrite(greenPin, green ? HIGH : LOW);
    digitalWrite(bluePin, blue ? HIGH : LOW);
}

void RgbLed::setColor(Color color) {
    switch(color) {
        case Color::Red:
            RgbLed::setColor(1, 0, 0);
            break;
        case Color::Green:
            RgbLed::setColor(0, 1, 0);
            break;
        case Color::Blue:
            RgbLed::setColor(0, 0, 1);
            break;
    }
}

void RgbLed::resetColor() {
    // Turn off all colors
    RgbLed::setColor(0, 0, 0);
}

void RgbLed::bumpDetected() {
    currentEvent = Event::Bump;
    lastEventTime = std::chrono::steady_clock::now();
}

void RgbLed::potholeDetected() {
    currentEvent = Event::Pit;
    lastEventTime = std::chrono::steady_clock::now();
}

void RgbLed::configurationChanged(int blinks) {
    currentEvent = Event::ConfigChange;
    configNumber = blinks;
    blinkCount = 0;
    blinkState = 0;
    lastEventTime = std::chrono::steady_clock::now();
}

void RgbLed::update() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastEventTime).count();

    switch (currentEvent){

        case Event::Bump:
            if (elapsed <= hazardChangeLedDelayMs) {
                RgbLed::setColor(1, 1, 0); // Yellow
            } else {
                RgbLed::resetColor();
                currentEvent = Event::None;
            }
            break;
        
        case Event::Pit:
            if (elapsed <= hazardChangeLedDelayMs) {
                RgbLed::setColor(1, 0, 0); // Red
            } else {
                RgbLed::resetColor();
                currentEvent = Event::None;
            }
            break;
        
        case Event::ConfigChange:
            if (elapsed >= configChangeLedDelayMs) { // 400 ms on/off cycle
                if (blinkCount < configNumber) { // Each blink consists of an on and off state
                    blinkState = !blinkState;
                    if (blinkState) {
                        RgbLed::setColor(0, 0, 1); // Blue
                    } else {
                        RgbLed::resetColor();
                        blinkCount++;
                    }
                    lastEventTime = now; // Reset the timer for the next blink
                } else {
                    currentEvent = Event::None;
                    RgbLed::resetColor(); // Ensure the LED is off at the end of the sequence
                }
            }
            break;
        
        default:
            RgbLed::resetColor();
            break;
    }
}