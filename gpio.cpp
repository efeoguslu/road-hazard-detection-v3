#include "gpio.h"


// Function to blink the LED a specific number of times
void blinkLED(int ledPin, int blinkCount) {
    const int blinkDurationMs = 200; // Each blink lasts 100 ms
    for (int i = 0; i < blinkCount; ++i) {
        digitalWrite(ledPin, HIGH); // Turn the LED on
        std::this_thread::sleep_for(std::chrono::milliseconds(blinkDurationMs)); // Wait for the blink duration
        digitalWrite(ledPin, LOW); // Turn the LED off
        std::this_thread::sleep_for(std::chrono::milliseconds(blinkDurationMs)); // Wait for the blink duration
    }
}