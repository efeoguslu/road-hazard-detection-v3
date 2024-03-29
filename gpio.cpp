#include "gpio.h"

// Global flag used to exit from the main loop
bool RUNNING = true;

// Function to blink an LED a specified number of times
void blink_led(int ledPin, int blinkCount, int delayTime) {
    for (int i = 0; i < blinkCount; i++) {
        digitalWrite(ledPin, HIGH); // Turn on the LED
        delay(delayTime); // Wait for the specified delay time
        digitalWrite(ledPin, LOW); // Turn off the LED
        delay(delayTime); // Wait for the specified delay time
    }
}