#include "button.h"
#include "rgbled.h"
// #include "sensitivity.h"

void Button::checkButton() {
        unsigned int buttonState = digitalRead(pin) == LOW ? 1 : 0;

        // If the button is pressed and was not pressed in the previous iteration
        if (buttonState && !pressed) {
            pressed = true;
            pressTime = std::chrono::high_resolution_clock::now(); // Record the time when the button is pressed
        }
        // If the button is not pressed and was pressed in the previous iteration
        else if (!buttonState && pressed) {
            pressed = false;
            auto releaseTime = std::chrono::high_resolution_clock::now();
            auto pressDuration = std::chrono::duration_cast<std::chrono::milliseconds>(releaseTime - pressTime).count();
            std::cout << "Button on pin " << pin << " was pressed for: " << pressDuration << " ms" << std::endl;
            
        }
}

std::string Button::getButtonStateStr() {
        return (digitalRead(pin) == LOW ? "1" : "0");
}


int Button::getButtonState() {
        return (digitalRead(pin) == LOW ? 1 : 0);
}