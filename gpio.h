#ifndef GPIO_H
#define GPIO_H

#include <wiringPi.h>


void blink_led(int ledPin, int blinkCount, int delayTime);


#endif // GPIO_H