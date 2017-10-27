#include "ros/ros.h"
#include "wiringPi/wiringPi.h"
#include "yeti_snowplow/gpio_inputs.h"

/**
 * RASPBERRY PI 3B PINS
 * 
 *  Pin values for each given component controlled using GPIO.
 * 
 * https://www.raspberrypi.org/documentation/usage/gpio-plus-and-raspi2/
 */
#define ESTOP 2;
#define BUTTON1 3;
#define SWITCH1 4;

#define LED1 14; // meant for drive mode, can change name later

ros::Publisher gpioPub;
ros::Subscriber gpioSub;

yeti_snowplow::gpio_inputs;
bool estop;
bool button1;
bool switch1;

yeti_snowplow::gpio_leds;
bool led1;

void initPins() {
    pinMode(ESTOP, INPUT);
    pinMode(BUTTON1, INPUT);
    pinMode(SWITCH1, INPUT);

    pinMode(LED1, OUTPUT);
}

void updateState() {
    estop = digitalRead(ESTOP) == 0 ? false : true;
    button1 = digitalRead(BUTTON1) == 0 ? false : true;
    switch1 = digitalRead(SWITCH1) == 0 ? false : true;
    gpio_inputs.estop = estop;
    gpio_inputs.button1 = button1;
    gpio_inputs.switch1 = switch1;
    gpioPub.publish(gpio_inputs);
}

void updateLeds() {
    if(led1) {
        digitalWrite(LED1, 1);
    }
}

void ledCallback(const yeti_snowplow::gpio_leds &leds) {
    led1 = leds->led1;
    updateLeds();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gpio_monitor");
    ros::NodeHandle n;
    wiringPiSetup(); // Enables WiringPi for GPIO control

    initPins();
    updateState();

    gpioSub = n.subscribe("gpio/leds", 5, ledCallback);
    gpioPub = n.advertise<yeti_snowplow::gpio_inputs>("gpio/inputs", 5);

    ros::spin();
    return 0;
}
