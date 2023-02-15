/**
 * @author Pontus Staaf
 * 
 * Justification: 
 * This file contains pin declerations for the accelerometer and barometer intended
 * to be used on Elpis 1
 */

// Include Guard
#ifndef PINS_H_INC
#define PINS_H_INC


enum pins_IO{

    // IO PINS FOR ACCELEROMETER MPU6050
    ACC_IN  = 1,    //GPIO1
    ACC_OUT = 3,    //GPIO3

    // IO PINS FOR BAROMETER BMP280
    BAR_IN  = 0,    //GPIO0
    BAR_OUT = 2,    //GPIO2
};


#endif
