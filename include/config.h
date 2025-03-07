#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
// #include "debug.h"

//Power
#define POWER_BUTTON 39//Power On/Off Button
#define POWER_ENABLE 38// Power Latch Enable
#define POWER_AUTO_OFF_TIMEOUT 15*60*1000
#define LONG_PRESS_TIME 1000

//Output 
#define LASER_PIN 40
#define LASER_POWER 40 //Percentage of power to be given to the laser. 0-100%

#define NEOPIXEL_PIN 7
#define NEOPIXEL_COUNT 10//2 Row of 5 Neopixels


#define BUZZER_PIN 48
#define BUZZER_FREQ 2731//Buzzer Max output frequency

//THETA previously called MOTOR_Y
#define MOTOR_Y_STEP 10//Step Pin
#define MOTOR_Y_DIR 9//Direction Pin
#define MOTOR_Y_ENABLE 21//Enable Pin
#define MOTOR_Y_DIAG 11//StallGard Detection Pin(Homing)


//R previously called MOTOR_X
#define MOTOR_X_STEP 13//Step Pin
#define MOTOR_X_DIR 12//Direction Pin
#define MOTOR_X_ENABLE 47//Enable Pin
#define MOTOR_X_DIAG 14//StallGard Detection Pin(Homing)


#define R_SENSE 0.11f
#define THETA_SENSE 0.11f
#define DRIVER_ADDRESS_R 0b00
#define DRIVER_ADDRESS_THETA 0b01

//PDN UART
#define PDN_TX 15
#define PDN_RX 16

//Lidar 
#define LIDAR_TX 18
#define LIDAR_RX 17

//Charging (BQ25887)
#define BQ_SDA 2
#define BQ_SCL 1
#define BQ_INT 41
#define BQ_PG 42//Power Good Pin


//BLe
#define BLE_NAME_PREFIX "HELIOS"
#define BLE_NAME_LENGTH 20











#endif