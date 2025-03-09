#pragma once

#include <Arduino.h>
#include "config.h"
#include "TMCStepper.h"

// UART communication for TMC drivers
#define TMCUART Serial2

/**
 * @brief Motor Configuration Parameters
 * These values can be modified through setter functions
 */
// Motor Theta (Rotation) Parameters
extern uint16_t theta_steps_per_rev;        // Full steps per revolution (200 for 1.8° motor)
extern uint16_t theta_microsteps;           // Microstepping setting (16, 32, etc.)
extern float theta_gear_ratio;              // Output gear / Motor gear (e.g., 60/16)
extern uint16_t theta_run_current;          // Running current in mA
extern float theta_hold_current_factor;      // Current reduction when holding (0.0-1.0)
extern float theta_max_speed_deg_sec;       // Maximum speed in degrees/second
extern float theta_home_speed_deg_sec;      // Homing speed in degrees/second
extern float theta_run_speed_deg_sec;       // Running speed in degrees/second
extern uint16_t theta_stallguard_threshold; // StallGuard threshold for homing
extern uint16_t theta_home_current;         // Current during homing in mA
extern volatile bool theta_is_homed;        // Homing status flag
extern volatile bool theta_stall_detected;   // Stall detection flag
extern volatile int32_t theta_current_pos_steps; // Current position in steps
extern volatile float theta_current_pos_deg;    // Current position in degrees
extern volatile bool theta_home_mode;

// Motor Phi (Elevation) Parameters
extern uint16_t phi_steps_per_rev;
extern uint16_t phi_microsteps;
extern float phi_gear_ratio;
extern uint16_t phi_run_current;
extern float phi_hold_current_factor;
extern float phi_max_speed_deg_sec;
extern float phi_home_speed_deg_sec;
extern uint16_t phi_stallguard_threshold;
extern uint16_t phi_home_current;
extern volatile bool phi_is_homed;
extern volatile bool phi_stall_detected;
extern volatile int32_t phi_current_pos_steps;
extern volatile float phi_current_pos_deg;
extern volatile bool phi_home_mode;

// TMC2209 Driver instances
extern TMC2209Stepper driver_theta;
extern TMC2209Stepper driver_phi;

/**
 * @brief Convert degrees to steps for a given motor
 * @param degrees Angle in degrees
 * @param steps_per_rev Motor's full steps per revolution
 * @param microsteps Microstepping setting
 * @param gear_ratio Gear reduction ratio
 * @return Number of steps needed
 */
int32_t degrees_to_steps(float degrees, uint16_t steps_per_rev, 
                        uint16_t microsteps, float gear_ratio);

/**
 * @brief Convert steps to degrees for a given motor
 * @param steps Number of steps
 * @param steps_per_rev Motor's full steps per revolution
 * @param microsteps Microstepping setting
 * @param gear_ratio Gear reduction ratio
 * @return Angle in degrees
 */
float steps_to_degrees(int32_t steps, uint16_t steps_per_rev,
                      uint16_t microsteps, float gear_ratio);

/**
 * @brief Convert degrees per second to steps per second
 * @param deg_per_sec Speed in degrees per second
 * @param steps_per_rev Motor's full steps per revolution
 * @param microsteps Microstepping setting
 * @param gear_ratio Gear reduction ratio
 * @return Speed in steps per second
 */
int32_t speed_degrees_to_steps(float deg_per_sec, uint16_t steps_per_rev, 
                              uint16_t microsteps, float gear_ratio);

// Position tracking functions for Theta motor
float theta_get_current_angle(void);
void theta_set_current_angle(float degrees);
void theta_update_position(int32_t steps);
void theta_reset_position(void);
void theta_calibrate_position(float known_angle);

// Position tracking functions for Phi motor
float phi_get_current_angle(void);
void phi_set_current_angle(float degrees);
void phi_update_position(int32_t steps);
void phi_reset_position(void);
void phi_calibrate_position(float known_angle);

// Theta (Rotation) Motor Functions
bool theta_init(uint8_t enable_pin, uint8_t dir_pin, uint8_t step_pin, uint8_t diag_pin);
void theta_move_to_angle(float degrees);
void theta_move_relative(float degrees);
void theta_set_speed(float deg_per_sec);
void theta_home(int direction);
void theta_emergency_stop(void);
void theta_enable(void);
void theta_disable(void);
void theta_set_current(uint16_t current_ma, float hold_factor);
void theta_take_step(int steps, int steps_delay);

// Theta motor parameter setters
void theta_set_microsteps(uint16_t microsteps);
void theta_set_gear_ratio(float ratio);
void theta_set_stallguard(uint16_t threshold);
void theta_set_home_current(uint16_t current_ma);
void theta_set_max_speed(float deg_per_sec);

// Phi (Elevation) Motor Functions
bool phi_init(uint8_t enable_pin, uint8_t dir_pin, uint8_t step_pin, uint8_t diag_pin);
void phi_move_to_angle(float degrees);
void phi_move_relative(float degrees);
void phi_set_speed(float deg_per_sec);
void phi_home(int direction);
void phi_emergency_stop(void);
void phi_enable(void);
void phi_disable(void);
void phi_set_current(uint16_t current_ma, float hold_factor);

// Phi motor parameter setters
void phi_set_microsteps(uint16_t microsteps);
void phi_set_gear_ratio(float ratio);
void phi_set_stallguard(uint16_t threshold);
void phi_set_home_current(uint16_t current_ma);
void phi_set_max_speed(float deg_per_sec);

// Interrupt Service Routines
void IRAM_ATTR theta_stall_isr(void);
void IRAM_ATTR phi_stall_isr(void);

// Position calculation helpers
inline float normalize_angle(float angle) {
    // Normalize angle to 0-360 range
    angle = fmod(angle, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle;
}

// Default motor configurations
void set_theta_defaults(void) {
    theta_steps_per_rev = 200;          // 1.8° step angle
    theta_microsteps = 16;              // 16 microsteps
    theta_gear_ratio = 1.0f;     // 60T:16T gear ratio
    theta_run_current = 800;            // 800mA running current
    theta_hold_current_factor = 0.3f;    // 30% current when holding
    theta_max_speed_deg_sec = 360.0f;   // 360°/s maximum speed
    theta_home_speed_deg_sec = 90.0f;   // 90°/s homing speed
    theta_stallguard_threshold = 25;     // StallGuard threshold
    theta_home_current = 250;           // 250mA during homing
    theta_current_pos_steps = 0;        // Reset position
    theta_current_pos_deg = 0.0f;       // Reset angle
    phi_home_mode = false;

}

void set_phi_defaults(void) {
    phi_steps_per_rev = 200;           // 1.8° step angle
    phi_microsteps = 16;               // 16 microsteps
    phi_gear_ratio = 60.0f/16.0f;      // 60T:16T gear ratio
    phi_run_current = 800;             // 800mA running current
    phi_hold_current_factor = 0.3f;     // 30% current when holding
    phi_max_speed_deg_sec = 180.0f;    // 180°/s maximum speed
    phi_home_speed_deg_sec = 45.0f;    // 45°/s homing speed
    phi_stallguard_threshold = 25;      // StallGuard threshold
    phi_home_current = 250;            // 250mA during homing
    phi_current_pos_steps = 0;         // Reset position
    phi_current_pos_deg = 0.0f;        // Reset angle
    phi_home_mode = false;
}

// Utility functions for step calculations
inline int32_t calculate_steps_per_rev(uint16_t steps_per_rev, 
                                     uint16_t microsteps, 
                                     float gear_ratio) {
    return static_cast<int32_t>(steps_per_rev * microsteps * gear_ratio);
}