#include "motor.h"

// Global variables initialization for Theta motor
uint16_t theta_steps_per_rev = 200;
uint16_t theta_microsteps = 16;
float theta_gear_ratio = 1.0f;
uint16_t theta_run_current = 800;
float theta_hold_current_factor = 0.3f;
float theta_max_speed_deg_sec = 360.0f;
float theta_run_speed_deg_sec = 90.0f;
float theta_home_speed_deg_sec = 90.0f;
uint16_t theta_stallguard_threshold = 25;
uint16_t theta_home_current = 250;
volatile bool theta_is_homed = false;
volatile bool theta_stall_detected = false;
volatile int32_t theta_current_pos_steps = 0;
volatile float theta_current_pos_deg = 0.0f;
volatile bool theta_home_mode = false;

// GPIO pins for Theta motor
uint8_t theta_enable_pin;
uint8_t theta_dir_pin;
uint8_t theta_step_pin;
uint8_t theta_diag_pin;

// Global variables initialization for Phi motor
uint16_t phi_steps_per_rev = 200;
uint16_t phi_microsteps = 16;
float phi_gear_ratio = 60.0f / 16.0f;
uint16_t phi_run_current = 800;
float phi_hold_current_factor = 0.3f;
float phi_max_speed_deg_sec = 180.0f;
float phi_home_speed_deg_sec = 45.0f;
uint16_t phi_stallguard_threshold = 25;
uint16_t phi_home_current = 250;
volatile bool phi_is_homed = false;
volatile bool phi_stall_detected = false;
volatile int32_t phi_current_pos_steps = 0;
volatile float phi_current_pos_deg = 0.0f;
volatile bool phi_home_mode = false;

// GPIO pins for Phi motor
uint8_t phi_enable_pin;
uint8_t phi_dir_pin;
uint8_t phi_step_pin;
uint8_t phi_diag_pin;

// TMC2209 driver instances
TMC2209Stepper driver_theta(&TMCUART, R_SENSE, DRIVER_ADDRESS_THETA);
TMC2209Stepper driver_phi(&TMCUART, R_SENSE, DRIVER_ADDRESS_R);




// Utility Functions Implementation
int32_t degrees_to_steps(float degrees, uint16_t steps_per_rev,
                         uint16_t microsteps, float gear_ratio)
{
    float steps_per_rev_total = steps_per_rev * microsteps * gear_ratio;
    return static_cast<int32_t>((degrees * steps_per_rev_total) / 360.0f);
}

float steps_to_degrees(int32_t steps, uint16_t steps_per_rev,
                       uint16_t microsteps, float gear_ratio)
{
    float steps_per_rev_total = steps_per_rev * microsteps * gear_ratio;
    return (steps * 360.0f) / steps_per_rev_total;
}



int32_t speed_degrees_to_steps(float deg_per_sec, uint16_t steps_per_rev,
                               uint16_t microsteps, float gear_ratio)
{
    float steps_per_rev_total = steps_per_rev * microsteps * gear_ratio;
    return static_cast<int32_t>((deg_per_sec * steps_per_rev_total) / 360.0f);
}

// Theta Motor Functions Implementation
bool theta_init(uint8_t enable_pin, uint8_t dir_pin, uint8_t step_pin, uint8_t diag_pin)
{
    // Save pin assignments
    theta_enable_pin = enable_pin;
    theta_dir_pin = dir_pin;
    theta_step_pin = step_pin;
    theta_diag_pin = diag_pin;

    // Configure pins
    pinMode(enable_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);
    pinMode(diag_pin, INPUT);

    // Initialize TMC driver
    driver_theta.begin();
    int version = driver_theta.version();
    if (version != 0)
    {
        Serial.printf("TMC2209 Version: %d\n", version);
    }
    else
    {
        Serial.println("TMC2209 not found!");
        return false;
    }

    driver_theta.en_spreadCycle(false);
    driver_theta.I_scale_analog(false);
    driver_theta.internal_Rsense(false);
    driver_theta.microsteps(theta_microsteps);
    driver_theta.SGTHRS(theta_stallguard_threshold);
    driver_theta.rms_current(theta_run_current, theta_run_current);
    driver_theta.TCOOLTHRS(0xFFFFF);

    // Attach interrupt for stall detection
    attachInterrupt(digitalPinToInterrupt(diag_pin), theta_stall_isr, RISING);

    // Enable motor
    theta_enable();
    return true;
}

void theta_move_to_angle(float degrees)
{
    // Normalize target angle
    degrees = normalize_angle(degrees);

    // Calculate shortest path
    float current_angle = theta_get_current_angle();
    float delta = degrees - current_angle;
    if (delta > 180.0f)
        delta -= 360.0f;
    else if (delta < -180.0f)
        delta += 360.0f;

    // Calculate steps needed
    int32_t steps = degrees_to_steps(delta, theta_steps_per_rev,
                                     theta_microsteps, theta_gear_ratio);

    // Set direction

    // Convert speed from deg/s to steps/s
    int32_t speed_steps = speed_degrees_to_steps(theta_run_speed_deg_sec,
                                                 theta_steps_per_rev,
                                                 theta_microsteps,
                                                 theta_gear_ratio);

    // TODO: Implement step generation with acceleration
    // This is where you'd implement your stepping algorithm
    // Remember to update position after each step

    int32_t steps_delay = 1000000 / speed_steps;
    theta_take_step(steps, steps_delay);
}

void theta_take_step(int steps, int steps_delay)
{
    digitalWrite(theta_dir_pin, steps >= 0 ? HIGH : LOW);
    for (int i = 0; i < abs(steps); i++)
    {
        digitalWrite(theta_step_pin, HIGH);
        delayMicroseconds(1);
        digitalWrite(theta_step_pin, LOW);
        delayMicroseconds(steps_delay);
        theta_update_position(steps > 0 ? 1 : -1);
        if (theta_stall_detected && theta_home_mode)
        {
            Serial.println("Stall detected!");
            theta_stall_detected = false;
            break;
        }
    }
}

float theta_get_current_angle(void)
{
    return theta_current_pos_deg;
}

void theta_update_position(int32_t steps)
{
    theta_current_pos_steps += steps;
    theta_current_pos_deg = normalize_angle(steps_to_degrees(
        theta_current_pos_steps,
        theta_steps_per_rev,
        theta_microsteps,
        theta_gear_ratio));
}


// Homing function implementation
void theta_home(int direction)
{
    // Set homing current and speed
    driver_theta.rms_current(theta_home_current);
    int32_t home_speed_steps = speed_degrees_to_steps(theta_home_speed_deg_sec,
                                                      theta_steps_per_rev,
                                                      theta_microsteps,
                                                      theta_gear_ratio);

    // Enable StallGuard
    driver_theta.TCOOLTHRS(0xFFFFF);
    driver_theta.SGTHRS(theta_stallguard_threshold);

    // Set direction
    digitalWrite(theta_dir_pin, direction > 0 ? HIGH : LOW);

    // Reset stall flag
    theta_stall_detected = false;

    theta_home_mode = true;
    theta_take_step(1000, 1000);

    driver_theta.rms_current(theta_run_current);
}

// ISR for stall detection
void IRAM_ATTR theta_stall_isr(void)
{
    theta_stall_detected = true;
}

// Phi Motor Functions Implementation
// TODO: Implement similar functions for phi motor
// Copy and modify theta functions for phi motor

// Emergency stop implementation
void theta_emergency_stop(void)
{
    // TODO: Implement emergency stop
    // Consider:
    // - Immediate step pulse stop
    // - Current reduction
    // - Position tracking update
}

void theta_enable(void)
{
    digitalWrite(theta_enable_pin, LOW); // Assuming active LOW
}

void theta_disable(void)
{
    digitalWrite(theta_enable_pin, HIGH); // Assuming active LOW
}

// Add remaining function implementations as needed