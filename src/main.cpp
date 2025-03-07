#include <Arduino.h>
#include "config.h"
#include "TMCStepper.h"

#define TMCUART Serial2
#define REPORT_DELAY 4

TMC2209Stepper driverX(&TMCUART, R_SENSE, DRIVER_ADDRESS_R);
TMC2209Stepper driverY(&TMCUART, R_SENSE, DRIVER_ADDRESS_THETA);
int motor_speedX = 500;
int microstepsX = 16;
int steps_per_revX = 200 * microstepsX;
int stallguardthreshX = 25;
int rms_currentX = 250;
float rms_holdX = 0.4;
bool home_modeX = false;
bool is_homedX = false;



int motor_speedY = 500;
int microstepsY= 16;
int steps_per_revY = 200 * microstepsY;
int stallguardthreshY = 25;
int rms_currentY = 250;
float rms_holdY = 0.4;
bool home_modeY = false;
bool is_homedY = false;

void moveMotorX(int dir, int steps, int speed);
void motorGoX(float deg);
void moveMotorY(int dir, int steps, int speed);
void motorGoY(float deg);

volatile bool stallflagX = false;
volatile bool stallflagY = false;

void IRAM_ATTR stallISRX()
{
  stallflagX = true;
}

void IRAM_ATTR stallISRY()
{
  stallflagY = true;
}

void initMotorsPins()
{
  pinMode(MOTOR_X_ENABLE, OUTPUT);
  pinMode(MOTOR_X_DIR, OUTPUT);
  pinMode(MOTOR_X_STEP, OUTPUT);
  pinMode(MOTOR_X_DIAG, INPUT);
  digitalWrite(MOTOR_X_ENABLE, LOW);
  pinMode(MOTOR_Y_ENABLE, OUTPUT);
  pinMode(MOTOR_Y_DIR, OUTPUT);
  pinMode(MOTOR_Y_STEP, OUTPUT);
  pinMode(MOTOR_Y_DIAG, INPUT);
  digitalWrite(MOTOR_Y_ENABLE, LOW);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, HIGH);
}

void setup()
{
  Serial.begin(115200);
  initMotorsPins();
  TMCUART.begin(115200, SERIAL_8N1, PDN_RX, PDN_TX);
delay(100);
  driverX.begin();
delay(100);
  // driverY.begin();
  delay(100);

  uint8_t versionX = driverX.version();
  uint8_t versionY = driverY.version();

  Serial.printf("X Driver Version: %d\n", versionX);
  Serial.printf("Y Driver Version: %d\n", versionY);

  driverX.pdn_disable(true);
  driverX.en_spreadCycle(false);
  driverX.I_scale_analog(false);
  driverX.internal_Rsense(false);
  driverX.microsteps(microstepsX);
  driverX.SGTHRS(stallguardthreshX);
  driverX.rms_current(rms_currentX, rms_holdX);
  driverX.TCOOLTHRS(0xFFFFF);


  driverX.pdn_disable(true);
  driverX.en_spreadCycle(false);
  driverX.I_scale_analog(false);
  driverX.internal_Rsense(false);
  driverX.microsteps(microstepsX);
  driverX.SGTHRS(stallguardthreshX);
  driverX.rms_current(rms_currentX, rms_holdX);
  driverX.TCOOLTHRS(0xFFFFF);


  attachInterrupt(MOTOR_X_DIAG, stallISRX, FALLING);
  attachInterrupt(MOTOR_Y_DIAG, stallISRY, FALLING);

  driverX.enn();
  driverY.enn();
}

void loop()
{
  static bool printflag = false;
  if (Serial.available())
  {
    char c = Serial.read();

    if (c == 'e')
    {
      digitalWrite(MOTOR_X_ENABLE, !digitalRead(MOTOR_X_ENABLE));

    }
    else if (c == 'E')
    {
      digitalWrite(MOTOR_Y_ENABLE, !digitalRead(MOTOR_Y_ENABLE));
    }
    else if (c == 'x')
    {
      float deg = Serial.parseFloat();
      motorGoX(deg);
      Serial.printf("Moving %f degrees\n", deg);
    }
    else if (c == 'y')
    {
      float deg = Serial.parseFloat();
      motorGoY(deg);
      Serial.printf("Moving %f degrees\n", deg);
    }
    else if (c == 's')
    {
      int speed = Serial.parseInt();
      motor_speedX = speed;
      Serial.printf("Setting speed to %d\n", speed);
    }
    else if (c == 'S')
    {
      int speed = Serial.parseInt();
      motor_speedY = speed;
      Serial.printf("Setting speed to %d\n", speed);
    }
    else if (c == 'm')
    {

      microstepsX = Serial.parseInt();
      driverX.microsteps(microstepsX);
      Serial.printf("Setting microsteps to %d\n", microstepsX);
    }
    else if (c == 'M')
    {
      microstepsY = Serial.parseInt();
      driverY.microsteps(microstepsY);
      Serial.printf("Setting microsteps to %d\n", microstepsY);
    }
    else if (c == 't')
    {
      stallguardthreshX = Serial.parseInt();
      driverX.SGTHRS(stallguardthreshX);
      Serial.printf("Setting stall threshold to %d\n", stallguardthreshX);
    }
    else if (c == 'T')
    {
      stallguardthreshY = Serial.parseInt();
      driverY.SGTHRS(stallguardthreshY);
      Serial.printf("Setting stall threshold to %d\n", stallguardthreshY);
    }

    else if (c == 'r')
    {
      rms_currentX = Serial.parseInt();
      driverX.rms_current(rms_currentX, rms_holdX);
      Serial.printf("Setting current to %d\n", rms_currentX);
    }
    else if (c == 'R')
    {
      rms_currentY = Serial.parseInt();
      driverY.rms_current(rms_currentY, rms_holdY);
      Serial.printf("Setting current to %d\n", rms_currentY);
    }
    else if (c == 'c')
    {
      int tcool = Serial.parseInt();
      driverX.TCOOLTHRS(tcool);
      Serial.printf("Setting cool threshold to %d\n", tcool);
    }
    else if (c == 'C')
    {
      int tcool = Serial.parseInt();
      driverY.TCOOLTHRS(tcool);
      Serial.printf("Setting cool threshold to %d\n", tcool);
    }
    else if (c == 'p')
    {
      printflag = !printflag;
    }
    else if (c == 'd')
    {
      delay(Serial.parseInt());
    }
    else if (c == 'h')
    {
      rms_holdX = Serial.parseFloat();
      driverX.rms_current(rms_currentX, rms_holdX);
    }
    else if (c == 'H')
    {
      rms_holdY = Serial.parseFloat();
      driverY.rms_current(rms_currentY, rms_holdY);
    }
  }

  if (printflag)
  {
  }
}






void motorGoX(float deg)
{
  steps_per_revX = 200 * microstepsX;
  int steps = deg * steps_per_revX / 360;
  steps = abs(steps);
  if (deg < 0)
  {
    moveMotorX(0, steps, motor_speedX);
  }
  else
  {
    moveMotorX(1, steps, motor_speedX);
  }
}

void moveMotorX(int dir, int steps, int speed)
{
  digitalWrite(MOTOR_X_DIR, dir);
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(MOTOR_X_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(MOTOR_X_STEP, LOW);
    delayMicroseconds(speed);
    if (stallflagX)
    {
      Serial.println("Stall Detected");
      stallflagX = false;
      break;
    }
  }
}

void motorGoY(float deg)
{
  steps_per_revY = 200 * microstepsY;
  int steps = deg * steps_per_revY / 360;
  steps = abs(steps);
  if (deg < 0)
  {
    moveMotorY(0, steps, motor_speedY);
  }
  else
  {
    moveMotorY(1, steps, motor_speedY);
  }
}

void moveMotorY(int dir, int steps, int speed)
{
  digitalWrite(MOTOR_Y_DIR, dir);
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(MOTOR_Y_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(MOTOR_Y_STEP, LOW);
    delayMicroseconds(speed);
    if (stallflagY)
    {
      Serial.println("Stall Detected");
      stallflagY = false;
      break;
    }
  }
}