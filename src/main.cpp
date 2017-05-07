/* This example uses the front proximity sensor on the Zumo 32U4
Front Sensor Array to locate an opponent robot or any other
reflective object. Using the motors to turn, it scans its
surroundings. If it senses an object, it turns on its yellow LED
and attempts to face towards that object. */

#include <Wire.h>
#include <Zumo32U4.h>


Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4Buzzer buzzer;

// A sensors reading must be greater than or equal to this
// threshold in order for the program to consider that sensor as
// seeing an object.
const uint8_t sensorThreshold = 4;

// The maximum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t forwardSpeed = 400       ;

// The maximum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t turnSpeedMax = 400;

// The minimum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t turnSpeedMin = 100;

// The amount to decrease the motor speed by during each cycle
// when an object is seen.
const uint16_t deceleration = 10;

// The amount to increase the speed by during each cycle when an
// object is not seen.
const uint16_t acceleration = 10;


/* BEGIN LINE SENSOR STUFF */
#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];
// These might need to be tuned for different motor types.
#define QTR_THRESHOLD     1000  // microseconds
#define REVERSE_SPEED     400  // 0 is stopped, 400 is full speed
#define TURN_SPEED        400
#define FORWARD_SPEED     400
#define REVERSE_DURATION  300  // ms
#define TURN_DURATION     400  // ms
bool useEmitters = true;
/* END LINE SENSOR STUFF */


#define LEFT 0
#define RIGHT 1

// Stores the last indication from the sensors about what
// direction to turn to face the object.  When no object is seen,
// this variable helps us make a good guess about which direction
// to turn.
bool senseDir = RIGHT;

// True if the robot is turning left (counter-clockwise).
bool turningLeft = false;

// True if the robot is turning right (clockwise).
bool turningRight = false;

// True if the robot is going forward.
bool running = false;

// If the robot is turning, this is the speed it will use.
uint16_t turnSpeed = turnSpeedMax;

// The time, in milliseconds, when an object was last seen.
uint16_t lastTimeObjectSeen = 0;

void setup()
{
  Serial.begin(115200);
  proxSensors.initFrontSensor();
  lineSensors.initThreeSensors();

  // Wait for the user to press A before driving the motors.
  lcd.clear();
  lcd.print(F("Press A"));
  buttonA.waitForButton();
  lcd.clear();
  for (int i = 0; i < 3; i++)
  {
    buzzer.playNote(NOTE_G(3), 200, 15);
    delay(1000);
  }
  buzzer.playNote(NOTE_G(4), 400, 15);
}

void turnRight()
{
  motors.setSpeeds(turnSpeed, -turnSpeed);
  turningLeft = false;
  turningRight = true;
  running = false;
}

void turnLeft()
{
  motors.setSpeeds(-turnSpeed, turnSpeed);
  turningLeft = true;
  turningRight = false;
  running = false;
}

void stop()
{
  motors.setSpeeds(0, 0);
  turningLeft = false;
  turningRight = false;
  running = false;
}

void run_forest()
{
  motors.setSpeeds(forwardSpeed, forwardSpeed);
  turningLeft = false;
  turningRight = false;
  running = true;
}

void loop()
{
  // Read the front proximity sensor and gets its left value (the
  // amount of reflectance detected while using the left LEDs)
  // and right value.
  proxSensors.read();
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue  = proxSensors.countsFrontWithRightLeds();

  // Determine if an object is visible or not.
  // bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;



  if (lineSensorValues[0] < QTR_THRESHOLD)
  {
    // If leftmost sensor detects line, reverse and turn to the
    // right.
    lcd.clear();
    lcd.print(F("LL"));
    // buttonA.waitForButton();
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (lineSensorValues[NUM_SENSORS - 1] < QTR_THRESHOLD)
  {
    // If rightmost sensor detects line, reverse and turn to the
    // left.
    lcd.clear();
    lcd.print(F("LR"));
    // buttonA.waitForButton();
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;
  if (objectSeen)
  {
    // An object is visible, so we will start decelerating in
    // order to help the robot find the object without
    // overshooting or oscillating.
    lcd.clear();
    lcd.print(F("Obj visible"));
    turnSpeed -= deceleration;
    turnSpeed = constrain(turnSpeed, turnSpeedMin, turnSpeedMax);
    ledYellow(1);

    if (abs(leftValue - rightValue) < 2) {
      run_forest();
    }
    else if (leftValue < rightValue)
    {
      // The right value is greater, so the object is probably
      // closer to the robot's right LEDs, which means the robot
      // is not facing it directly.  Turn to the right to try to
      // make it more even.
      turnRight();
      senseDir = RIGHT;
    }
    else if (leftValue > rightValue)
    {
      // The left value is greater, so turn to the left.
      turnLeft();
      senseDir = LEFT;
    }
    else
    {
      // The values are equal, so stop the motors.
      run_forest();
    }
  }
  else {

    // An object is not visible, so we will accelerate in order
    // to help find the object sooner.
    turnSpeed += acceleration;
    lcd.clear();
    lcd.print(F("Blind"));
    ledYellow(0);

    if (senseDir == RIGHT)
    {
      turnRight();
    }
    else
    {
      turnLeft();
    }
  }
}
