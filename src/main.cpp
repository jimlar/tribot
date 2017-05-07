#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;

#define ATTACK_SPEED 400
#define EVADE_REVERSE_SPEED 400
#define EVADE_REVERSE_TIME 300
#define EVADE_SPEED 400
#define EVADE_TIME 300

#define SEARCH_TURN_SPEED_MAX 400
#define SEARCH_TURN_SPEED_MIN 200
#define SEARCH_DECELERATION 10
#define SEARCH_ACCELERATION 10

#define WHITE_THRESHOLD 200
#define PROX_THRESHOLD 3

#define MODE_SEARCH 0
#define MODE_ATTACK 1

int active_mode = MODE_SEARCH;

uint16_t lineSensorValues[3];
uint8_t leftProxValue;
uint8_t rightProxValue;

void log(String msg) {
  Serial.print(millis());
  Serial.print(": ");
  Serial.println(msg);
}

void wait_for_start() {
  lcd.clear();
  lcd.print("Press A");
  buttonA.waitForButton();
  lcd.clear();
  lcd.print("1 sec");
  delay(1000);
}

void printLineReadingsToLCD()
{
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print(lineSensorValues[0]);
  lcd.gotoXY(0, 1);
  lcd.print(lineSensorValues[2]);
}

void printProxReadingsToLCD()
{
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print(leftProxValue);
  lcd.gotoXY(0, 1);
  lcd.print(rightProxValue);
}

void printAllReadingsToLCD()
{
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print(leftProxValue);
  lcd.gotoXY(4, 0);
  lcd.print(lineSensorValues[0]);

  lcd.gotoXY(0, 1);
  lcd.print(rightProxValue);
  lcd.gotoXY(4, 1);
  lcd.print(lineSensorValues[2]);
}

boolean objectSeen(uint8_t threshold) {
  return leftProxValue >= threshold || rightProxValue >= threshold;
}

void search() {
  static int turnSpeed = SEARCH_TURN_SPEED_MAX;

  ledYellow(true);
  boolean objSeen = objectSeen(PROX_THRESHOLD);

  if (objSeen) {
    turnSpeed -= SEARCH_DECELERATION;
  } else {
    turnSpeed += SEARCH_ACCELERATION;
  }
  turnSpeed = constrain(turnSpeed, SEARCH_TURN_SPEED_MIN, SEARCH_TURN_SPEED_MAX);

  if (objSeen) {
    if (leftProxValue < rightProxValue) {
         motors.setSpeeds(-turnSpeed, turnSpeed);
         log("Search - object seen, searching right");

     } else if (leftProxValue > rightProxValue) {
         motors.setSpeeds(-turnSpeed, turnSpeed);
         log("Search - object seen, searching left");
     } else {
         active_mode = MODE_ATTACK;
         log("Search - object seen, switching to attack");
         turnSpeed = SEARCH_TURN_SPEED_MAX;
     }

  } else {
    log("Search - nothing seen");
    motors.setSpeeds(-turnSpeed, turnSpeed);
  }
}

void attack() {
  log("Attack");
  motors.setSpeeds(ATTACK_SPEED, ATTACK_SPEED);
}

void evadeLeft() {
  ledRed(true);
  log("Evade left");
  motors.setSpeeds(-EVADE_REVERSE_SPEED, -EVADE_REVERSE_SPEED);
  delay(EVADE_REVERSE_TIME);
  motors.setSpeeds(-EVADE_SPEED, EVADE_SPEED);
  delay(EVADE_TIME);
}

void evadeRight() {
  ledRed(true);
  log("Evade right");
  motors.setSpeeds(-EVADE_REVERSE_SPEED, -EVADE_REVERSE_SPEED);
  delay(EVADE_REVERSE_TIME);
  motors.setSpeeds(EVADE_SPEED, -EVADE_SPEED);
  delay(EVADE_TIME);
}

void setup()
{
  proxSensors.initFrontSensor();
  lineSensors.initThreeSensors();
  wait_for_start();
}

void readProximity() {
  proxSensors.read();
  leftProxValue = proxSensors.countsFrontWithLeftLeds();
  rightProxValue = proxSensors.countsFrontWithRightLeds();
}

void loop() {
  ledYellow(false);
  ledRed(false);

  boolean lineRight = false;
  boolean lineLeft = false;

  static uint16_t lastSampleTime = 0;

  readProximity();

  if ((uint16_t)(millis() - lastSampleTime) >= 50)
  {
    lastSampleTime = millis();
    lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
    lineLeft = lineSensorValues[0] < WHITE_THRESHOLD;
    lineRight = lineSensorValues[2] < WHITE_THRESHOLD;
    printAllReadingsToLCD();
  }

  if (lineLeft) {
    evadeRight();
    active_mode = MODE_SEARCH;
  } else if (lineRight) {
    evadeLeft();
    active_mode = MODE_SEARCH;
  }

  switch (active_mode) {
    case MODE_ATTACK:
      attack();
      break;
    case MODE_SEARCH:
      search();
      break;
  }
}
