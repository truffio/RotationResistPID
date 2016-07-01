/* This demo shows how the Zumo can use its gyroscope to detect
when it is being rotated, and use the motors to resist that
rotation.

This code was tested on a Zumo with 4 NiMH batteries and two 75:1
HP micro metal gearmotors.  If you have different batteries or
motors, you might need to adjust the PID constants.

Be careful to not move the robot for a few seconds after starting
it while the gyro is being calibrated.  During the gyro
calibration, the yellow LED is on and the words "Gyro cal" are
displayed on the LCD.

After the gyro calibration is done, press button A to start the
demo.  If you try to turn the Zumo, or put it on a surface that
is turning, it will drive its motors to counteract the turning.

This demo only uses the Z axis of the gyro, so it is possible to
pick up the Zumo, rotate it about its X and Y axes, and then put
it down facing in a new position. */

#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const int16_t maxSpeed = 400;
const float   kp = 56;
const float   ki = 0.;
const float   kd = 1/20;

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
L3G gyro;

int x = 0;
  
void setup()
{
  turnSensorSetup();
  delay(500);
  turnSensorReset();

  lcd.clear();
  lcd.print(F("Try to"));
  lcd.gotoXY(0, 1);
  lcd.print(F("turn me!"));
  Serial.begin(9600);
  Serial.println("------Starting------");
  delay(2000);
  lcd.clear();
}

void loop()
{

  
  // Read the gyro to update turnAngle, the estimation of how far
  // the robot has turned, and turnRate, the estimation of how
  // fast it is turning.
  turnSensorUpdate();

  // Calculate the motor turn speed using proportional and
  // derivative PID terms.  Here we are a using a proportional
  // constant of 56 and a derivative constant of 1/20.

  // This is thte original line
  int32_t turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 56) - turnRate / 20;

  //int32_t tA = (int32_t)turnAngle;
  //int32_t turnSpeed = -tA * 56 / (turnAngle1);// - turnRate / 20;
  //int32_t turnSpeed = -(int32_t)tA / (turnAngle1 / 56) - turnRate / 20;
  //  int32_t turnSpeed = -(int32_t)turnAngle * 56 / (turnAngle1) - (int16_t)(kd * turnRate);
    
  //int32_t turnSpeed = -(int32_t)turnAngle/turnAngle1 * kp - turnRate* kd;

  if(x > 0 && x % 10 == 0){

  /*
    //Serial.print("turnAngle=");
    Serial.print((int32_t)turnAngle/turnAngle1);
    //Serial.print("  turnRate=");
    Serial.print(",");
    Serial.print(turnRate);
    //Serial.print("  turnSpeed=");
    Serial.print(",");
    Serial.println(turnSpeed);
    */
    /*Serial.print(turnAngle);
    Serial.print(",");
    Serial.print((int32_t)turnAngle);
    Serial.print(",");
    Serial.print(tA);
    Serial.print(",");
    Serial.print(tA*56/turnAngle1);
    Serial.print(",");
    Serial.println(tA/(turnAngle1/56));
    */
    /*Serial.print(",");
    Serial.print(turnAngle);
    Serial.print(",");
    Serial.print(turnAngle);
    Serial.print(",");
    Serial.print(turnAngle);
    Serial.print(",");
    Serial.print(turnAngle);
    Serial.print(",");
    Serial.print(turnAngle);
    Serial.print(",");
    Serial.print(turnAngle);
    Serial.print(",");
    Serial.print(turnAngle);
    Serial.print(",");*/
    
    x = 0;
  }
  x++;


  // Constrain our motor speeds to be between
  // -maxSpeed and maxSpeed.
  turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);
  
  lcd.gotoXY(0, 0);
  lcd.print((int32_t)turnSpeed);
  lcd.print(F("   "));

  motors.setSpeeds(-turnSpeed, turnSpeed);
}
