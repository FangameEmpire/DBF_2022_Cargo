#include <Wire.h>
#include <Servo.h>;

Servo servo_01;
Servo servo_02;
Servo servo_03;
Servo servo_04;
Servo servoMotors[] = {servo_01, servo_02, servo_03, servo_04};
const int output[] = {3, 5, 6, 11};
int servoPos[] = {90, 90, 90, 90};

void setup()
{
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output

  for (int i = 0; i < 4; i++) {
    servoMotors[i].attach(output[i]);
  }
}

void loop()
{
  for (int i = 0; i < 4; i++) {
    servoMotors[i].write(servoPos[i]);
    Serial.print(servoPos[i]);
    Serial.print("  ");
    Serial.println(i);
  }
  delay(1);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  int servoVals[4];
  servoVals[0] = Wire.read();
  servoVals[1] = Wire.read();
  servoVals[2] = Wire.read();
  servoVals[3] = Wire.read();
  
  if (false) {
    Serial.print(servoVals[0]);
    Serial.print(" ");
    Serial.print(servoVals[1]);
    Serial.print(" ");
    Serial.print(servoVals[2]);
    Serial.print(" ");
    Serial.print(servoVals[3]);
    Serial.println(" ");
  }
  for (int i = 0; i < 4; i++) {
    servoPos[i] = servoVals[i];
  }
}
