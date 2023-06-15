/*  
 * Arduino wiring:
 * 
 * Servo PWM output pins: Defined by output[]
 * Continuous servo feedback pins: Defined by feedbackPin[]
 * Servo reset (within software) pins: Defined by zeroServos[]
 * Reset limit switches are NO between Arduino and ground
 * Make sure all arrays are in the same order
 * 
 * Ramp servo: Digital pins 11, 3
 * 
 * Radio input: RX0
 * Manual override: Controlled by radio
  */

// Libraries
#include <IBusBM.h>
#include <Servo.h>
#include <Wire.h>
IBusBM IBus; // IBus object
Servo rampServoL;
Servo rampServoR;
const int rampOpen = 160;
const int rampClosed = 20;
const int rampPinL = 3; // Plane's left
const int rampPinR = 11;
int rampAngle[2] = {rampClosed, rampOpen};

const int boxWidth = 730; // Experimental
const int deployPos = -3420; // Experimental
const int center[] = {-2250, -2050}; // Experimental
const int maxBoxes = 7;
int numBoxes = maxBoxes; // Once operation begins, ranges from 0 to 6
// Implement something to return if carts are in right spot

// Declare servo values
const int output[] = {5, 6};
int servoPos[] = {90, 90}; // Starting servo positions
const int numServos = 2; // USER DEFINED

int contFeedback[numServos]; // Stores current servo angles
int feedbackPin[] = {9, 10}; // Code will loop indefinitely without servo feedback

int totalPos[] = {0, 0}; // Plate position
int prevPos[] = {0, 0}; // Previous position value
int feedbackOffset[] = {0, 0, 0, 0}; // Helps zero servo positions.  Length: 2*numServos

const int zeroServos[] = {7, 8}; // Zeros stored servo position value

// Store PWM input from radio receiver
const int numInputs = 5;
int feedback[numInputs];
int radioState = 2;
int waitForUpdate = 0;

Servo frontPlate; // Furthest from ramp
Servo backPlate; // Closest to ramp
Servo servoMotors[] = {frontPlate, backPlate};

void setup() {
  Wire.begin(4);
  Serial.begin(115200); // Developer input required from PC
  IBus.begin(Serial); // For radio
  
  for (int i = 0; i < numServos; i++) {
    servoMotors[i].attach(output[i]);
    sendServoData();
    pinMode(zeroServos[i], INPUT_PULLUP);
  }
  rampServoL.attach(rampPinL);
  rampServoR.attach(rampPinR);
  closeRamp();

  initialZero();
}

int started = 0;
void loop() {
  for (int i = 0; i < numServos; i++) {
    /*if (servoMotors[i].read() != servoPos[i]) {
      sendServoData();
      Serial.println("Changing in loop");
    }*/
    sendServoData();
    contFeedback[i] = readPos(feedbackPin[i]);
    calcPos(i, contFeedback[i]);
    if (!digitalRead(zeroServos[i]) || started < numServos) {
      resetServo(i);
      started += 1;
    }
    // Note: Opening Serial monitor sends a reset signal
  }

  boolean useRadio = true;
  // User input from radio or PC
  if (useRadio) {
    for (int i = 0; i < numInputs; i++) {
      feedback[i] = IBus.readChannel(i + 5);
      feedback[i] = roundFeedback(feedback[i]);
    }
    decodeFeedback();
    decodeState();
  } else {
    if (Serial.available()) {
      char ch = Serial.read(); // Cases: 0-7, +, -, r, R
      decodeSerialState(ch);
    }
    decodeFeedback();
    decodeState();
  }
  printStatus(1);
}

// Servo 0, Servo 1, First motor to move (0 or 1)
void goTo(int frontDest, int backDest, int first) {
  int dest[] = {frontDest, backDest};
  int dir[] = {2*(totalPos[0] > frontDest) - 1, 2*(totalPos[1] > backDest) - 1};
  printStatus(1);
  int servoSpeed = 75;
    
  if (abs(totalPos[0] - dest[0]) > 0 || abs(totalPos[1] - dest[1]) > 0) { // Oscillation checker
    for (int j = 0; j <2; j++) {
      int i = abs(first - j);
      //Serial.println(abs(totalPos[i]) - abs(dest[i]));
      if (totalPos[i] > dest[i]) {
        while (totalPos[i] > dest[i]) {
          servoPos[i] = 90 + dir[i] * servoSpeed; // Full forward or reverse
          sendServoData();
          contFeedback[i] = readPos(feedbackPin[i]);
          calcPos(i, contFeedback[i]);
          if (!digitalRead(zeroServos[i])) {
            resetServo(i);
          }
          if (true) {
            Serial.print("{");
            Serial.print(totalPos[i]);
            Serial.print(", ");
            Serial.print(dest[i]);
            Serial.print(", ");
            Serial.print(i);
            Serial.println("}");
          }
        }
        if (!digitalRead(zeroServos[i])) {
          resetServo(i);
        }
      } else if (totalPos[i] < dest[i]) {
        while (totalPos[i] < dest[i] && digitalRead(zeroServos[i])) {
          servoPos[i] = 90 + dir[i] * servoSpeed;
          sendServoData();
          contFeedback[i] = readPos(feedbackPin[i]);
          calcPos(i, contFeedback[i]);
          if (!digitalRead(zeroServos[i])) {
            resetServo(i);
          }
          if (true) {
            Serial.print("{");
            Serial.print(totalPos[i]);
            Serial.print(", ");
            Serial.print(dest[i]);
            Serial.print(", ");
            Serial.print(i);
            Serial.println("}");
          }
        }
      }
  
      // End servo motion
      servoPos[i] = 90;
      sendServoData();
      contFeedback[i] = readPos(feedbackPin[i]);
      calcPos(i, contFeedback[i]);
    }
    if (!digitalRead(zeroServos[0])) {
      resetServo(0);
    }
    if (!digitalRead(zeroServos[1])) {
      resetServo(1);
    }
  }
  printStatus(0);
}

// Moves servos out to edges and resets them
// Servos move until they hit a limit switch
void initialZero() {
  for (int i = 0; i < numServos; i++) {
    while (digitalRead(zeroServos[i])) {
      servoPos[i] = 0;
      sendServoData();
      contFeedback[i] = readPos(feedbackPin[i]);
      calcPos(i, contFeedback[i]);      
    }
    servoPos[i] = 90;
    sendServoData();
    contFeedback[i] = readPos(feedbackPin[i]);
    calcPos(i, contFeedback[i]);
  }
  resetServo(0);
  resetServo(1);
}

// Overrides current value of numBoxes
// Does nothing if given the current value
// First determines which servo moves first, used to prevent crushing and jams
// Servo has 1200 mA stall current requirement
void setBoxes(int n, int first) {
  if (n < 0) {
    setBoxes(0, first);
  } else if (n > maxBoxes) {
    setBoxes(maxBoxes, first);
  } else { // n between 0 and 6
    int frontDest = center[0] + (0.5 * n * boxWidth) + 20;
    int backDest = center[1] + (0.5 * n * boxWidth) + 20;
    if (false) {
      Serial.println(totalPos[0]);
      Serial.println(frontDest);
      Serial.println(totalPos[1]);
      Serial.println(backDest);
    }
    goTo(frontDest, backDest, first);
    numBoxes = n;
    printStatus(1);
  }
}

// Deploys one box and decrements numServos
void deploy() {
  if (numBoxes > 0) {
    goTo(0, deployPos + (numBoxes - 1) * boxWidth, 0);
    openRamp();
    delay(1000);
    closeRamp();
    delay(500);
    setBoxes(numBoxes - 1, 1);
  } else {
    Serial.println("No boxes to deploy!");
  }
}

// Sets internal representation of servo position to zero
// Does not move servo
void resetServo(int i) {
  feedbackOffset[2*i] += totalPos[i];
  feedbackOffset[2*i + 1] += totalPos[i];
  totalPos[i] = 0;
  while (feedbackOffset[2*i + 1] < 0) {
    feedbackOffset[2*i + 1] += 360;
  }
  feedbackOffset[2*i + 1] = feedbackOffset[2*i + 1] % 360;
  prevPos[i] = 0;
}

// Index i for totalPos[i] and prevPos[i], Servo readPos value
int calcPos(int i, int feedback) {
  //Serial.println(feedbackOffset[2*i + 1]);
  feedback = feedback - feedbackOffset[2*i + 1];
  
  while (feedback < 0) {
    feedback += 360;
  }
  while (feedback > 360) {
    feedback -= 360;
  }
  while (prevPos[i] < 0) {
    prevPos[i] += 360;
  }
  while (prevPos[i] > 360) {
    prevPos[i] -= 360;
  }

  //Serial.println(feedback);

  if (prevPos[i] < 60 && feedback > 300) {
    totalPos[i] -= 360;
  } else if (prevPos[i] > 300 && feedback < 60) {
    totalPos[i] += 360;
  }
  
  totalPos[i] += (feedback - prevPos[i]);
  prevPos[i] = feedback;

  //Serial.println(totalPos[i]);
  //delay(50); // Test removing this
  return totalPos[i];
}

// Maps feedback to [-1 0 1]
int roundFeedback(int raw) {
  if (raw == 1000) {
    return -1;
  } else if (raw == 2000) {
    return 1;
  } else {
    return 0;
  }
}

// Open the ramp slowly
void openRamp() {
  for (int i = rampAngle[1]; i <= rampOpen; i += 5) {
    rampAngle[1] = i;
    rampAngle[0] = 180 - i;
    delay(5);
    sendServoData();
  }
}

void closeRamp() {
  rampAngle[1] = rampClosed;
  rampAngle[0] = rampOpen;
  sendServoData();
}

void sendServoData() {
  Wire.beginTransmission(4); // transmit to device #4
  uint8_t dataOut[4];

  dataOut[0] = rampAngle[0];
  dataOut[1] = servoPos[0];
  dataOut[2] = servoPos[1];
  dataOut[3] = rampAngle[1];

  if(false) {
    Serial.print(dataOut[0]);
    Serial.print(" ");
    Serial.print(dataOut[1]);
    Serial.print(" ");
    Serial.print(dataOut[2]);
    Serial.print(" ");
    Serial.print(dataOut[3]);
    Serial.println(" ");
  }
  if (true) {
    rampServoL.write(dataOut[0]);
    servoMotors[0].write(dataOut[1]);
    servoMotors[1].write(dataOut[2]);
    rampServoR.write(dataOut[3]);
  }
  
  Wire.write((uint8_t*)dataOut, sizeof(dataOut));
  Wire.endTransmission();    // stop transmitting
}

// 5-Input system
void decodeFeedback() {
  if (feedback[1] == -1) {
    if (feedback[0] == 1) {
      // Manual disabled, auto deploy start
      radioState = 0;
    } else if (feedback[0] == -1) {
      // Manual disabled, auto deploy end
      // Close ramp
      radioState = 1;
    } else {
      // Manual disabled, auto disabled
      radioState = 2;
      // Ground crew: Watch for box falling before sending plates back
    }
  } else { // Manual override
    if (feedback[2] == 1) {
      // Ramp open
      radioState = 4;
    } else if (feedback[4] == 1){
      // Close ramp if open
      // Incrementing numBoxes mode
      radioState = 5;
    } else {
      if (feedback[3] == 0) {
        // Manual enabled and idle, ramp closed
        radioState = 3;
      } else if (feedback[3] == -1) {
        // Move plates forward toward ramp until limit switch impact
        radioState = 6;
      } else if (feedback[3] == 1) {
        // Move plates backward away from ramp until limit switch impact
        radioState = 7;
      }
    }
  }
}

// 4-Input system
void decodeFeedbackOld() {
  if (feedback[0] == -1) {
    if (feedback[1] == 1) {
      // Manual disabled, auto deploy start
      radioState = 0;
    } else if (feedback[1] == -1) {
      // Manual disabled, auto deploy end
      // Close ramp
      radioState = 1;
    } else {
      // Manual disabled, auto disabled
      radioState = 2;
      // Ground crew: Watch for box falling before sending plates back
    }
  } else { // Manual override
    if (feedback[2] == 0 && feedback[3] == 0) {
      // Manual enabled and idle, ramp closed
      radioState = 3;      
    } else if (feedback[3] == 1) {
      radioState = 4;
      // Open ramp
    } else if (feedback[3] == -1) {
      radioState = 5;
      // Close ramp if open
      // Incrementing numBoxes mode
    } else if (feedback[2] == -1 && feedback[3] == 0) {
      // Move plates forward toward ramp until limit switch impact
      radioState = 6;
    } else if (feedback[2] == 1 && feedback[3] == 0) {
      // Move plates backward away from ramp until limit switch impact
      radioState = 7;
    }
  }
}

// Dependent on 5 inputs
boolean safeToCenter; // Could numBoxes have changed?
int safeToAuto = 0;
void decodeState() {
  if (radioState == 0) {
    // Manual disabled, auto deploy start
    if ((abs(totalPos[1]) > 0 || abs(totalPos[0] - (deployPos + numBoxes * boxWidth)) > 0) && safeToAuto != 1) { // Oscillation?
      goTo(0, deployPos + (numBoxes - 1) * boxWidth, 0);
      safeToCenter = true;
      safeToAuto = feedback[0];
    }
    // Open ramp
    openRamp();
    if (feedback[2] == 1) {
      closeRamp();
      while (IBus.readChannel(2 + 5) == 2000) {
        delay(1);
      }
    }
  } else if (radioState == 1) {
    // Manual disabled, auto deploy end
    // Close ramp
    closeRamp();
    // Have the plates moved back recently and are there boxes remaining?
    if (safeToCenter && numBoxes > 0 && safeToAuto != -1) {
      setBoxes(numBoxes - 1, 1);
      safeToCenter = false;
      safeToAuto = feedback[0];
    } else if (safeToAuto != -1){
      setBoxes(numBoxes, 1);
    }
  } else if (radioState == 2) {
    // Manual disabled, auto disabled
    // Ground crew: Watch for box falling before sending plates back
    safeToAuto = feedback[0];
  } else if (radioState == 3) { // Manual override states 3-7
    // Manual enabled and idle, ramp closed
    if (waitForUpdate == 0) {
      waitForUpdate = 1;
    }
    for (int i = 0; i < 2; i++) {
      servoPos[i] = 90;
      sendServoData();
    }
    closeRamp();

    if (feedback[0] == -1) { // Reset and center
      initialZero();
      setBoxes(numBoxes, 1);
    } else if (feedback[0] == 1) { // Reset without centering
      initialZero();
      numBoxes = maxBoxes;
    }
  } else if (radioState == 4) {
    // Open ramp
    openRamp();
  } else if (radioState == 5) {
    // Close ramp if open
    closeRamp();
    if (feedback[3] == 1 && waitForUpdate == 1) {
      waitForUpdate = 0;
      setBoxes(numBoxes + 1, 1);
      // Increment numBoxes
    } else if (feedback[3] == -1 && waitForUpdate == 1) {
      waitForUpdate = 0;
      setBoxes(numBoxes - 1, 1);
      // Decrement numBoxes
    }
    if (feedback[3] == 0 && waitForUpdate == 0) {
      waitForUpdate = 1;
    }
  } else if (radioState == 6) {
    // Suggest adding LEDs in view of possible internal camera
    // Move plates forward toward ramp until limit switch impact
    if (digitalRead(zeroServos[1])) {
      for (int i = 0; i < 2; i++) {
        servoPos[i] = 90 + 20*(1 - 2*i);
        sendServoData();
        contFeedback[i] = readPos(feedbackPin[i]);
        calcPos(i, contFeedback[i]);
      }
    } else {
      for (int i = 0; i < 2; i++) {
        servoPos[i] = 90;
        sendServoData();
        contFeedback[i] = readPos(feedbackPin[i]);
        calcPos(i, contFeedback[i]);
      }
    }
  } else if (radioState == 7) {
    // Move plates backward away from ramp until limit switch impact
    if (digitalRead(zeroServos[0])) {
      for (int i = 0; i < 2; i++) {
        servoPos[i] = 90 - 20*(1 - 2*i);
        sendServoData();
        contFeedback[i] = readPos(feedbackPin[i]);
        calcPos(i, contFeedback[i]);
      }
    } else {
      for (int i = 0; i < 2; i++) {
        servoPos[i] = 90;
        sendServoData();
        contFeedback[i] = readPos(feedbackPin[i]);
        calcPos(i, contFeedback[i]);
      }
    }
  }
}

// Takes keyboard commands instead of radio commands
void decodeSerialState(char ch) {
  if (ch == '0') {
    Serial.println("Case 0");
    feedback[0] = 1;
    feedback[1] = -1;
    feedback[2] = 0; // X
    feedback[3] = 0; // X
    feedback[4] = 0; // X
  } else if (ch == '1') { 
    Serial.println("Case 1");
    feedback[0] = -1;
    feedback[1] = -1;
    feedback[2] = 0; // X
    feedback[3] = 0; // X
    feedback[4] = 0; // X
  } else if (ch == '2') {
    Serial.println("Case 2");
    feedback[0] = 0;
    feedback[1] = -1;
    feedback[2] = 0; // X
    feedback[3] = 0; // X
    feedback[4] = 0; // X
  } else if (ch == '3') {
    Serial.println("Case 3");
    feedback[0] = 0;
    feedback[1] = 1;
    feedback[2] = 0;
    feedback[3] = 0;
    feedback[4] = 0;
  } else if (ch == '4') {
    Serial.println("Case 4");
    feedback[0] = 0;
    feedback[1] = 1;
    feedback[2] = 1;
    feedback[3] = 0;
    feedback[4] = 0;
  } else if (ch == '5') {
    Serial.println("Case 5");
    feedback[0] = 0;
    feedback[1] = 1;
    feedback[2] = 0;
    feedback[3] = 0;
    feedback[4] = 1;
  } else if (ch == '6') {
    Serial.println("Case 6");
    feedback[0] = 0;
    feedback[1] = 1;
    feedback[2] = 0;
    feedback[3] = -1;
    feedback[4] = 0;
  } else if (ch == '7') {
    Serial.println("Case 7");
    feedback[0] = 0;
    feedback[1] = 1;
    feedback[2] = 0;
    feedback[3] = 1;
    feedback[4] = 0;
  } else if (ch == '+') {
    Serial.println("Case +");
    feedback[0] = 0;
    feedback[1] = 1;
    feedback[2] = 0;
    feedback[3] = -1;
    feedback[4] = 1;
  } else if (ch == '-') {
    Serial.println("Case -");
    feedback[0] = 0;
    feedback[1] = 1;
    feedback[2] = 0;
    feedback[3] = 1;
    feedback[4] = 1;
  } else if (ch == 'r') {
    Serial.println("Case r");
    feedback[0] = -1;
    feedback[1] = 1;
    feedback[2] = 0;
    feedback[3] = 0;
    feedback[4] = 0;
  } else if (ch == 'R') {
    Serial.println("Case R");
    feedback[0] = 1;
    feedback[1] = 1;
    feedback[2] = 0;
    feedback[3] = 0;
    feedback[4] = 0;
  }
}

// Function reads the PWM feedback signal coming from the servo feedback wire
// Causes infinite loop if PWM feedback is not present
int readPos(int feedbackPin)
{
  int tHigh; // time when feedback signal is at 3.3V
  int tLow;  // time feedback signal is at 0V
  int tCycle; // period of full cycle

  float theta = 0; // angle servo is at
  float dc = 0; // duty cycle of PWM signal
  int unitsFC = 360; // units in a full circle in this case, degrees
  float dcMin = 0.029; //minimum duty cycle, this corresponds to theta = 0
  float dcMax = 0.971; // max duty cycle when theta is approching 360
  float dutyScale = 1; 
  while(1) {
    pulseIn(feedbackPin, LOW);
    tHigh = pulseIn(feedbackPin, HIGH); //measures how long the PWM signal is on, measure in microseconds
    tLow =  pulseIn(feedbackPin, LOW); // measures how long the PWM signal is off
    tCycle = tHigh + tLow; // time full cycle = time on + time off
    if (false) {
      Serial.print(tHigh);
      Serial.print("   ");
      Serial.println(tLow);
    }
    if ((tCycle > 1000) && ( tCycle < 1200)) break; // tCycle should always be approx. 1.1 ms or 1100 microseconds
  }

  dc = (dutyScale * tHigh) / tCycle; // calculate the duty cycle of the PWM signal
  theta = ((dc - dcMin) * unitsFC) / (dcMax - dcMin); // calculate theta from the PWM signal
  return theta; //return theta
}

void printStatus (int statusMode) {
  if (statusMode == 0) {
    1+1;
  } else if (statusMode == 1) {
    Serial.print("[");
    Serial.print(totalPos[0]);
    Serial.print(", ");
    Serial.print(totalPos[1]);
    Serial.print(", ");
    Serial.print(radioState);
    Serial.print(", ");
    Serial.print(numBoxes);
    Serial.println("]");
  } else if (statusMode == 2) { // Plotter
    Serial.print(totalPos[0]);
    Serial.print(", ");
    Serial.println(totalPos[1]);
  } else if (statusMode == 3) {
    Serial.print("[");
    Serial.print(feedback[0]);
    Serial.print(", ");
    Serial.print(feedback[1]);
    Serial.print(", ");
    Serial.print(feedback[2]);
    Serial.print(", ");
    Serial.print(feedback[3]);
    Serial.print(", ");
    Serial.print(feedback[4]);
    Serial.println("]");
  } else if (statusMode == 4) {
    Serial.print("[");
    Serial.print(rampServoL.read());
    Serial.print(", ");
    Serial.print(rampServoR.read());
    Serial.println("]");
  } else if (statusMode == 5) {
    Serial.print("[");
    Serial.print(radioState);
    Serial.print(", ");
    Serial.print(numBoxes);
    Serial.println("]");
  } else if (statusMode == 6) {
    Serial.print("[");
    Serial.print(digitalRead(zeroServos[0]));
    Serial.print(", ");
    Serial.print(digitalRead(zeroServos[1]));
    Serial.println("]");
  }else if (statusMode == 7) {
    Serial.print("[");
    Serial.print(servoPos[0]);
    Serial.print(", ");
    Serial.print(servoPos[1]);
    Serial.println("]");
  }
}
