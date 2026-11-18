#include <Wire.h>                          // I2C library (Nano: A4 = SDA, A5 = SCL)
#include <Adafruit_PWMServoDriver.h>       // library that talks to the PCA9685

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // 0x40 = default I2C address of the board

// ================= SETTINGS (edit these) =================
const int  NUM_SERVOS    = 5;      // how many values Python sends per message (thumb..pinky). Don't change.
const int  ACTIVE_SERVOS = 1;      // how many servos are actually plugged in, starting at CH0. Change to 5 later.
const bool RUN_SELF_TEST = true;   // true = sweep each active servo once at power-up to prove it works

const int MIN_US = 600;            // pulse width (microseconds) for 0 degrees (safe range for SG90)
const int MAX_US = 2400;           // pulse width for 180 degrees

// Safe travel per finger, in degrees. Start narrow, widen slowly.
// Swap a finger's min and max numbers to reverse its direction.
const int MIN_ANGLE[NUM_SERVOS] = {30, 30, 30, 30, 30};      // "open" position
const int MAX_ANGLE[NUM_SERVOS] = {150, 150, 150, 150, 150}; // "closed" position

char lineBuf[40];                  // holds one incoming line, e.g. "0,90,180,45,10"
byte lineLen = 0;                  // how many characters are stored so far

// Send one servo to an exact angle (0-180) by converting it to a pulse width
void setServo(int ch, int deg) {
  deg = constrain(deg, 0, 180);                      // safety clamp
  int us = map(deg, 0, 180, MIN_US, MAX_US);         // angle -> pulse width in microseconds
  pwm.writeMicroseconds(ch, us);                     // PCA9685 outputs that pulse on channel ch
}

// Move a finger using the 0-180 value from Python, squeezed into that finger's safe range
void moveFinger(int i, int a) {
  a = constrain(a, 0, 180);                                  // 0 = open, 180 = fully closed
  setServo(i, map(a, 0, 180, MIN_ANGLE[i], MAX_ANGLE[i]));   // rescale into the safe range
}

// Slow sweep open -> closed -> open on each active servo
void selfTest() {
  for (int i = 0; i < ACTIVE_SERVOS; i++) {
    for (int a = 0; a <= 180; a += 3)   { moveFinger(i, a); delay(15); }   // opening to closing
    for (int a = 180; a >= 0; a -= 3)   { moveFinger(i, a); delay(15); }   // closing to opening
  }
}

// Turn one received text line into servo movement
void parseAndMove() {
  int v[NUM_SERVOS];
  // Pull 5 integers out of the text; sscanf returns how many it found
  if (sscanf(lineBuf, "%d,%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3], &v[4]) == 5) {
    for (int i = 0; i < ACTIVE_SERVOS; i++) {        // only drive servos that are really plugged in
      moveFinger(i, v[i]);
    }
  }                                                  // garbled lines are ignored
}

void setup() {
  Serial.begin(115200);                    // must match BAUD_RATE in Python
  pwm.begin();                             // start I2C and the PCA9685
  pwm.setOscillatorFrequency(27000000);    // chip's internal clock (typical value)
  pwm.setPWMFreq(50);                      // 50 Hz = standard servo signal
  delay(10);
  for (int i = 0; i < ACTIVE_SERVOS; i++) moveFinger(i, 0);   // start with fingers open
  delay(500);                              // give the servo time to get there
  if (RUN_SELF_TEST) selfTest();           // optional power-up test
  Serial.println(F("READY"));              // Python waits for this word before sending data
}

void loop() {
  while (Serial.available() > 0) {         // any byte waiting from the laptop?
    char c = Serial.read();                // read one character
    if (c == '\n') {                       // newline = a complete message has arrived
      lineBuf[lineLen] = '\0';             // end the string
      parseAndMove();                      // act on it
      lineLen = 0;                         // start a fresh line
    } else if (c != '\r') {                // ignore carriage returns
      if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;   // store the character
      else lineLen = 0;                    // buffer overflow: discard and restart
    }
  }
}