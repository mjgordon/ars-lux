
/*
  Instruction set:
  B : B B B
  command : data0 data1 data2

  X 00 : RRR GGG BBB - set tri
  X 01 : __R __G __B - set rgb
    02 : ___ ___ __R - set R
    03 : ___ ___ __G - set G
    04 : ___ ___ __B - set B
    05 : III III III - set fade speed for tri
    06 : III III III - set advance speed
    07 : ___ ___ ___ - Add next to list
    08 : ___ ___ ___ - Clear list

    10 : __R __G __B - Toggle rgb
    11 : ___ ___ ___ - Toggle R
    12 : ___ ___ ___ - Toggle G
    13 : ___ ___ ___ - Toggle B

    20 : III III III - Set blink speed for next (TRI/RGB)
    21 : III III III - Set independent blink speed for R
    22 : III III III - Set independent blink speed for G
    23 : III III III - Set independent blink speed for B


*/

/*
   TODO:
   While values are being set, turn off totally
   Tune debounce timing? Can KRL even register smaller than 0.1 for WAIT?

*/

union intConverter {
  byte asBytes[4];
  int asInt;
} intConvert;

union floatConverter {
  byte asBytes[4];
  float asFloat;
} floatConvert;

int serialCount;

// Sends current to single color LEDs
const int pinR = 6;
const int pinG = 7;
const int pinB = 8;

//Sends PWM current to 3color LEDs (common cathode, so values are inverted)
const int pin3ColorR = 9;
const int pin3ColorG = 10;
const int pin3ColorB = 11;

//Toggles whether any color LED's can be lit.
const int pinInputPower = 12;
//Toggles whether the individual or 3-color LED's are controlled.
const int pinInputMode = A5;
// Because pin 13 is the one pin that doesn't work for input_pullup, of course. :P
// The presence of the built in LED drains enough current that its stuck in one position.
// You could conceivably cut the trace to the LED, but heres its too much hassle for too little gain.
// Oh, also A6, and A7 don't have pullup resistors. Because of course.

//The Echo toggle directly controls flow of current to control-box LED's, is not read by the arduino.

//Sends current to yellow LED's to echo relay state.
const int pinEchoA = A3;
const int pinEchoB = A2;
const int pinEchoC = A1;
const int pinEchoD = A0;
const int echoPins[] = {A3, A2, A1, A0};

byte byteInput[] = {0, 0, 0, 0};
int intValue;
float floatValue;

int triFadeTime = 0;
int triFadeStart = 0;

int triValueRLast = 255;
int triValueGLast = 255;
int triValueBLast = 255;

int triValueR = 255;
int triValueG = 255;
int triValueB = 255;

boolean rgbValueR = LOW;
boolean rgbValueG = LOW;
boolean rgbValueB = LOW;

const byte CMD_SET_TRI = 0;
const byte CMD_SET_RGB = 1;
const byte CMD_SET_R = 2;
const byte CMD_SET_G = 3;
const byte CMD_SET_B = 4;
const byte CMD_SET_FADE = 5;
const byte CMD_SET_ADVANCE = 6;
const byte CMD_LIST_ADD = 7;
const byte CMD_LIST_CLEAR = 8;
const byte CMD_TOGGLE_RGB = 10;
const byte CMD_TOGGLE_R = 11;
const byte CMD_TOGGLE_G = 12;
const byte CMD_TOGGLE_B = 13;
const byte CMD_BLINK_SPEED = 20;
const byte CMD_BLINK_R = 21;
const byte CMD_BLINK_G = 22;
const byte CMD_BLINK_B = 23;


void setup() {
  pinMode(pinR, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinB, OUTPUT);

  //PWM output is default, no pinMode needed.

  pinMode(pinInputMode, INPUT_PULLUP);
  pinMode(pinInputPower, INPUT_PULLUP);

  //Setting these here may not be necessary);
  digitalWrite(pinInputMode, HIGH);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);

  pinMode(pinEchoA, OUTPUT);
  pinMode(pinEchoB, OUTPUT);
  pinMode(pinEchoC, OUTPUT);
  pinMode(pinEchoD, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  checkSerial();

  int mode = digitalRead(pinInputMode);
  int power = digitalRead(pinInputPower) == 0;

  // 3 LED Mode
  if (mode) {
    digitalWrite(pinR, rgbValueR & power);
    digitalWrite(pinG, rgbValueG & power);
    digitalWrite(pinB, rgbValueB  & power);

    analogWrite(pin3ColorR, 255);
    analogWrite(pin3ColorG, 255);
    analogWrite(pin3ColorB, 255);
  }

  // Single LED Mode
  else {
    digitalWrite(pinR, LOW);
    digitalWrite(pinG, LOW);
    digitalWrite(pinB, LOW);

    if (power) {
      int r = triValueR;
      int g = triValueG;
      int b = triValueB;
      if (triFadeTime > 0) {
        float pos = 1.0f * (millis() - triFadeStart) / triFadeTime;
        if (pos > 1) pos = 1;
        r = (int)((triValueR - triValueRLast) * pos + triValueRLast);
        g = (int)((triValueG - triValueRLast) * pos + triValueGLast);
        b = (int)((triValueB - triValueRLast) * pos + triValueBLast);
      }
      analogWrite(pin3ColorR, r);
      analogWrite(pin3ColorG, g);
      analogWrite(pin3ColorB, b);
    }
    else {
      analogWrite(pin3ColorR, 255);
      analogWrite(pin3ColorG, 255);
      analogWrite(pin3ColorB, 255);
    }
  }
}


void runCommand() {
  byte command = byteInput[0];

  switch (command) {
    case CMD_SET_TRI:
      triValueRLast = triValueR;
      triValueGLast = triValueG;
      triValueBLast = triValueB;
      triValueR = 255 - byteInput[1];
      triValueG = 255 - byteInput[2];
      triValueB = 255 - byteInput[3];
      triFadeStart = millis();
      
      break;
    case CMD_SET_RGB:
      rgbValueR = byteInput[1] != 0;
      rgbValueG = byteInput[2] != 0;
      rgbValueB = byteInput[3] != 0;
      break;
    case CMD_SET_R:
      rgbValueR = byteInput[3] != 0;
      break;
    case CMD_SET_G:
      rgbValueG = byteInput[3] != 0;
      break;
    case CMD_SET_B:
      rgbValueB = byteInput[3] != 0;
      break;
    case CMD_SET_FADE:
      triFadeTime = intValue;
      break;


    default:
      for (int i = 0; i < 4; i++) {
        digitalWrite(echoPins[i], HIGH);
      }
      delay(100);
      for (int i = 0; i < 4; i++) {
        digitalWrite(echoPins[i], LOW);
      }
      delay(100);
      for (int i = 0; i < 4; i++) {
        digitalWrite(echoPins[i], HIGH);
      }
      delay(100);
      for (int i = 0; i < 4; i++) {
        digitalWrite(echoPins[i], LOW);
      }
  }

}

/*
  Reads 4-byte integers or floats. The value sent from the robot will be available
  in the intValue or floatValue variables.
*/
void checkSerial() {
  while (Serial.available() > 0) {
    
    if (serialCount > 0) {
      int id = 3 - serialCount;
      intConvert.asBytes[id] = Serial.peek();
      floatConvert.asBytes[id] = Serial.peek();
    }
    byteInput[serialCount] = Serial.read();


    if (serialCount == 3) {
      intValue = intConvert.asInt;
      floatValue = floatConvert.asFloat;
      runCommand();
    }
    serialCount++;
    serialCount %= 4;

    for (int i = 0; i < 4; i++) {
      digitalWrite(echoPins[i], i <= serialCount);
    }
  }
}


