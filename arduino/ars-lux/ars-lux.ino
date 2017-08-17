
/*
Instruction set:
B : B B B B
command : data0 data1 data2 data3

00 : ___ RRR GGG BBB - set tri
01 : ___ __R __G __B - set rgb
02 : ___ ___ ___ __R - set R
03 : ___ ___ ___ __G - set G
04 : ___ ___ ___ __B - set B
05 : III III III III - set fade speed for tri
06 : III III III III - set advance speed
07 : ___ ___ ___ ___ - Add next to list
08 : ___ ___ ___ ___ - Clear list

10 : ___ __R __G __B - Toggle rgb
11 : ___ ___ ___ ___ - Toggle R
12 : ___ ___ ___ ___ - Toggle G
13 : ___ ___ ___ ___ - Toggle B

20 : III III III III - Set blink speed for next (TRI/RGB)
21 : III III III III - Set independent blink speed for R
22 : III III III III - Set independent blink speed for G
23 : III III III III - Set independent blink speed for B


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

byte byteInput[] = {0,0,0,0,0};
int intValue;
float floatValue;

int triValueR = 255;
int triValueG = 255;
int triValueB = 255;

boolean rgbValueR = false;
boolean rgbValueG = false;
boolean rgbValueB = false;



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
      analogWrite(pin3ColorR, triValueR);
      analogWrite(pin3ColorG, triValueG);
      analogWrite(pin3ColorB, triValueB);
    }
    else {
      analogWrite(pin3ColorR, 255);
      analogWrite(pin3ColorG, 255);
      analogWrite(pin3ColorB, 255);
    }
  }
}

/*
Reads 4-byte integers or floats. The value sent from the robot will be available 
in the intValue or floatValue variables.
*/
void checkSerial() {
  while (Serial.available() > 0) {
    byteInput[serialCount] = Serial.peek();
    if (serialCount > 0) {
      int id = 4 - (serialCount - 1);
      intConvert.asBytes[id] = Serial.peek();
      floatConvert.asBytes[id] = Serial.read();
    }
    

    if (serialCount == 4) {
      intValue = intConvert.asInt;
      floatValue = floatConvert.asFloat;
    }
    serialCount++;
    serialCount %= 5;
  }
}


