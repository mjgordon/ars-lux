/*
   TODO:
   While values are being set, turn off totally
   Tune debounce timing? Can KRL even register smaller than 0.1 for WAIT?

*/


//Receives input from relays
const int pinInputA = 2;
const int pinInputB = 3;
const int pinInputC = 4;
const int pinInputD = 5;

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
// Oh, also A6, and A7 don't have pullup resistors. Because of course.

//The Echo toggle directly controls flow of current to control-box LED's, is not read by the arduino.

//Sends current to yellow LED's to echo relay state.
const int pinEchoA = A3;
const int pinEchoB = A2;
const int pinEchoC = A1;
const int pinEchoD = A0;

// = 3Color LED variables =

//Signals will control the 0 - red, 1 - green, or 2 - blue component of the LED.
int pos = 0;

//The previous state of the D relay.
int lastD = 0;

//When the D relay was last changed (after debouncing).
long lastFlip = 0;

int rValue = 255;
int gValue = 255;
int bValue = 255;

int count = 0;


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


  pinMode(pinInputA, INPUT_PULLUP);
  pinMode(pinInputB, INPUT_PULLUP);
  pinMode(pinInputC, INPUT_PULLUP);
  pinMode(pinInputD, INPUT_PULLUP);

  

  Serial.begin(9600);

  lastD = digitalRead(pinInputD) == 0;
  lastFlip = millis();
  
}

void loop() {
  int stateA = digitalRead(pinInputA) == 0;
  int stateB = digitalRead(pinInputB) == 0;
  int stateC = digitalRead(pinInputC) == 0;
  int stateD = digitalRead(pinInputD) == 0;

  int mode = digitalRead(pinInputMode);
  int power = digitalRead(pinInputPower) == 0;

  digitalWrite(pinEchoA, stateA);
  digitalWrite(pinEchoB, stateB);
  digitalWrite(pinEchoC, stateC);
  digitalWrite(pinEchoD, stateD);

  // 3 LED Mode
  if (mode) {
    digitalWrite(pinR, stateA & power);
    digitalWrite(pinG, stateB & power);
    digitalWrite(pinB, stateC  & power);

    analogWrite(pin3ColorR, 255);
    analogWrite(pin3ColorG, 255);
    analogWrite(pin3ColorB, 255);

    pos = 0;
  }

  // Single LED Mode
  else {
    digitalWrite(pinR, LOW);
    digitalWrite(pinG, LOW);
    digitalWrite(pinB, LOW);

    if (stateD != lastD) {
      //Debounce the relays

        Serial.print("Delta d: ");
        Serial.println(millis() - lastFlip);

      if (millis() - lastFlip > 12) {
        if (pos == 0) {
          Serial.println("=============================");
          Serial.print("Count: ");
          Serial.println(count);
          count++;
        }

        

        //Serial.print("D: ");
        //Serial.println(stateD);
        //Serial.print("Last D: ");
        //Serial.println(lastD);
        Serial.print("Pos: ");
        Serial.println(pos);
        switch (pos) {
          case 0:
            rValue = 255 - decode(stateA, stateB, stateC);
            break;
          case 1:
            gValue = 255 - decode(stateA, stateB, stateC);
            break;
          case 2:
            bValue = 255 - decode(stateA, stateB, stateC);
            break;
          default:
            Serial.println("pos error");
        }
        pos = (pos + 1) % 3;
        lastFlip = millis();
      }
    }
    lastD = stateD;

    if (power) {
      analogWrite(pin3ColorR, rValue);
      analogWrite(pin3ColorG, gValue);
      analogWrite(pin3ColorB, bValue);
    }
    else {
      analogWrite(pin3ColorR, 255);
      analogWrite(pin3ColorG, 255);
      analogWrite(pin3ColorB, 255);
    }
  }
}


/**
   Converte 3 binary bits into quantified values between 0 - 255
*/
int decode(int A, int B, int C) {
  int out = 0;
  if (A) out += 4;
  if (B) out += 2;
  if (C) out += 1;

  float temp = out;
  temp /= 7.0;
  temp *= 255.0;
  out = (int)temp;
  Serial.println(out);
  return (out);
}

