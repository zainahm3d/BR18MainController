#include <Arduino.h>
#include <FlexCan.h>
#include <kinetis_flexcan.h>

int rpm = 0;
double temp = 0;

#define Steering_ID 0
#define RPM_ID 218099784
#define TEMP_ID 218101064
#define led 13
#define shiftUp A4
#define shiftDown A5
#define fuelRelay A8
#define sparkCut 12
#define fan A6
#define pump A7
#define neutralSwitch 6

CAN_message_t inMsg;
CAN_message_t outMsg;

elapsedMillis ECUTimer;
elapsedMillis pumpTimer;
elapsedMillis messageTimer;

boolean timerFlag = 0;
int cylinderDeactivationEnabled = 0;

void setup() {

  pinMode(led, OUTPUT);                 // LED
  pinMode(shiftUp, OUTPUT);             // Shift up
  pinMode(shiftDown, OUTPUT);           // Shift down
  pinMode(fuelRelay, OUTPUT);           // Fuel Relay
  pinMode(sparkCut, OUTPUT);            // Spark cut (to ECU)
  pinMode(fan, OUTPUT);                 // Fan
  pinMode(pump, OUTPUT);                // Water Pump
  pinMode(neutralSwitch, INPUT_PULLUP); // CBR Neutral switch

  digitalWrite(led, HIGH);
  digitalWrite(sparkCut, HIGH);
  digitalWrite(fuelRelay, HIGH);
  digitalWrite(fan, LOW);
  digitalWrite(pump, LOW);

  Serial.begin(9600);
  Serial.println("online");

  Can0.begin(250000); // PE3 ECU SPEED

  // Allow Extended CAN id's through
  CAN_filter_t allPassFilter;
  allPassFilter.ext = 1;
  for (uint8_t filterNum = 1; filterNum < 16; filterNum++) {
    Can0.setFilter(allPassFilter, filterNum);
  }

  inMsg.ext = true;
  outMsg.ext = true;
  outMsg.len = 8;

  ECUTimer = 0;
  pumpTimer = 0;
}

void loop() {

  if (Can0.available()) {

    Can0.read(inMsg);
    digitalWrite(led, !digitalRead(led)); // show that a message was recieved

    if (inMsg.id == RPM_ID) {
      int lowByte = inMsg.buf[0];
      int highByte = inMsg.buf[1];
      int newRPM = ((highByte * 256) + lowByte);
      rpm = newRPM;

      Serial.print("RPM: ");
      Serial.println(rpm);
      pumpTimer = 0;
      ECUTimer = 0;
      timerFlag = 0;

      if (rpm > 1500) { // Do not not enable pump while cranking
        digitalWrite(pump, HIGH);
      }
    }

    if (inMsg.id == Steering_ID) {

      int b0 = inMsg.buf[0];

      // shift up
      if (b0 == 10) { // 0x0A

        digitalWrite(sparkCut, LOW);
        delay(10);
        digitalWrite(shiftUp, HIGH);
        delay(20);
        digitalWrite(shiftUp, LOW);
        digitalWrite(sparkCut, HIGH);

        // Serial.println("Shifted up.");
      }

      // shift down
      else if (b0 == 11) { // 0x0B

        digitalWrite(sparkCut, LOW);
        delay(10);
        digitalWrite(shiftDown, HIGH);
        delay(20);
        digitalWrite(shiftDown, LOW);
        digitalWrite(sparkCut, HIGH);

        // Serial.println("Shifted down.");
      }
    }

    if (inMsg.id == TEMP_ID) {
      double lowByte = inMsg.buf[4];
      double highByte = inMsg.buf[5];
      double newTemp = ((highByte * 256) + lowByte);

      if (newTemp > 32767) {
        newTemp = newTemp - 65536;
      }

      temp = newTemp / 10;
      Serial.print("Temp: ");
      Serial.println(temp);
    }
  }

  if (ECUTimer >= 1500 && digitalRead(pump) == 1 &&
      timerFlag == 0) // ENGINE HAS TURNED OFF and pump was on
  {
    digitalWrite(pump, HIGH);
    digitalWrite(fan, LOW);
    rpm = 0;
    pumpTimer = 0;
    timerFlag = 1;
  }

  // run pump for 15 seconds if it was already on (engine running)
  if (pumpTimer >= 13500) {
    digitalWrite(pump, LOW);
    pumpTimer = 0;
  }

  if (rpm > 2200 && temp >= 150) {
    digitalWrite(fan, HIGH);
    // Serial.println("Fan High.");
  } else if (rpm > 2200 && temp <= 130) {
    digitalWrite(fan, LOW);
  }
  if (rpm < 1000) {
    digitalWrite(fan, LOW);
  }

  // ---------- Cylinder Deactivation ----------
  if (rpm > 10500 && digitalRead(neutralSwitch) == 0) {
    digitalWrite(fuelRelay, LOW);
    cylinderDeactivationEnabled = 0;
  } else {
    digitalWrite(fuelRelay, HIGH);
    cylinderDeactivationEnabled = 1;
  }

  if (messageTimer > 10) {
    messageTimer = 0;

    outMsg.id = 6;
    outMsg.buf[0] = digitalRead(neutralSwitch);
    outMsg.buf[1] = cylinderDeactivationEnabled;

    Can0.write(outMsg);
  }
}
