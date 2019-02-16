#include <Arduino.h>
#include <FlexCan.h>
//#include <SimpleTimer.h>

int rpm = 0;
double temp = 0;
int neutralStatus = 0;

#define Steering_ID 0
#define RPM_ID 218099784
#define TEMP_ID 218101064
#define ANALOG_ID 218100296 // analog input 1-4, assuming 1 for code.
#define led 13
#define shiftUp A4
#define shiftDown A5
#define fuelCut A8
#define sparkCut 6
#define fan A6
#define pump A7
#define Can1 3
#define Can2 4

CAN_message_t inMsg;

void setup() {

  pinMode(led, OUTPUT);       // LED
  pinMode(shiftUp, OUTPUT);   // shhift up
  pinMode(shiftDown, OUTPUT); // shift down
  pinMode(fuelCut, OUTPUT);   // fuelCut
  pinMode(sparkCut, OUTPUT);  // sparkCut
  pinMode(fan, OUTPUT);       // fan
  pinMode(pump, OUTPUT);      // pump

  pinMode(Can1, OUTPUT);
  pinMode(Can2, OUTPUT);

  // Timer ECUTimer;
  // Timer pumpTimer;

  // LOW is Neutral
  // DigitalIn neutralSwitch = DigitalIn(D, PullUp);

  digitalWrite(led, HIGH);
  digitalWrite(sparkCut, LOW);
  digitalWrite(fuelCut, LOW);
  digitalWrite(fan, LOW);
  digitalWrite(pump, LOW);

  // ECUTimer.start();

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
}

void loop() {

  if (Can0.available()) {
    Can0.read(inMsg);

    digitalWrite(led, !digitalRead(led)); // show that a message was recieved

    // ECUTimer.start();
    // ECUTimer.reset();

    if (inMsg.id == RPM_ID) {
      int lowByte = inMsg.buf[0];
      int highByte = inMsg.buf[1];
      int newRPM = ((highByte * 256) + lowByte);
      rpm = newRPM;

      Serial.print("RPM: ");
      Serial.println(rpm);

      if (rpm > 1500) // make sure to not engage the water pump while cranking
                      // starter
      {
        digitalWrite(pump, HIGH);
      }
    }

    if (inMsg.id == Steering_ID) {

      int b0 = inMsg.buf[0];

      // shift up
      if (b0 == 10) {

        digitalWrite(sparkCut, HIGH);
        delay(10);
        digitalWrite(shiftUp, HIGH);
        delay(200);
        digitalWrite(shiftUp, LOW);
        digitalWrite(sparkCut, LOW);

        Serial.println("Shifted up.");

      }

      // shift down
      else if (b0 == 11) {

        digitalWrite(sparkCut, HIGH);
        delay(10);
        digitalWrite(shiftDown, HIGH);
        delay(200);
        digitalWrite(shiftDown, LOW);
        digitalWrite(sparkCut, LOW);

        Serial.println("Shifted down.");
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

      // printf("\nRPM %d", rpm); // REMOVE
      // printf("\tTEMP: %f", temp);
    }

    if (inMsg.id == TEMP_ID) {
      int lowByte = inMsg.buf[0];
      int highByte = inMsg.buf[1];
      int data = ((highByte * 256) + lowByte);
      neutralStatus = data;
    }
  }

  // int pumpVal = digitalRead(pump);

  // if (ECUTimer.read_ms() >= 3000 &&
  //    pumpVal == 1) // ENGINE HAS TURNED OFF and pump was on

  // {
  //   digitalWrite(pump, HIGH);
  //   digitalWrite(fan, LOW);
  //   rpm = 0;
  //   ECUTimer.stop();
  //   ECUTimer.reset();
  //   pumpTimer.reset();
  //   pumpTimer.start();
  // }

  // run pump for 15 seconds if it was already on (engine running)

  // if (pumpTimer.read() >= 12) {
  //   digitalWrite(pump, 0);
  //   pumpTimer.stop();
  //   pumpTimer.reset();

  if (rpm > 2200 && temp >= 150) {
    digitalWrite(fan, HIGH);
    Serial.println("Fan High.");

  } else if (rpm > 2200 && temp <= 130) {
    digitalWrite(fan, LOW);
  }
  if (rpm < 1000) {
    digitalWrite(fan, LOW);
  }
}
