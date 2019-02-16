#include <Arduino.h>
#include <FlexCan.h>

int rpm = 0;
double temp = 0;

#define Steering_ID 0
#define RPM_ID 218099784
#define TEMP_ID 218101064
#define ANALOG_ID 218100296 // analog input 1-4, assuming 1 for code.
#define led 13
#define shiftUp A4
#define shiftDown A5
#define fuelRelay A8
#define sparkCut 12
#define fan A6
#define pump A7
#define neutralSwitch 6

CAN_message_t inMsg;

elapsedMillis ECUTimer;
elapsedMillis pumpTimer;

boolean timerFlag = 0;

void setup()
{

  pinMode(led, OUTPUT);       // LED
  pinMode(shiftUp, OUTPUT);   // shhift up
  pinMode(shiftDown, OUTPUT); // shift down
  pinMode(fuelRelay, OUTPUT); // fuelRela
  pinMode(sparkCut, OUTPUT);  // sparkCut
  pinMode(fan, OUTPUT);       // fan
  pinMode(pump, OUTPUT);      // pump
  pinMode(neutralSwitch, INPUT_PULLUP);

  digitalWrite(led, HIGH);
  digitalWrite(sparkCut, LOW);
  digitalWrite(fuelRelay, HIGH);
  digitalWrite(fan, LOW);
  digitalWrite(pump, LOW);

  Serial.begin(9600);
  Serial.println("online");

  Can0.begin(250000); // PE3 ECU SPEED

  // Allow Extended CAN id's through
  CAN_filter_t allPassFilter;
  allPassFilter.ext = 1;
  for (uint8_t filterNum = 1; filterNum < 16; filterNum++)
  {
    Can0.setFilter(allPassFilter, filterNum);
  }

  inMsg.ext = true;

  ECUTimer = 0;
  pumpTimer = 0;
}

void loop()
{

  if (Can0.available())
  {

    Can0.read(inMsg);
    digitalWrite(led, !digitalRead(led)); // show that a message was recieved

    if (inMsg.id == RPM_ID)
    {
      int lowByte = inMsg.buf[0];
      int highByte = inMsg.buf[1];
      int newRPM = ((highByte * 256) + lowByte);
      rpm = newRPM;

      Serial.print("RPM: ");
      Serial.println(rpm);
      pumpTimer = 0;
      ECUTimer = 0;
      timerFlag = 0;

      if (rpm > 1500) // make sure to not engage the water pump while cranking
                      // starter
      {
        digitalWrite(pump, HIGH);
      }
    }

    if (inMsg.id == Steering_ID)
    {

      int b0 = inMsg.buf[0];

      // shift up
      if (b0 == 10)
      {

        digitalWrite(sparkCut, HIGH);
        delay(10);
        digitalWrite(shiftUp, HIGH);
        delay(200);
        digitalWrite(shiftUp, LOW);
        digitalWrite(sparkCut, LOW);

        Serial.println("Shifted up.");
      }

      // shift down
      else if (b0 == 11)
      {

        digitalWrite(sparkCut, HIGH);
        delay(10);
        digitalWrite(shiftDown, HIGH);
        delay(200);
        digitalWrite(shiftDown, LOW);
        digitalWrite(sparkCut, LOW);

        Serial.println("Shifted down.");
      }
    }

    if (inMsg.id == TEMP_ID)
    {
      double lowByte = inMsg.buf[4];
      double highByte = inMsg.buf[5];
      double newTemp = ((highByte * 256) + lowByte);

      if (newTemp > 32767)
      {
        newTemp = newTemp - 65536;
      }

      temp = newTemp / 10;
      Serial.print("Temp: ");
      Serial.println(temp);
    }
  }

  if (ECUTimer >= 1500 && digitalRead(pump) == 1 && timerFlag == 0) // ENGINE HAS TURNED OFF and pump was on
  {
    digitalWrite(pump, HIGH);
    digitalWrite(fan, LOW);
    rpm = 0;
    pumpTimer = 0;
    timerFlag = 1;
  }

  // run pump for 15 seconds if it was already on (engine running)
  if (pumpTimer >= 13500)
  {
    digitalWrite(pump, LOW);
    pumpTimer = 0;
  }

  if (rpm > 2200 && temp >= 150)
  {
    digitalWrite(fan, HIGH);
    Serial.println("Fan High.");
  }
  else if (rpm > 2200 && temp <= 130)
  {
    digitalWrite(fan, LOW);
  }
  if (rpm < 1000)
  {
    digitalWrite(fan, LOW);
  }

  //////////////////////cylinder deactivation/////////////////////
  if (rpm > 10500 && digitalRead(neutralSwitch) == 0)
  {
    digitalWrite(fuelRelay, LOW);
  }
  else
  {
    digitalWrite(fuelRelay, HIGH);
  }
}
