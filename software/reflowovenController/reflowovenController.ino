#include <MsTimer2.h>
#include <PID_v1.h>
#include <SPI.h>
//#include <LiquidCrystal.h>

#define RELAYPIN 5
#define SLAVE 10

// State
#define READY 0
#define RISE_FOR_PREHEAT 1
#define PREHEAT 2
#define RISE_FOR_PEAK 3
#define PEAK 4
#define COOLDOWN 5

char state = 0;

unsigned long time = 0;
double preheatTemperature = 150;
int preheatTime = 90; /* sec */
double heatTemperature = 210;
int heatTime = 30; /* sec */

double SetPoint, Output, Input;
int windowSize = 2000;
unsigned long windowStartTime;

PID myPID(&Input, &Output, &SetPoint, 100, 0.025, 20, DIRECT);

int heaterOn = 0; // for debug

void setup() {
  state = RISE_FOR_PREHEAT;

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, LOW);
  pinMode(SLAVE, OUTPUT);
  digitalWrite(SLAVE, HIGH);

  windowStartTime = millis();
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTime(10);
  myPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE0);
  
  MsTimer2::set(1000, countDown);
  MsTimer2::start();
}

void loop() {
  double currentTemperature = getTemperature();
  Output = 0;

  switch (state) {
    case READY: // wait for start-switch
      break;
    case RISE_FOR_PREHEAT:
      SetPoint = preheatTemperature;
      keepHeat(currentTemperature);
      if (currentTemperature >= SetPoint) {
        state = PREHEAT;
      }
      break;
    case PREHEAT: // keep preheat time. main logic is countDown()
      //MsTimer2::start();
      keepHeat(currentTemperature);
      break;
    case RISE_FOR_PEAK:
      //MsTimer2::stop();
      SetPoint = heatTemperature;
      myPID.SetTunings(300, 0.05, 100);
      keepHeat(currentTemperature);
      if (currentTemperature >= SetPoint) {
        state = PEAK;
      }
      break;
    case PEAK: // keep peak time. main logic is countDown()
      //MsTimer2::start();
      keepHeat(currentTemperature);
      break;
    case COOLDOWN: //
      MsTimer2::stop();
      break;
  }

  char str[32];
  char temp[16];
  char temp2[16];
  char temp3[16];
  sprintf(str, "%ld, %s, %s, %s, %s, %d",
          time,
          dtostrf(currentTemperature, 5, 2, temp),
          dtostrf(Output, 5, 2, temp2),
          dtostrf(millis() - windowStartTime, 5, 2, temp3),
          heaterOn ? "ON" : "OFF",
          state);
  Serial.println(str);

  time++;
}

double getTemperature()
{
  unsigned int thermocouple; // 14-Bit Thermocouple Temperature Data + 2-Bit
  unsigned int internal; // 12-Bit Internal Temperature Data + 4-Bit

  digitalWrite(SLAVE, LOW);  //  Enable the chip
  thermocouple = (unsigned int)SPI.transfer(0x00) << 8;  //  Read high byte thermocouple
  thermocouple |= (unsigned int)SPI.transfer(0x00);  //  Read low byte thermocouple
  internal = (unsigned int)SPI.transfer(0x00) << 8;  //  Read high byte internal
  internal |= (unsigned int)SPI.transfer(0x00);  //  Read low byte internal
  digitalWrite(SLAVE, HIGH);  //  Disable the chip

  if ((thermocouple & 0x0001) != 0) {
    Serial.println("ERROR: ");
  } else {
    if ((thermocouple & 0x8000) == 0) { // above 0 Degrees Celsius
      return (thermocouple >> 2) * 0.25;
    } else {  // below zero
      return (0x3fff - (thermocouple >> 2) + 1)  * -0.25;
    }
  }
}

void keepHeat(double currentTemperature)
{
  Input = currentTemperature;
  myPID.Compute();

  if ((millis() - windowStartTime) > windowSize) {
    windowStartTime += windowSize;
  }

  if (Output > (millis() - windowStartTime)) {
    heaterOn = 1;
    digitalWrite(RELAYPIN, HIGH);
  } else {
    heaterOn = 0;
    digitalWrite(RELAYPIN, LOW);
  }
}

void countDown()
{
  // keep "SetPoint" temperature until preheatTime/heatTime
  switch (state) {
    case PREHEAT:
      if (preheatTime <= 0) {
        state = RISE_FOR_PEAK;
      }
      preheatTime--;
      //Serial.println(preheatTime);
      break;
    case PEAK:
      if (heatTime <= 0) {
        state = COOLDOWN;
      }
      heatTime--;
      //Serial.println(heatTime);
      break;
  }
}

