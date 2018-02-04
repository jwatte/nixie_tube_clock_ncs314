
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "OneWire.h"
#include "Debounce.h"
#include "Tone.h"


#define INIT_ANIMATE_COUNT 3

const byte LEpin = 10; //pin Latch Enabled data accepted while HI level
const byte DHVpin = 5; // off/on MAX1771 Driver  Hight Voltage(DHV) 110-220V
const byte RedLedPin = 9; //MCU WDM output for red LEDs 9-g
const byte GreenLedPin = 6; //MCU WDM output for green LEDs 6-b
const byte BlueLedPin = 3; //MCU WDM output for blue LEDs 3-r
const byte pinSet = A0;
const byte pinUp = A2;
const byte pinDown = A1;
const byte pinBuzzer = 2;
const byte pinUpperDots = 12; //HIGH value light a dots
const byte pinLowerDots = 8; //HIGH value light a dots
const word fpsLimit = 16666; // 1/60*1.000.000 //limit maximum refresh rate on 60 fps
const byte pinTemp = 7;
bool RTC_present;
float gTemperature;

OneWire ds(pinTemp);
bool TempPresent = false;

char displayString[7] = "123456";

#define DS1307_ADDRESS 0x68
byte zero = 0x00; //workaround for issue #527
int RTC_hours, RTC_minutes, RTC_seconds, RTC_day, RTC_month, RTC_year, RTC_day_of_week;

void setRTCDateTime(byte h, byte m, byte s, byte d, byte mon, byte y, byte w);

Debounce setButton(pinSet);
Debounce upButton(pinUp);
Debounce downButton(pinDown);

Tone tone1;

enum Mode {
  DisplayTime,
  DisplayDate,
  DisplayTemp,
  ModeEnd
};

enum SetMode {
  SetModeNone,
  SetModeHour,
  SetModeMinute,
  SetModeSecond,
  SetModeYear,
  SetModeMonth,
  SetModeDay,
  SetModeEnd
};

#define UpperDotsMask 0x80000000                                                                                        
#define LowerDotsMask 0x40000000                                                                                        
                                                                                                                        

/*******************************************************************************************************
  Init Programm
*******************************************************************************************************/
void setup()
{
  digitalWrite(DHVpin, LOW);    // off MAX1771 Driver  Hight Voltage(DHV) 110-220V
  Wire.begin();

  Serial.begin(115200);

  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);
  pinMode(BlueLedPin, OUTPUT);

  pinMode(pinBuzzer, OUTPUT);
  tone1.begin(pinBuzzer);

  pinMode(LEpin, OUTPUT);
  pinMode(DHVpin, OUTPUT);

  // SPI setup

  SPI.begin(); //
  SPI.setDataMode (SPI_MODE2); // Mode 3 SPI
  SPI.setClockDivider(SPI_CLOCK_DIV8); // SCK = 16MHz/128= 125kHz

  //buttons pins inits
  pinMode(pinSet,  INPUT_PULLUP);
  pinMode(pinUp,  INPUT_PULLUP);
  pinMode(pinDown,  INPUT_PULLUP);

  doTest();
  getRTCTime();

  byte prevSeconds = RTC_seconds;
  unsigned long RTC_ReadingStartTime = millis();
  RTC_present = true;
  while (prevSeconds == RTC_seconds)
  {
    getRTCTime();
    if ((millis() - RTC_ReadingStartTime) > 3000)
    {
      Serial.println(F("Warning! RTC DON'T RESPOND!"));
      RTC_present = false;
      break;
    }
  }
  generateDisplay();
  newMode(DisplayTime);
  digitalWrite(DHVpin, LOW); // off MAX1771 Driver  Hight Voltage(DHV) 110-220V
  digitalWrite(DHVpin, HIGH); // on MAX1771 Driver  Hight Voltage(DHV) 110-220V
}


Mode mode = DisplayTime;
uint32_t modeTime;
const uint32_t modeDuration[]= { 25000, 7000, 7000 };
byte lastSec;
Mode oldDisplayMode = DisplayTime;
SetMode setMode = SetModeNone;
uint32_t setModeTime = 0;

const uint32_t displayBits[10] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512
};

char displayed[6];
uint32_t lastSwapTime;
bool animating = false;
int animatecount = INIT_ANIMATE_COUNT;
uint32_t dots = 0;

#define MIN_TEMP 66
#define MED_TEMP 71
#define HI_TEMP 76

void writeTempColor() {
  unsigned char red = 0;
  unsigned char green = 0;
  unsigned char blue = 0;
  if (gTemperature < MIN_TEMP) {
    blue = 255;
  } else if (gTemperature < MED_TEMP) {
    blue = 255 * (MED_TEMP - gTemperature) / (MED_TEMP - MIN_TEMP);
    green = 255 * (gTemperature - 64) / (MED_TEMP - MIN_TEMP);
  } else if (gTemperature < HI_TEMP) {
    green = 255 * (HI_TEMP - gTemperature) / (HI_TEMP - MED_TEMP);
    red = 255 * (gTemperature - MED_TEMP) / (HI_TEMP - MED_TEMP);
  } else {
    red = 255;
  }
  analogWrite(RedLedPin, red>>2);
  analogWrite(GreenLedPin, green>>3);
  analogWrite(BlueLedPin, blue>>2);
}

void newMode(Mode nm) {
  if (setMode != SetModeNone) {
    getRTCTime();
  }
  animating = true;
  animatecount = INIT_ANIMATE_COUNT;
  switch (nm) {
    case DisplayTime:
      dots = 0;
      break;
    case DisplayDate:
      dots = 0xc0000000UL;
      break;
    case DisplayTemp:
      dots = 0x40000000UL;
      break;
  }
}

void generateDisplay() {
  if ((mode == DisplayTime && setMode == SetModeNone) || setMode == SetModeHour || setMode == SetModeMinute || setMode == SetModeSecond) {
    sprintf(displayString, "%02d%02d%02d", RTC_hours, RTC_minutes, RTC_seconds);
  } else if (mode == DisplayDate || setMode == SetModeYear || setMode == SetModeMonth || setMode == SetModeDay) {
    sprintf(displayString, "%02d%02d%02d", RTC_year, RTC_month, RTC_day);
  } else {
    float f = gTemperature;
    sprintf(displayString, "%04d%02d", (int)f, (int)(100*(f - (int)f)));
  }
}

uint32_t getSetMode(int dig) {
  uint32_t ret = 0x3ff;
  if (!(millis() & 256)) {
    ret = 0;
  }
  switch (dig) {
    case 0: case 1: if (setMode == SetModeYear || setMode == SetModeHour) return ret; break;
    case 2: case 3: if (setMode == SetModeMonth || setMode == SetModeMinute) return ret; break;
    case 4: case 5: if (setMode == SetModeDay || setMode == SetModeSecond) return ret; break;
  }
  return 0x3ff;
}

void doDisplay(char *ds) {

  if (!animating && !memcmp(displayed, ds, 6) && setMode == SetModeNone) {
    return;
  }
  if (animating && setMode == SetModeNone) {
    uint32_t now = millis();
    if (now - lastSwapTime < 100) {
      return;
    }
    lastSwapTime = now;
    bool differ = false;
    if (animatecount > 0) {
      --animatecount;
      differ = true;
    }
    for (int i = 0; i != 6; ++i) {
      if (differ) {
        displayed[i] = (rand() % 10) + '0';
      } else if (displayed[i] != ds[i]) {
        displayed[i] = ((displayed[i] - '0') + 7) % 10 + '0';
      }
    }
    if (!memcmp(displayed, ds, 6)) {
      animating = false;
      animatecount = INIT_ANIMATE_COUNT;
    }
  } else {
    memcpy(displayed, ds, 6);
    lastSwapTime = millis();
  }
  
  uint32_t dv = 0;

  digitalWrite(LEpin, LOW);
  
  dv = (displayBits[displayed[5]-'0'] & getSetMode(5)) << 20;
  dv |= (displayBits[displayed[4]-'0'] & getSetMode(4)) << 10;
  dv |= (displayBits[displayed[3]-'0'] & getSetMode(3));
  dv |= dots;

  SPI.transfer(dv>>24);                                                                                              
  SPI.transfer(dv>>16);                                                                                              
  SPI.transfer(dv>>8);                                                                                               
  SPI.transfer(dv);                                                                                                  

  dv = (displayBits[displayed[2]-'0'] & getSetMode(2)) << 20;
  dv |= (displayBits[displayed[1]-'0'] & getSetMode(1)) << 10;
  dv |= (displayBits[displayed[0]-'0'] & getSetMode(0));
  dv |= dots;

  SPI.transfer(dv>>24);                                                                                              
  SPI.transfer(dv>>16);                                                                                              
  SPI.transfer(dv>>8);                                                                                               
  SPI.transfer(dv);                                                                                                  

  digitalWrite(LEpin, HIGH);                                                                                            
}

void loop() {

  uint32_t now = millis();
  byte newSec = (now / 1000);
  if (newSec != lastSec) {
    gTemperature = getTemperature(true);
    writeTempColor();
    lastSec = newSec;
    ++RTC_seconds;
    if (RTC_seconds == 60) {
      RTC_seconds = 0;
      ++RTC_minutes;
      if (RTC_minutes == 60) {
        RTC_minutes = 0;
        ++RTC_hours;
        if (RTC_hours == 24) {
          RTC_hours = 0;
          getRTCTime();
        }
      }
    }
    generateDisplay();
  }

  if (now - modeTime >= modeDuration[mode]) {
    mode = (Mode)(mode + 1);
    if (mode == ModeEnd) {
      mode = DisplayTime;
    }
    newMode(mode);
    modeTime = now;
  }

  setButton.update();
  upButton.update();
  downButton.update();

  if (setButton.clicked()) {
    tone1.play(1000, 70);
    setModeTime = millis();
    if (setButton.longclick() || (setMode + 1 == SetModeEnd)) {
      setMode = SetModeNone;
      setRTCDateTime(RTC_hours, RTC_minutes, RTC_seconds, RTC_day, RTC_month, RTC_year, 1); //  weekday is wrong!
      newMode(DisplayTime);
    } else {
      setMode = (SetMode)(setMode + 1);
    }
  }
  if (setMode != SetModeNone) {
    if (now - setModeTime > 10000) {
      setMode = SetModeNone;
      getRTCTime();
      newMode(DisplayTime);
    } else {
      int delta = 0;
      if (upButton.clicked()) {
        delta = 1;
        tone1.play(1500, 70);
      }
      if (downButton.clicked()) {
        delta = -1;
        tone1.play(750, 70);
      }
      switch (setMode) {
        case SetModeHour:   RTC_hours   = wrap(RTC_hours+delta, 0, 23); break;
        case SetModeMinute: RTC_minutes = wrap(RTC_minutes+delta, 0, 59); break;
        case SetModeSecond: RTC_seconds = wrap(RTC_seconds+delta, 0, 59); break;
        case SetModeYear:   RTC_year    = wrap(RTC_year+delta, 0, 99); break;
        case SetModeMonth:  RTC_month   = wrap(RTC_month+delta, 1, 12); break;
        case SetModeDay:    RTC_day     = wrap(RTC_day+delta, 1, monthlength(RTC_year, RTC_month)); break;
      }
    }
  }

  doDisplay(displayString);
  
  uint32_t then = millis();
  if (then - now < 8) {
    delay(8 - (then - now));
  }
}

int monthlength(int year, int month) {
  switch (month) {
    case 1: case 3: case 5: case 7: case 8: case 10: case 12: return 31;
    case 2: return (year & 3) ? 28 : 29;  //  assume 00 is 2000 which is also leap year
    default: return 30;
  }
}

int wrap(int arg, int from, int to) {
  if (arg < from) return to;
  if (arg > to) return from;
  return arg;
}

void doTest()
{
  Serial.print(F("Firmware version: "));
  Serial.println(__DATE__ __TIME__);

  analogWrite(RedLedPin, 255);
  delay(400);
  analogWrite(RedLedPin, 0);
  analogWrite(GreenLedPin, 255);
  delay(400);
  analogWrite(GreenLedPin, 0);
  analogWrite(BlueLedPin, 255);
  delay(400);
  analogWrite(BlueLedPin, 0);

  digitalWrite(DHVpin, HIGH);

  for (char c = '0'; c != (char)('0'+10); ++c) {
    char str[7] = { c, c, c, c, c, c };
    doDisplay(str);
    delay(200);
  };
}

void setRTCDateTime(byte h, byte m, byte s, byte d, byte mon, byte y, byte w)
{
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(zero); //  write pointer

  Wire.write(decToBcd(s));
  Wire.write(decToBcd(m));
  Wire.write(decToBcd(h));
  Wire.write(decToBcd(w));
  Wire.write(decToBcd(d));
  Wire.write(decToBcd(mon));
  Wire.write(decToBcd(y));

  Wire.write(zero); //start
  Wire.endTransmission();
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(zero);
  Wire.endTransmission();
}

byte decToBcd(byte val) {
  // Convert normal decimal numbers to binary coded decimal
  return ( (val / 10 * 16) + (val % 10) );
}

byte bcdToDec(byte val)  {
  // Convert binary coded decimal to normal decimal numbers
  return ( (val / 16 * 10) + (val % 16) );
}

void getRTCTime()
{
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(zero);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 7);

  RTC_seconds = bcdToDec(Wire.read());
  RTC_minutes = bcdToDec(Wire.read());
  RTC_hours = bcdToDec(Wire.read() & 0b111111); //24 hour time
  RTC_day_of_week = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
  RTC_day = bcdToDec(Wire.read());
  RTC_month = bcdToDec(Wire.read());
  RTC_year = bcdToDec(Wire.read());
}

float getTemperature (boolean bTempFormat)
{
  byte TempRawData[2];
  ds.reset();
  ds.write(0xCC); //skip ROM command
  ds.write(0x44); //send make convert to all devices
  ds.reset();
  ds.write(0xCC); //skip ROM command
  ds.write(0xBE); //send request to all devices

  TempRawData[0] = ds.read();
  TempRawData[1] = ds.read();
  int16_t raw = (TempRawData[1] << 8) | TempRawData[0];
  float celsius = (float)raw / 16.0;
  return (bTempFormat) ? (celsius * 1.8 + 32.0) : celsius;
}

