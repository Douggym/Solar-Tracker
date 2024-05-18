#include <ACS712.h>
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>

// LCD Display configuration using I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Define BME280 connection via SPI
#define BME_SCK 48
#define BME_MISO 47
#define BME_MOSI 49
#define BME_CS 46
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

// Voltage sensor pins
const byte SOLAR_PIN_V = A4;   // Solar panel voltage sensor
const byte BATTERY_PIN_V = A6; // Battery voltage sensor

// ACS712 objects for current sensing
ACS712 solarSensor(ACS712_30A, A5); // ACS712 object for solar current sensing
ACS712 batterySensor(ACS712_30A, A7); // ACS712 object for battery current sensing

const byte PHOTO_SENSOR_PINS[] = {A0, A1, A2, A3}; // Photosensitive diode pins
const byte SD_CS_PIN = 53;
const byte FIXED_POSITION_SWITCH_PIN = 40; // Kill switch / Fixed position pin

// Motor driver pins
const int ENA = 10;   // Enable Motor A
const int IN1 = 28;   // Motor A Direction 1
const int IN2 = 27;   // Motor A Direction 2
const int ENB = 8;  // Enable Motor B
const int IN3 = 30;  // Motor B Direction 1
const int IN4 = 31;   // Motor B Direction 2

int currentPage = 0; // Define globally for LCD

RTC_DS3231 rtc;
char filename[16];

// Current offsets
float solarCurrentOffset = 0.18; // Example offset value for solar current
float batteryCurrentOffset = 0.3; // Example offset value for battery current

void setup() {
  Serial.begin(9600);
  Wire.begin();
  SPI.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");

  if (!bme.begin()) {
    lcdErrorDisplay("BME280 Error");
  }

  if (!rtc.begin()) {
    lcdErrorDisplay("RTC Error");
  }
  DateTime now = rtc.now();
  if (now.year() < 2022)
  {
    Serial.println("RTC time needs to be set.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("RTC time set.");
  }
  if (!SD.begin(SD_CS_PIN)) {
    lcdErrorDisplay("SD Card Error");
  } else {
    lcd.setCursor(0, 1);
    lcd.print("SD Card Ready");
  }

  // Set up the motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(FIXED_POSITION_SWITCH_PIN, INPUT_PULLUP); // Use internal pull-up resistor

  // Initialize both motors to be stopped at startup
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  sprintf(filename, "%04d%02d%02d.csv", now.year(), now.month(), now.day());
  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println("DateTime,LDR1,LDR2,LDR3,LDR4,LDR Average,Solar Voltage,Solar Current,Battery Voltage,Battery Current,Temperature,Humidity,Pressure");
    dataFile.close();
  }
}

void loop() {
  static unsigned long lastDataLog = 0;
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastTrackingTime = 0;
  static bool isTracking = false;

  unsigned long currentTime = millis();

  if (currentTime - lastDataLog >= 1800000) { // Log every 5 min
    logData();
    lastDataLog = currentTime;
  }

  if (currentTime - lastDisplayUpdate > 5000) {  // Change display every 5 seconds
    displaySensorData();
    lastDisplayUpdate = currentTime;
    currentPage = (currentPage + 1) % 6; // Increment page, wrap around after the last page
  }

  if (digitalRead(FIXED_POSITION_SWITCH_PIN) == HIGH) {
    if (!isTracking && (currentTime - lastTrackingTime >= 1800000)) { // Start tracking every 15 min
      lastTrackingTime = currentTime;
      isTracking = true;
      trackSun();
    }
    else if (isTracking && (currentTime - lastTrackingTime >= 3000)) { // Move for 3 seconds during tracking
      isTracking = false;
      emergencyStop(); // Stop tracking after 3 seconds
    }
  } else {
    emergencyStop();  // Stop and fix position
  }
}

void trackSun() {
  unsigned long startTime = millis(); // Record the start time of tracking
  while (millis() - startTime <= 3000) { // Track for 3 seconds
    int ldrValues[4];
    for (int i = 0; i < 4; i++) {
      ldrValues[i] = analogRead(PHOTO_SENSOR_PINS[i]);
    }

    int avgVertical = ((ldrValues[0] + ldrValues[1]) / 2) - ((ldrValues[2] + ldrValues[3]) / 2);
    int avgHorizontal = ((ldrValues[0] + ldrValues[2]) / 2) - ((ldrValues[1] + ldrValues[3]) / 2);

    driveMotor(ENA, IN1, IN2, avgHorizontal, false);
    driveMotor(ENB, IN3, IN4, avgVertical, true);
  }
}

void driveMotor(int enablePin, int inPin1, int inPin2, int value, bool isVertical) {
  int sensitivityMultiplier = isVertical ? 5 : 5;
  int speed = min(abs(value) * sensitivityMultiplier, 75);
  analogWrite(enablePin, speed);

  if (value > 0) {
    digitalWrite(inPin1, HIGH);
    digitalWrite(inPin2, LOW);
  } else if (value < 0) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, HIGH);
  } else {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
    analogWrite(enablePin, 0);
  }
}

void emergencyStop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Emergency stop activated!");
}

void logData() {
  File dataFile = SD.open(filename, FILE_WRITE);
  DateTime now = rtc.now();
  if (dataFile) {
    int ldrValues[4];
    int ldrSum = 0;
    for (int i = 0; i < 4; i++) {
      ldrValues[i] = analogRead(PHOTO_SENSOR_PINS[i]);
      ldrSum += ldrValues[i];
    }
    int ldrAverage = ldrSum / 4;

    float solarVoltage = readVoltage(SOLAR_PIN_V);
    float solarCurrent = solarSensor.getCurrentDC() - solarCurrentOffset;
    float batteryVoltage = readVoltage(BATTERY_PIN_V);
    float batteryCurrent = batterySensor.getCurrentDC() - batteryCurrentOffset;
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;

    dataFile.print(now.year(), DEC);
    dataFile.print("-");
    dataFile.print(now.month(), DEC);
    dataFile.print("-");
    dataFile.print(now.day(), DEC);
    dataFile.print(" ");
    dataFile.print(now.hour(), DEC);
    dataFile.print(":");
    dataFile.print(now.minute(), DEC);
    dataFile.print(":");
    dataFile.print(now.second(), DEC);
    dataFile.print(",");

    for (int i = 0; i < 4; i++) {
      dataFile.print(ldrValues[i]);
      dataFile.print(",");
    }
    dataFile.print(ldrAverage);
    dataFile.print(",");
    dataFile.print(solarVoltage, 2);
    dataFile.print(",");
    dataFile.print(solarCurrent, 2);
    dataFile.print(",");
    dataFile.print(batteryVoltage, 2);
    dataFile.print(",");
    dataFile.print(batteryCurrent, 2);
    dataFile.print(",");
    dataFile.print(temperature, 2);
    dataFile.print(",");
    dataFile.print(humidity, 2);
    dataFile.print(",");
    dataFile.println(pressure, 2);

    dataFile.close();
  }

  // Print readings to serial monitor
  printReadingsSerial();
}

void printReadingsSerial() {
  DateTime now = rtc.now();
  int ldrValues[4];
  int ldrSum = 0;
  for (int i = 0; i < 4; i++) {
    ldrValues[i] = analogRead(PHOTO_SENSOR_PINS[i]);
    ldrSum += ldrValues[i];
  }
  int ldrAverage = ldrSum / 4;

  float solarVoltage = readVoltage(SOLAR_PIN_V);
  float solarCurrent = solarSensor.getCurrentDC() - solarCurrentOffset;
  float batteryVoltage = readVoltage(BATTERY_PIN_V);
  float batteryCurrent = batterySensor.getCurrentDC() - batteryCurrentOffset;
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;

  Serial.print("DateTime: ");
  Serial.print(now.year(), DEC);
  Serial.print("-");
  Serial.print(now.month(), DEC);
  Serial.print("-");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print(", LDR1: ");
  Serial.print(ldrValues[0]);
  Serial.print(", LDR2: ");
  Serial.print(ldrValues[1]);
  Serial.print(", LDR3: ");
  Serial.print(ldrValues[2]);
  Serial.print(", LDR4: ");
  Serial.print(ldrValues[3]);
  Serial.print(", LDR Average: ");
  Serial.print(ldrAverage);
  Serial.print(", Solar Voltage: ");
  Serial.print(solarVoltage, 2);
  Serial.print("V, Solar Current: ");
  Serial.print(solarCurrent, 2);
  Serial.print("A, Battery Voltage: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V, Battery Current: ");
  Serial.print(batteryCurrent, 2);
  Serial.print("A, Temperature: ");
  Serial.print(temperature, 2);
  Serial.print("C, Humidity: ");
  Serial.print(humidity, 2);
  Serial.print("%, Pressure: ");
  Serial.print(pressure, 2);
  Serial.println(" hPa");
}

void displaySensorData() {
  lcd.clear();
  DateTime now = rtc.now();
  Serial.print("Displaying page: ");
  Serial.println(currentPage);
  switch (currentPage) {
    case 0:  // Display date and time
      Serial.println("Displaying Date/Time");
      lcd.print("Date/Time:");
      lcd.setCursor(0, 1);
      lcd.print(now.year(), DEC);
      lcd.print("-");
      lcd.print(now.month(), DEC);
      lcd.print("-");
      lcd.print(now.day(), DEC);
      lcd.setCursor(0, 2);
      lcd.print(now.hour(), DEC);
      lcd.print(":");
      lcd.print(now.minute(), DEC);
      lcd.print(":");
      lcd.print(now.second(), DEC);
      break;
    case 1:  // Display LDR values
      Serial.println("Displaying LDR values");
      for (int i = 0; i < 4; i++) {
        int ldrReading = analogRead(PHOTO_SENSOR_PINS[i]);
        lcd.setCursor(0, i);
        lcd.print("LDR" + String(i + 1) + ": " + String(ldrReading));
      }
      break;
    case 2:  // Display solar voltage and current
      {
        Serial.println("Displaying Solar values");
        float solarVoltage = readVoltage(SOLAR_PIN_V);
        float solarCurrent = solarSensor.getCurrentDC() - solarCurrentOffset;
        lcd.print("Solar V: " + String(solarVoltage, 2) + "V");
        lcd.setCursor(0, 1);
        lcd.print("Solar I: " + String(solarCurrent, 2) + "A");
      }
      break;
    case 3:  // Display battery voltage and current
      {
        Serial.println("Displaying Battery values");
        float batteryVoltage = readVoltage(BATTERY_PIN_V);
        float batteryCurrent = batterySensor.getCurrentDC() - batteryCurrentOffset;
        lcd.print("Battery V: " + String(batteryVoltage, 2) + "V");
        lcd.setCursor(0, 1);
        lcd.print("Battery I: " + String(batteryCurrent, 2) + "A");
      }
      break;
    case 4:  // Display temperature and humidity
      {
        Serial.println("Displaying Temperature and Humidity");
        float temperature = bme.readTemperature();
        float humidity = bme.readHumidity();
        lcd.print("Temp: " + String(temperature, 2) + "C");
        lcd.setCursor(0, 1);
        lcd.print("Humidity: " + String(humidity, 2) + "%");
      }
      break;
    case 5:  // Display pressure
      {
        Serial.println("Displaying Pressure");
        float pressure = bme.readPressure() / 100.0F;
        lcd.print("Pressure:");
        lcd.setCursor(0, 1);
        lcd.print(String(pressure, 2) + " hPa");
      }
      break;
  }
}

float readVoltage(byte pin) {
  int sensorValue = analogRead(pin);
  return sensorValue * (5.0 / 1023.0) * 4.725;  // Assuming a scaling from 0-25V
}

void lcdErrorDisplay(String error) {
  Serial.println(error);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(error);
  while (true) delay(1);
}
