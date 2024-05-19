#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>

// Standard I2C pins on Arduino Mega
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Define BME280 connection via SPI
#define BME_SCK 26
#define BME_MISO 28
#define BME_MOSI 27
#define BME_CS 29
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

// Pin Definitions for sensors and SD card
#define LDR_PIN_1 A0
#define LDR_PIN_2 A1
#define LDR_PIN_3 A2
#define LDR_PIN_4 A3
#define MAX471_SOLAR_PIN_V A4
#define MAX471_SOLAR_PIN_I A5
#define MAX471_BATTERY_PIN_V A6
#define MAX471_BATTERY_PIN_I A7
#define SD_CS_PIN 9

// Motor pins for solar tracking
#define MOTOR_A_ENA 2
#define MOTOR_A_IN1 3
#define MOTOR_A_IN2 4
#define MOTOR_B_ENB 5
#define MOTOR_B_IN3 6
#define MOTOR_B_IN4 7

const float MAX471_SOLAR_VOLTAGE_CORRECTION_FACTOR = 5.0;
const float MAX471_SOLAR_CURRENT_CORRECTION_FACTOR = 1.0;
const float MAX471_BATTERY_VOLTAGE_CORRECTION_FACTOR = 5.0;
const float MAX471_BATTERY_CURRENT_CORRECTION_FACTOR = 1.0;

RTC_DS3231 rtc;
File dataFile;

unsigned long lastDisplayUpdate = 0;
const long displayInterval = 5000; // Time between display updates in milliseconds
const long trackingInterval = 30000; // Time between tracking adjustments in milliseconds
unsigned long lastTrackingTime = 0;

int displayPage = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  SPI.begin();

  // Initialize BME280 sensor
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // Initialize SD Card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  } else {
    Serial.println("SD Card initialized successfully.");
  }

  // Simplified filename handling
  char filename[] = "data_log.csv"; // Simpler, static filename
  if (!SD.exists(filename)) {
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Date,Time,LDR1,LDR2,LDR3,LDR4,LDR_Avg,SolarVoltage,SolarCurrent,SolarPower,BatteryVoltage,BatteryCurrent,BatteryPower,Temperature,Humidity,Pressure");
      dataFile.close();
      Serial.println("File created and header written successfully.");
    } else {
      Serial.println("Failed to create file on SD.");
      while (1); // Halt on failure
    }
  }

  initializeMotors();
  Serial.println("Setup complete. System is ready.");
}
void loop() {
  static unsigned long previousMillis = 0;
  const long interval = 1000; // Interval for sensor monitoring and data logging (10 seconds)

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    monitorAndLogData();
  }

  if (currentMillis - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = currentMillis;
    updateLCD();
  }

  if (currentMillis - lastTrackingTime >= trackingInterval) {
    lastTrackingTime = currentMillis;
    performSolarTracking();
  }
}

void initializeSDCard() {
  DateTime now = rtc.now();
  char filename[20];
  sprintf(filename, "data%02d%02d%02d.csv", now.year() % 100, now.month(), now.day());
  if (!SD.exists(filename)) {
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Date,Time,LDR1,LDR2,LDR3,LDR4,LDR_Avg,SolarVoltage,SolarCurrent,SolarPower,BatteryVoltage,BatteryCurrent,BatteryPower,Temperature,Humidity,Pressure");
      dataFile.close();
    } else {
      Serial.print("Failed to create '");
      Serial.print(filename);
      Serial.println("' on SD.");
      while (1); // Halt on failure
    }
  }
}

void initializeMotors() {
  pinMode(MOTOR_A_ENA, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_ENB, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  analogWrite(MOTOR_A_ENA, 0);
  analogWrite(MOTOR_B_ENB, 0);
  Serial.println("Motors initialized.");
}

void performSolarTracking() {
  int ldr1 = analogRead(LDR_PIN_1);
  int ldr2 = analogRead(LDR_PIN_2);
  int ldr3 = analogRead(LDR_PIN_3);
  int ldr4 = analogRead(LDR_PIN_4);

  int errorX = ldr1 - ldr2;
  int errorY = ldr3 - ldr4;

  adjustMotor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_ENA, errorX);
  adjustMotor(MOTOR_B_IN3, MOTOR_B_IN4, MOTOR_B_ENB, errorY);
}

void adjustMotor(int pinForward, int pinBackward, int pinEnable, int error) {
  if (abs(error) > 50) { // Adjust threshold as needed
    digitalWrite(pinForward, error > 0 ? HIGH : LOW);
    digitalWrite(pinBackward, error > 0 ? LOW : HIGH);
    analogWrite(pinEnable, min(abs(error), 255));
  } else {
    digitalWrite(pinForward, LOW);
    digitalWrite(pinBackward, LOW);
    analogWrite(pinEnable, 0);
  }
}

void monitorAndLogData() {
  DateTime now = rtc.now();
  String dateStamp = formatTimeOrDate(now.year()) + "-" + formatTimeOrDate(now.month()) + "-" + formatTimeOrDate(now.day());
  String timeStamp = formatTimeOrDate(now.hour()) + ":" + formatTimeOrDate(now.minute()) + ":" + formatTimeOrDate(now.second());

  float temp = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;
  int ldr1 = analogRead(LDR_PIN_1);
  int ldr2 = analogRead(LDR_PIN_2);
  int ldr3 = analogRead(LDR_PIN_3);
  int ldr4 = analogRead(LDR_PIN_4);
  int ldrAvg = (ldr1 + ldr2 + ldr3 + ldr4) / 4;

  float solarVoltage = analogRead(MAX471_SOLAR_PIN_V) * MAX471_SOLAR_VOLTAGE_CORRECTION_FACTOR;
  float solarCurrent = analogRead(MAX471_SOLAR_PIN_I) * MAX471_SOLAR_CURRENT_CORRECTION_FACTOR;
  float solarPower = solarVoltage * solarCurrent;
  float batteryVoltage = analogRead(MAX471_BATTERY_PIN_V) * MAX471_BATTERY_VOLTAGE_CORRECTION_FACTOR;
  float batteryCurrent = analogRead(MAX471_BATTERY_PIN_I) * MAX471_BATTERY_CURRENT_CORRECTION_FACTOR;
  float batteryPower = batteryVoltage * batteryCurrent;

  logData(dateStamp, timeStamp, ldr1, ldr2, ldr3, ldr4, ldrAvg, solarVoltage, solarCurrent, solarPower, batteryVoltage, batteryCurrent, batteryPower, temp, humidity, pressure);
}

void logData(String dateStamp, String timeStamp, int ldr1, int ldr2, int ldr3, int ldr4, int ldrAvg, float solarVoltage, float solarCurrent, float solarPower, float batteryVoltage, float batteryCurrent, float batteryPower, float temp, float humidity, float pressure) {
  // Use FILE_WRITE to append to the file if it exists; this does not truncate existing content.
  dataFile = SD.open("data_log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(dateStamp + "," + timeStamp + ",");
    dataFile.print(ldr1 + ",");
    dataFile.print(ldr2 + ",");
    dataFile.print(ldr3 + ",");
    dataFile.print(ldr4 + ",");
    dataFile.print(ldrAvg + ",");
    dataFile.print(solarVoltage, 2);
    dataFile.print(",");
    dataFile.print(solarCurrent, 2);
    dataFile.print(",");
    dataFile.print(solarPower, 2);
    dataFile.print(",");
    dataFile.print(batteryVoltage, 2);
    dataFile.print(",");
    dataFile.print(batteryCurrent, 2);
    dataFile.print(",");
    dataFile.println(batteryPower, 2);
    dataFile.print(",");
    dataFile.print(temp, 1);
    dataFile.print(",");
    dataFile.print(humidity, 1);
    dataFile.print(",");
    dataFile.println(pressure, 2);
    dataFile.close();  // Close the file to ensure data is written and saved properly.
    Serial.println("Data logged to SD card.");
  } else {
    Serial.println("Error opening data file!");
  }
}

void updateLCD() {
  float temp = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F; // Convert to hPa
  int ldrAvg = (analogRead(LDR_PIN_1) + analogRead(LDR_PIN_2) + analogRead(LDR_PIN_3) + analogRead(LDR_PIN_4)) / 4;

  lcd.clear();
  switch (displayPage) {
    case 0:
      lcd.print("Temp: ");
      lcd.print(temp, 1);
      lcd.print(" C");
      lcd.setCursor(0, 1);
      lcd.print("Hum: ");
      lcd.print(humidity, 1);
      lcd.print(" %");
      break;
    case 1:
      lcd.print("Pressure: ");
      lcd.print(pressure, 2);
      lcd.print(" hPa");
      lcd.setCursor(0, 1);
      lcd.print("LDR Avg: ");
      lcd.print(ldrAvg);
      break;
    default:
      break;
  }

  displayPage = (displayPage + 1) % 2; // Cycle through pages
}

String formatTimeOrDate(int value) {
  if (value < 10) {
    return "0" + String(value);
  }
  return String(value);
}
