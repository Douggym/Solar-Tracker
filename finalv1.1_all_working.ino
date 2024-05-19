// Include necessary libraries
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

// Pin assignments
const byte LDR_PIN_1 = A0;
const byte LDR_PIN_2 = A1;
const byte LDR_PIN_3 = A2;
const byte LDR_PIN_4 = A3;
const byte MAX471_SOLAR_PIN_V = A4;
const byte MAX471_SOLAR_PIN_I = A5;
const byte MAX471_BATTERY_PIN_V = A6;
const byte MAX471_BATTERY_PIN_I = A7;
const byte SD_CS_PIN = 53;
const byte FIXED_POSITION_SWITCH_PIN = 40;
const byte MOTOR_A_ENA = 1;
const byte MOTOR_A_IN1 = 2;
const byte MOTOR_A_IN2 = 3;
const byte MOTOR_B_IN3 = 5;
const byte MOTOR_B_IN4 = 6;
const byte MOTOR_B_ENB = 7;
const byte LIMIT_SWITCH_X_POS_PIN = 22;
const byte LIMIT_SWITCH_X_NEG_PIN = 23;
const byte LIMIT_SWITCH_Y_POS_PIN = 24;
const byte LIMIT_SWITCH_Y_NEG_PIN = 25;

// Timing and control constants
unsigned long previousMillis = 0;
unsigned long sensorMillis = 0;
unsigned long trackStartMillis = 0;
const long interval = 3000;   // Interval for operations
const long sensorInterval = 1000;
const long trackInterval = 300000; // Tracking interval in milliseconds
const unsigned int DEBOUNCE_DELAY = 50; // Debouncing delay for switches

// System state management
int currentPage = 0; // Current LCD display page
bool trackingEnabled = true; // Is tracking active
const float LOW_BATTERY_THRESHOLD = 0.0; // Low battery threshold
bool errorDetected = false; // Flag to detect errors
String errorMessage = ""; // Error message string

// RTC management
RTC_DS3231 rtc;

bool isHeaderWritten() {
  // Open the data file
  File dataFile = SD.open("datalog.csv");
  if (dataFile) {
    // Check if the file is empty
    bool isEmpty = dataFile.size() == 0;
    dataFile.close();
    return !isEmpty;
  }
  return false;
}

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

  if (!SD.begin(SD_CS_PIN)) {
    lcdErrorDisplay("SD Card Error");
  }

  initializePins();
}

void loop() {
  unsigned long currentMillis = millis();
  manageTimers(currentMillis);
  manageTrackingSwitch();

  static unsigned long lastDisplayUpdate = 0;
  if (currentMillis - lastDisplayUpdate > 5000) {
    displaySensorData();
    lastDisplayUpdate = currentMillis;
  }
}

void manageTrackingSwitch() {
  static bool lastSwitchState = HIGH;
  bool currentSwitchState = digitalRead(FIXED_POSITION_SWITCH_PIN);

  if (currentSwitchState != lastSwitchState) {
    delay(DEBOUNCE_DELAY);
    lastSwitchState = currentSwitchState;
    trackingEnabled = !trackingEnabled;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(trackingEnabled ? "Tracking On" : "Fixed Position");
  }
}

void initializePins() {
  pinMode(FIXED_POSITION_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A_ENA, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_ENB, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  pinMode(LIMIT_SWITCH_X_POS_PIN, INPUT);
  pinMode(LIMIT_SWITCH_X_NEG_PIN, INPUT);
  pinMode(LIMIT_SWITCH_Y_POS_PIN, INPUT);
  pinMode(LIMIT_SWITCH_Y_NEG_PIN, INPUT);
  Serial.println("All pins initialized.");
}

void lcdErrorDisplay(String error) {
  Serial.println(error);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(error);
  while (true) delay(1);
}

void manageTimers(unsigned long currentMillis) {
  DateTime now = rtc.now();
  int currentHour = now.hour();
  bool isNightTime = (currentHour >= 23 || currentHour < 5);

  if (!isNightTime) {
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      logData(now);
    }
    if (currentMillis - sensorMillis >= sensorInterval) {
      sensorMillis = currentMillis;
      monitorSensors();
    }
    if (trackingEnabled && currentMillis - trackStartMillis >= trackInterval) {
      trackStartMillis = currentMillis;
      performSolarTracking();
    }
  } else {
    digitalWrite(MOTOR_A_ENA, LOW);
    digitalWrite(MOTOR_B_ENB, LOW);
    Serial.println("Nighttime - Motors off.");
  }
}

void logData(DateTime now) {
  // Example: Logging to SD card (ensure your data structure and format)

  // Open the data file
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (dataFile) {
    // Write header if it's the first time
    if (!isHeaderWritten()) {
      dataFile.println("Timestamp, Solar Voltage (V), Solar Current (A), Battery Voltage (V), Battery Current (A), Temperature (C), Humidity (%), Pressure (hPa), LDR1, LDR2, LDR3, LDR4, Solar Power (W), Battery Power (W)");
    }

    // Read sensor values and calculate solar/battery power
    // (same as before)

    // Print sensor values to serial monitor
    // (same as before)

    // Write sensor values to the data file
    // (same as before)

    dataFile.close();
    Serial.println("Data successfully logged.");
  } else {
    Serial.println("Error opening datalog.csv");
  }
}

void monitorSensors() {
  // Just an example, add actual sensor monitoring logic
  float batteryVoltage = readCompensatedVoltage(MAX471_BATTERY_PIN_V, 0.95);
  if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
    Serial.println("Low battery voltage detected!");
    errorDetected = true;
    errorMessage = "Low battery voltage detected!";
  }
}

void performSolarTracking() {
  int ldr1 = analogRead(LDR_PIN_1);
  int ldr2 = analogRead(LDR_PIN_2);
  int ldr3 = analogRead(LDR_PIN_3);
  int ldr4 = analogRead(LDR_PIN_4);

  int errorX = ldr1 - ldr2;
  int errorY = ldr3 - ldr4;

  controlMotors(errorX, errorY);
}

void controlMotors(int errorX, int errorY) {
  // Simple proportional control, expand to PID for more accuracy
  int motorSpeedX = constrain(errorX * 2, 0, 255);
  int motorSpeedY = constrain(errorY * 2, 0, 255);
  setMotorDirection(MOTOR_A_IN1, MOTOR_A_IN2, errorX > 0);
  setMotorDirection(MOTOR_B_IN3, MOTOR_B_IN4, errorY > 0);
  analogWrite(MOTOR_A_ENA, motorSpeedX);
  analogWrite(MOTOR_B_ENB, motorSpeedY);
}

void setMotorDirection(byte motorPinPos, byte motorPinNeg, bool direction) {
  digitalWrite(motorPinPos, direction ? HIGH : LOW);
  digitalWrite(motorPinNeg, !direction ? HIGH : LOW);
}

void displaySensorData() {
  DateTime now = rtc.now();
  lcd.clear();
  switch (currentPage) {
    case 0:
      lcd.print("Date: ");
      lcd.print(now.toString("YYYY-MM-DD"));
      lcd.setCursor(0, 1);
      lcd.print("Time: ");
      lcd.print(now.toString("hh:mm:ss"));
      lcd.setCursor(0, 2);
      lcd.print(trackingEnabled ? "Tracking On" : "Fixed Position");
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("Solar V: ");
      lcd.print(readCompensatedVoltage(MAX471_SOLAR_PIN_V, 1.05), 2);
      lcd.print(" V");
      lcd.setCursor(0, 1);
      lcd.print("Solar I: ");
      lcd.print(readCompensatedCurrent(MAX471_SOLAR_PIN_I, 1.10), 2);
      lcd.print(" A");
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("Battery V: ");
      lcd.print(readCompensatedVoltage(MAX471_BATTERY_PIN_V, 0.95), 2);
      lcd.print(" V");
      lcd.setCursor(0, 1);
      lcd.print("Battery I: ");
      lcd.print(readCompensatedCurrent(MAX471_BATTERY_PIN_I, 0.90), 2);
      lcd.print(" A");
      break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(bme.readTemperature(), 1);
      lcd.print(" C");
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(bme.readHumidity(), 1);
      lcd.print(" %");
      lcd.setCursor(0, 2);
      lcd.print("Pressure: ");
      lcd.print(bme.readPressure() / 100.0F, 1);
      lcd.print(" hPa");
      break;
    case 4:
      lcd.setCursor(0, 0);
      lcd.print("LDR1: ");
      lcd.print(analogRead(LDR_PIN_1));
      lcd.setCursor(0, 1);
      lcd.print("LDR2: ");
      lcd.print(analogRead(LDR_PIN_2));
      lcd.setCursor(0, 2);
      lcd.print("LDR3: ");
      lcd.print(analogRead(LDR_PIN_3));
      lcd.setCursor(0, 3);
      lcd.print("LDR4: ");
      lcd.print(analogRead(LDR_PIN_4));
      break;
  }
  currentPage = (currentPage + 1) % 5; // Cycle through pages
}

float readCompensatedVoltage(int pin, float compensationFactor) {
  float rawVoltage = analogRead(pin) * (5.0 / 1023.0); // Convert analog reading to voltage
  return rawVoltage * compensationFactor; // Apply compensation factor
}

float readCompensatedCurrent(int pin, float compensationFactor) {
  float rawCurrent = analogRead(pin) * (5.0 / 1023.0); // Convert analog reading to current
  return rawCurrent * compensationFactor; // Apply compensation factor
}
