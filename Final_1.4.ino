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
#define BME_SCK 11
#define BME_MISO 9
#define BME_MOSI 10
#define BME_CS 8
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

// Pin assignments
const byte PHOTO_SENSOR_PINS[] = { A0, A1, A2, A3 };  // Photosensitive diode sensor pins
const byte SOLAR_PIN_V = A4;
const byte SOLAR_PIN_I = A5;
const byte BATTERY_PIN_V = A6;
const byte BATTERY_PIN_I = A7;
const byte SD_CS_PIN = 53;
const byte FIXED_POSITION_SWITCH_PIN = 40;
const byte MOTOR_A_ENA = 1;
const byte MOTOR_A_IN1 = 2;
const byte MOTOR_A_IN2 = 3;
const byte MOTOR_B_IN3 = 4;
const byte MOTOR_B_IN4 = 5;
const byte MOTOR_B_ENB = 6;
const byte LIMIT_SWITCH_X_POS_PIN = 22;
const byte LIMIT_SWITCH_X_NEG_PIN = 23;
const byte LIMIT_SWITCH_Y_POS_PIN = 24;
const byte LIMIT_SWITCH_Y_NEG_PIN = 25;

// Calibration factors for each photosensitive diode
const float CALIBRATION_FACTORS[] = {1.0, 0.95, 1.05, 0.98}; // Example factors, adjust based on your calibration

// Timing and control constants
unsigned long previousMillis = 0;
unsigned long sensorMillis = 0;
unsigned long trackStartMillis = 0;
const long interval = 30000;  // Interval for operations
const long sensorInterval = 1000;
const unsigned int DEBOUNCE_DELAY = 50;  // Debouncing delay for switches

// System state management
int currentPage = 0;                            // Current LCD display page
bool trackingEnabled = true;                    // Is tracking active
const float MIN_SOLAR_VOLTAGE_THRESHOLD = 0.0;  // Minimum solar panel voltage threshold to turn off tracking
const float LOW_BATTERY_THRESHOLD = 0.0;        // Set your desired threshold value in volts
bool errorDetected = false;                     // Flag to detect errors
String errorMessage = "";                       // Error message string

// RTC management
RTC_DS3231 rtc;

// Compensation factors
float voltageCompensationFactor = 4.681;  // Default 1.0 (no compensation)
float currentCompensationFactor = 0.571;  // Default 1.0 (no compensation)

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

  // Debugging RTC initialization
  int rtcInitResult = rtc.begin();
  Serial.print("RTC Initialization result: ");
  Serial.println(rtcInitResult);
  if (!rtcInitResult) {
    lcdErrorDisplay("RTC Error");
  }

  // Check if RTC time needs to be set
  DateTime now = rtc.now();
  if (now.year() < 2022) {
    Serial.println("RTC time needs to be set.");

    // Set the RTC time when compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    Serial.println("RTC time set.");
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
  if (currentMillis - lastDisplayUpdate > 2000) {
    displaySensorData();
    lastDisplayUpdate = currentMillis;
  }
}

void readAndCalibrateSensors(float *calibratedValues) {
    for (int i = 0; i < 4; i++) {
        int rawValue = analogRead(PHOTO_SENSOR_PINS[i]);
        calibratedValues[i] = rawValue * CALIBRATION_FACTORS[i];
    }
}

void monitorSensors() {
    float calibratedLDRValues[4];
    readAndCalibrateSensors(calibratedLDRValues);

    // Log calibrated values to Serial for calibration purposes
    Serial.println("Calibrated LDR Values:");
    for (int i = 0; i < 4; i++) {
        Serial.print("LDR");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(calibratedLDRValues[i], 2);
    }
    
    float ldrSum = 0;
    for (int i = 0; i < 4; i++) {
        ldrSum += calibratedLDRValues[i];
    }
    float ldrAverage = ldrSum / 4;

    // Your existing logic...
}

void performSolarTracking() {
    float calibratedLDRValues[4];
    readAndCalibrateSensors(calibratedLDRValues);

    int maxIndex = 0;
    for (int i = 1; i < 4; i++) {
        if (calibratedLDRValues[i] > calibratedLDRValues[maxIndex]) {
            maxIndex = i;
        }
    }

    switch (maxIndex) {
        case 0:
            setMotorDirection(MOTOR_A_IN1, MOTOR_A_IN2, true);
            setMotorDirection(MOTOR_B_IN3, MOTOR_B_IN4, true);
            break;
        case 1:
            setMotorDirection(MOTOR_A_IN1, MOTOR_A_IN2, false);
            setMotorDirection(MOTOR_B_IN3, MOTOR_B_IN4, true);
            break;
        case 2:
            setMotorDirection(MOTOR_A_IN1, MOTOR_A_IN2, true);
            setMotorDirection(MOTOR_B_IN3, MOTOR_B_IN4, false);
            break;
        case 3:
            setMotorDirection(MOTOR_A_IN1, MOTOR_A_IN2, false);
            setMotorDirection(MOTOR_B_IN3, MOTOR_B_IN4, false);
            break;
    }

    analogWrite(MOTOR_A_ENA, 20);
    analogWrite(MOTOR_B_ENB, 20);

    delay(2000);

    analogWrite(MOTOR_A_ENA, 0);
    analogWrite(MOTOR_B_ENB, 0);
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
      {
        float solarVoltage = readCompensatedVoltage(SOLAR_PIN_V, voltageCompensationFactor);
        float solarCurrent = readCompensatedCurrent(SOLAR_PIN_I, currentCompensationFactor);
        lcd.setCursor(0, 0);
        lcd.print("Solar V: ");
        lcd.print(solarVoltage, 2);
        lcd.print(" V");
        lcd.setCursor(0, 1);
        lcd.print("Solar I: ");
        lcd.print(solarCurrent, 2);
        lcd.print(" A");
        printSensorReadings(solarVoltage, solarCurrent, "Solar");
      }
      break;
    case 2:
      {
        lcd.clear();
        float batteryVoltage = readCompensatedVoltage(BATTERY_PIN_V, voltageCompensationFactor);
        float batteryCurrent = readCompensatedCurrent(BATTERY_PIN_I, currentCompensationFactor);
        lcd.setCursor(0, 0);
        lcd.print("Battery V: ");
        lcd.print(batteryVoltage, 2);
        lcd.print(" V");
        lcd.setCursor(0, 1);
        lcd.print("Battery I: ");
        lcd.print(batteryCurrent, 2);
        lcd.print(" A");
        printSensorReadings(batteryVoltage, batteryCurrent, "Battery");
      }
      break;
    case 3:
      {
        // Read averaged values from photo diodes
        float ldr1 = 0, ldr2 = 0, ldr3 = 0, ldr4 = 0;
        for (int i = 0; i < 5; i++) {
          ldr1 += analogRead(PHOTO_SENSOR_PINS[0]);
          ldr2 += analogRead(PHOTO_SENSOR_PINS[1]);
          ldr3 += analogRead(PHOTO_SENSOR_PINS[2]);
          ldr4 += analogRead(PHOTO_SENSOR_PINS[3]);
          delay(10);
        }
        ldr1 /= 5;
        ldr2 /= 5;
        ldr3 /= 5;
        ldr4 /= 5;

        lcd.setCursor(0, 0);  // Line 1
        lcd.print("LDR1: ");
        lcd.print(ldr1);
        lcd.setCursor(0, 1);  // Line 2
        lcd.print("LDR2: ");
        lcd.print(ldr2);
        lcd.setCursor(0, 2);  // Line 3
        lcd.print("LDR3: ");
        lcd.print(ldr3);
        lcd.setCursor(0, 3);  // Line 4
        lcd.print("LDR4: ");
        lcd.print(ldr4);
      }
      break;
    case 4:
      {
        // Read environmental sensor values
        float temperature = bme.readTemperature();
        float humidity = bme.readHumidity();
        float pressure = bme.readPressure() / 100.0F; // Convert to hPa
        lcd.setCursor(0, 0);  // Line 1
        lcd.print("Temp: ");
        lcd.print(temperature, 1);
        lcd.print(" C");

        // Move to the second line and print the humidity
        lcd.setCursor(0, 1);  // Line 2
        lcd.print("Humidity: ");
        lcd.print(humidity, 1);
        lcd.print("%");

        // Move to the third line and print the pressure
        lcd.setCursor(0, 2);  // Line 3
        lcd.print("Pressure: ");
        lcd.print(pressure, 1);
        lcd.print(" hPa");

        // Optionally, if you have additional data or messages to display, use the fourth line
        lcd.setCursor(0, 3);  // Line 4
        lcd.print("Data OK"); // Example message
      }
      break;
  }
  currentPage = (currentPage + 1) % 5; // Cycle through pages
}

void printSensorReadings(float voltage, float current, const char* sensorName) {
  Serial.print(sensorName);
  Serial.print(" Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V, ");
  Serial.print(sensorName);
  Serial.print(" Current: ");
  Serial.print(current, 2);
  Serial.println(" A");
}

// Function to print serial data
void printSerialData() {
  // Read sensor values
  float solarVoltage = readCompensatedVoltage(SOLAR_PIN_V, voltageCompensationFactor);
  float solarCurrent = readCompensatedCurrent(SOLAR_PIN_I, currentCompensationFactor);
  float batteryVoltage = readCompensatedVoltage(BATTERY_PIN_V, voltageCompensationFactor);
  float batteryCurrent = readCompensatedCurrent(BATTERY_PIN_I, currentCompensationFactor);
  int ldr1 = analogRead(PHOTO_SENSOR_PINS[0]);
  int ldr2 = analogRead(PHOTO_SENSOR_PINS[1]);
  int ldr3 = analogRead(PHOTO_SENSOR_PINS[2]);
  int ldr4 = analogRead(PHOTO_SENSOR_PINS[3]);
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F; // Convert to hPa

  // Print sensor values to serial monitor
  Serial.println("---- Sensor Readings ----");
  Serial.print("Solar Voltage: ");
  Serial.print(solarVoltage, 2);
  Serial.println(" V");
  Serial.print("Solar Current: ");
  Serial.print(solarCurrent, 2);
  Serial.println(" A");
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage, 2);
  Serial.println(" V");
  Serial.print("Battery Current: ");
  Serial.print(batteryCurrent, 2);
  Serial.println(" A");
  Serial.print("LDR1: ");
  Serial.print(ldr1);
  Serial.print(", LDR2: ");
  Serial.print(ldr2);
  Serial.print(", LDR3: ");
  Serial.print(ldr3);
  Serial.print(", LDR4: ");
  Serial.println(ldr4);
  Serial.print("Temperature: ");
  Serial.print(temperature, 1);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(humidity, 1);
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.print(pressure, 1);
  Serial.println(" hPa");
  Serial.println("--------------------------");
  delay(500); // Reduce delay to 500ms to allow quicker updates without flooding the serial output
}

float readCompensatedVoltage(int pin, float compensationFactor) {
  float rawVoltage = analogRead(pin) * (5.0 / 1023.0);  // Convert analog reading to voltage
  return rawVoltage * compensationFactor;               // Apply compensation factor
}

float readCompensatedCurrent(int pin, float compensationFactor) {
    // Read the sensor voltage from the analog pin
    float sensorVoltage = analogRead(pin) * (5.0 / 1023.0);

    // Calculate the current: (SensorVoltage - Vcc/2) / Sensitivity
    // Note: Vcc/2 (midpoint voltage) is 2.5V assuming a 5V supply
    float current = (sensorVoltage - 2.5) / 0.066;  // Convert voltage to current using 66 mV/A sensitivity

    return current * compensationFactor;  // Apply any additional calibration factor if needed
}

String getFormattedDate(DateTime now) {
  return String(now.year()) + "_" + String(now.month()) + "_" + String(now.day());
}
