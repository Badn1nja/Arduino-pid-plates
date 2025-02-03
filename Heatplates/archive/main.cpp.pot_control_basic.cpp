#include "Arduino.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);
// --I2C Pins--
// SDA = A4 
// SCL = A5

#include <AutoPID.h>
// PID settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 0.12
#define KI 0.0003
#define KD 0
#define KP2 0.12
#define KI2 0.0003
#define KD2 0

#include <SPI.h>
#include <Adafruit_MAX31865.h>
const int MAX31865_CS_0 = 9;
const int MAX31865_CS_1 = 10;
#define ResistanceREF      430.0
#define ResistanceNOMINAL  100.0
Adafruit_MAX31865 temp_sensor_0 = Adafruit_MAX31865(MAX31865_CS_0);
Adafruit_MAX31865 temp_sensor_1 = Adafruit_MAX31865(MAX31865_CS_1);

// Constants
#define RESISTANCE_REF 430.0
#define RESISTANCE_NOMINAL 100.0
#define MAX_TEMP_CUTOFF_VALUE 150 // safety cut off
#define PID_UPDATE_INTERVAL 4000
#define TEMP_READ_DELAY 500


// Heater Pins
#define PLATE_HEATER1 7
#define PLATE_HEATER2 6

// POT PIN for tempDemand method
#define POT_PIN 1

// Global Variables
unsigned long lastTempUpdate;
unsigned long faultCheckCounter = 10; //because we can run on boot with this set to 10 - every 10 cycles we will check faults
double requested_temp = 0;
double plate_temp1, plate_temp2, AMBIENT_TEMP, pid_out1, pid_out2;

// int
AutoPID PLATE1PID(&plate_temp1, &requested_temp, &pid_out1, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID PLATE2PID(&plate_temp2, &requested_temp, &pid_out2, OUTPUT_MIN, OUTPUT_MAX, KP2, KI2, KD2);

// get-Data functions
void readSensors() {
    plate_temp1 = temp_sensor_0.temperature(RESISTANCE_NOMINAL, RESISTANCE_REF);
    plate_temp2 = temp_sensor_1.temperature(RESISTANCE_NOMINAL, RESISTANCE_REF);
    Serial.printf("Plate 1 Temperature: %.2f\n", plate_temp1);
    Serial.printf("Plate 2 Temperature: %.2f\n", plate_temp2);
}
void updateTemperature() {
    if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
        readSensors();
        lastTempUpdate = millis();
    }
}
void startTemps()
    {
        double avg = ((plate_temp1 + plate_temp2) / 2);
        AMBIENT_TEMP = round(avg);
    }

// Input functions
// set control method in loop - default will be rotary encoder for better usability 
//
// Use tempDemand if using a conventional potentiometer 
void tempDemand(){
    int potValue = analogRead(POT_PIN);
    requested_temp = map(potValue, 0, 1023, 60, 120);
    // Optionally, print the requested temperature for debugging
    Serial.print("Requested Temperature: ");
    Serial.println(requested_temp);  
}

// Output Functions
void heaterPWM()
    {
        analogWrite(PLATE_HEATER1, pid_out1);
        analogWrite(PLATE_HEATER2, pid_out2);
    }

// validity functions
void devianceCheck() {  
    int deviance = fabs(plate_temp1 - plate_temp2);
    Serial.printf("Deviance: %d\n", deviance);
}

// Safety functions
void stop_failure() {
    digitalWrite(PLATE_HEATER1, LOW);
    digitalWrite(PLATE_HEATER2, LOW);
    Serial.println("Issue detected! Heaters off.");
} 

void tempSanity() {
    if (plate_temp1 >= MAX_TEMP_CUTOFF_VALUE || plate_temp2 >= MAX_TEMP_CUTOFF_VALUE) {
        Serial.println("OVERTEMP Triggered");
        stop_failure();
    }
}
// Fault Management functions
void checkSensorFault(Adafruit_MAX31865 &sensor, const char* sensorName) {
    uint8_t fault = sensor.readFault();
    if (fault) {
        Serial.print(sensorName);
        Serial.print(" Fault: ");
        if (fault & MAX31865_FAULT_HIGHTHRESH) Serial.println("High Threshold");
        if (fault & MAX31865_FAULT_LOWTHRESH) Serial.println("Low Threshold");
        if (fault & MAX31865_FAULT_REFINLOW) Serial.println("REFIN- > 0.85 x Bias");
        if (fault & MAX31865_FAULT_REFINHIGH) Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
        if (fault & MAX31865_FAULT_RTDINLOW) Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
        if (fault & MAX31865_FAULT_OVUV) Serial.println("Under/Over Voltage");
        stop_failure();
    }
}
void checkFaults() {
    const int checkInterval = 10;
    if (faultCheckCounter >= checkInterval) {
        checkSensorFault(temp_sensor_0, "Sensor 1");
        checkSensorFault(temp_sensor_1, "Sensor 2");
        faultCheckCounter = 0;
    } else {
        faultCheckCounter++;
    }
}

void setup() {
    // begin comms
    Serial.begin(115200);  
    SPI.begin();
    // set pins
    pinMode(PLATE_HEATER1, OUTPUT);
    pinMode(PLATE_HEATER2, OUTPUT);
    lcd.init();     
    if (!temp_sensor_0.begin(MAX31865_3WIRE) || !temp_sensor_1.begin(MAX31865_3WIRE)) {
        lcd.println("Sensor initialization failed!");
        while (1); // Halt execution
    }
    lcd.backlight();
    // validate sensors
    checkFaults();
    lastTempUpdate = millis(); // Initialize lastTempUpdate
    readSensors();
    devianceCheck();
    startTemps();
    // set pid interval
    PLATE1PID.setTimeStep(PID_UPDATE_INTERVAL);
    PLATE2PID.setTimeStep(PID_UPDATE_INTERVAL);

}

void loop() {
    checkFaults();
    tempSanity();
    updateTemperature();
    tempDemand(); // must assign pot to POT_PIN
  
    // Run PID control
    PLATE1PID.run();
    PLATE2PID.run();
    heaterPWM();
}