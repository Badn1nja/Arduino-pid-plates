//***********************************************
//    * Project: Heat Plate PID Controller System
//    * Author: Badn1nja
//    * Description:
//    *  This project controls the temperature of heat plates using PID controllers.
//    *  It reads temperature data from MAX31865 RTD sensors (Pt100), applies PID control
//    *  to two heaters (PLATE_HEATER0 and PLATE_HEATER1), and displays key information
//    *  on an LCD screen (16x2, I2C).
//    *
//    *  The user interacts with the system via an LCD menu interface, where they can:
//    *   - Start the system with default settings
//    *   - Adjust temperature settings
//    *   - Perform an autotune to automatically adjust PID parameters (WIP)
//    *   - Monitor real-time temperature readings and control feedback
//    *
//    *  The system uses AutoPID and sTune libraries for PID control and tuning,
//    *  along with the SimpleRotary library to handle rotary encoder inputs.
//    *
//    *  ** Hardware: **
//    *   - Arduino (e.g., Uno, Nano)
//    *   - MAX31865 RTD sensor (Pt100) for temperature measurement
//    *   - LCD I2C (16x2) display for UI
//    *   - Rotary Encoder for user interaction
//    *   - 2x heaters
//    *
//    *  ** Libraries Used: **
//    *   - QuickPID (PID Control))
//    *   - sTune (PID Tuning)
//    *   - LcdMenu (Menu System)
//    *   - SimpleRotary (Rotary Encoder)
//    *   - MAX31865 (Temperature Sensing)
//    **************************************************/
//    * DECLARATIONS / Settings ******************************************************/
//    **************************************************/

#include "Arduino.h"
#include <SPI.h>
#include <PwFusion_MAX31865.h>
//#include <pt100rtd.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include <sTune.h>
#include <QuickPID.h>
#include <LcdMenu.h>
#include <MenuScreen.h>
#include <display/LCD_I2CAdapter.h>
#include <SimpleRotary.h>
#include <input/SimpleRotaryAdapter.h>
#include <renderer/CharacterDisplayRenderer.h>
#include <ItemWidget.h>
// #include <widget/WidgetBool.h>
// #include <widget/WidgetList.h>
#include <widget/WidgetRange.h>
#include <ItemCommand.h>

// Function declarations
void ScreenFlash();
void start_heating();
void sampleUntilNonZero();
void PrintRTDStatus(uint8_t status);
void UpdateTemps();
void Temp_Sanity_Tests();
void stop_failure();
void callback(int pos);
void toggleBacklight(bool isOn);
void PrintTemps();
void loop_pid();
void heating_loop();
void stop_heating();
void returntomenu();

const int PLATE_HEATER0 = 5;
const int PLATE_HEATER1 = 6;
const int ENCODER_PIN_A = 2;
const int ENCODER_PIN_B = 3; 
const int ENCODER_PIN_SW = 4;
const int CS0_PIN = 9;
const int CS1_PIN = 8;
boolean heating = false;
#define RREF 430.0 // for pt100rtd

#define PID_UPDATE_INTERVAL 4000
#define TEMP_READ_DELAY 500
#define LCD_ROWS 2
#define LCD_COLS 16
#define Boot_MSGTIME 500 
// time between boot-sequence messages, longer is more readable, reduce for speed.
#define ScreenFlashDelay 100
#define Revision "Version 1.2"

float setpoint = 0;
const char* options[] = {"Stop", "Start"};

// clang-format off
MENU_SCREEN(mainScreen, mainItems,
  ITEM_WIDGET(
    "Temp",
    [](int settemp) { setpoint = settemp; },
    WIDGET_RANGE(80, 5, 10, 130, "%dC", 1)),
  ITEM_COMMAND("Start", start_heating));
// clang-format on

struct STuneSettings {
  uint32_t settleTimeSec = 10;
  uint32_t testTimeSec = 500;
  uint16_t samples = 500;
  float inputSpan = 140;
  float outputSpan = 255;
  float outputStart = 0;
  float outputStep = 50;
  float tempLimit = 150;
  uint8_t debounce = 1;
};

struct PlateData
{
  float temperature;   // Current temperature of plate
  float output; 
  float tunedoutput;       // PID output specific to this plate
  int heaterPin;
  int csPin;       // GPIO pin for the heater
  float kp;
  float ki;
  float kd;

  PlateData(float temp, float out, float tunedout, int heaterpin, int csPin, float kp, float ki, float kd)
      : temperature(temp),
        output(out),
        tunedoutput(tunedout),
        heaterPin(heaterpin), 
        csPin(csPin),
        kp(kp),
        ki(ki),
        kd(kd) {}
};

/// ************************************************************************************************

struct Plate
{
  MAX31865 sensor;     // RTD sensor object (assuming RTD is the sensor class)45
  sTune tuner;         // sTune object
  QuickPID quickpid;   // QuickPID object
  PlateData data;
  STuneSettings sTuneConfig;  

  Plate(MAX31865 sensor, sTune tuner, QuickPID pid, PlateData data, STuneSettings stconf)
      : sensor(sensor),
        tuner(tuner),
        quickpid(pid),
        data(data),
        sTuneConfig(stconf) {}

  void beginSensor(int csPin, uint8_t wireType, uint8_t rtdType)
  {
    sensor.begin(csPin, wireType, rtdType);
  }

  void initStune()
  {
    tuner.Configure(sTuneConfig.inputSpan, sTuneConfig.outputSpan, sTuneConfig.outputStart, sTuneConfig.outputStep, sTuneConfig.testTimeSec, sTuneConfig.settleTimeSec, sTuneConfig.samples);
    tuner.SetEmergencyStop(sTuneConfig.tempLimit);
  }
  // PIDPlate1.SetOutputLimits(0, sTuneConfig.outputSpan * 0.1);
  // PIDPlate1.SetSampleTimeUs((sTuneConfig.outputSpan - 1) * 1000);
  // PIDPlate1.SetMode(QuickPID::Control::manual); // the PID is turned on
  // PIDPlate1.SetProportionalMode(QuickPID::pMode::pOnMeas);
  // PIDPlate1.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  // PIDPlate1.SetTunings(plate1.pidConfig.kp, plate1.pidConfig.ki, plate1.pidConfig.kd); // set PID gains
  float updateTemperature()
  {
    sensor.sample();
    data.temperature = sensor.getTemperature();
    return data.temperature;
  }

};

// Global Classes
STuneSettings sTuneConfig;

PlateData PlateData0(0.0, 0.0, 0.0, PLATE_HEATER0, CS0_PIN, 0.0, 0.0, 0.0);
PlateData PlateData1(0.0, 0.0, 0.0, PLATE_HEATER1, CS1_PIN, 0.0, 0.0, 0.0);

sTune p0tuner = sTune(&PlateData0.temperature, &PlateData0.output, p0tuner.ZN_PID, p0tuner.directIP, p0tuner.printOFF);
sTune p1tuner = sTune(&PlateData1.temperature, &PlateData1.output, p1tuner.ZN_PID, p1tuner.directIP, p1tuner.printOFF);

QuickPID PIDPlate0(&PlateData0.temperature, &PlateData0.output, &setpoint);
QuickPID PIDPlate1(&PlateData1.temperature, &PlateData1.output, &setpoint);

Plate plate0(MAX31865(), p0tuner, PIDPlate0, PlateData0, sTuneConfig);
Plate plate1(MAX31865(), p1tuner, PIDPlate1, PlateData1, sTuneConfig);

LCD_I2C lcd(0x27, 16, 2);
LCD_I2CAdapter lcdAdapter(&lcd);
CharacterDisplayRenderer renderer(&lcdAdapter, LCD_COLS, LCD_ROWS);
LcdMenu menu(renderer);
SimpleRotary encoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_PIN_SW);
SimpleRotaryAdapter rotaryInput(&menu, &encoder);

//pt100rtd PT100 = pt100rtd();

// setup function initializes the system
void setup()
{
  // Begin
  Serial.begin(115200);
  lcd.begin();
  ScreenFlash();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print(F("Heat Plate:"));
  lcd.setCursor(3, 1);
  lcd.print(F(Revision));
  delay(Boot_MSGTIME * 2);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(F("Begin."));
  SPI.begin();
  pinMode(PLATE_HEATER0, OUTPUT);
  pinMode(PLATE_HEATER1, OUTPUT);
  digitalWrite(PLATE_HEATER0, HIGH);
  digitalWrite(PLATE_HEATER1, HIGH);
  pinMode(CS0_PIN, OUTPUT);
  pinMode(CS1_PIN, OUTPUT);
  plate0.beginSensor(CS0_PIN, static_cast<uint8_t>(RTD_3_WIRE), static_cast<uint8_t>(RTD_TYPE_PT100));
  plate1.beginSensor(CS1_PIN, static_cast<uint8_t>(RTD_3_WIRE), static_cast<uint8_t>(RTD_TYPE_PT100));
  plate0.initStune();
  plate1.initStune();
  lcd.setCursor(0, 1);
  lcd.print(F(" Wait for Temp. "));
  sampleUntilNonZero();
  Temp_Sanity_Tests();
  ScreenFlash();
  lcd.clear();
  renderer.begin();
  menu.setScreen(mainScreen);       
}

void loop()
{
  rotaryInput.observe();
}

void start_heating()
{
  menu.hide();
  lcd.clear();
  lcd.print(F("Heating Started"));
  heating = true;
  plate0.quickpid.SetMode(QuickPID::Control::automatic);
  plate1.quickpid.SetMode(QuickPID::Control::automatic);
  delay(Boot_MSGTIME);
  heating_loop();
}
void stop_heating()
{
  lcd.clear();
  lcd.print(F("Heating Stopped"));
  delay(Boot_MSGTIME);
  plate0.quickpid.SetMode(QuickPID::Control::manual);
  plate1.quickpid.SetMode(QuickPID::Control::manual);
  digitalWrite(PLATE_HEATER0, HIGH);
  digitalWrite(PLATE_HEATER1, HIGH);
  heating = false;
  lcd.clear();
  menu.show();
  menu.setScreen(mainScreen); 
}

void heating_loop() {
    if (!heating) {
        stop_heating();
        return;
    }
    plate0.data.tunedoutput = plate0.tuner.softPwm(plate0.data.heaterPin, plate0.data.temperature, plate0.data.output, setpoint, plate0.sTuneConfig.outputSpan, plate0.sTuneConfig.debounce);
    plate1.data.tunedoutput = plate1.tuner.softPwm(plate1.data.heaterPin, plate1.data.temperature, plate1.data.output, setpoint, plate1.sTuneConfig.outputSpan, plate1.sTuneConfig.debounce);
    UpdateTemps();
    PrintTemps();
    Temp_Sanity_Tests();
    returntomenu();
    auto handleTuner = [](auto& plate, auto& PIDPlate) {
        switch (plate.tuner.Run()) {
            case plate.tuner.sample:
                plate.tuner.plotter(plate.data.temperature, plate.data.output, setpoint, 0.5f, 3);
                break;

            case plate.tuner.tunings:
                plate.tuner.GetAutoTunings(&plate.data.kp, &plate.data.ki, &plate.data.kd);
                PIDPlate.SetOutputLimits(0, sTuneConfig.outputSpan * 0.1);
                PIDPlate.SetSampleTimeUs((sTuneConfig.outputSpan - 1) * 1000);
                sTuneConfig.debounce = 0;
                plate.data.output = plate.sTuneConfig.outputStep;
                PIDPlate.SetMode(QuickPID::Control::automatic);
                PIDPlate.SetProportionalMode(QuickPID::pMode::pOnMeas);
                PIDPlate.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
                PIDPlate.SetTunings(plate.data.kp, plate.data.ki, plate.data.kd);
                break;

            case plate.tuner.runPid:
                PIDPlate.Compute();
                plate.tuner.plotter(plate.data.temperature, plate.data.tunedoutput, setpoint, 0.5f, 3);
                break;
        }
    };

    handleTuner(plate0, PIDPlate0);
    handleTuner(plate1, PIDPlate1);
    returntomenu();
    heating_loop();
  
  
}
void stop_failure()
{
  digitalWrite(PLATE_HEATER0, HIGH);
  digitalWrite(PLATE_HEATER1, HIGH);
}
void sampleUntilNonZero() {
    do {
        plate0.sensor.sample();
        plate0.updateTemperature();
        delay(TEMP_READ_DELAY);
        plate1.sensor.sample();
        plate1.updateTemperature();
        delay(TEMP_READ_DELAY);
    } while (plate0.data.temperature == 0 || plate1.data.temperature == 0);
}
void PrintTemps() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("Plate 0: ");
  lcd.print(plate0.data.temperature);
  lcd.setCursor(0, 1);
  lcd.printf("Plate 1: ");
  lcd.print(plate1.data.temperature);
  delay(TEMP_READ_DELAY);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("Setpoint: ");
  lcd.print(setpoint);
  lcd.setCursor(0, 1);
  lcd.printf("Output: ");
  lcd.print(plate0.data.output);
  delay(TEMP_READ_DELAY);
}
void UpdateTemps() {
  plate0.updateTemperature();
  plate1.updateTemperature();
}

void ScreenFlash()
{
  for (int i = 0; i < 2; i++)
  {
    lcd.noBacklight();
    delay(ScreenFlashDelay);
    lcd.backlight();
    delay(ScreenFlashDelay);
  }
}
// Define the callbacks
void toggleBacklight(bool isOn)
{
  lcdAdapter.setBacklight(isOn);
}
void Temp_Sanity_Tests()
{
  if (plate0.data.temperature >= sTuneConfig.tempLimit || plate1.data.temperature >= sTuneConfig.tempLimit)
  {
    stop_failure();
  }
}
void returntomenu(){
  byte i;
  // 0 = not pushed, 1 = pushed
  i = encoder.push();
  if(i == 1){
    heating = false;
    stop_heating();
  }
}

  // void loop_pid(STuneSettings& sTuneConfig){
  //   updateTemperature();
  //   float optimumOutput = tuner.softPwm(heaterPin, temperature, output, setpoint, sTuneConfig.outputSpan, sTuneConfig.debounce);
  //   switch (tuner.Run())
  //   {
  //   case tuner.sample: // active once per sample during test
  //     updateTemperature();
  //     tuner.plotter(temperature, output, setpoint, 0.5f, 3); // output scale 0.5, plot every 3rd sample
  //     break;

  //   case tuner.tunings:                    // active just once when sTune is done
  //     tuner.GetAutoTunings(pidConfig.kp, pidConfig.ki, pidConfig.kd); // sketch variables updated by sTune
  //     quickpid.SetSampleTimeUs((sTuneConfig.outputSpan - 1) * 1000);
  //     sTuneConfig.debounce = 0; // ssr mode
  //     output = sTuneConfig.outputStep;
  //     quickpid.SetMode(quickpid.Control::automatic); // the PID is turned on
  //     quickpid.SetProportionalMode(quickpid.pMode::pOnMeas);
  //     quickpid.SetAntiWindupMode(quickpid.iAwMode::iAwClamp);
  //     quickpid.SetTunings(pidConfig.kp, pidConfig.ki, pidConfig.kd); // update PID with the new tunings
  //     break;

  //   case tuner.runPid: // active once per sample after tunings
  //     updateTemperature();
  //     quickpid.Compute();
  //     tuner.plotter(temperature, optimumOutput, setpoint, 0.5f, 3);
  //     break;
  //   }
  // }


  // void invert_pid()
// {
//   float p1outinvert, p0outinvert;
//   // Map the output from the range 1000-0 to the PWM range 0-255, inverted
//   p0outinvert = map(plate0.output, 0, 255, 255, 0);  // 1000 -> 255 (off), 0 -> 0 (full power)
//   p1outinvert = map(plate1.output, 0, 255, 255, 0);

//   // Control the heaters with the mapped inverted output
//   analogWrite(PLATE_HEATER0, p0outinvert);
//   analogWrite(PLATE_HEATER1, p1outinvert);
// }

// struct Plate
// {
//   float temperature;   // Current temperature of plate
//   float output;        // PID output specific to this plate
//   int heaterPin;
//   int csPin;       // GPIO pin for the heater
//   PIDConfig pidConfig; // PID parameters
//   MAX31865 sensor;     // RTD sensor object (assuming RTD is the sensor class)45
//   sTune tuner;         // sTune object
//   QuickPID quickpid;   // QuickPID object  

//   Plate(float temp, float out, int heaterpin, int csPin, PIDConfig pidCfg, MAX31865 sensor, sTune tuner, QuickPID pid)
//       : temperature(temp),
//         output(out),
//         heaterPin(heaterpin), 
//         csPin(csPin),
//         pidConfig(pidCfg),
//         sensor(sensor),
//         tuner(tuner),
//         quickpid(pid) {}

//   void beginSensor(int csPin, uint8_t wireType, uint8_t rtdType)
//   {
//     sensor.begin(csPin, wireType, rtdType);
//   }

//   void initStune()
//   {
//     tuner.Configure(sTuneConfig.inputSpan, sTuneConfig.outputSpan, sTuneConfig.outputStart, sTuneConfig.outputStep, sTuneConfig.testTimeSec, sTuneConfig.settleTimeSec, sTuneConfig.samples);
//     tuner.SetEmergencyStop(sTuneConfig.tempLimit);
//   }

//   float updateTemperature()
//   {
//     sensor.sample();
//     temperature = sensor.getTemperature();
//     return temperature;
//   }

// };
