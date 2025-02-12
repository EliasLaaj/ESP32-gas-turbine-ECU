#include <Arduino.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include "max6675.h"
#include <BluetoothSerial.h>
#include <EEPROM.h>

// Define UART for communication with instrument cluster
#define TX2 17  // GPIO pin for TX
#define RX2 16  // GPIO pin for RX

// Define pins
const int idlePin = 33;               // GPIO pin for idle potentiometer 
const int StopSwitch = 15;            // GPIO pin for stop switch
const int StartSwitch = 13;           // GPIO pin for start switch
const int hallSensorPin = 14;         // GPIO pin for RPM Hall sensor
const int throttlePotPin = 32;        // GPIO pin for throttle potentiometer 
const int flameDetectorPin = 26;      // GPIO pin for Flame detector
const int oilPressureSensorPin = 34;  // GPIO pin for oil pressure sensor

const int ledPin = 2;                 // GPIO pin for onboard led 
const int pwmPin = 4;                 // GPIO pin for oil pump PWM output
const int igniterPin = 21;            // GPIO pin for igniter MOSFET
const int starterPin = 23;            // GPIO pin for starter ESC
const int throttlePin = 22;           // GPIO pin for throttle ESC

// Define the pins for the MAX6675
const int thermoCLK = 5;              // Clock pin, SCK
const int thermoCS = 18;              // Chip Select pin, CS
const int thermoDO = 19;              // Data Out pin (MISO), SO

// Constants
const int MIN_RPM = 32000;                      // Minimum rpm before EPO activates.
const int RPM_LIMIT = 100000;                   // Maximum safe operating rpm.
const int FLAME_THRESHOLD = 1200;               // Raw value from flame sensor to detect flame. 0 if no flame, ~1000 with spark and 4095 with flame.
const float TOT_THRESHOLD = 750.0;              // Maximum safe TOT (in Celsius).
const int NUM_OF_AVERAGE_POINTS = 5;            // Number of points for averaging.
const int PULSES_PER_REVOLUTION = 1;            // Amount of pulses per rotation for rpm calculation.
const unsigned long READ_INTERVAL = 300;        // 300 ms interval for TOT reading. Sensor is limited to max of 4hz.
const int RELIGHT_ACTIVATION_THRESHOLD = 42000; // Rpm needed which after relight is in use.
const int START_FUEL_MULTIPLIER = 1.2;         // Fuel pump multiplier for ignition and initial ramp up.

// Predefined variables
int minThrottleValue = 800;                     // Default value for throttle range min value
int maxThrottleValue = 3440;                    // Default value for throttle range max value
int minimumOilPressureLimit = 1.5;              // Minimum oil pressure intially in startup

const int MIN_THROTTLE_ESC_VALUE = 1000;        // Min and max values of esc lines in pulse width (in ms).
const int MAX_THROTTLE_ESC_VALUE = 2050;
const int MIN_STARTER_ESC_VALUE = 1500;
const int MAX_STARTER_ESC_VALUE = 2050;

// Flags for function presets
bool skipSafetyChecks = 0;     // Skips safety checks, such as oil pressure minumums, rpm minimums, flame checks etc.
bool enableSensorData = 1;     // Enables non nescessary data feed to SerialBT for debugging. 
bool allowRunningRestart = 0;  // Allows Relight function to turn on starter if it detects drop in rpm or it is called to do so by start switch.

// Flags
bool runningState = 0;
bool startupActive = 0;
bool checkFlameout = 0;
bool relightActivator = 0;

// Variables for timing and state tracking
float TOT = 0.0;
int currentRPM = 0;
int throttleEscValue = 0;
int throttleEscIdleValue = 0; 
unsigned long lastReadTime = 0;
int throttleEscRequestedValue = 0;
volatile unsigned long pulseCount = 0;

// Global Variables for averaging
double RPM_list[NUM_OF_AVERAGE_POINTS] = {0};
double TOT_list[NUM_OF_AVERAGE_POINTS] = {0};
int currentIndexRpm = 0;
int currentIndexTOT = 0;
double avgRPM = 0;
double avgTOT = 0;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
BluetoothSerial SerialBT;
Servo throttleEsc;
Servo starterEsc;

// Function declarations
void startup();
void shutdown();
void setOilValue();
void calculateRPM();
void hallSensorISR();
void checkStopSwitch();
void listenForCommands();
void checkNeedForOilFeed();
void loadCalibrationValues();
void readAndReportTemperature();
void sleep(int duration);
void setOilRpm(int position);
void RPMAverage (double newRPMValue);
void TOTAverage (double newTOTValue);
void relight(unsigned long currentTime);
void updateThrottleESC(int requestedValue);
void setStarterESC(int targetSetting, float accelerationFactor);
void saveCalibrationValues(int minThrottleValue, int maxThrottleValue);
float readPressure();
bool flameDetector();


void setup() {
  SerialBT.begin("JetECU"); // Bluetooth device name
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2); // "S:#number1-11#", "T:TOT", "R:RPM", "P;Pressure"

  pinMode(pwmPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(igniterPin, OUTPUT);
  pinMode(idlePin, INPUT);
  pinMode(throttlePotPin, INPUT);
  pinMode(flameDetectorPin, INPUT);
  pinMode(oilPressureSensorPin, INPUT);
  pinMode(StopSwitch, INPUT_PULLUP);
  pinMode(StartSwitch, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT_PULLUP);

  // Set unused GPIO pins as OUTPUT and pull them LOW
  pinMode(25, OUTPUT); digitalWrite(25, LOW); // Unused GPIO
  pinMode(12, OUTPUT); digitalWrite(12, LOW); // Use caution with this pin
  pinMode(27, OUTPUT); digitalWrite(27, LOW); // Unused GPIO
  
  // Configure input-only pins with pull-down resistors
  pinMode(35, INPUT_PULLDOWN); // Input-only, pull-down
  pinMode(36, INPUT_PULLDOWN); // Input-only, pull-down
  pinMode(39, INPUT_PULLDOWN); // Input-only, pull-down


  EEPROM.begin(6); // Initialize EEPROM with size of 6 bytes
  loadCalibrationValues();

  attachInterrupt(digitalPinToInterrupt(hallSensorPin), hallSensorISR, FALLING);

  throttleEsc.attach(throttlePin, MIN_THROTTLE_ESC_VALUE, MAX_THROTTLE_ESC_VALUE);
  starterEsc.attach(starterPin, MIN_STARTER_ESC_VALUE, MAX_STARTER_ESC_VALUE);

  digitalWrite(ledPin, HIGH);
  setOilRpm(0);
  updateThrottleESC(0);
  setStarterESC(0, 1);
  digitalWrite(igniterPin, LOW);
  delay(3000);  // Wait for ESCs to initialize
  digitalWrite(ledPin, LOW);
  
  SerialBT.println("System Initialized. Ready to start.");
  Serial2.println("S:4");//"Ready to start."
}

void loop() {
  
  digitalWrite(ledPin, !digitalRead(ledPin));
  unsigned long currentTime = millis();

  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    readAndReportTemperature();
  }

  if (runningState) {

    updateThrottleESC(1); // 1 = Pump is automatic, 0 = Turn pump off, otherwice pump will be set to requested value

    if (relightActivator) {
      relight(currentTime);
    }

    if (!relightActivator && avgRPM > RELIGHT_ACTIVATION_THRESHOLD) {
      relightActivator = 1;
    }

    if (digitalRead(StopSwitch) == LOW) {
      delay(200);
      if (digitalRead(StopSwitch) == LOW) {
        SerialBT.println("Stop switch activated.");
        shutdown();
      }
    }

    if (avgRPM < (MIN_RPM * 0.8) && avgRPM > (MIN_RPM * 0.6) && !skipSafetyChecks) {
      SerialBT.println("Rpm too low to maintain running state!");
      shutdown();
    }
  }

  if (!runningState) {
    listenForCommands();
    checkNeedForOilFeed();

    if (digitalRead(StartSwitch) == LOW) {
      delay(200);

      if (digitalRead(StopSwitch) == LOW) {
        Serial2.println("S:5");//"Stop switch active!"
        SerialBT.println("Stop switch active!");
      }
      else if (digitalRead(StartSwitch) == LOW) {
        SerialBT.println("Start switch activated");
        startup();
      }
    }
  }
}

void startup() {
  SerialBT.println("Starting...");
  Serial2.println("S:6");//"Starting..."
  startupActive = 1;
  int idleThrottleValue = analogRead(idlePin);
  int minThrottleEscValueAtStart = map(idleThrottleValue, 0, 4095, MIN_THROTTLE_ESC_VALUE, MIN_THROTTLE_ESC_VALUE + (MAX_THROTTLE_ESC_VALUE - MIN_THROTTLE_ESC_VALUE) / 2);
  int ignitedCounter = 0;
  setOilValue(3.4, 0);
  setStarterESC(18, 1.1);
  setStarterESC(32, 0.7);
  checkStopSwitch();
  
  readAndReportTemperature();
  digitalWrite(igniterPin, HIGH);
  updateThrottleESC(minThrottleEscValueAtStart);

  unsigned long startupTimer = millis();
  while (ignitedCounter < 3 && millis() - startupTimer < 5000) { // Wait five seconds for ignition or get 3 high flame values
    delay(300);
    readAndReportTemperature();
    checkStopSwitch();

    if (flameDetector()) {
      ignitedCounter++;
    }
    else {
      ignitedCounter = 0;
    }
  }

  digitalWrite(igniterPin, LOW);
  
  if (!flameDetector()) {
      updateThrottleESC(0);
      Serial2.println("S:10");//"Ignition failed!"
      shutdown();
  }
  runningState = 1;
  checkFlameout = 1;//TODO
  Serial2.println("S:7");//"Ignited..."
  setStarterESC(60, 0.75);
  updateThrottleESC(minThrottleEscValueAtStart * START_FUEL_MULTIPLIER);
  setStarterESC(100, 0.85);
  checkStopSwitch();
  

  startupTimer = millis();
  while (avgRPM < 36000 && millis() - startupTimer < 10000) { // Wait rpm to increase over idle or 10 seconds to pass
    sleep(1);
    if (enableSensorData) {
      SerialBT.println("Waiting rpm to pass 36k.");
    }
    if (StartSwitch == LOW) {
      break;
    }
  }

  updateThrottleESC(minThrottleEscValueAtStart);
  setStarterESC(0, 1);
  startupActive = 0;
  checkFlameout = 0;

  checkStopSwitch();

  if (!skipSafetyChecks && (avgRPM < 32000 && avgRPM > 3000 || !flameDetector())) {
    SerialBT.println("RPM too low for sustainable idle, or flameout!");
    shutdown();
  }
  minimumOilPressureLimit = 2.5; // Set minimum oil pressure to be 2.5 bars when engine is running
  Serial2.println("S:1");//"Running"
}

void shutdown() {
  SerialBT.println("Shutting down.");
  Serial2.println("S:8");//"Shutting down."

  digitalWrite(igniterPin, LOW);
  updateThrottleESC(0);
  setStarterESC(0, 1);
  bool extendedCoolDown = 0;
  startupActive = 0;
  checkFlameout = 0;
  relightActivator = 0;
  minimumOilPressureLimit = 1.5;
  
  if (runningState){
    extendedCoolDown = 1;
  }
  runningState = 0;
  setOilValue(2.5, 0);

  unsigned long shutdownTimer = millis();
  while (currentRPM > 5000 && millis() - shutdownTimer < 15000) {
    sleep(1);
    SerialBT.println("Waiting rpm to drop...");
  }

  setStarterESC(20, 0.8); // Cool turbine down
  if (extendedCoolDown){ 
    shutdownTimer = millis();
    while (TOT > 150 && millis() - shutdownTimer < 90000) {
      sleep(1);

      if (digitalRead(StopSwitch) == LOW && digitalRead(StartSwitch) == LOW) { // Skip cooldown if stop switch is active and start switch is pressed over 1s
        delay(1000);
        if (digitalRead(StartSwitch == 0)){
          SerialBT.println("Cooldown skipped!");
          break;
        }
      }
    }
  }

  setStarterESC(0, 1);
  shutdownTimer = millis();
  while (currentRPM > 0 && millis() - shutdownTimer < 10000) { // Wait shaft to stop
    sleep(1);
  }

  setOilValue(0.0, 0);
  Serial2.println("S:9");//"Shutdown complete."
  Serial2.println("P:0.0");
  ESP.restart(); // Restart ECU
}

void setOilRpm(int oilPercentage) {
  int oilPwmSetting = map(oilPercentage, 0, 100, 255, 0);
  analogWrite(pwmPin, oilPwmSetting);
  delay(5);
  analogWrite(pwmPin, oilPwmSetting);
}

float readPressure() {
  float voltage = analogRead(oilPressureSensorPin) * (3.3 / 4095.0);
  float pressure = -0.2439 * voltage * voltage + 3.2112 * voltage - 1.1526;
  return pressure < 0 ? 0 : pressure; // Ensure no negative pressure
}

void setOilValue(float oilPressureTarget, bool singleAdjustment) { // oil pressure, for example 2.0, 0 for setting pressure to wanted value and 1 for gradual adjustment towards it
  // Ensure the target pressure is within the achievable range (0.0 - 5.0 bars)
  oilPressureTarget = constrain(oilPressureTarget, 0.0, 5.0);

  static int oilTargetPercentage = 0;
  int maxIterations = 10; // Fail-safe: maximum number of iterations
  int iterationCount = 0;  // Track the number of iterations

  // Special case: if target pressure is 0.0, turn off the pump
  if (oilPressureTarget == 0.0) {
    oilTargetPercentage = 0;
    setOilRpm(oilTargetPercentage);
    if (enableSensorData) {
      SerialBT.println("Oil pressure target is 0.0 bars, pump turned off.");
    }
    return; // Exit the function
  }

  do {
    // If Stop switch is activated in startup, return
    if (startupActive) {
      checkStopSwitch();
    }

    // Read the sensor and map the value to pressure in bars
    float pressure = readPressure();

    // Calculate the pressure difference
    float pressureDifference = oilPressureTarget - pressure;

    // Calculate adjustment based on pressure difference (before casting to int)
    float rawAdjustment = pressureDifference * 7; // Linear scaling

    // Ensure adjustment is at least 1 or -1 before casting
    if (rawAdjustment >= 0 && rawAdjustment < 1) {
      rawAdjustment = 1;  // Ensure a minimum positive adjustment
    } else if (rawAdjustment < 0 && rawAdjustment > -1) {
      rawAdjustment = -1; // Ensure a minimum negative adjustment
    }

    // Cast to int after ensuring the minimum adjustment
    int adjustment = static_cast<int>(rawAdjustment);


    // Adjust the oilTargetPercentage proportionally
    oilTargetPercentage += adjustment;

    // Constrain oilTargetPercentage to safe operating range
    oilTargetPercentage = constrain(oilTargetPercentage, 18, 100);

    // Set the oil pump speed
    setOilRpm(oilTargetPercentage);

    // If oil pressure is under minimum limit bars when running, shut down the engine
    if (pressure < minimumOilPressureLimit && runningState && !skipSafetyChecks) {
      SerialBT.println("Oil pressure too low!!");
      shutdown();
      return;
    }

    if (enableSensorData) {
      SerialBT.print("Pressure: ");
      SerialBT.println(pressure);
      SerialBT.print(" bars, Oil Target %: ");
      SerialBT.println(oilTargetPercentage);
    }

    Serial2.print("P:");
    Serial2.println(pressure);

    // If singleAdjustment is true, exit after one iteration
    if (singleAdjustment) {
      return;
    }

    if (enableSensorData) {
      SerialBT.print("Target: ");
      SerialBT.println(oilPressureTarget);
    }


    // Check if we are close enough to the target pressure
    if (fabs(pressureDifference) <= 0.1) {
      delay(500);
      pressure = readPressure();
      pressureDifference = oilPressureTarget - pressure;

      if (fabs(pressureDifference) <= 0.1) {
        if (enableSensorData) {
          SerialBT.println("Target pressure reached.");
        }
        break; // Exit the loop if the target pressure is reached
      }
    }
    
    // Increment iteration count
    iterationCount++;

    // Fail-safe: Check if maximum iterations are reached
    if (iterationCount >= maxIterations) {
      if (enableSensorData) {
        SerialBT.println("Failed to reach target pressure within maximum iterations.");
      }
      break;
    }

    if (iterationCount > 3) {
      // Fail-safe: Check if the pump is at its minimum and pressure is not improving
      if (oilTargetPercentage == 18 && pressureDifference > 0.1) {
        if (enableSensorData) {
          SerialBT.println("Cannot reduce pressure further; pump is at minimum operating speed.");
        }
        break;
      }
    }

    // Slow down the loop for stability
    delay(250); // Increased delay for slower adjustments
  } while (true);
}

void hallSensorISR() {
  pulseCount++;
}

void calculateRPM() {
  static unsigned long lastRpmTime = 0;
  unsigned long rpmTime = millis();
  
  // Use a critical section for atomic access
  noInterrupts();
  unsigned long localPulseCount = pulseCount;
  pulseCount = 0;
  interrupts();

  unsigned long deltaTime = rpmTime - lastRpmTime;

  if (localPulseCount > 0 && deltaTime > 0) {   
    currentRPM = (localPulseCount * 60000UL) / (PULSES_PER_REVOLUTION * deltaTime);
  } else {
    currentRPM = 0;
  }

  lastRpmTime = rpmTime;
  
  RPMAverage(currentRPM);
}


void sleep(int duration) {
  int cycles = (int)(duration * (1000.0 / READ_INTERVAL));
  for (int i = 0; i < cycles; i++) {
    readAndReportTemperature();
    // If Stop switch is activated in startup, return
    if (startupActive) {
      checkStopSwitch();
    }
    delay(READ_INTERVAL);
  }
}

void relight(unsigned long currentTime) {
  static bool starterRunning = 0;
  static unsigned long startSwitchPressedTime = 0;
  if (digitalRead(StartSwitch) == LOW) {
    if (startSwitchPressedTime == 0) {
      startSwitchPressedTime = currentTime;
    }

    else if (currentTime - startSwitchPressedTime >= 200 && !starterRunning) {
      digitalWrite(igniterPin, HIGH);
      starterRunning = 1;
      if (allowRunningRestart) {
        setStarterESC(75,8);
        setStarterESC(100, 2);
      }
      Serial2.println("S:2");//"Relight active!"
      SerialBT.println("Relight activated!");
    }
  }

  else if (avgRPM < MIN_RPM && avgRPM > 20000 && allowRunningRestart) {
    Serial2.println("S:2");//"Relight active!"
    digitalWrite(igniterPin, HIGH);
    SerialBT.println("Relight activated (Flame sensor or Min RPM)!");
    starterRunning = 1;
    setStarterESC(75, 10);
    setStarterESC(100, 4);
  }

  else if (!flameDetector()) {
    delay(20);
    if (!flameDetector()) {
      SerialBT.println("Flameout detected!!! Igniting!");
      digitalWrite(igniterPin, HIGH);
      starterRunning = 1; // To run if case in else, for status update
    }
  }
  
  else {
    digitalWrite(igniterPin, LOW);
    if (starterRunning) {
      setStarterESC(0, 1);
      starterRunning = 0;
      Serial2.println("S:1");//"Running"
    }
    startSwitchPressedTime = 0;
  }
}

void readAndReportTemperature() {
  float tempC = thermocouple.readCelsius();
  
  // Validate temperature reading
  if (tempC < 0.0 || tempC > 1370.0) {
    tempC = -1; // Invalid temperature
  }

  // Process temperature if valid
  if (tempC != -1) {
    TOT = tempC;
    TOTAverage(TOT);
  }

  // Calculate RPM before serial communication
  calculateRPM();

  // Serial communication: combine multiple prints into single calls
  Serial2.print("R:"); Serial2.println(currentRPM);
  Serial2.print("T:"); Serial2.println(TOT);

  if (enableSensorData) {
    SerialBT.print("Temperature: "); SerialBT.println(TOT);
    SerialBT.print("Current RPM: "); SerialBT.println(currentRPM);
    SerialBT.print("Flame sensor value: "); SerialBT.println(analogRead(flameDetectorPin));
  }

  // Oil pump setting calculation
  if (runningState) {  // Oil pressure based on power setting, from 3.4 to 4.2 bars
    int oilPumpSetting = map(throttleEscRequestedValue, max(throttleEscIdleValue, throttleEscValue), MAX_THROTTLE_ESC_VALUE, 34, 42);
    oilPumpSetting = constrain(oilPumpSetting, 34, 42); // Ensure within range
    setOilValue(oilPumpSetting / 10, 1); // Integer division
  }

  // Safety Check: Over-Temperature Shutdown
  if (avgTOT > TOT_THRESHOLD && !skipSafetyChecks) {
    SerialBT.print("TOT too high: ");
    Serial2.println("S:11"); // "TOT over limit!"
    SerialBT.println(avgTOT);
    shutdown();
  }
}

bool flameDetector() {
  if (analogRead(flameDetectorPin) > FLAME_THRESHOLD) {
    return 1;
  }
  return 0;
}

void checkStopSwitch() { // Checks if stopswitch is activated or flamout has occured during startup

  if ((digitalRead(StopSwitch) == LOW || (!flameDetector() && checkFlameout))) {
    delay(200);
    if (digitalRead(StopSwitch) == LOW || (!flameDetector() && checkFlameout)) {
      SerialBT.println("Stop switch interruption or flameout!");
      shutdown();
    }
  }
}

void setStarterESC(int targetSetting, float accelerationFactor) {
  static int starterESCValue = MIN_STARTER_ESC_VALUE;
  if (starterESCValue == 0 || targetSetting == 0) {
    starterESCValue = MIN_STARTER_ESC_VALUE;
    starterEsc.write(MIN_STARTER_ESC_VALUE);
    return;
  }
  unsigned long currentTime = millis();
  unsigned long starterLastReadTime = 0;
  unsigned long reportLastReadTime = currentTime;

  // Clamp the target setting to the max allowable ESC setting
  targetSetting = map((constrain(targetSetting, 0, 100)), 0, 100, MIN_STARTER_ESC_VALUE, MAX_STARTER_ESC_VALUE);

  // Calculate step interval in milliseconds, default to 20 steps per second (50 ms) and adjust by acceleration (Range about 1500 - 2050, with factor of one, ~30s to get to max rpm)
  int stepInterval = (1000 / 50) / accelerationFactor;  // in milliseconds

  while (starterESCValue != targetSetting) {
    currentTime = millis();

    // Adjust ESC value gradually toward the target setting
    if (currentTime - starterLastReadTime >= stepInterval) {
      if (starterESCValue < targetSetting) {
        starterESCValue++;
      } else if (starterESCValue > targetSetting || targetSetting == MIN_STARTER_ESC_VALUE) {
        starterESCValue = MIN_STARTER_ESC_VALUE; // Reset ESC value if greater
        starterEsc.write(starterESCValue);
        return; // Exit function once stopped
      }
      starterEsc.write(starterESCValue);
      starterLastReadTime = currentTime; // Update the last read time
    }

    // Call readAndReportTemperature() periodically
    if (currentTime - reportLastReadTime >= READ_INTERVAL) {
      readAndReportTemperature();
      reportLastReadTime = currentTime;
    }

    // Check stop switch and exit if triggered
    if (startupActive) {
      checkStopSwitch();
    }

    delay(10);
  }
}


void updateThrottleESC(int requestedValue) {

  if (requestedValue == 1) {
    int throttleValue = analogRead(throttlePotPin);
    int idleThrottleValue = analogRead(idlePin);

    throttleEscIdleValue = map(idleThrottleValue, 0, 4095, MIN_THROTTLE_ESC_VALUE, MIN_THROTTLE_ESC_VALUE + (MAX_THROTTLE_ESC_VALUE - MIN_THROTTLE_ESC_VALUE) / 2);

    // Ensure throttleValue is within the specified range
    throttleValue = constrain(throttleValue, minThrottleValue, maxThrottleValue);

    throttleEscRequestedValue = map(throttleValue, minThrottleValue, maxThrottleValue, throttleEscIdleValue, MAX_THROTTLE_ESC_VALUE);

    if (throttleEscRequestedValue > throttleEscValue) {  // Set values based on required change speed over time
      throttleEscValue = throttleEscValue + 1;
      delay(1);
    }

    else if (throttleEscRequestedValue < throttleEscValue) {
      throttleEscValue = throttleEscValue - 1;
      delay(2);
    } 

    if (currentRPM > RPM_LIMIT || (TOT > (TOT_THRESHOLD - 20) && !skipSafetyChecks)) {
      if (throttleEscValue > throttleEscIdleValue + 1) {
        throttleEscValue -= 2;
      }
    }

    throttleEsc.write(max(throttleEscValue, throttleEscIdleValue));
    return;
  }

  else if (requestedValue == 0) {
    throttleEscValue = MIN_THROTTLE_ESC_VALUE;
    throttleEsc.write(throttleEscValue);
    return;
  }

  else {
    throttleEscValue = (int)constrain(requestedValue, MIN_THROTTLE_ESC_VALUE, MAX_THROTTLE_ESC_VALUE);
    throttleEsc.write(throttleEscValue);
  }
}

void listenForCommands() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();  // Remove any trailing whitespace

    if (command.equals("FuelPrime")) {
      SerialBT.println("Executing FuelPrime...");
      throttleEsc.write(70);
      sleep(3);
      throttleEsc.write(180);
      sleep(2);
      throttleEsc.write(0);
    } 
    else if (command.equals("IGNtest")) {
      SerialBT.println("Executing IGNtest...");
      digitalWrite(igniterPin, HIGH);
      sleep(3);
      if (enableSensorData) {
        SerialBT.println(analogRead(flameDetectorPin));
      }
      digitalWrite(igniterPin, LOW);
    } 
    else if (command.equals("StartTest")) {
      SerialBT.println("Executing StartTest...");
      setOilRpm(30);
      setStarterESC(30, 0.5);
      sleep(1);
      setStarterESC(0, 1);
      sleep(2);
      setOilRpm(0);
    } 
    else if (command.equals("OilPrime")) {
      SerialBT.println("Executing OilPrime...");
      setOilRpm(50);
      sleep(60);
      setOilRpm(0);
    } 

    else if (command.equals("StartUp")) {
      if (digitalRead(StopSwitch) == HIGH) {
        SerialBT.println("Startup activated");
        startup();
      }
      else{
        SerialBT.println("Stop switch active!");
      }
    }

    else if (command.equals("ToggleSafetyChecks")) {
      skipSafetyChecks = !skipSafetyChecks;
      if (skipSafetyChecks) {
        SerialBT.println("Safety checks not in use!!!");
      }
      else {
        SerialBT.println("Safety checks active.");
      }
    }

    else if (command.equals("ToggleSensorData")) {
      enableSensorData = !enableSensorData;
      if (enableSensorData) {
        SerialBT.println("Sensor data enabled.");
      }
      else {
        SerialBT.println("Sensor data disabled.");
      }
    }

    else if (command.startsWith("SetOilPressure ")) {
      String testPressureString = command.substring(15);  // "xx" starts at position 15
      float testPressure = testPressureString.toFloat();       // Convert it to an float

      // Check if the pressure is valid (between 0 and 5)
      if (testPressure >= 0 && testPressure <= 5) {
        SerialBT.println("Setting Oil pressure to " + String(testPressure) + "bars for 10 seconds...");
        setOilValue(testPressure, false);   // Set the RPM based on the input value
        sleep(10);
        setOilRpm(0);
        }
      else {
        SerialBT.println("Invalid pressure. Please enter a value between 0.0 and 5.0 bars.");
      }
    }

    else if (command.startsWith("SetOil ")) { // Check if the command is "SetOil xx"
      String percentageString = command.substring(7);  // "xx" starts at position 7
      int percentage = percentageString.toInt();       // Convert it to an integer

      // Check if the percentage is valid (between 0 and 100)
      if (percentage >= 0 && percentage <= 100) {
        SerialBT.println("Setting Oil RPM to " + String(percentage) + "% for 10 seconds...");
        setOilRpm(percentage);   // Set the RPM based on the input value
        sleep(10);
        setOilRpm(0);
        }
      else {
        SerialBT.println("Invalid percentage. Please enter a value between 0 and 100.");
      }
    }
    else if (command.equals("PotRead")) {
      int throttleVal = analogRead(throttlePotPin);
      SerialBT.print("Throttle pot value is ");
      SerialBT.println(throttleVal);
      int idleVal = analogRead(idlePin);
      int idlePercent = map(idleVal, 0, 4095, 0, 100);
      SerialBT.print("Idle pot value is ");
      SerialBT.println(idleVal);
      SerialBT.print("Idle pot percent is ");
      SerialBT.print(idlePercent);
      SerialBT.println("%.");
      delay(5000);
    } 
    else if (command.equals("ThrottleCalibration")) {
      SerialBT.println("Starting Throttle Calibration...");
      SerialBT.println("Reading closed state of throttle in 1 second.");
      delay(1000);
      int firstThrottleValue = analogRead(throttlePotPin) + 50; // Added 50 to ensure idle activation
      firstThrottleValue = constrain(firstThrottleValue, 0, 4095);
      SerialBT.println("Open throttle. Reading wide open state of throttle in 3 seconds.");
      delay(1000);
      SerialBT.println("3");
      delay(1000);
      SerialBT.println("2");
      delay(1000);
      SerialBT.println("1");
      delay(1000);
      int secondThrottleValue = analogRead(throttlePotPin) - 20;
      secondThrottleValue = constrain(secondThrottleValue, 0, 4095);

      if (firstThrottleValue < 1365 && secondThrottleValue > 2730) { // 1/3 and 2/3 of 4095
        saveCalibrationValues(firstThrottleValue, secondThrottleValue);
      }
      else {
        SerialBT.println("Throttle calibration failed. Ensure the throttle position and range was correct.");
        delay(5000);
      }
    }

    else {
      SerialBT.println("Unknown command: " + command);
      SerialBT.println("Possible commands are: FuelPrime, IGNtest, StartTest, OilPrime, PotRead,  SetOil xx (xx is in %), ");
      SerialBT.println("SetOilPressure x.x, ToggleSafetyChecks, ToggleSensorData, StartUp and ThrottleCalibration.");
      delay(5000);
    }
  }
}

void saveCalibrationValues(int minThrottleValue, int maxThrottleValue) {
  uint8_t minLowByte = minThrottleValue & 0xFF;
  uint8_t minHighByte = (minThrottleValue >> 8) & 0xFF;
  uint8_t maxLowByte = maxThrottleValue & 0xFF;
  uint8_t maxHighByte = (maxThrottleValue >> 8) & 0xFF;

  uint8_t checksum = minLowByte + minHighByte + maxLowByte + maxHighByte;

  if (EEPROM.length() < 6) {
  SerialBT.println("EEPROM size insufficient for calibration values.");
  delay(5000);
  return;
  }

  EEPROM.write(0, minLowByte);
  EEPROM.write(1, minHighByte);
  EEPROM.write(2, maxLowByte);
  EEPROM.write(3, maxHighByte);
  EEPROM.write(4, checksum);
  EEPROM.write(5, 0xA5); // Validation byte
  EEPROM.commit();
  SerialBT.println("Throttle calibration values saved to EEPROM.");
  delay(5000);
}

void loadCalibrationValues() {
  if (EEPROM.read(5) == 0xA5) { // Check validation byte
    int minThrottleValueRead = EEPROM.read(0) + (EEPROM.read(1) << 8);
    int maxThrottleValueRead = EEPROM.read(2) + (EEPROM.read(3) << 8);
    uint8_t checksum = EEPROM.read(4);

    uint8_t calculatedChecksum = (minThrottleValueRead & 0xFF) + ((minThrottleValueRead >> 8) & 0xFF) + 
                                 (maxThrottleValueRead & 0xFF) + ((maxThrottleValueRead >> 8) & 0xFF);

    if (checksum == calculatedChecksum) {
      SerialBT.println("Loaded throttle calibration values successfully.");
      minThrottleValue = minThrottleValueRead;
      maxThrottleValue = maxThrottleValueRead;
    } else {
      SerialBT.println("Throttle calibration data is corrupted.");
    }
  } else {
    SerialBT.println("No valid throttle calibration data found. Using preset values.");
  }
}

void checkNeedForOilFeed() {
  if (avgRPM > 1000) {
    setOilRpm(40);
    SerialBT.println("Standby oil feed activated");
  }
  else {
    setOilRpm(0);
  }
}

void RPMAverage (double newRPMValue) {
  RPM_list[currentIndexRpm] = newRPMValue;
  // Move to the next index, wrapping around if necessary
  currentIndexRpm = (currentIndexRpm + 1) % NUM_OF_AVERAGE_POINTS;
  double RPMsum = 0.0;
  int RPMvalidCount = 0;
  for (int i = 0; i < NUM_OF_AVERAGE_POINTS; i++) {
    if (RPM_list[i] != 0) {  // Only include proper values
            RPMsum += RPM_list[i];
            RPMvalidCount++;
  }
  // Calculate the average of valid values
  avgRPM = (RPMvalidCount > 0) ? (RPMsum / RPMvalidCount) : 0;
  }
}

void TOTAverage (double newTOTValue) {
  TOT_list[currentIndexTOT] = newTOTValue;
  // Move to the next index, wrapping around if necessary
  currentIndexTOT = (currentIndexTOT + 1) % NUM_OF_AVERAGE_POINTS;
  double TOTsum = 0.0;
  int TOTvalidCount = 0;
  for (int i = 0; i < NUM_OF_AVERAGE_POINTS; i++) {
    if (TOT_list[i] != 0) {  // Only include proper values
            TOTsum += TOT_list[i];
            TOTvalidCount++;
    }
  }
  // Calculate the average of valid values
  avgTOT = (TOTvalidCount > 0) ? (TOTsum / TOTvalidCount) : 0;
}