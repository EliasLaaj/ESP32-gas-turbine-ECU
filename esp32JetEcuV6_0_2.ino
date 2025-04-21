#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <math.h>
#include "driver/pcnt.h"
#include "esp_task_wdt.h"
#include <BluetoothSerial.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <ESP32Servo.h>                         // By Kevin Harrington
#include "max6675.h"                            // By Adafruit

// Serial2 pins for instrument cluster 
#define TX2 17                                  // GPIO pin for TX
#define RX2 16                                  // GPIO pin for RX

// Define input pins
const int idlePin = 33;                         // GPIO pin for idle potentiometer 
const int StopSwitch = 15;                      // GPIO pin for stop switch
const int StartSwitch = 13;                     // GPIO pin for start switch
const int hallSensorPin = 14;                   // GPIO pin for RPM Hall sensor
const int throttlePotPin = 32;                  // GPIO pin for throttle potentiometer 
const int flameDetectorPin = 35;                // GPIO pin for Flame detector
const int oilPressureSensorPin = 34;            // GPIO pin for oil pressure sensor

// Define output pins
const int ledPin = 2;                           // GPIO pin for onboard led 
const int pwmPin = 4;                           // GPIO pin for oil pump PWM output
const int igniterPin = 21;                      // GPIO pin for igniter MOSFET
const int starterPin = 23;                      // GPIO pin for starter ESC
const int throttlePin = 22;                     // GPIO pin for throttle ESC
const int EscActivationPin = 25;                // GPIO pin for starter esc activation switch

// Define the pins for the MAX6675
const int thermoCLK = 5;                        // Clock pin, SCK
const int thermoCS = 18;                        // Chip Select pin, CS
const int thermoDO = 19;                        // Data Out pin (MISO), SO

// Constants
const int MIN_RPM = 30000;                      // Minimum rpm for underspeed protection. Set to minimum self sustained speed. When reached, fuel is cut.
const int TGT_RPM = 36000;                      // Target rpm for dynamic idle adjustment.
const int RPM_LIMIT = 100000;                   // Maximum safe operating rpm. Used for overspeed protection.
const int WDT_TIMEOUT = 1000;                   // Timeout (in ms) before hardware reboot in case of core crash when runningState is set to 1. (Timer resets every READ_INTERVAL)
const int FLAME_THRESHOLD = 400;                // Raw value from flame sensor to detect flame.  Rough readings are 0 without flame, ~100 with spark and 2000 with flame.
const float TOT_THRESHOLD = 750.0;              // Maximum safe TOT (in Celsius) for overheat protection.
const int PULSES_PER_REVOLUTION = 1;            // Amount of pulses per rotation for rpm calculation.
const int DYNAMIC_IDLE_DELAY = 4000;            // Minimum time needed to keep throttle at idle before dynamic idle starts to adjust idle speed.
const float MIN_IDLE_MULTIPLIER = 0.65;         // Minimum idle multiplier of static idle value, when dynamic idle is enabled.
const int START_FUEL_MULTIPLIER = 1.3;          // Fuel pump multiplier for initial ramp up.
const int NUM_OF_TOT_AVERAGE_POINTS = 3;        // Number of points for averaging TOT values.
const int NUM_OF_RPM_AVERAGE_POINTS = 5;        // Number of points for averaging RPM values.
const unsigned long READ_INTERVAL = 300;        // Interval for TOT and RPM reading in ms. MAX6675 is limited to max of 4hz.
const int RELIGHT_ACTIVATION_THRESHOLD = 34000; // Rpm limit, which passing activates relight.

// Flags for function presets
bool skipSafetyChecks = 0;                      // Skips Safety checks, for example, starting min rpm, oilpressure monitoring etc.
bool enableSensorData = 1;                      // Enables all the sensor data feed on SerialBT. Causes excess amount of bluetooth communications.
bool dynamicIdleEnabled = 1;                    // Enables Dynamic idle adjustment, if disabled idlePot is directly used to set minimum throttle ESC value.
bool forceIdleCalibration = 0;                  // Forces successfull idle calibration on setup. Reboots the esp untill calibration is successfull.
bool disableStarterWhenRunning = 0;             // Disables starter ESC when runningState is set to 1. Saves battery, but disables starter from running by relight.

// Predefined variables
int minThrottleValue = 900;                     // Default value for throttle range min value.
int maxThrottleValue = 2150;                    // Default value for throttle range max value.
int minimumOilPressureLimit = 1.5;              // Minimum oil pressure intially in startup.

// Limits for servo signal generation
const int MIN_THROTTLE_ESC_VALUE = 1000;        // Min and max values of esc signal pulse width (in ms).
const int MAX_THROTTLE_ESC_VALUE = 2050;
const int MIN_STARTER_ESC_VALUE = 1500;
const int MAX_STARTER_ESC_VALUE = 2050;

// Flags
bool runningState = 0;
bool startupActive = 0;
bool checkFlameout = 0;
bool relightActivator = 0;

// Variables for timing and state tracking
float TOT = 0.0;
int currentRPM = 0;
unsigned long lastReadTime = 0;
int throttleEscRequestedValue = 0;
const pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
int finalIdleEscValue = MIN_THROTTLE_ESC_VALUE;

// Global Variables for averaging
double RPM_list[NUM_OF_RPM_AVERAGE_POINTS] = {0};
double TOT_list[NUM_OF_TOT_AVERAGE_POINTS] = {0};
int currentIndexRpm = 0;
int currentIndexTOT = 0;
double avgRPM = 0;
double avgTOT = 0;

// ADC Calibration Data
esp_adc_cal_characteristics_t *adc_chars;

// Classes and function declarations
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
BluetoothSerial SerialBT;
Servo throttleEsc;
Servo starterEsc;

// Map GPIO to ADC1 channel
adc1_channel_t gpioToADC1Channel(int gpio) {
  switch (gpio) {
    case 32: return ADC1_CHANNEL_4;
    case 33: return ADC1_CHANNEL_5;
    case 34: return ADC1_CHANNEL_6;
    case 35: return ADC1_CHANNEL_7;
    case 36: return ADC1_CHANNEL_0;
    case 37: return ADC1_CHANNEL_1;
    case 38: return ADC1_CHANNEL_2;
    case 39: return ADC1_CHANNEL_3;
    default: return ADC1_CHANNEL_MAX;
  }
}

void startup();
void shutdown();
void setupADC();
void setupPCNT();
void calculateRPM();
void setupWatchdog();
void resetWatchdog();
void enableWatchdog();
void disableWatchdog();
void checkStopSwitch();
void listenForCommands();
void checkNeedForOilFeed();
void calibrateMinThrottle();
void loadCalibrationValues();
void readAndReportTemperature();
void sleep(int duration);
void setOilRpm(int position);
void RPMAverage (int newRPMValue);
void TOTAverage (float newTOTValue);
void relight(unsigned long currentTime);
void updateThrottleESC(int requestedValue);
void setStarterESC(int targetSetting, float accelerationFactor);
void setOilValue(float oilPressureTarget, bool singleAdjustment);
void saveCalibrationValues(int minThrottleValue, int maxThrottleValue);
bool flameDetector();
float readPressure();
int readDebouncedThrottle();
int getAverageADC(int pin, int samples);

void setup() {
  // Setup serial communications
  SerialBT.begin("JetECU"); // Bluetooth device name
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2); // "S:#number1-12#", "T:TOT", "R:RPM", "P;Pressure"

  // Configure GPIO pins
  pinMode(pwmPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(idlePin, INPUT);
  pinMode(igniterPin, OUTPUT);
  pinMode(throttlePotPin, INPUT);
  pinMode(flameDetectorPin, INPUT);
  pinMode(EscActivationPin, OUTPUT);
  pinMode(oilPressureSensorPin, INPUT);
  pinMode(StopSwitch, INPUT_PULLUP);
  pinMode(StartSwitch, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT_PULLUP);

  // Set unused GPIO pins as OUTPUT and pull them LOW
  pinMode(12, OUTPUT); digitalWrite(12, LOW); // Use caution with this pin at boot
  pinMode(26, OUTPUT); // digitalWrite(26, LOW); // Unused GPIO TODO
  pinMode(27, OUTPUT); digitalWrite(27, LOW); // Unused GPIO
  
  // Configure input-only pins with pull-down resistors
  pinMode(36, INPUT_PULLDOWN); // Input-only, pull-down
  pinMode(39, INPUT_PULLDOWN); // Input-only, pull-down

  // Initialize EEPROM and load throttle calibration values
  EEPROM.begin(6); 
  loadCalibrationValues();

  // Setup ADC averaging, rpm measuring and watchdog 
  setupADC();
  setupPCNT(); 
  setupWatchdog();

  // Initialize servo signal generation
  throttleEsc.attach(throttlePin, MIN_THROTTLE_ESC_VALUE, MAX_THROTTLE_ESC_VALUE);
  starterEsc.attach(starterPin, MIN_STARTER_ESC_VALUE, MAX_STARTER_ESC_VALUE);

  // Set everything to safe initial state and calibrate idle value
  digitalWrite(ledPin, HIGH);
  digitalWrite(EscActivationPin, LOW);
  setOilRpm(0);
  updateThrottleESC(0);
  setStarterESC(0, 1);
  digitalWrite(igniterPin, LOW);
  calibrateMinThrottle();
  delay(2000);  // Wait for ESCs to initialize
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

    if (runningState) {
      resetWatchdog();  // Reset watchdog in runningState
    }
  }

  if (runningState) {

    updateThrottleESC(1); // 1 = Pump is automatic, 0 = Turn pump off, else means to set pump to that value

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

    if (avgRPM < (MIN_RPM) && avgRPM > (MIN_RPM * 0.6) && !skipSafetyChecks) {
      SerialBT.println("Rpm too low!");
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

// Function containing startup sequence
void startup() {
  SerialBT.println("Starting...");
  Serial2.println("S:6");//"Starting..."
  digitalWrite(EscActivationPin, HIGH); // Enable starter esc
  startupActive = 1;
  int idleThrottleValue = getAverageADC(idlePin, 10);
  int minThrottleEscValueAtStart = map(idleThrottleValue, 0, 4095, MIN_THROTTLE_ESC_VALUE, MIN_THROTTLE_ESC_VALUE + (MAX_THROTTLE_ESC_VALUE - MIN_THROTTLE_ESC_VALUE) / 2);
  int ignitedCounter = 0;
  int startCounter = 0;
  setOilValue(2.0, 0);

  while (currentRPM < 1000) {
    setStarterESC(25, 1.2);
    if (startCounter > 2) {
      setOilValue(0, 0);
      digitalWrite(EscActivationPin, LOW); // Disable starter esc
      Serial2.println("S:4");//"Ready to start."
      return; // Go back to main loop if starter is stuck
    }
    else if (currentRPM < 700) {
      setStarterESC(0, 1.0);
    }
    startCounter ++;
  }
  setOilValue(3.4, 0);
  setStarterESC(36, 0.7);
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
  updateThrottleESC(minThrottleEscValueAtStart);
  Serial2.println("S:7");//"Ignited..."
  setStarterESC(100, 0.9);
  updateThrottleESC(minThrottleEscValueAtStart * START_FUEL_MULTIPLIER);
  checkFlameout = 1;
  checkStopSwitch();

  startupTimer = millis();
  while (avgRPM < 32000 && millis() - startupTimer < 12000) { // Wait rpm to increase over idle or 12 seconds to pass
    checkStopSwitch();
    if (enableSensorData) {
      SerialBT.println("Waiting rpm to pass 32k.");
    }
    if (digitalRead(StartSwitch) == LOW) {
      break;
    }
    sleep(1);
  }

  updateThrottleESC(minThrottleEscValueAtStart);
  setStarterESC(0, 1);
  startupActive = 0;
  checkFlameout = 0;

  checkStopSwitch();

  if (!skipSafetyChecks && (avgRPM < 31000 && avgRPM > 3000 || !flameDetector())) {
    delay(100);
    if (!skipSafetyChecks && (avgRPM < 31000 && avgRPM > 3000 || !flameDetector())) {
      SerialBT.println("RPM too low for sustainable idle, or flameout!");
      shutdown();
    }
  }

  if (disableStarterWhenRunning) {
    digitalWrite(EscActivationPin, LOW); // Disable starter esc
  }

  setStarterESC(0, 1);
  minimumOilPressureLimit = 2.5; // Set minimum oil pressure to be 2.5 bars when engine is running
  enableWatchdog(); // Enable watchdog for system responsiveness monitoring
  Serial2.println("S:1");//"Running"
}

// Function containing shutdown sequence from all states including cooldown run. Function restarts ESP32 when it finishes.
void shutdown() {
  SerialBT.println("Shutting down.");
  Serial2.println("S:8");//"Shutting down."

  disableWatchdog(); // Disable watchdog
  digitalWrite(igniterPin, LOW);
  updateThrottleESC(0);
  setStarterESC(0, 1);

  bool extendedCoolDown = 0;
  startupActive = 0;
  checkFlameout = 0;
  relightActivator = 0;
  minimumOilPressureLimit = 1.5;
  updateThrottleESC(0);
  
  if (runningState){
    extendedCoolDown = 1;
  }
  runningState = 0;

  if (disableStarterWhenRunning) {
    digitalWrite(EscActivationPin, HIGH); // Enable starter esc
    sleep(2);
  }

  setOilValue(2.5, 0);

  unsigned long shutdownTimer = millis();
  while (currentRPM > 5000 && millis() - shutdownTimer < 15000) {
    sleep(1);
    SerialBT.println("Waiting rpm to drop...");
  }

  setStarterESC(18, 0.8); // Cool turbine down
  SerialBT.println("Starter activated!");
  if (extendedCoolDown){ 
    shutdownTimer = millis();
    while (avgTOT > 150 && millis() - shutdownTimer < 180000) {
      sleep(1);

      if (digitalRead(StopSwitch) == LOW && digitalRead(StartSwitch) == LOW) { // Skip cooldown if stop switch is active and start switch is held over 1s
        delay(1000);
        if (digitalRead(StartSwitch) == LOW){
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
  digitalWrite(EscActivationPin, LOW); // Disable starter Esc
  Serial2.println("S:9");//"Shutdown complete."
  Serial2.println("P:0.0");
  ESP.restart();// Restart ESP32 to clean everything and start loop again
}

// Function to generate PWM signal for oil pump from oil percentage.
void setOilRpm(int oilPercentage) {
  int oilPwmSetting = map(oilPercentage, 0, 100, 255, 0);
  analogWrite(pwmPin, oilPwmSetting);
  delay(5);
  analogWrite(pwmPin, oilPwmSetting);
}

// Function to read oilpressure from pressure transducer
float readPressure() {
  float voltage = getAverageADC(oilPressureSensorPin, 10) * (3.3 / 4095.0);
  float pressure = -0.2439 * voltage * voltage + 3.2112 * voltage - 1.1526;
  return pressure < 0 ? 0 : pressure; // Ensure no negative pressure
}

// Function to set oil pressure. Pressure target is in bars. Second parameter is for mode, 0 runs function until target is reached and 1 gradually adjusts toward target by one iteration at time.
void setOilValue(float oilPressureTarget, bool singleAdjustment) { 

  // Ensure the target pressure is within the achievable range (0.0 - 5.0 bars)
  oilPressureTarget = constrain(oilPressureTarget, 0.0, 5.0);

  static int oilTargetPercentage = 0;
  int maxIterations = 15; // Fail-safe: maximum number of iterations
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
    float rawAdjustment = pressureDifference * 9; // Linear scaling

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

// Initializes ESP32 hardware pulse counter
void setupPCNT() {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = hallSensorPin,  // GPIO pin connected to Hall sensor
    .ctrl_gpio_num = PCNT_PIN_NOT_USED, // No control pin needed
    .lctrl_mode = PCNT_MODE_KEEP,       // No control mode
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,         // Increment on rising edge
    .neg_mode = PCNT_COUNT_DIS,         // Ignore falling edge
    .counter_h_lim = 32767,             // Prevent overflow
    .counter_l_lim = -32767             // Not needed for single direction
  };

  // Initialize PCNT
  pcnt_unit_config(&pcnt_config);
    
  // Enable noise filtering (removes glitches)
  pcnt_set_filter_value(PCNT_UNIT, 1023); // Debounce time.
  pcnt_filter_enable(PCNT_UNIT);

  // Start counter
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

// Function to read RPM using PCNT
void calculateRPM() {
    static unsigned long lastRpmTime = 0;
    unsigned long rpmTime = millis();
    
    int16_t pulseCount = 0;
    pcnt_get_counter_value(PCNT_UNIT, &pulseCount);
    pcnt_counter_clear(PCNT_UNIT); // Reset count after reading

    unsigned long deltaTime = rpmTime - lastRpmTime;

    if (pulseCount > 0 && deltaTime > 0) {   
        currentRPM = (pulseCount * 60000UL) / (PULSES_PER_REVOLUTION * deltaTime);
    } else {
        currentRPM = 0;
    }

    lastRpmTime = rpmTime;
    RPMAverage(currentRPM);
}

// Function for longer delays, reading and sending sensor data regularly while waiting.
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

// Function to reignite the engine in case of flameout or underspeed.
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

      if (!disableStarterWhenRunning) {
        /*setStarterESC(75,8);TODO
        setStarterESC(100, 2); */
      }

      Serial2.println("S:2");//"Relight active!"
      SerialBT.println("Relight activated!");
    }
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

// Function to read sensor data and to send nescessary data to other devices. Also calls oil pressure function when engine is running.
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
  Serial2.print("R:"); Serial2.println(avgRPM);
  Serial2.print("T:"); Serial2.println(avgTOT);

  if (enableSensorData) {
    SerialBT.print("Temperature: "); SerialBT.println(TOT);
    SerialBT.print("Current RPM: "); SerialBT.println(currentRPM);
    SerialBT.print("Flame sensor value: "); SerialBT.println(getAverageADC(flameDetectorPin, 10));
  }

  // Oil pump setting calculation
  if (runningState) {  // Oil pressure based on power setting, from 3.4 to 4.2 bars
    int oilPumpSetting = map(throttleEscRequestedValue, finalIdleEscValue, MAX_THROTTLE_ESC_VALUE, 34, 42);
    oilPumpSetting = constrain(oilPumpSetting, 34, 42); // Ensure within range
    setOilValue(oilPumpSetting / 10.0, 1);
  }

  // Safety Check: Over-Temperature Shutdown
  if (avgTOT > TOT_THRESHOLD && !skipSafetyChecks) {
    SerialBT.print("TOT too high: ");
    Serial2.println("S:11"); // "TOT over limit!"
    SerialBT.println(avgTOT);
    shutdown();
  }
}

// Function to determine flame persistence.
bool flameDetector() {
  if (getAverageADC(flameDetectorPin, 10) > FLAME_THRESHOLD) {
    return 1;
  }
  return 0;
}

// Function to call shutdown if stop switch is activated or flameout is detected.
void checkStopSwitch() { // Checks if stopswitch is activated or flamout has occured during startup
  if ((digitalRead(StopSwitch) == LOW || (!flameDetector() && checkFlameout))) {
    delay(200);
    if (digitalRead(StopSwitch) == LOW || (!flameDetector() && checkFlameout)) {
      SerialBT.println("Stop switch interruption or flameout!");
      shutdown();
    }
  }
}

// Sets starter ESC servo signal to requested percentage. If value is higher than previous one, signal is increased towards new value.
// AccelerationFactor determines how fast signal increases. If value is smaller than previous one, Starter will be set to 0%.
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

// Function generates throttle ESC servo signal. If requested value is 1, signal is set by throttle and idle pots or throttle and rpm data. If requested value is 0, Servo signal will be set to minimum value.
// Any other value will set throttle servo signal to requested value constrained between min and max.
void updateThrottleESC(int requestedValue) {
  static int throttleEscValue = MIN_THROTTLE_ESC_VALUE;

  if (requestedValue == 1) {
    int throttleValue = readDebouncedThrottle();
    int idleThrottleValue = getAverageADC(idlePin, 10);
    int throttleEscIdleValue = 0;

    // Update throttle idle value
    throttleEscIdleValue = map(idleThrottleValue, 0, 4095, MIN_THROTTLE_ESC_VALUE, MIN_THROTTLE_ESC_VALUE + (MAX_THROTTLE_ESC_VALUE - MIN_THROTTLE_ESC_VALUE) / 2);


    static float idleMaintainingValue = throttleEscIdleValue;
    static unsigned long idleStartTime = 0;  // Retains values across function calls

    if (dynamicIdleEnabled) {
          
      // idleStartTime tracking
      if (throttleValue <= (minThrottleValue + 160) && idleStartTime == 0) {
        idleStartTime = millis();  // Start tracking idle time
      } 
      else if (throttleValue > (minThrottleValue + 250)) {  // Small buffer to avoid tiny fluctuations resetting idle time
        idleStartTime = 0;  // Reset tracking only when throttle is consistently raised
      }

      // Prevent false high timeSinceIdle values
      unsigned long timeSinceIdle = (idleStartTime > 0) ? (millis() - idleStartTime) : 0;

      // Calculate values
      float minIdleLimit = throttleEscIdleValue * MIN_IDLE_MULTIPLIER;
      int rpmDiff = TGT_RPM - avgRPM;

      // Adaptive idle adjustment. Negative rpmDiff means that idle is higher than it should.
      if (throttleValue <= minThrottleValue && timeSinceIdle > DYNAMIC_IDLE_DELAY) {
        if (rpmDiff > 1000) {  
          idleMaintainingValue = fmin(idleMaintainingValue + 0.0005, throttleEscIdleValue);
        } else if (rpmDiff > 500) {  
          idleMaintainingValue = fmin(idleMaintainingValue + 0.00015, throttleEscIdleValue);
        } else if (rpmDiff < -10000) {  
          idleMaintainingValue = fmax(idleMaintainingValue - 0.0003, minIdleLimit);
        } else if (rpmDiff < -50) {  
          idleMaintainingValue = fmax(idleMaintainingValue - 0.00015, minIdleLimit);
        }
      }

      finalIdleEscValue = (int)idleMaintainingValue;
      finalIdleEscValue = constrain(finalIdleEscValue, minIdleLimit, throttleEscIdleValue);
      static int printcounter = 0;
      if (printcounter > 300) {
        SerialBT.print("Idle maintaining value is ");
        SerialBT.println(idleMaintainingValue);
        printcounter = 0;
      }
      printcounter ++;
    }
      
    else {
      finalIdleEscValue = throttleEscIdleValue;
    }

    // Read and constrain throttle input
    throttleValue = constrain(throttleValue, minThrottleValue, maxThrottleValue);
    throttleEscRequestedValue = map(throttleValue, minThrottleValue, maxThrottleValue, finalIdleEscValue, MAX_THROTTLE_ESC_VALUE);

    // Gradual throttle adjustment with delays
    if (throttleEscRequestedValue > throttleEscValue) {
      throttleEscValue++;
      delay(1);
    } else if (throttleEscRequestedValue < throttleEscValue) {
      throttleEscValue--;
      delay(2);
    }

    // Safety limiters for high RPM or temperature
    if (currentRPM > RPM_LIMIT || (TOT > (TOT_THRESHOLD - 20) && !skipSafetyChecks)) {
      if (throttleEscValue > finalIdleEscValue + 1) {
        throttleEscValue -= 2;
      }
    }

    // Apply the adjusted idle value
    throttleEsc.write(max(throttleEscValue, finalIdleEscValue));

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

// Function to debounce and smooth out throttle pot output
int readDebouncedThrottle() {
  constexpr int NUM_SAMPLES = 10;               // Running average sample count
  constexpr unsigned long DEBOUNCE_MS = 100;   // must persist this long
  constexpr int DELTA_THRESHOLD = 10;          // ADC ticks difference to trigger debounce

  static int sampleBuffer[NUM_SAMPLES] = {0};
  static int bufIndex = 0;
  static long sum = 0;
  static int stableValue = 0;
  static bool debounceActive = false;
  static unsigned long changeStart = 0;

  // 1) Read raw and update circular buffer
  int raw = getAverageADC(throttlePotPin, 10);
  sum -= sampleBuffer[bufIndex];
  sampleBuffer[bufIndex] = raw;
  sum += raw;
  bufIndex = (bufIndex + 1) % NUM_SAMPLES;

  // 2) Compute filtered value
  int filtered = sum / NUM_SAMPLES;

  // 3) Debounce: only accept new filtered if it stays > threshold for DEBOUNCE_MS
  if (abs(filtered - stableValue) > DELTA_THRESHOLD) {
    if (!debounceActive) {
      debounceActive = true;
      changeStart = millis();
    } else if (millis() - changeStart >= DEBOUNCE_MS) {
      // confirmed change
      stableValue = filtered;
      debounceActive = false;
    }
  } else {
    // if it drifts back, cancel pending change
    debounceActive = false;
  }

  return stableValue;
}


// Function to listen commands from bluetooth serial. Function aslo runs those commands.
void listenForCommands() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();  // Remove any trailing whitespace

    if (command.equals("FuelPrime")) {
      SerialBT.println("Executing FuelPrime...");
      throttleEsc.write(70);
      sleep(3);
      throttleEsc.write(120);
      sleep(2);
      throttleEsc.write(0);
    } 
    else if (command.equals("IGNtest")) {
      SerialBT.println("Executing IGNtest...");
      digitalWrite(igniterPin, HIGH);
      sleep(3);
      if (enableSensorData) {
        SerialBT.println(getAverageADC(flameDetectorPin, 10));
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

        else if (command.equals("ToggleIdleMode")) {
      dynamicIdleEnabled = !dynamicIdleEnabled;
      if (dynamicIdleEnabled) {
        SerialBT.println("Idle in dynamic mode.");
      }
      else {
        SerialBT.println("Idle in static mode.");
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
      float testPressure = testPressureString.toFloat();  // Convert it to an float

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

    else if (command.startsWith("CalibrateOilSensor")) {
      int oilPercentage = 20;
      const int stepDuration = 10; // seconds
      const int stepSize = 5;

      while (oilPercentage <= 100) {
        SerialBT.println("Setting Oil PWM to " + String(oilPercentage) + "% for " + String(stepDuration) + " seconds...");

        setOilRpm(oilPercentage); // Set the oil PWM output based on percentage

        // --- Measurement loop ---    
        long totalReading = 0;
        int numReadings = 0;

        for (int i = 0; i < stepDuration; i++) {
          int adcReading = getAverageADC(oilPressureSensorPin, 10);
          totalReading += adcReading;
          numReadings++;
          delay(1000); // wait 1 second between measurements
        }

        int averageReading = totalReading / numReadings;
        SerialBT.println("Average sensor reading at " + String(oilPercentage) + "% = " + String(averageReading));
        oilPercentage += stepSize;
      }

      SerialBT.println("Oil sensor calibration complete.");
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
      int throttleVal = getAverageADC(throttlePotPin, 10);
      SerialBT.print("Throttle pot value is ");
      SerialBT.println(throttleVal);
      float throttlePercent = ((float)(throttleVal - minThrottleValue) / (maxThrottleValue - minThrottleValue)) * 100;
      SerialBT.print("Throttle percent is ");
      SerialBT.println(throttlePercent);
      int idleVal = getAverageADC(idlePin, 10);
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
      int firstThrottleValue = getAverageADC(throttlePotPin, 50) + 100; // Add 100 to ensure idle activation
      firstThrottleValue = constrain(firstThrottleValue, 0, 4095);
      SerialBT.println("Open throttle. Reading wide open state of throttle in 3 seconds.");
      delay(1000);
      SerialBT.println("3");
      delay(1000);
      SerialBT.println("2");
      delay(1000);
      SerialBT.println("1");
      delay(1000);
      int secondThrottleValue = getAverageADC(throttlePotPin, 50) - 20;
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
      SerialBT.println("Possible commands are: FuelPrime, IGNtest, StartTest, OilPrime, PotRead,  SetOil xx (xx is in %), CalibrateOilSensor");
      SerialBT.println("SetOilPressure x.x, ToggleSafetyChecks, ToggleSensorData, ToggleIdleMode, StartUp and ThrottleCalibration.");
      delay(8000);
    }
  }
}

// Function to save throttle calibration values to EEPROM
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

// Function to load throttle calibration values from EEPROM
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

// Function to turn on oil pump if shaft is spinning.
void checkNeedForOilFeed() {
  if (avgRPM > 1000) {
    setOilRpm(40);
    SerialBT.println("Standby oil feed activated");
  }
  else {
    setOilRpm(0);
  }
}

// Function to reset watchdog timer
void resetWatchdog() {
   esp_task_wdt_reset();  // Feed watchdog to prevent reset
}

// Function to disable watchdog timer
void disableWatchdog() {
  if (esp_task_wdt_status(NULL) == ESP_OK) {
    esp_task_wdt_delete(NULL);  // Safely detach the task if currently monitored
  }
}

// Function to enable watchdog timer
void enableWatchdog() {
  if (esp_task_wdt_status(NULL) != ESP_OK) {
    esp_task_wdt_add(NULL);  // Safely attach the task if not already monitored
  }
}

// Function to setup watchdog timer
void setupWatchdog() {
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT,
    .idle_core_mask = (1 << 0) | (1 << 1),  // Enable WDT on both CPU cores
    .trigger_panic = true  // Restart ESP32 on timeout
  };

  esp_task_wdt_init(&wdtConfig);  // Pass the struct, not an int
}

// Initializes ADC reading. Note that this function only works on ADC1 channels on ESP32. These channels are GPIOs 32–39 on ESP32.
void setupADC() {
  adc1_config_width(ADC_WIDTH_BIT_12);

  adc1_config_channel_atten(gpioToADC1Channel(idlePin), ADC_ATTEN_DB_11);
  adc1_config_channel_atten(gpioToADC1Channel(throttlePotPin), ADC_ATTEN_DB_11);
  adc1_config_channel_atten(gpioToADC1Channel(flameDetectorPin), ADC_ATTEN_DB_11);
  adc1_config_channel_atten(gpioToADC1Channel(oilPressureSensorPin), ADC_ATTEN_DB_11);

  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);
}

// Function to calculate average of raw ADC values from ADC1 of ESP32. Note that this function only works on ADC1 channels which are GPIOs 32–39 on ESP32.
int getAverageADC(int gpio, int samples) {
  adc1_channel_t channel = gpioToADC1Channel(gpio);
  if (channel == ADC1_CHANNEL_MAX) return -1;

  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += adc1_get_raw(channel);
  }
  return sum / samples;
}

// Function to calibrate throttle value at setup
void calibrateMinThrottle() {
  int sum = 0;
  int count = 0;

  for (int i = 0; i < 20; i++) {
    int reading = getAverageADC(throttlePotPin, 50);
    if (reading < 1200) {  // Only include values under 1200
      sum += reading;
      count++;
    }
    delay(20);  // Spread 10 samples evenly over 1000ms
  }

  if (count > 16) { // At least 16 proper samples are needed
    int measuredMinThrottlePotValue = sum / count;
    if (measuredMinThrottlePotValue < 1000) {
      minThrottleValue = (measuredMinThrottlePotValue + 100); // Add 100 to value to make sure to activate idle
    }
  }
  else if (forceIdleCalibration) {
    Serial2.println("S:12"); // "Calibration failed!"
    digitalWrite(EscActivationPin, LOW);
    sleep(1);
    SerialBT.println("Throttle calibration failed. Rebooting...");
    ESP.restart();// Restart ESP32 to calibrate throttle again
  }
}

// Function to calculate average rpm to smooth out sensor reading.
void RPMAverage(int newRPMValue) {
    RPM_list[currentIndexRpm] = newRPMValue;
    // Move to the next index, wrapping around if necessary
    currentIndexRpm = (currentIndexRpm + 1) % NUM_OF_RPM_AVERAGE_POINTS;

    long RPMsum = 0;  // Use long to handle larger values
    int RPMvalidCount = 0;

    // Loop through the stored values to calculate the sum and valid count
    for (int i = 0; i < NUM_OF_RPM_AVERAGE_POINTS; i++) {
        if (RPM_list[i] != 0) {  // Only include non-zero values
            RPMsum += RPM_list[i];
            RPMvalidCount++;
        }
    }

    // Calculate the average of valid values
    avgRPM = (RPMvalidCount > 0) ? (RPMsum / RPMvalidCount) : 0;
}

// Function to calculate average TOT to smooth out sensor reading.
void TOTAverage (float newTOTValue) {
  TOT_list[currentIndexTOT] = newTOTValue;
  // Move to the next index, wrapping around if necessary
  currentIndexTOT = (currentIndexTOT + 1) % NUM_OF_TOT_AVERAGE_POINTS;
  float TOTsum = 0.0;
  int TOTvalidCount = 0;
  for (int i = 0; i < NUM_OF_TOT_AVERAGE_POINTS; i++) {
    if (TOT_list[i] != 0) {  // Only include proper values
            TOTsum += TOT_list[i];
            TOTvalidCount++;
    }
  }
  // Calculate the average of valid values
  avgTOT = (TOTvalidCount > 0) ? (TOTsum / TOTvalidCount) : 0;
}