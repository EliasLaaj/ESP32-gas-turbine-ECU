
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
const int StopSwitch = 15;       // GPIO pin for stop switch
const int StartSwitch = 13;      // GPIO pin for start switch
const int throttlePin = 22;      // GPIO pin for throttle ESC
const int starterPin = 23;       // GPIO pin for starter ESC
const int igniterPin = 21;       // GPIO pin for igniter MOSFET
const int throttlePotPin = 32;   // GPIO pin for throttle potentiometer 
const int idlePin = 33;          // GPIO pin for idle potentiometer 
const int hallSensorPin = 14;    // GPIO pin for RPM Hall sensor

// Define the pins for the MAX6675
const int thermoCLK = 5;  // Clock pin, SCK
const int thermoCS = 18;  // Chip Select pin, CS
const int thermoDO = 19;  // Data Out pin (MISO), SO

// Define the pins for X9C103S digital potentiometer
const int INC_PIN = 26;    // Increment pin
const int UD_PIN = 25;     // Up/Down pin
const int CS_PIN = 27;     // Chip Select pin

// Constants
const unsigned long READ_INTERVAL = 300;    // 300 ms interval
const unsigned int RPM_DEC_MAX = -2000;     // Amount of rpm loss in second to activate EPO
const int IGNITED_THRESHOLD_INCREMENT = 30; // Increase of TOT to detect ignition (in Celsius)
const float TOT_THRESHOLD = 700.0;          // Maximum safe TOT (in Celsius)
const int MIN_RPM = 25000;                  // Minimum rpm before EPO activates
const int MIN_OIL_VALUE = 60;               // Minimum value which oil pump can be set while running. (pwm %)

// Flags
bool runningState = false;
bool cooldownRun = false;

// Variables for timing and state tracking
unsigned long lastReadTime = 0;
unsigned long lastRpmTime = 0;
double lastRPM = 0;
double currentRPM = 0;
double deltaRPM = 0;
volatile unsigned long pulseCount = 0;
unsigned long startSwitchPressedTime = 0;
float TOT = 0.0;
int escPosition = 0;

// Other variables
int minThrottleValue = 800;                 // Default value for throttle range min value
int maxThrottleValue = 3440;                // Default value for throttle range max value
int currentPosition = 50;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
BluetoothSerial SerialBT;

// Function declarations
void startup();
void shutdown();
void setOilRpm(int position);
void calculateRPM();
void hallSensorISR();
void sleep(int duration);
void EPO(unsigned long currentTime);
void readAndReportTemperature();
void updateESCs();
void listenForCommands();
void loadCalibrationValues();

Servo throttleEsc;
Servo starterEsc;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("JetECU"); // Bluetooth device name
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);

  pinMode(StopSwitch, INPUT_PULLUP);
  pinMode(StartSwitch, INPUT_PULLUP);
  pinMode(igniterPin, OUTPUT);
  pinMode(INC_PIN, OUTPUT);
  pinMode(UD_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(hallSensorPin, INPUT_PULLUP);

  loadCalibrationValues();

  attachInterrupt(digitalPinToInterrupt(hallSensorPin), hallSensorISR, FALLING);
  digitalWrite(CS_PIN, HIGH);

  throttleEsc.attach(throttlePin, 1000, 2200);
  starterEsc.attach(starterPin, 1500, 2100);

  throttleEsc.write(0);
  starterEsc.write(0);
  digitalWrite(igniterPin, LOW);
  setOilRpm(0);
  delay(3000);  // Wait for ESCs to initialize

  Serial.println("System Initialized. Ready to start.");
  SerialBT.println("System Initialized. Ready to start.");
  Serial2.println("status:Ready to start.");
}

void loop() {
  unsigned long currentTime = millis();

  if (digitalRead(StopSwitch) == LOW && cooldownRun) {
    SerialBT.println("Stop switch activated. Shutting down.");
    shutdown();
  }

  if (runningState) {
    EPO(currentTime);
    updateESCs();
  }
  else if (!runningState) {
    listenForCommands();
  }
  else if (digitalRead(StartSwitch) == LOW && digitalRead(StopSwitch) == HIGH) {
    delay(200);
    if (digitalRead(StartSwitch) == LOW) {
      SerialBT.print("Startup activated");
      startup();
    }
  }
  else if (digitalRead(StopSwitch) == LOW) {
    Serial2.println("status:Stop switch active.");
  }

  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    readAndReportTemperature();
  }

  delay(10);
}

void startup() {
  SerialBT.println("Starting...");
  Serial2.println("status:Starting...");
  int minEscValue = analogRead(idlePin);
  int minEscPosition = map(minEscValue, 0, 4095, 0, 90);
  setOilRpm(50);
  starterEsc.write(25);
  sleep(5);
  
  readAndReportTemperature();
  int ignitedThreshold = TOT + IGNITED_THRESHOLD_INCREMENT;
  Serial2.println("status:Igniting...");
  digitalWrite(igniterPin, HIGH);
  throttleEsc.write(minEscPosition);
  delay(250);
  sleep(1);
  starterEsc.write(50);
  sleep(2);
  cooldownRun = true;

  unsigned long startTime = millis();
  while (TOT < ignitedThreshold && millis() - startTime < 5000) {
    delay(300);
    readAndReportTemperature();
  }
  
  digitalWrite(igniterPin, LOW);
  if (TOT > ignitedThreshold) {
    runningState = true;
    Serial2.println("status:Ignited");
    starterEsc.write(90);
    setOilRpm(75);
  } else {
    starterEsc.write(50);
    Serial2.println("status:Ignition failed.");
    shutdown();
  }
  sleep(5);
  starterEsc.write(0);
  Serial2.println("status:Running");
}

void shutdown() {
  SerialBT.println("Shutting down.");
  Serial2.println("status:Shutting down.");

  runningState = false;
  setOilRpm(50);
  throttleEsc.write(0);
  sleep(5);
  starterEsc.write(50);
  sleep(20);
  starterEsc.write(0);
  sleep(5);
  setOilRpm(0);
  cooldownRun = false;
  Serial2.println("status:Shutdown complete.");
}

void setOilRpm(int position) {
  position = constrain(position, 0, 99);
  int steps = abs(position - currentPosition);
  if (steps == 0) return;

  digitalWrite(UD_PIN, (position > currentPosition) ? HIGH : LOW);
  digitalWrite(CS_PIN, LOW);

  for (int i = 0; i < steps; i++) {
    digitalWrite(INC_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(INC_PIN, HIGH);
    delayMicroseconds(1);
  }

  digitalWrite(CS_PIN, HIGH);
  currentPosition = position;
}

void hallSensorISR() {
  pulseCount++;
}

void calculateRPM() {
  unsigned long rpmTime = millis();
  const unsigned int pulsesPerRevolution = 1;
  noInterrupts();
  double localPulseCount = pulseCount;
  pulseCount = 0;
  interrupts();

  unsigned long deltaTime = rpmTime - lastRpmTime;
  if (deltaTime > 0) {
    currentRPM = (localPulseCount / pulsesPerRevolution) * (60000.0 / deltaTime);
    deltaRPM = (currentRPM - lastRPM) / (deltaTime / 1000.0);
  } else {
    currentRPM = 0;
    deltaRPM = 0;
  }

  lastRpmTime = rpmTime;
  lastRPM = currentRPM;
}

void sleep(int duration) {
  int cycles = duration * 3;
  for (int i = 0; i < cycles; i++) {
    readAndReportTemperature();
    delay(333);
  }
}

void EPO(unsigned long currentTime) {
  if (digitalRead(StartSwitch) == LOW) {
    if (startSwitchPressedTime == 0) {
      startSwitchPressedTime = currentTime;
    }

    if (currentTime - startSwitchPressedTime >= 500) {
      digitalWrite(igniterPin, HIGH);
      starterEsc.write(75);
      Serial2.println("status:EPO activated!");
    }
  }

  if (deltaRPM < RPM_DEC_MAX || MIN_RPM > currentRPM > 0) {
    Serial2.println("status:EPO activated!");
    digitalWrite(igniterPin, HIGH);
    starterEsc.write(75);
    startSwitchPressedTime = 1;
  } else if (startSwitchPressedTime != 0) {
    digitalWrite(igniterPin, LOW);
    starterEsc.write(0);
    Serial2.println("status:Running");
    startSwitchPressedTime = 0;
  }
}

void readAndReportTemperature() {
  TOT = thermocouple.readCelsius();  // Update global TOT variable
  calculateRPM();
  SerialBT.print("Temperature: ");
  SerialBT.println(TOT);
  SerialBT.print("Current RPM: ");
  SerialBT.println(currentRPM);

  Serial2.print("RPM:");
  Serial2.println(currentRPM);
  Serial2.print("TOT:");
  Serial2.println(TOT);

  if (runningState) {
    int oilPumpSetting = map(escPosition, 0, 90, MIN_OIL_VALUE, 99);
    setOilRpm(oilPumpSetting);
  }

  if (!isnan(TOT) && TOT > TOT_THRESHOLD) {
    SerialBT.print("TOT too high: ");
    Serial2.println("status:TOT over 700c!");
    SerialBT.println(TOT);
    shutdown();
  }
}

void updateESCs() {
  int escValue = analogRead(throttlePotPin);
  int minEscValue = analogRead(idlePin);

  int minEscPosition = map(minEscValue, 0, 4095, 0, 90);

  // Ensure escValue is within the specified range
  escValue = constrain(escValue, minThrottleValue, maxThrottleValue);

  escPosition = map(escValue, minThrottleValue, maxThrottleValue, minEscPosition, 90);

  throttleEsc.write(max(escPosition, minEscPosition));
}

void listenForCommands() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();  // Remove any trailing whitespace

    if (command.equals("FuelPrime")) {
      SerialBT.println("Executing FuelPrime...");
      throttleEsc.write(60);
      sleep(3);
      throttleEsc.write(90);
      sleep(2);
      throttleEsc.write(0);
    } 
    else if (command.equals("IGNtest")) {
      SerialBT.println("Executing IGNtest...");
      digitalWrite(igniterPin, HIGH);
      sleep(3);
      digitalWrite(igniterPin, LOW);
    } 
    else if (command.equals("StartTest")) {
      SerialBT.println("Executing StartTest...");
      starterEsc.write(45);
      sleep(5);
      starterEsc.write(0);
    } 
    else if (command.equals("OilPrime")) {
      SerialBT.println("Executing OilPrime...");
      setOilRpm(50);
      sleep(5);
      setOilRpm(99);
      sleep(5);
      setOilRpm(0);
    } 
    else if (command.equals("PotRead")) {
      SerialBT.println("Throttle pot value is " + analogRead(throttlePotPin));
      SerialBT.println("Idle pot value is " + analogRead(idlePin));
    } 
    else if (command.equals("ThrottleCalibration")) {
      SerialBT.println("Starting Throttle Calibration...");
      SerialBT.println("Reading closed state of throttle in 1 second.");
      delay(1000);
      SerialBT.println("Open throttle. Reading wide open state of throttle in 3 second.");
      delay(1000);
      int firstThrottleValue = analogRead(throttlePotPin) + 50; // Added 50 to ensure idle activation
      SerialBT.println("3");
      delay(1000);
      SerialBT.println("2");
      delay(1000);
      SerialBT.println("1");
      delay(1000);
      int secondThrottleValue = analogRead(throttlePotPin) - 20;

      if (firstThrottleValue < 1365 && secondThrottleValue > 2730) { // 1/3 and 2/3 of 4095
        uint8_t checksum = (firstThrottleValue & 0xFF) + ((firstThrottleValue >> 8) & 0xFF) + (secondThrottleValue & 0xFF) + ((secondThrottleValue >> 8) & 0xFF);

        EEPROM.begin(6);
        EEPROM.write(0, firstThrottleValue & 0xFF);
        EEPROM.write(1, (firstThrottleValue >> 8) & 0xFF);
        EEPROM.write(2, secondThrottleValue & 0xFF);
        EEPROM.write(3, (secondThrottleValue >> 8) & 0xFF);
        EEPROM.write(4, checksum);
        EEPROM.write(5, 0xA5); // Validation byte
        EEPROM.commit();
        SerialBT.println("Throttle calibration values saved to EEPROM.");
      } else {
        SerialBT.println("Throttle calibration failed. Ensure the throttle position and range was correct.");
      }
    } 
    else {
      SerialBT.println("Unknown command: " + command);
      SerialBT.println("Possible commands are: FuelPrime, IGNtest, StartTest, OilPrime, PotRead, and ThrottleCalibration.");
    }
  }
}

void loadCalibrationValues() {
  EEPROM.begin(6);
  if (EEPROM.read(5) == 0xA5) { // Check validation byte
    int minThrottleValueRead = EEPROM.read(0) + (EEPROM.read(1) << 8);
    int maxThrottleValueRead = EEPROM.read(2) + (EEPROM.read(3) << 8);
    uint8_t checksum = EEPROM.read(4);

    uint8_t calculatedChecksum = (minThrottleValueRead & 0xFF) + ((minThrottleValueRead >> 8) & 0xFF) + (maxThrottleValueRead & 0xFF) + ((maxThrottleValueRead >> 8) & 0xFF);

    if (checksum == calculatedChecksum) {
      SerialBT.print("Loaded throttle calibration values succesfully.");
      int minThrottleValue = minThrottleValueRead;
      int maxThrottleValue = maxThrottleValueRead;
    } else {
      SerialBT.println("Throttle calibration data is corrupted.");
    }
  } else {
    SerialBT.println("No valid throttle calibration data found. Using preset values.");
  }
}
