#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h> // Include the Servo library for ESP8266
#include <deque>

// Variables to store RPM and TOT values
float N1 = 0.0;
int TOT = 0;

// Define constants
const unsigned long StatusLimit = 1000; // Time limit to consider multiple status messages (in milliseconds)
const unsigned long statusInterval = 2000; // Interval to cycle through status messages (in milliseconds)
const int N1servoMaxAngle = 180; // Adjusted to full servo range
const int totservoMaxAngle = 180; // Adjusted to full servo range

// Arrays to store last 3 values of RPM and TOT
float N1Values[3] = {0.0, 0.0, 0.0};
int TOTValues[3] = {0, 0, 0};
int N1Index = 0; // Index for N1Values array
int TOTIndex = 0; // Index for TOTValues array

// Variables to handle status messages
std::deque<String> statusMessages; // Deque to store status messages
unsigned long lastStatusReceivedTime = 0; // Last time a status message was received
unsigned long lastStatusDisplayTime = 0; // Last time a status message was displayed
int currentStatusIndex = 0; // Index of the current status message being displayed

// Define the OLED display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

// Create an instance of the SSD1306 display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Servo objects
Servo N1Servo;
Servo TOTServo;

// Define servo pins
#define N1_SERVO_PIN 12
#define TOT_SERVO_PIN 13

void setup() {
  // Attach servos
  N1Servo.attach(N1_SERVO_PIN);
  TOTServo.attach(TOT_SERVO_PIN);

  // Move servos to initial position
  N1Servo.write(N1servoMaxAngle);
  TOTServo.write(totservoMaxAngle);

  // Start the I2C communication
  Wire.begin(4, 5); // GPIO 4 is SDA, GPIO 5 is SCL

  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize the display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Stay here if the display initialization fails
  }

  // Clear the buffer
  display.clearDisplay();

  // Set text color
  display.setTextColor(SSD1306_WHITE);

  // Display initialization message
  display.setTextSize(1); // Initial text size
  display.setCursor(0, 0);
  display.println("Initializing...");

  // Update the display with the buffer contents
  display.display();
  delay(1000);
  
  // Move servos to zero position
  N1Servo.write(0);
  TOTServo.write(0);
  delay(1000); // Delay to show the initialization message
}

void loop() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    Serial.println(message);

    // Parse message to extract RPM, TOT, or status
    if (message.startsWith("RPM:")) {
      float newN1 = message.substring(4).toFloat(); // Store RPM value (skip "RPM:") and convert to float
      N1Values[N1Index] = newN1;
      N1Index = (N1Index + 1) % 3; // Increment index and wrap around using modulo
    } else if (message.startsWith("TOT:")) {
      int newTOT = message.substring(4).toInt(); // Store TOT value (skip "TOT:") and convert to int
      TOTValues[TOTIndex] = newTOT;
      TOTIndex = (TOTIndex + 1) % 3; // Increment index and wrap around using modulo
    } else if (message.startsWith("status:")) {
      unsigned long currentMillis = millis();
      if (currentMillis - lastStatusReceivedTime > StatusLimit) {
        statusMessages.clear(); // Clear all previous status messages
      }
      statusMessages.push_back(message.substring(7)); // Add new status message
      lastStatusReceivedTime = currentMillis; // Update the time the last status message was received
    }
  }

  // Calculate averages
  float averageN1 = (N1Values[0] + N1Values[1] + N1Values[2]) / 3.0;
  int averageTOT = (TOTValues[0] + TOTValues[1] + TOTValues[2]) / 3;

  // Clear the display
  display.clearDisplay();

  // Display TOT on the first row
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("TOT ");
  display.print(averageTOT);

  // Display the Celsius unit with smaller text size
  display.setTextSize(1);
  display.cp437(true); // Enable code page 437 for degree symbol
  display.write(248); // Degree symbol
  display.print("C");

  // Display N1 (rpm) on the second row
  display.setTextSize(2);
  display.setCursor(0, 24);
  display.print("N1 ");
  display.print(averageN1 / 1000.0, 1); // Convert RPM to krpm and print with 1 decimal place

  // Display the rpm unit with smaller text size
  display.setTextSize(1);
  display.print(" krpm");

  // Display the status message on the third row
  unsigned long currentMillis = millis();
  if (!statusMessages.empty()) {
    if (currentMillis - lastStatusDisplayTime >= statusInterval) {
      currentStatusIndex = (currentStatusIndex + 1) % statusMessages.size(); // Cycle to next status message
      lastStatusDisplayTime = currentMillis; // Update the time the status message was last displayed
    }
    display.setTextSize(1.5);
    display.setCursor(0, 54);
    display.println(statusMessages[currentStatusIndex]);
  } else {
    display.setTextSize(1.5);
    display.setCursor(0, 54);
    display.println(" ");
  }

  // Update the display with the buffer contents
  display.display();

  // Move servos based on average values
  int N1ServoPosition = map(averageN1, 0, 100000, 0, N1servoMaxAngle); // Map RPM (0 to 100000) to servo angle (0 to 180)
  int TOTServoPosition = map(averageTOT, 0, 800, 0, totservoMaxAngle); // Map TOT (0 to 800) to servo angle (0 to 180)

  N1Servo.write(N1ServoPosition);
  TOTServo.write(TOTServoPosition);
}
