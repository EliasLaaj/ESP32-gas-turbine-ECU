#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

//
// Use board serial pins(tx, rx)
// Set demo mode to 0 to read data from serial
bool demoMode = 1;

// Define ILI9341 pins
#define TFT_CS    15
#define TFT_RST   4
#define TFT_DC    2
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Value ranges
const int MAX_TOT = 750;       // TOT maximum value
const int MAX_N1 = 110;        // N1 maximum value in kRPM (110,000 RPM / 1000)
const int MAX_OIL = 5;         // Oil pressure maximum value

// Alarm limits
const int TOT_LIMIT = 650;
const int N1_LIMIT = 90000;
const float OIL_LIMIT = 2.0;

// Constants for ui
const int BAR_X_LOC = 210;
const int BAR_WIDTH = 12;
const int BAR_HEIGHT = 88;
const int TOT_POS = 5;
const int N1_POS = 103;
const int OIL_POS = 201;
const float VALUE_POS = 0.6;

// Current and previous values
int tot = 0, prevTot = -1;
int n1 = 0, prevN1 = -1;
float oil = 0.0, prevOil = -1.0;
String status = "";
String prevStatus = "";
unsigned long criticalStatusStartTime = 0;
bool blinkState = 0;
int blinkCounter = 0;

// Function prototypes
void updateValues();
void updateValuesDemo();
void updateScreen();
void drawVerticalBar(int x, int y, int width, int height, float value, float maxValue, uint16_t normalColor, uint16_t warningColor, uint16_t colorVar);
void updateStatus();
void drawVerticalBarOutLine(int x, int y, int width, int height, float maxValue);

void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(0); // Vertical orientation
  tft.fillScreen(ILI9341_BLACK);

  // Draw static parts of the screen (labels, bar outlines)
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);

  // Draw TOT Label
  tft.setCursor(10, TOT_POS + (BAR_HEIGHT * VALUE_POS));
  tft.print("TOT");

  // Draw N1 Label
  tft.setCursor(10, N1_POS + (BAR_HEIGHT * VALUE_POS));
  tft.print("N1");

  // Draw Oil Pressure Label
  tft.setCursor(10, OIL_POS + (BAR_HEIGHT * VALUE_POS));
  tft.print("OIL");

  drawVerticalBarOutLine(BAR_X_LOC, TOT_POS, BAR_WIDTH, BAR_HEIGHT, MAX_TOT);
  drawVerticalBarOutLine(BAR_X_LOC, N1_POS, BAR_WIDTH, BAR_HEIGHT, MAX_N1);
  drawVerticalBarOutLine(BAR_X_LOC, OIL_POS, BAR_WIDTH, BAR_HEIGHT, MAX_OIL);
}

void loop() {

  // Simulate changes in values
  if (demoMode){
    updateValuesDemo();
  }
  else{
    updateValues();
  }

  // Update only changed areas
  updateScreen();

  if (blinkCounter > 1) {
    blinkState = !blinkState;
    blinkCounter = 0;
  }
  else{
    blinkCounter++;
  }

  delay(100); // Refresh rate
}

void updateStatus() {
  if (criticalStatusStartTime != 0) {
    if (millis() - criticalStatusStartTime > 5000) {
      criticalStatusStartTime = 0;
      tft.fillRect(10, 285, 180, 20, ILI9341_BLACK);
    }
  }  
}

void updateScreen() {
  // TOT
  if (tot != prevTot) {
    uint16_t colorVar = (tot > TOT_LIMIT) ? ILI9341_RED : ILI9341_WHITE;
    // Alternate between showing the value and a black box if the color is red
    if (colorVar == ILI9341_RED && blinkState) {
      // Draw black box to "hide" the value
      tft.fillRect(10, 60, 120, 20, ILI9341_BLACK);
    }
    else{
    tft.setTextColor(colorVar, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, TOT_POS + (BAR_HEIGHT * VALUE_POS));
    tft.print("TOT ");
    tft.printf("%d", tot);
    tft.print(" ");
    tft.setTextSize(1);
    tft.print("C  ");
    drawVerticalBar(BAR_X_LOC, TOT_POS, BAR_WIDTH, BAR_HEIGHT, tot, MAX_TOT, ILI9341_WHITE, ILI9341_RED, colorVar);
    prevTot = tot;
    }
  }


  // N1
  if (n1 != prevN1) {
    uint16_t colorVar = (n1 > N1_LIMIT) ? ILI9341_RED : ILI9341_WHITE;
    // Alternate between showing the value and a black box if the color is red
    if (colorVar == ILI9341_RED && blinkState) {
      // Draw black box to "hide" the value
      tft.fillRect(10, 140, 140, 20, ILI9341_BLACK);
    }
    else {
      tft.setTextColor(colorVar, ILI9341_BLACK);
      tft.setTextSize(2);
      tft.setCursor(10, N1_POS + (BAR_HEIGHT * VALUE_POS));
      tft.print("N1 ");
      tft.printf("%.1f", n1 / 1000.0);
      tft.print(" ");
      tft.setTextSize(1);
      tft.print("kRPM   ");
      drawVerticalBar(BAR_X_LOC, N1_POS, BAR_WIDTH, BAR_HEIGHT, n1 / 1000.0, MAX_N1, ILI9341_WHITE, ILI9341_RED, colorVar);
      prevN1 = n1;
    }
  }

  // Oil Pressure
  if (oil != prevOil) {
    uint16_t colorVar = (oil < OIL_LIMIT) ? ILI9341_RED : ILI9341_WHITE;
     // Alternate between showing the value and a black box if the color is red
    if (colorVar == ILI9341_RED && blinkState) {
      // Draw black box to "hide" the value
      tft.fillRect(10, 220, 120, 20, ILI9341_BLACK);
    }
    else {
      tft.setTextColor(colorVar, ILI9341_BLACK);
      tft.setTextSize(2);
      tft.setCursor(10, OIL_POS + (BAR_HEIGHT * VALUE_POS));
      tft.print("OIL ");
      tft.printf("%.1f", oil);
      tft.print(" ");
      tft.setTextSize(1);
      tft.print("bar  ");
      drawVerticalBar(BAR_X_LOC, OIL_POS, BAR_WIDTH, BAR_HEIGHT, oil, MAX_OIL, ILI9341_WHITE, ILI9341_RED, colorVar);
      prevOil = oil;
    }
  }

  // Status
  if (status != prevStatus) {
    tft.fillRect(10, 300, 360, 20, ILI9341_BLACK); // Clear previous status
    tft.setCursor(10, 300);
    tft.setTextSize(2);
    uint16_t color = ILI9341_WHITE;
    if (status == "1") {
      color = ILI9341_GREEN;
    }
    else if (status == "2" || status == "10" || status == "11") {
      color = ILI9341_RED;
    }
    tft.setTextColor(color);
    
    // Status codes
    if (status == "1") tft.print("Running");
    else if (status == "2") tft.print("Relight active!");
    else if (status == "3") tft.print("Standby");
    else if (status == "4") tft.print("Ready to start.");
    else if (status == "5") tft.print("Stop switch active!");
    else if (status == "6") tft.print("Starting...");
    else if (status == "7") tft.print("Ignited...");
    else if (status == "8") tft.print("Shutting down.");
    else if (status == "9") tft.print("Shutdown complete.");
    else if (status == "10" || status == "11") { // If special status is active
      criticalStatusStartTime = millis();
      tft.setTextSize(1);
      tft.setCursor(10, 285);
      if (status == "10") tft.print("Ignition failed!");
      else if (status == "11") tft.print("TOT over limit!");
      tft.setTextSize(2);
    }
  }
  prevStatus = status;
  updateStatus();
}

// Function to draw a vertical bar (optimized for partial updates)
void drawVerticalBar(int x, int y, int width, int height, float value, float maxValue, uint16_t normalColor, uint16_t warningColor, uint16_t colorVar) {
  static float lastValue[3] = {0, 0, 0}; // Store previous values for 3 bars
  static uint16_t lastColor[3] = {0, 0, 0}; // Store previous colors for 3 bars

  int barIndex = (y == TOT_POS) ? 0 : (y == N1_POS) ? 1 : 2; // Identify bar by y-position
  height = height - 2;
  y = y + 2;

  // Restrict the value to the max value
  if (value > maxValue) value = maxValue;

  // Calculate new and old bar heights
  int newBarHeight = (value / maxValue) * height;
  int oldBarHeight = (lastValue[barIndex] / maxValue) * height;

  // Check if the color has changed
  if (colorVar != lastColor[barIndex]) {
    // Clear and redraw the entire bar
    tft.fillRect(x + 1, y + 1, width - 2, height - 2, ILI9341_BLACK);
    tft.fillRect(x + 1, y + height - newBarHeight - 1, width - 2, newBarHeight, colorVar);
  } else {
    // Only update changed pixels
    if (newBarHeight > oldBarHeight) {
      // Add new pixels
      tft.fillRect(x + 1, y + height - newBarHeight - 1, width - 2, newBarHeight - oldBarHeight, colorVar);
    } else if (newBarHeight < oldBarHeight) {
      // Remove excess pixels (clear only the difference)
      tft.fillRect(x + 1, y + height - oldBarHeight - 1, width - 2, oldBarHeight - newBarHeight, ILI9341_BLACK);
    }
  }

  // Store current value and color for next update
  lastValue[barIndex] = value;
  lastColor[barIndex] = colorVar;
}

// Function to draw a vertical bar outline (keeps the outline intact)
void drawVerticalBarOutLine(int x, int y, int width, int height, float maxValue) {
  // Draw bar outline
  tft.drawRect(x, y, width, height, ILI9341_WHITE);

  // Draw min and max values (without decimals)
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);

  // Min value at the bottom of the bar
  tft.setCursor(x - 20 , y + height - 8);
  tft.print("  0");

  // Max value at the top of the bar
  tft.setCursor(x - 20, y);
  if (maxValue == MAX_TOT) {
    tft.print("750"); // Max value for TOT
  } else if (maxValue == MAX_N1) {
    tft.print("110"); // Max value for N1 in kRPM
  } else {
    tft.print("  5"); // Max value for Oil Pressure
  }

  // Reset font size to 2 for other text
  tft.setTextSize(2);
}



void updateValues() { // Version of updateValues to read actual data
  while (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n'); // Read a line from Serial
    receivedData.trim(); // Remove any extra whitespace or newlines

    if (receivedData.startsWith("P:")) {
      oil = receivedData.substring(2).toFloat(); // Extract and convert Oil Pressure
    } 
    else if (receivedData.startsWith("T:")) {
      tot = receivedData.substring(2).toInt(); // Extract and convert TOT
    } 
    else if (receivedData.startsWith("R:")) {
      n1 = receivedData.substring(2).toInt(); // Extract and convert RPM
    } 
    else if (receivedData.startsWith("S:")) {
      status = receivedData.substring(2); // Extract Status as a string
    }
  }
}

void updateValuesDemo() { // Version of updateValues to simulate changes
  static int totDir = 1;  
  static int n1Dir = 1;  
  static int oilDir = 1;  
  // Simulate TOT (0 to 750)
  int change = random(5, 20) * totDir; // Random step in current direction
  tot += change;
  
  if (tot >= MAX_TOT) { 
    tot = MAX_TOT;
    totDir = -1;  // Reverse direction
  } 
  if (tot <= 0) { 
    tot = 0;
    totDir = 1;  // Start increasing again
  }

  // Simulate N1 RPM (0 to 100000)
  int changeN1 = random(500, 2000) * n1Dir;
  n1 += changeN1;
  
  if (n1 >= MAX_N1 * 1000) { 
    n1 = MAX_N1 * 1000;
    n1Dir = -1;
  } 
  if (n1 <= 0) { 
    n1 = 0;
    n1Dir = 1;
  }

  // Simulate Oil Pressure (0 to 4 bar)
  float changeOil = random(5, 20) / 100.0 * oilDir;
  oil += changeOil;
  
  if (oil >= MAX_OIL) { 
    oil = MAX_OIL;
    oilDir = -1;
  } 
  if (oil <= 0) { 
    oil = 0;
    oilDir = 1;
  }


  // Randomly set status to be a string between "1" and "11"
  status = String(random(1, 12));  // Generates a number between 1 and 11, converts to string
}
