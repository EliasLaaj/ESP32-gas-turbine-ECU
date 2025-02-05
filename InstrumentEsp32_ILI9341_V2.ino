#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// Define ILI9341 pins
#define TFT_CS    15
#define TFT_RST   4
#define TFT_DC    2
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Value ranges
const int MAX_TOT = 750;       // TOT maximum value
const int MAX_N1 = 110;        // N1 maximum value in kRPM (110,000 RPM / 1000)
const int MAX_OIL = 5;         // Oil pressure maximum value

// Current and previous values
int tot = 0, prevTot = -1;
int n1 = 0, prevN1 = -1;
float oil = 0.0, prevOil = -1.0;
String status = "";
String prevStatus = "";
unsigned long criticalStatusStartTime = 0;
bool blinkState = 0;

// Function prototypes
void updateValues();
void updateScreen();
void drawVerticalBar(int x, int y, int width, int height, float value, float maxValue, uint16_t normalColor, uint16_t warningColor, uint16_t colorVar);
void updateStatus();

void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(0); // Vertical orientation
  tft.fillScreen(ILI9341_BLACK);

  // Draw static parts of the screen (labels, bar outlines)
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);

  // Draw TOT Label
  tft.setCursor(10, 60);
  tft.print("TOT");

  // Draw N1 Label
  tft.setCursor(10, 140);
  tft.print("N1");

  // Draw Oil Pressure Label
  tft.setCursor(10, 220);
  tft.print("OIL");
}

void loop() {
  // Simulate changes in values
  updateValues();

  // Update only changed areas
  updateScreen();

  blinkState = !blinkState;

  delay(180); // Refresh rate
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
    uint16_t colorVar = (tot > 680) ? ILI9341_RED : ILI9341_WHITE;
    // Alternate between showing the value and a black box if the color is red
  if (colorVar == ILI9341_RED && blinkState) {
    // Draw black box to "hide" the value
    tft.fillRect(10, 60, 120, 20, ILI9341_BLACK);
  }
  else{
    tft.setTextColor(colorVar);
    tft.setTextSize(2);
    tft.setCursor(10, 60);
    tft.print("TOT");
    tft.fillRect(60, 60, 120, 20, ILI9341_BLACK); // Clear previous value
    tft.setCursor(60, 60);
    tft.setTextSize(2);
    tft.printf("%d", tot);
    tft.setTextSize(1);
    tft.print(" C");
    drawVerticalBar(200, 20, 20, 60, tot, MAX_TOT, ILI9341_WHITE, ILI9341_RED, colorVar);
    prevTot = tot;
  }
  }

  // N1
  if (n1 != prevN1) {
    uint16_t colorVar = (n1 > 90000) ? ILI9341_RED : ILI9341_WHITE;
    // Alternate between showing the value and a black box if the color is red
  if (colorVar == ILI9341_RED && blinkState) {
    // Draw black box to "hide" the value
    tft.fillRect(10, 140, 140, 20, ILI9341_BLACK);
  }
  else{
    tft.setTextColor(colorVar);
    tft.setTextSize(2);
    tft.setCursor(10, 140);
    tft.print("N1");
    tft.fillRect(60, 140, 120, 20, ILI9341_BLACK); // Clear previous value
    tft.setCursor(60, 140);
    tft.setTextSize(2);
    tft.printf("%.1f", n1 / 1000.0);
    tft.setTextSize(1);
    tft.print(" kRPM");
    drawVerticalBar(200, 100, 20, 60, n1 / 1000.0, MAX_N1, ILI9341_WHITE, ILI9341_RED, colorVar);
    prevN1 = n1;
  }
  }

  // Oil Pressure
  if (oil != prevOil) {
    uint16_t colorVar = (oil < 2.0) ? ILI9341_RED : ILI9341_WHITE;
     // Alternate between showing the value and a black box if the color is red
  if (colorVar == ILI9341_RED && blinkState) {
    // Draw black box to "hide" the value
    tft.fillRect(10, 220, 120, 20, ILI9341_BLACK);
  }
  else{
    tft.setTextColor(colorVar);
    tft.setTextSize(2);
    tft.setCursor(10, 220);
    tft.print("OIL");
    tft.fillRect(60, 220, 120, 20, ILI9341_BLACK); // Clear previous value
    tft.setCursor(60, 220);
    tft.setTextSize(2);
    tft.printf("%.1f", oil);
    tft.setTextSize(1);
    tft.print(" bar");
    drawVerticalBar(200, 180, 20, 60, oil, MAX_OIL, ILI9341_WHITE, ILI9341_RED, colorVar);
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
    } else if (status == "2" || status == "10" || status == "11") {
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

// Function to draw a vertical bar
void drawVerticalBar(int x, int y, int width, int height, float value, float maxValue, uint16_t normalColor, uint16_t warningColor, uint16_t colorVar) {
  // Restrict the value to the max value
  if (value > maxValue) value = maxValue;

  // Calculate bar height
  int barHeight = (value / maxValue) * height;

  // Clear previous bar area
  tft.fillRect(x, y, width, height, ILI9341_BLACK);

  // Draw filled bar with the color passed as parameter
  tft.fillRect(x, y + height - barHeight, width, barHeight, colorVar);

  // Draw bar outline
  tft.drawRect(x, y, width, height, ILI9341_WHITE);

  // Draw min and max values (without decimals)
  tft.setTextSize(1);  // Smaller font size for min/max
  tft.setTextColor(ILI9341_WHITE);

  // Min value at the bottom of the bar
  tft.setCursor(x - 30, y + height);  // Adjust position for min value
  tft.print("0");

  // Max value at the top of the bar
  tft.setCursor(x - 30, y);  // Adjust position for max value
  if (maxValue == MAX_TOT) {
    tft.print("750"); // Max value for TOT
  } else if (maxValue == MAX_N1) {
    tft.print("110"); // Max value for N1 in kRPM
  } else {
    tft.print("5"); // Max value for Oil Pressure
  }

  // Reset font size to 2 for other text
  tft.setTextSize(2);
}

// Function to simulate changes in values
// Direction variables to track increase/decrease


void updateValues() {
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
