#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include "GS_Bluetooth_BLE.h"
#include <OneBitDisplay.h>
#include <AnimatedGIF.h>
#include "intro-gif.h"
#include "Cipher.h"
#include <esp_system.h>
#include <esp_wifi.h>
#include "SPIFFSTest.h"

#include "TFTMirror.h"  // ST7789 mirroring without removing SSD1306

bool validateDeviceAuthenticity();
void protectionLoop();
void showProtectionMessage();

CSPIFFS mSpiffs;
Cipher *cipher = new Cipher();

enum SensitivityLevel {
  SENS_VERY_LOW = 1,
  SENS_LOW = 2,
  SENS_MEDIUM = 3,
  SENS_HIGH = 4,
  SENS_VERY_HIGH = 5
};

const char* sensitivityLabels[5] = {
  "VERY LOW", "LOW", "MEDIUM", "HIGH", "VERY HIGH"
};

struct SensitivitySnapshot {
  adsGain_t gain;
  int threshold;
  long min;
  long max;
  float smoothing;
};

SensitivitySnapshot initialSensitivity0;
bool sensitivity0Stored = false;

struct SensitivitySettings {
  adsGain_t gain;
  int detectionThreshold;
  long mapMin;
  long mapMax;
  float smoothingFactor;
};

// BLE configuration - no additional config needed for ESP32 BLE

const unsigned char linkIcon[] PROGMEM = {
  0x0C, 0x12, 0x12, 0x0C, 0x30, 0x48, 0x48, 0x30
};

const unsigned char bluetoothIcon[] PROGMEM = {
0xff, 0xff, 0xfb, 0xff, 0xf9, 0xff, 0xb8, 0xff, 0x9a, 0x7b, 0xc3, 0x29, 0xe2, 0x6d, 0xf0, 0xe5, 
0xf0, 0xe5, 0xe2, 0x6d, 0xc3, 0x29, 0x9a, 0x7b, 0xb8, 0xff, 0xf9, 0xff, 0xfb, 0xff, 0xff, 0xff
};

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

const int UP_BUTTON = 12;     
const int DOWN_BUTTON = 13;  
const int LEFT_BUTTON = 14;   
const int SELECT_BUTTON = 26; 
const int SEND_BUTTON = 5;   
const int BATTERY_PIN = 33;    

#define STARTUP_TONE 1000  
#define TONE_DURATION 100 
#define BUTTON_TONE_FREQ 800     
#define ALERT_TONE_FREQ 1500     
#define CONFIRM_TONE_FREQ 1200   
#define ERROR_TONE_FREQ 600      

#define STARTUP_MELODY_LENGTH 8
const int startupMelody[] = {523, 659, 784, 1047, 784, 659, 523, 1047}; // نغمة do-mi-sol-do-sol-mi-do-do
const int noteDurations[] = {100, 100, 100, 200, 100, 100, 100, 300};

// ST7789 pins/settings (keep OLED intact)
#define TFT_CS   17
#define TFT_RST  16
#define TFT_DC   25
#define TFT_BLK  27

TFTMirror tftMirror(TFT_CS, TFT_DC, TFT_RST, TFT_BLK);

GSBluetoothBLE SerialBT;
AnimatedGIF gif;
ONE_BIT_DISPLAY obd;
uint8_t ucFrameBuffer[(128*64) + ((128 * 64)/8)];
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;
const int buzzerPin = 2;

const float METAL_THRESHOLD = 5.0;  
const float VOID_THRESHOLD = -5.0;  
String statusMessage = "";
bool isBluetoothConnected = false;
unsigned long bluetoothConnectTime = 0;
String deviceName = "GS MAGNOVA PLUS"; 

float calibrationField1 = 0;
float calibrationField2 = 0;
bool isCalibrated = false;
unsigned long calibrationStartTime = 0;
const int CALIBRATION_DURATION = 5000; 
const int CALIBRATION_TIME = 5000;  
const int HISTORY_SIZE = 128;      
const int PULSE_HEIGHT = 20;       
float calibrationValues[100];     
float calibrationAverage = 0;      
int calibrationCount = 0;        
float pulseHistory[HISTORY_SIZE];  
int historyIndex = 0;             
bool gsSmartCalibrating = true;    
unsigned long gsSmartCalibrationStartTime = 0; 

int gsSmart2Selection = 0;
const int GS_SMART_2_COUNT = 2;
const char* TEXT_GS_SMART_2_EN[] = {"GS VISUAL", "GS MANUAL"};

bool gsManualCalibrating = true;
unsigned long gsManualCalibrationStartTime = 0;
bool gsManualSendButtonPressed = false;

float batteryVoltage = 0.0;
int batteryPercentage = 0;
unsigned long lastBatteryReadTime = 0;
const unsigned long batteryReadInterval = 30000; 
bool batteryIconVisible = true;
unsigned long lastBlinkTime = 0;

// معاملات معايرة البطارية
const float batteryCalibrationFactor = 1.15;  // معامل تصحيح لقراءة الجهد (تم زيادته بشكل كبير لضمان امتلاء المؤشر)
const float batteryFullVoltage = 4.15;       // جهد البطارية عند الشحن الكامل
const float batteryEmptyVoltage = 3.0;       // جهد البطارية عند الفراغ

unsigned long lastLinkIconBlinkTime = 0;
const unsigned long linkIconBlinkInterval = 500; 
bool linkIconState = true; 

bool autoScanActive = false;
unsigned long lastAutoScanTime = 0;
const unsigned long autoScanInterval = 2500; 


bool liveScanCalibrated = false;
bool liveScanCalibrationInProgress = false;
unsigned long liveScanCalibrationStartTime = 0;
const int LIVE_SCAN_CALIBRATION_TIME = 3000; 
const int LIVE_SCAN_CALIBRATION_SAMPLES = 100; 
int16_t liveScanCalibrationValues[LIVE_SCAN_CALIBRATION_SAMPLES]; 
int liveScanCalibrationCount = 0;
float liveScanCalibrationAverage = 0;
const int LIVE_SCAN_DETECTION_THRESHOLD = 300; 

unsigned long lastToneTime = 0;

enum MenuState {
  MAIN_MENU,
  SCAN_MENU,
  MANUAL_SCAN,
  AUTO_SCAN,
  LIVE_SCAN,
  SETTINGS,
  GS_SMART,
  GS_SMART_2,
  GS_MANUAL
};

int16_t text_x1, text_y1;
uint16_t text_w, text_h;

MenuState currentState = MAIN_MENU;
int menuSelection = 0;
const int menuItemCount = 3;
const char* TEXT_MAIN_MENU_EN[] = {"Scan Mode", "Settings", "GS SMART"};
const char* TEXT_MAIN_MENU[][3] = {
  {TEXT_MAIN_MENU_EN[0], ""},
  {TEXT_MAIN_MENU_EN[1], ""},
  {TEXT_MAIN_MENU_EN[2], ""}
};

String menuItems[menuItemCount];

const int scanMenuItemCount = 3;
const char* TEXT_SCAN_MENU_EN[] = {"MANUAL SCAN", "AUTO SCAN", "LIVE SCAN"};
const char* TEXT_SCAN_MENU[][3] = {
  {TEXT_SCAN_MENU_EN[0], ""},
  {TEXT_SCAN_MENU_EN[1], ""},
  {TEXT_SCAN_MENU_EN[2], ""}
};
String scanMenuItems[scanMenuItemCount];
int scanMenuSelection = 0;

const int SETTINGS_COUNT = 2; 
const char* TEXT_SETTINGS_EN[] = {"SENSITIVITY", "VOLUME"};
String settingsItems[SETTINGS_COUNT];
int currentSettingsSelection = 0;

int sensitivityLevel = 3; 

bool soundEnabled = true; 
int volumeLevel = 3; 

long DETECTION_THRESHOLD = 120; 
long mapMin = 0; 
long mapMax = 20000; 
float smoothingFactor = 0.3;


SensitivitySettings sensitivityPresets[5] = {
  {GAIN_TWOTHIRDS, 200, -20000, 20000, 0.2}, // VERY LOW - Extended range to prevent saturation
  {GAIN_TWOTHIRDS, 150, -16000, 16000, 0.25}, // LOW - Extended range to prevent saturation
  {GAIN_TWOTHIRDS, 120, -13000, 13000, 0.3},   // MEDIUM - Extended range to prevent saturation
  {GAIN_TWOTHIRDS, 100, -10000, 10000, 0.35},  // HIGH - Extended range to prevent saturation
  {GAIN_TWOTHIRDS,  80,  -8000,  8000, 0.4}    // VERY HIGH - Extended range to prevent saturation
};


int languageSelection = 0; // 0: English only

const char* TEXT_MISC_EN[] = {
  "CONNECTED", "NOT CONNECTED", "Press to select",
  "READY", "Scanning...", "Press to start", "Press to stop",
  "CALIBRATION", "METAL", "VOID", "NORMAL",
  "Value", "Press to change/hold to select"
};

String getText(const char* textEN) {
  return textEN;
}

void updateSensitivity() {
  const char* levelName = (sensitivityLevel == SENS_VERY_LOW) ? "VERY LOW" :
                           (sensitivityLevel == SENS_LOW) ? "LOW" :
                           (sensitivityLevel == SENS_MEDIUM) ? "MEDIUM" :
                           (sensitivityLevel == SENS_HIGH) ? "HIGH" : "VERY HIGH";
  
  showCalibrationMessage(levelName);
  
  showCalibrationProgress();
  
  int presetIndex = sensitivityLevel - 1;
  
  ads.setGain(sensitivityPresets[presetIndex].gain);
  delay(50);
  
  DETECTION_THRESHOLD = sensitivityPresets[presetIndex].detectionThreshold;
  mapMin = sensitivityPresets[presetIndex].mapMin;
  mapMax = sensitivityPresets[presetIndex].mapMax;
  smoothingFactor = sensitivityPresets[presetIndex].smoothingFactor;
  
  Serial.println("\n==== SENSITIVITY UPDATE ====\nLevel: " + String(levelName));
  Serial.println("Threshold: " + String(DETECTION_THRESHOLD));
  Serial.println("Range: " + String(mapMin) + " to " + String(mapMax));
  Serial.println("Smoothing: " + String(smoothingFactor));
  Serial.println("============================");
}
void drawText(int x, int y, const char* text) {
  display.setCursor(x, y);
  display.print(text);
}
int buttonToneDuration = 50;
void playButtonTone() {
  if (!soundEnabled) return;
  tone(buzzerPin, BUTTON_TONE_FREQ, buttonToneDuration);
}
void playConfirmTone() {
  if (!soundEnabled) return;
  
  int duration = 50; 
  tone(buzzerPin, CONFIRM_TONE_FREQ, duration);
  delay(duration);
  tone(buzzerPin, CONFIRM_TONE_FREQ + 200, duration);
}

void playAlertTone() {
  if (!soundEnabled) return;
  
  int duration = 80; 
  tone(buzzerPin, ALERT_TONE_FREQ, duration);
  delay(duration);
  tone(buzzerPin, ALERT_TONE_FREQ - 200, duration);
}

void playErrorTone() {
  if (!soundEnabled) return;
  
  int duration = 100; 
  tone(buzzerPin, ERROR_TONE_FREQ, duration);
  delay(duration / 2);
  tone(buzzerPin, ERROR_TONE_FREQ, duration);
}
void play_intro(){
   int iFrame;
  char szTemp[64];

  gif.begin(GIF_PALETTE_1BPP_OLED); // Choose the "OLED" version of 1-bpp output (vertical bytes, LSB on top)
  if (gif.open((uint8_t *)intro_gif, sizeof(intro_gif), NULL))
  {
    gif.setFrameBuf(ucFrameBuffer);
    gif.setDrawType(GIF_DRAW_COOKED);
    while (gif.playFrame(false, NULL)) { // live dangerously; run unthrottled :)
      obd.display(); 
    }
    gif.close();
  }
}

// Helper to flush mirrored display
inline void flushDisplay() {
  tftMirror.flush();
}

void setup() {
  if (sensitivityLevel < SENS_VERY_LOW || sensitivityLevel > SENS_VERY_HIGH) {
    sensitivityLevel = SENS_MEDIUM; // قيمة افتراضية آمنة
  }
  
  int initialPresetIndex = sensitivityLevel - 1;
  DETECTION_THRESHOLD = sensitivityPresets[initialPresetIndex].detectionThreshold;
  mapMin = sensitivityPresets[initialPresetIndex].mapMin;
  mapMax = sensitivityPresets[initialPresetIndex].mapMax;
  smoothingFactor = sensitivityPresets[initialPresetIndex].smoothingFactor;
  
  if (!sensitivity0Stored) {
    initialSensitivity0.gain = sensitivityPresets[0].gain;
    initialSensitivity0.threshold = sensitivityPresets[0].detectionThreshold;
    initialSensitivity0.min = sensitivityPresets[0].mapMin;
    initialSensitivity0.max = sensitivityPresets[0].mapMax;
    initialSensitivity0.smoothing = sensitivityPresets[0].smoothingFactor;
    sensitivity0Stored = true;
  }

  Serial.begin(9600); 
  SerialBT.begin(deviceName); 
  Serial.println("\n===================================");
  Serial.println("GS MAGNOVA - SYSTEM STARTUP");
  Serial.println("-----------------------------------");
  Serial.println("Device Name: " + String(deviceName));
  Serial.print("Initial Sensitivity Level: ");
  const char* initialLevelName;
  switch(sensitivityLevel) {
    case SENS_VERY_LOW: initialLevelName = "VERY LOW"; break;
    case SENS_LOW: initialLevelName = "LOW"; break;
    case SENS_MEDIUM: initialLevelName = "MEDIUM"; break;
    case SENS_HIGH: initialLevelName = "HIGH"; break;
    case SENS_VERY_HIGH: initialLevelName = "VERY HIGH"; break;
    default: initialLevelName = "UNKNOWN";
  }
  Serial.println(initialLevelName);
  
  int presetIndex = sensitivityLevel - 1;
  Serial.print("Gain: ");
  switch(sensitivityPresets[presetIndex].gain) {
    case GAIN_TWOTHIRDS: Serial.println("GAIN_TWOTHIRDS (2/3x)"); break;
    case GAIN_ONE: Serial.println("GAIN_ONE (1x)"); break;
    case GAIN_TWO: Serial.println("GAIN_TWO (2x)"); break;
    case GAIN_FOUR: Serial.println("GAIN_FOUR (4x)"); break;
    case GAIN_EIGHT: Serial.println("GAIN_EIGHT (8x)"); break;
    case GAIN_SIXTEEN: Serial.println("GAIN_SIXTEEN (16x)"); break;
    default: Serial.println("UNKNOWN");
  }
  
  Serial.print("Detection Threshold: ");
  Serial.println(sensitivityPresets[presetIndex].detectionThreshold);
  
  Serial.print("Map Range: ");
  Serial.print(sensitivityPresets[presetIndex].mapMin);
  Serial.print(" to ");
  Serial.println(sensitivityPresets[presetIndex].mapMax);
  
  Serial.print("Smoothing Factor: ");
  Serial.println(sensitivityPresets[presetIndex].smoothingFactor);
  
  Serial.println("Available Sensitivity Levels:");
  for (int i = 0; i < 5; i++) {
    Serial.print(i+1);
    Serial.print(": ");
    switch(i+1) {
      case SENS_VERY_LOW: Serial.print("VERY LOW"); break;
      case SENS_LOW: Serial.print("LOW"); break;
      case SENS_MEDIUM: Serial.print("MEDIUM"); break;
      case SENS_HIGH: Serial.print("HIGH"); break;
      case SENS_VERY_HIGH: Serial.print("VERY HIGH"); break;
    }
    Serial.print(" - Threshold: ");
    Serial.print(sensitivityPresets[i].detectionThreshold);
    Serial.print(", Range: ");
    Serial.print(sensitivityPresets[i].mapMin);
    Serial.print(" to ");
    Serial.println(sensitivityPresets[i].mapMax);
  }
  
  Serial.println("===================================");

  Serial.println("🔒 GS MAGNOVA Device Protection System");
  Serial.println("======================================");
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS initialization failed");
    while(1); 
  }
  if (!validateDeviceAuthenticity()) {
    Serial.println("❌ Device protection activated - Unauthorized device detected");
    Serial.println("🚫 Access denied - Device will not start");
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    display.clearDisplay();
    // Initialize ST7789 mirror as well for protection screen
    tftMirror.begin(240, 280, 0, 22, -1, 21, -1);
    tftMirror.attachDisplay(&display);
    protectionLoop(); 
  }
  
  Serial.println("✅ Device authenticity verified - Starting normal operation");
  Serial.println("======================================");
  delay(1000);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(SELECT_BUTTON, INPUT_PULLUP);
  pinMode(SEND_BUTTON, INPUT_PULLUP);  
  pinMode(BATTERY_PIN, INPUT);    

  obd.I2Cbegin(OLED_128x64);
  obd.setBuffer(&ucFrameBuffer[128*64]); 
  obd.fillScreen(OBD_WHITE); 
  obd.display();
  delay(1000);
  play_intro();
  delay(2000);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  Serial.println(F("SSD1306 allocation failed"));

  display.clearDisplay();
  flushDisplay();
  
  // Initialize ADS1115
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  ads.setGain(GAIN_TWOTHIRDS);
  
  // Initialize ST7789 mirror now that OLED is ready
  tftMirror.begin(240, 280, 0, 22, -1, 21, -1);
  tftMirror.attachDisplay(&display);
  tftMirror.setOffsets((240 - SCREEN_WIDTH)/2, (280 - SCREEN_HEIGHT)/2);
  
  // قراءة مستوى البطارية فوراً عند بدء التشغيل
  readBatteryLevel();
  lastBatteryReadTime = millis();
  
  delay(500); 
  
  delay(100);
  
  Serial.println("\n==== DIAGNOSTIC AT STARTUP ====");
  int16_t rawValue = ads.readADC_Differential_0_1();
  int mappedValue = map(rawValue, sensitivityPresets[sensitivityLevel - 1].mapMin, sensitivityPresets[sensitivityLevel - 1].mapMax, 0, 1023);
  int constrainedValue = constrain(mappedValue, 0, 1023);
  Serial.println("Raw ADS1115 Value: " + String(rawValue));
  Serial.println("Mapped Value (0-1023): " + String(mappedValue));
  Serial.println("Constrained Value (0-1023): " + String(constrainedValue));
  Serial.println("Initial mapMin: " + String(sensitivityPresets[sensitivityLevel - 1].mapMin));
  Serial.println("Initial mapMax: " + String(sensitivityPresets[sensitivityLevel - 1].mapMax));
  Serial.println("Initial Smoothing Factor: " + String(sensitivityPresets[sensitivityLevel - 1].smoothingFactor));
  Serial.print("Initial Gain: ");
  switch(sensitivityPresets[sensitivityLevel - 1].gain) {
    case GAIN_TWOTHIRDS: Serial.println("GAIN_TWOTHIRDS (2/3x)"); break;
    case GAIN_ONE: Serial.println("GAIN_ONE (1x)"); break;
    case GAIN_TWO: Serial.println("GAIN_TWO (2x)"); break;
    case GAIN_FOUR: Serial.println("GAIN_FOUR (4x)"); break;
    case GAIN_EIGHT: Serial.println("GAIN_EIGHT (8x)"); break;
    case GAIN_SIXTEEN: Serial.println("GAIN_SIXTEEN (16x)"); break;
    default: Serial.println("UNKNOWN");
  }
  
  Serial.println("==== END STARTUP DIAGNOSTIC ====");
  for (int i = 0; i < menuItemCount; i++) {
    menuItems[i] = TEXT_MAIN_MENU_EN[i];
  }
  
  for (int i = 0; i < scanMenuItemCount; i++) {
    scanMenuItems[i] = TEXT_SCAN_MENU_EN[i];
  }
  
  for (int i = 0; i < SETTINGS_COUNT; i++) {
    settingsItems[i] = TEXT_SETTINGS_EN[i];
  }
  
  display.clearDisplay();
  flushDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);  
  display.clearDisplay();
  display.setTextSize(2); // حجم الخط 2
  display.setTextColor(SSD1306_WHITE);
  String splashText = "MAGNOVA";
  display.getTextBounds(splashText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
  display.setCursor((SCREEN_WIDTH - text_w) / 2, 17);
  display.println(splashText);
  flushDisplay();
  const int barWidth = 100;
  const int barHeight = 6;
  const int barX = (SCREEN_WIDTH - barWidth) / 2;
  const int barY = 45;
  display.setTextSize(1);
  
  for (int progress = 0; progress <= 100; progress += 4) {
    display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
    display.fillRect(barX + 1, barY + 1, barWidth - 2, barHeight - 2, SSD1306_BLACK);
    int fillWidth = (progress * (barWidth - 2)) / 100;
    display.fillRect(barX + 1, barY + 1, fillWidth, barHeight - 2, SSD1306_WHITE);
    String percentText = String(progress) + "%";
    display.getTextBounds(percentText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.fillRect((SCREEN_WIDTH - text_w) / 2 - 2, barY + barHeight + 5, 
                    text_w + 4, 10, SSD1306_BLACK);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, barY + barHeight + 5);
    display.print(percentText);
    
    flushDisplay();
    delay(30);
  }
  
  delay(500); 
  
  for (int i = 0; i < STARTUP_MELODY_LENGTH; i++) {
    tone(buzzerPin, startupMelody[i], noteDurations[i]);
    delay(noteDurations[i] + 20); 
  }
  delay(500); 
  
  showMainMenu();
}

void loop() {
  checkBluetoothConnection();
  SerialBT.handle(); // معالجة Classic Bluetooth و BLE (مطابقة لـ LITE)
  SerialBT.loop(); // إضافة heartbeat للحفاظ على الاتصال
  
  if (millis() - lastBatteryReadTime > batteryReadInterval) {
    readBatteryLevel();
    lastBatteryReadTime = millis();
  }
  if (millis() - lastLinkIconBlinkTime > linkIconBlinkInterval) {
    lastLinkIconBlinkTime = millis();
    linkIconState = !linkIconState;
  }
  
  handleNavigation();
  switch (currentState) {
    case MAIN_MENU:
      showMainMenu();  
      break;
      
    case SCAN_MENU:
      showScanMenu();
      break;
      
    case MANUAL_SCAN:
      showManualScanScreen();
      handleManualScan();
      break;
      
    case AUTO_SCAN:
      runAutoScan();
      break;
      
    case LIVE_SCAN:
      showLiveScanScreen();
      break;
      
    case SETTINGS:
      runSettings();
      break;
      
    case GS_SMART:
      runGSSmart();
      break;
      
    case GS_SMART_2:
      runGSSmart2();
      break;
      
    case GS_MANUAL:
      runGSManual();
      break;
  }
  
  delay(100);
}

void checkBluetoothConnection() {
  SerialBT.handleReconnection(); // إدارة إعادة الاتصال التلقائي
  
  // فحص الاتصال الصحيح: BLE أو Classic SPP
  if (SerialBT.isConnected()) {
    isBluetoothConnected = true;
    bluetoothConnectTime = millis();
  } else {
    isBluetoothConnected = false;
  }
}

void handleNavigation() {
  static bool leftButtonPressed = false;
  if (digitalRead(LEFT_BUTTON) == LOW) {
    if (!leftButtonPressed) {  
      leftButtonPressed = true;
      if (currentState != MAIN_MENU) {
        if (currentState == MANUAL_SCAN || currentState == AUTO_SCAN || currentState == LIVE_SCAN) {
          currentState = SCAN_MENU;
        } else if (currentState == GS_SMART) {
          currentState = GS_SMART_2;
        } else if (currentState == GS_MANUAL) {
          currentState = GS_SMART_2;
        } else {
          currentState = MAIN_MENU;
          scanMenuSelection = 0;  
        }
        menuSelection = 0; 
        currentSettingsSelection = 0;  
        playButtonTone(); 
        delay(200);
      }
    }
  } else {
    leftButtonPressed = false; 
  }
  
  if (digitalRead(UP_BUTTON) == LOW) {
    switch (currentState) {
      case MAIN_MENU:
        menuSelection = (menuSelection + menuItemCount - 1) % menuItemCount;
        showMainMenu();
        break;
      case SCAN_MENU:
        scanMenuSelection = (scanMenuSelection + scanMenuItemCount - 1) % scanMenuItemCount;
        showScanMenu();
        break;
      case SETTINGS:
        currentSettingsSelection = (currentSettingsSelection + SETTINGS_COUNT - 1) % SETTINGS_COUNT;
        break;
      case GS_SMART_2:
        gsSmart2Selection = (gsSmart2Selection + GS_SMART_2_COUNT - 1) % GS_SMART_2_COUNT;
        break;
    }
    playButtonTone(); 
    delay(200);
  }
  
  if (digitalRead(DOWN_BUTTON) == LOW) {
    switch (currentState) {
      case MAIN_MENU:
        menuSelection = (menuSelection + 1) % menuItemCount;
        showMainMenu();
        break;
      case SCAN_MENU:
        scanMenuSelection = (scanMenuSelection + 1) % scanMenuItemCount;
        showScanMenu();
        break;
      case SETTINGS:
        currentSettingsSelection = (currentSettingsSelection + 1) % SETTINGS_COUNT;
        break;
      case GS_SMART_2:
        gsSmart2Selection = (gsSmart2Selection + 1) % GS_SMART_2_COUNT;
        break;
    }
    playButtonTone(); 
    delay(200);
  }
  
  if (digitalRead(SELECT_BUTTON) == LOW) {
    switch (currentState) {
      case MAIN_MENU:
        if (menuSelection == 0) {
          currentState = SCAN_MENU;
          playButtonTone(); 
          showScanMenu();
        } else if (menuSelection == 1) {
          currentState = SETTINGS;
          playButtonTone(); 
        } else if (menuSelection == 2) {
          currentState = GS_SMART_2;
          gsSmart2Selection = 0; 
          playButtonTone(); 
        }
        break;
        
      case SCAN_MENU:
        switch (scanMenuSelection) {
          case 0:
            currentState = MANUAL_SCAN;
            playButtonTone(); 
            break;
          case 1:
            currentState = AUTO_SCAN;
            playButtonTone(); 
            break;
          case 2:
            currentState = LIVE_SCAN;
            playButtonTone(); 
            break;
        }
        break;
        
      case SETTINGS:
        switch (currentSettingsSelection) {
          case 0: // SENSITIVITY
            selectSensitivityMenu(); 
            break;
          case 1: 
            soundEnabled = !soundEnabled; 
            updateVolume();
            break;
        }
        playButtonTone(); 
        delay(200);
        break;
        
      case GS_SMART_2:
        switch (gsSmart2Selection) {
          case 0: 
            currentState = GS_SMART;
            gsSmartCalibrating = true;
            gsSmartCalibrationStartTime = 0;
            playButtonTone(); 
            break;
          case 1: 
            currentState = GS_MANUAL; 
            playButtonTone(); 
            break;
        }
        break;
    }
  }
}

void showMainMenu() {
  display.clearDisplay();
  display.setTextSize(1.5); 
  display.setTextColor(SSD1306_WHITE);
  
  drawTopBar("MAIN MENU");
  
  display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
  
  for (int i = 0; i < menuItemCount; i++) {
    String menuText = TEXT_MAIN_MENU_EN[i];
    display.getTextBounds(menuText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    int xPos = (SCREEN_WIDTH - text_w) / 2;
    
    if (i == menuSelection) {
      display.fillRoundRect(10, 16 + (i * 16), SCREEN_WIDTH - 20, 14, 4, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(xPos, 19 + (i * 16));
      display.println(menuText);
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(xPos, 19 + (i * 16));
      display.println(menuText);
    }
  }
  
  flushDisplay();
}

void showScanMenu() {
  display.clearDisplay();
  display.setTextSize(1.5);  
  display.setTextColor(SSD1306_WHITE);
  drawTopBar("SCAN MODE");
  display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
  for (int i = 0; i < scanMenuItemCount; i++) {
    String menuText = TEXT_SCAN_MENU_EN[i];
    display.getTextBounds(menuText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    int xPos = (SCREEN_WIDTH - text_w) / 2;
    
    if (i == scanMenuSelection) {
      display.fillRoundRect(10, 16 + (i * 16), SCREEN_WIDTH - 20, 14, 4, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(xPos, 19 + (i * 16));
      display.println(menuText);
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(xPos, 19 + (i * 16));
      display.println(menuText);
    }
  }
  
  flushDisplay();
}

void showManualScanScreen() {
  display.clearDisplay();
  display.setTextSize(1); 
  display.setTextColor(SSD1306_WHITE);
  drawTopBar("MANUAL SCAN");
  display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
  int16_t difference = ads.readADC_Differential_0_1();
  if (!isBluetoothConnected) {
    display.drawBitmap(
      (SCREEN_WIDTH - 16) / 2,
      (SCREEN_HEIGHT - 16) / 2 - 5,
      bluetoothIcon, 16, 16, SSD1306_WHITE
    );
    
    String connectMsg = "Connect Bluetooth";
    display.getTextBounds(connectMsg.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, (SCREEN_HEIGHT / 2) + 15);
    display.println(connectMsg);
  } else {
    display.setTextSize(2);
    String readyMsg = "READY";
    display.getTextBounds(readyMsg.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 30);
    display.println(readyMsg);
    display.setTextSize(1);
    
    String sendMsg = "Press to Send";
    display.getTextBounds(sendMsg.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 50);
    display.println(sendMsg);
  }
  
  flushDisplay();
}

void handleManualScan() {
  if (!digitalRead(SEND_BUTTON) && isBluetoothConnected) {  

    int16_t channel0 = ads.readADC_SingleEnded(0);
    int16_t channel1 = ads.readADC_SingleEnded(1);
    int16_t channel2 = ads.readADC_SingleEnded(2);
    int16_t channel3 = ads.readADC_SingleEnded(3);
    
    int16_t difference = ads.readADC_Differential_0_1();
  
    Serial.println("\n==== MANUAL SCAN DEBUG ====");
    Serial.println("Channel 0 (Single): " + String(channel0));
    Serial.println("Channel 1 (Single): " + String(channel1));
    Serial.println("Channel 2 (Single): " + String(channel2));
    Serial.println("Channel 3 (Single): " + String(channel3));
    Serial.println("Differential 0-1: " + String(difference));
    Serial.println("Manual Difference (Ch0-Ch1): " + String(channel0 - channel1));
    Serial.println("Current mapMin: " + String(mapMin));
    Serial.println("Current mapMax: " + String(mapMax));
    Serial.println("Sensitivity Level: " + String(sensitivityLevel));
    
    int visualizer_data = map(difference, mapMin, mapMax, 0, 1023);
    visualizer_data = constrain(visualizer_data, 0, 1023); 
    
    Serial.println("Mapped Value (before constrain): " + String(map(difference, mapMin, mapMax, 0, 1023)));
    Serial.println("Final Sent Value: " + String(visualizer_data));
    Serial.println("==========================\n");
    
    SerialBT.println(visualizer_data);
    Serial.println("Sent: " + String(visualizer_data) + ", CH0:" + String(channel0) + ", CH1:" + String(channel1) + ", DIFF:" + String(difference));
    
    playConfirmTone();
    playButtonTone(); 
    
    delay(500);  
  }
}

void runAutoScan() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  drawTopBar("AUTO SCAN");
  display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
  
  if (!isBluetoothConnected) {
    display.drawBitmap(
      (SCREEN_WIDTH - 16) / 2,
      (SCREEN_HEIGHT - 16) / 2 - 5,
      bluetoothIcon, 16, 16, SSD1306_WHITE
    );
    String connectMsg = "Connect Bluetooth";
    display.getTextBounds(connectMsg.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, (SCREEN_HEIGHT / 2) + 15);
    display.println(connectMsg);
  } else {
    if (!autoScanActive) {
      display.drawRoundRect(14, 20, 100, 24, 4, SSD1306_WHITE);
      display.setCursor(20, 28);
      display.println(F("READY TO SCAN"));
    } else {
      display.drawRoundRect(14, 20, 100, 24, 4, SSD1306_WHITE);
      display.setCursor(20, 28);
      display.println(F("AUTO SCANNING"));
      
      unsigned long timeUntilNextScan = 0;
      if (millis() - lastAutoScanTime < autoScanInterval) {
        timeUntilNextScan = (autoScanInterval - (millis() - lastAutoScanTime)) / 1000;
      }
      display.setCursor(31, 53); 
      display.print(F("NEXT STEP: "));
      display.print(timeUntilNextScan);
      display.print(F("s"));
    }
  }
  
  flushDisplay();
  
  if (digitalRead(SELECT_BUTTON) == LOW && isBluetoothConnected && !autoScanActive) {
    autoScanActive = true;
    lastAutoScanTime = millis() - autoScanInterval; 
    tone(buzzerPin, 1500, 50);
    playButtonTone();
    delay(300); 
  }
  
  if (digitalRead(LEFT_BUTTON) == LOW && autoScanActive) {
    autoScanActive = false;
    tone(buzzerPin, 800, 50);
    delay(300); 
  }
  
  if (autoScanActive && isBluetoothConnected && (millis() - lastAutoScanTime >= autoScanInterval)) {
    int16_t channel0 = ads.readADC_SingleEnded(0);
    int16_t channel1 = ads.readADC_SingleEnded(1);
    int16_t channel2 = ads.readADC_SingleEnded(2);
    int16_t channel3 = ads.readADC_SingleEnded(3);
    
    int16_t difference = ads.readADC_Differential_0_1();
    
    Serial.println("\n==== AUTO SCAN DEBUG ====");
    Serial.println("Channel 0: " + String(channel0));
    Serial.println("Channel 1: " + String(channel1));
    Serial.println("Channel 2: " + String(channel2));
    Serial.println("Channel 3: " + String(channel3));
    Serial.println("Differential 0-1: " + String(difference));
    Serial.println("=========================");
      
    int visualizer_data = map(difference, mapMin, mapMax, 0, 1023);
    visualizer_data = constrain(visualizer_data, 0, 1023); 
    SerialBT.println(visualizer_data);
    
    Serial.println("Auto Sent: " + String(visualizer_data) + ", CH0:" + String(channel0) + ", CH1:" + String(channel1) + ", DIFF:" + String(difference));
      
    tone(buzzerPin, 1500, 50);
    lastAutoScanTime = millis();
  }
}

void showLiveScanScreen() {
  static bool appConnectionRequested = false;
  static unsigned long lastDataSendTime = 0;
  static bool isScanning = false;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.fillRect(0, 0, SCREEN_WIDTH, 14, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(35, 3);  
  display.println("LIVE SCAN");
  display.setTextColor(SSD1306_WHITE);
  
  if (!SerialBT.hasClient()) {
    display.setTextSize(1);
    display.setCursor(5, 30);   
    display.println("DEVICE NOT CONNECTED");
    display.setCursor(25, 45);
    display.println("WITH GS APP");
    
    isScanning = false;
    appConnectionRequested = false;
  } else {
    if (!isScanning) {
      isScanning = true;
      SerialBT.sendLiveScanStart();
    }
    
    display.setTextSize(1);
    display.setCursor(35, 30);  
    display.println("LIVE SCAN");
    display.setCursor(35, 45);  
    display.println("ACTIVATED");
    
    if (millis() - lastDataSendTime > 100) {
      // Read sensor data
      int16_t sensorReading = ads.readADC_Differential_0_1();
      
      int visualizer_data = map(sensorReading, mapMin, mapMax, 0, 1023);
      visualizer_data = constrain(visualizer_data, 0, 1023);
      
      Serial.println("\n==== LIVE SCAN DEBUG ====");
      Serial.println("Differential 0-1: " + String(sensorReading));
      Serial.println("Current mapMin: " + String(mapMin));
      Serial.println("Current mapMax: " + String(mapMax));
      Serial.println("Mapped Value: " + String(visualizer_data));
      Serial.println("==========================\n");
      
      SerialBT.println(visualizer_data);
      
      lastDataSendTime = millis();
    }
  }
  
  flushDisplay();
}

MenuState previousState = MAIN_MENU;

void runSettings() {
  previousState = SETTINGS;
  
  display.clearDisplay();
  display.setTextSize(1.5); 
  display.setTextColor(SSD1306_WHITE);
  
  drawTopBar("SETTINGS");
  
  display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
  
  for (int i = 0; i < SETTINGS_COUNT; i++) {
    int yPos = 25 + (i * 20); 
    
    if (i == currentSettingsSelection) {
      display.fillRoundRect(20, yPos - 1, SCREEN_WIDTH - 40, 14, 4, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    
    display.setCursor(25, yPos + 2);
    display.print(TEXT_SETTINGS_EN[i]);
    
    uint16_t valueWidth = 0;
    String valueText = "";
    
    switch (i) {
      case 0: // SENSITIVITY
        valueText = String(sensitivityLevel);
        break;
      case 1: // VOLUME
        valueText = soundEnabled ? "ON" : "OFF";
        break;
    }
    
    display.getTextBounds(valueText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor(SCREEN_WIDTH - 25 - text_w, yPos + 2);
    display.print(valueText);
  }
  
  flushDisplay();
  
  static bool selectButtonPressed = false;
  static unsigned long lastButtonPressTime = 0;
  unsigned long currentTime = millis();
  
  if (digitalRead(SELECT_BUTTON) == HIGH) {
    lastButtonPressTime = currentTime;
  }
  
  if (currentTime - lastButtonPressTime < 250) { 
    return; 
  }
  
  if (digitalRead(SELECT_BUTTON) == LOW) {
    if (!selectButtonPressed) { 
      selectButtonPressed = true;
      lastButtonPressTime = currentTime;
      
      switch (currentSettingsSelection) {
        case 0: 
          selectSensitivityMenu(); 
          break;
        case 1: 
          if (soundEnabled) { 
            tone(buzzerPin, 1500, 100);
            delay(100);
          }
          
          soundEnabled = !soundEnabled;
          updateVolume();
          Serial.println("Sound toggled to: " + String(soundEnabled ? "ON" : "OFF")); 
          break;
      }
      
      if (currentSettingsSelection != 1) { 
        playButtonTone(); 
      }
      delay(150); 
    }
  } else {
    selectButtonPressed = false; 
  }
}

void selectSensitivityMenu() {
  int currentIndex = sensitivityLevel - 1;
  bool selecting = true;

  while (selecting) {
    display.clearDisplay();
    drawTopBar("SENSITIVITY");
    
    display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
    
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    for (int i = 0; i < 5; i++) {
      int yPos = 18 + (i * 9);
      
      if (i == currentIndex) {
        display.fillRoundRect(10, yPos - 1, SCREEN_WIDTH - 20, 9, 2, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
      } else {
        display.setTextColor(SSD1306_WHITE);
      }
      
      display.setCursor(15, yPos);
      display.print(sensitivityLabels[i]);
    }
    
    flushDisplay();
    delay(100);

    if (digitalRead(UP_BUTTON) == LOW) {
      currentIndex = (currentIndex - 1 + 5) % 5;
      if (soundEnabled) {
        tone(buzzerPin, BUTTON_TONE_FREQ, TONE_DURATION);
      }
      delay(200);
    }
    if (digitalRead(DOWN_BUTTON) == LOW) {
      currentIndex = (currentIndex + 1) % 5;
      if (soundEnabled) {
        tone(buzzerPin, BUTTON_TONE_FREQ, TONE_DURATION);
      }
      delay(200);
    }
    if (digitalRead(SELECT_BUTTON) == LOW) {
      sensitivityLevel = currentIndex + 1;
      if (soundEnabled) {
        tone(buzzerPin, CONFIRM_TONE_FREQ, TONE_DURATION);
      }
      updateSensitivity();
      selecting = false;
      delay(200);
    }
    if (digitalRead(LEFT_BUTTON) == LOW) {
      if (soundEnabled) {
        tone(buzzerPin, BUTTON_TONE_FREQ, TONE_DURATION);
      }
      selecting = false;
      delay(200);
    }
  }
}

void updateVolume() {

  if (soundEnabled) {
    volumeLevel = 3;
    buttonToneDuration = 50; 
    
    tone(buzzerPin, 2000, 100);
    delay(120);
    tone(buzzerPin, 2500, 100);
  } else {
    volumeLevel = 0;
    
  }
  
  Serial.println("Sound: " + String(soundEnabled ? "ON" : "OFF"));
}

void runGSSmart() {
  static bool leftButtonPressed = false;
  if (digitalRead(LEFT_BUTTON) == LOW) {
    if (!leftButtonPressed) {  
      leftButtonPressed = true;
      currentState = MAIN_MENU;
      menuSelection = 0;
      gsSmartCalibrating = true;
      gsSmartCalibrationStartTime = 0;
      playButtonTone(); 
      delay(200);
      return;
    }
  } else {
    leftButtonPressed = false;  
  }
  
  if (gsSmartCalibrating) {
    if (gsSmartCalibrationStartTime == 0) {
      gsSmartCalibrationStartTime = millis();
      calibrationCount = 0;
      calibrationAverage = 0;
      memset(calibrationValues, 0, sizeof(calibrationValues));
      memset(pulseHistory, 0, sizeof(pulseHistory));
      historyIndex = 0;
    }
    
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(SSD1306_WHITE);
    
    String title = "GS VISUAL";
    display.getTextBounds(title.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 0);
    display.println(title);
    
    display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
    
    String calibText = "CALIBRATION";
    display.getTextBounds(calibText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 20);
    display.println(calibText);
    
    unsigned long elapsedTime = millis() - gsSmartCalibrationStartTime;
    int progress = (elapsedTime * 100) / CALIBRATION_TIME;
    progress = constrain(progress, 0, 100);
    
    display.drawRect(10, 35, 108, 10, SSD1306_WHITE);
    display.fillRect(10, 35, progress, 10, SSD1306_WHITE);
    
    String percentText = String(progress) + "%";
    display.getTextBounds(percentText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 50);
    display.print(percentText);
    
    flushDisplay();
    
    if (elapsedTime < CALIBRATION_TIME) {
      int16_t difference = ads.readADC_Differential_0_1();
      
      int visualizer_data = map(difference, mapMin, mapMax, 0, 1023);
      visualizer_data = constrain(visualizer_data, 0, 1023);
      
      calibrationValues[calibrationCount] = visualizer_data; 
      calibrationAverage += visualizer_data;
      calibrationCount++;
    } else {
      if (calibrationCount > 0) {
        calibrationAverage /= calibrationCount; 
      }
      gsSmartCalibrating = false;
      delay(500); 
    }
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    String title = "GS VISUAL";
    display.getTextBounds(title.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 0);
    display.println(title);
    
    display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
    
    int16_t currentDifference = ads.readADC_Differential_0_1();
    
    int current_visualizer_data = map(currentDifference, mapMin, mapMax, 0, 1023);
    current_visualizer_data = constrain(current_visualizer_data, 0, 1023);
    
    float deviation = current_visualizer_data - calibrationAverage;
    
    pulseHistory[historyIndex] = deviation * 2.0; 
    historyIndex = (historyIndex + 2) % HISTORY_SIZE; 
    
    int centerY = 32;
    for (int i = 0; i < HISTORY_SIZE - 1; i++) {
      int x1 = i;
      int x2 = i + 1;
      int y1 = centerY - (pulseHistory[i] * PULSE_HEIGHT / 100); 
      int y2 = centerY - (pulseHistory[(i + 1)] * PULSE_HEIGHT / 100);
      
      y1 = constrain(y1, 15, 50);
      y2 = constrain(y2, 15, 50);
      
      display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
    }
    
    display.setTextSize(1);
    String sensitivityType = "";
    
    const int DETECTION_THRESHOLD = 40; 
    
    if (deviation > DETECTION_THRESHOLD) {
      sensitivityType = "METAL";
    } else if (deviation < -DETECTION_THRESHOLD) {
      sensitivityType = "VOID";
    } else {
      sensitivityType = "NORMAL";
    }
    
    display.getTextBounds(sensitivityType.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 45);
    display.print(sensitivityType);
    
    display.setCursor(5, 55);
    display.print("REF:");
    display.print(int(calibrationAverage));
    
    display.setCursor(80, 55); 
    display.print("CUR:");
    display.print(current_visualizer_data);
        
    flushDisplay();
    
    float deviationAbs = abs(deviation);
    
    int toneInterval = 300 - constrain(deviationAbs - DETECTION_THRESHOLD, 0, 200);
    toneInterval = constrain(toneInterval, 50, 300); 
    
    if (deviation > DETECTION_THRESHOLD) { 
      if (millis() - lastToneTime > toneInterval) {
        playAlertTone();
        lastToneTime = millis();
      }
    } else if (deviation < -DETECTION_THRESHOLD) { 
      if (millis() - lastToneTime > toneInterval) {
        playErrorTone();
        lastToneTime = millis();
      }
    }
    
    delay(20);
  }
}

void readBatteryLevel() {
  int rawValue = analogRead(BATTERY_PIN);
  
  // حساب جهد البطارية مع تطبيق معامل التصحيح
  batteryVoltage = ((rawValue / 4095.0) * 3.3 * 2.0) * batteryCalibrationFactor;
  
  // حساب النسبة المئوية بناءً على نطاق الجهد المعاير
  batteryPercentage = ((batteryVoltage - batteryEmptyVoltage) / (batteryFullVoltage - batteryEmptyVoltage)) * 100.0;
  
  // التأكد من أن النسبة المئوية ضمن النطاق الصحيح
  batteryPercentage = constrain(batteryPercentage, 0, 100);
  
  // طباعة معلومات تشخيصية عن البطارية (للتطوير فقط)
  Serial.print("Battery Raw: "); Serial.print(rawValue);
  Serial.print(", Voltage: "); Serial.print(batteryVoltage, 2);
  Serial.print("V, Percentage: "); Serial.print(batteryPercentage);
  Serial.println("%");
}

void drawBatteryIcon(int x, int y) {
  // تحديث وميض البطارية إذا كانت ضعيفة
  if (batteryPercentage <= 20) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBlinkTime > 500) { // نصف ثانية وميض
      batteryIconVisible = !batteryIconVisible;
      lastBlinkTime = currentMillis;
    }
    if (!batteryIconVisible) {
      return; // لا ترسم الأيقونة في هذه اللحظة
    }
  }

  // رسم الإطار الخارجي للبطارية
  display.drawRect(x, y, 18, 8, SSD1306_WHITE); // مستطيل البطارية
  display.drawRect(x + 18, y + 2, 2, 4, SSD1306_WHITE); // رأس البطارية

  // تعبئة داخل البطارية حسب النسبة
  int fillWidth = map(batteryPercentage, 0, 100, 0, 16);
  if (fillWidth > 0) {
    display.fillRect(x + 1, y + 1, fillWidth, 6, SSD1306_WHITE);
  }
}

void drawTopBar(String title) {
  display.fillRect(0, 0, SCREEN_WIDTH, 12, SSD1306_BLACK);
  
  display.setTextColor(SSD1306_WHITE);
  display.getTextBounds(title.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
  display.setCursor((SCREEN_WIDTH - text_w) / 2, 2);
  display.println(title);
  
  if (isBluetoothConnected) {
    if (linkIconState) {
      display.drawBitmap(4, 2, linkIcon, 8, 8, SSD1306_WHITE);
    }
  } else {
    display.drawBitmap(4, 2, linkIcon, 8, 8, SSD1306_WHITE);
  }
  
  drawBatteryIcon(SCREEN_WIDTH - 22, 2);
  
  display.setTextColor(SSD1306_WHITE);
}

void showWelcomeScreen() {
  if (!isCalibrated) {
    showCalibrationScreen();
    return;
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  drawTopBar("GS SMART");
  
  display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
  
  flushDisplay();
}

void showCalibrationScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  drawTopBar("CALIBRATION");
  
  display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
  
  display.setCursor(5, 25);
  display.println(F("Place on metal surface"));
  display.setCursor(5, 35);
  display.println(F("Press to calibrate"));
  
  if (digitalRead(SELECT_BUTTON) == LOW) {
    calibrationStartTime = millis();
    calibrationField1 = 0;
    calibrationField2 = 0;
    int samples = 0;
    
    tone(buzzerPin, 2000, 100);
    delay(100);
    tone(buzzerPin, 2500, 100);
    playButtonTone(); 
    
    while (millis() - calibrationStartTime < CALIBRATION_DURATION) {
      float readMagneticField0 = ads.readADC_SingleEnded(0);
      float readMagneticField1 = ads.readADC_SingleEnded(1);
      calibrationField1 += readMagneticField0; 
      calibrationField2 += readMagneticField1;
      samples++;
      
      int progress = map(millis() - calibrationStartTime, 0, CALIBRATION_DURATION, 0, 100);
      
      display.fillRect(0, 16, SCREEN_WIDTH, SCREEN_HEIGHT - 16, SSD1306_BLACK); 
      
      String calibText = "Calibrating...";
      display.getTextBounds(calibText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
      display.setCursor((SCREEN_WIDTH - text_w) / 2, 25);
      display.println(calibText);
      
      int barWidth = SCREEN_WIDTH - 40;
      int barHeight = 10;
      int barX = (SCREEN_WIDTH - barWidth) / 2;
      int barY = SCREEN_HEIGHT / 2;
      
      display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
      int fillWidth = map(progress, 0, 100, 0, barWidth);
      display.fillRect(barX, barY, fillWidth, barHeight, SSD1306_WHITE);
      
      String percentText = String(progress) + "%";
      display.getTextBounds(percentText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
      display.setCursor((SCREEN_WIDTH - text_w) / 2, barY + barHeight + 10);
      display.println(percentText);
      
      flushDisplay();
      delay(50);
    }
    
    calibrationField1 /= samples;
    calibrationField2 /= samples;
    isCalibrated = true;
    
    display.fillRect(0, 16, SCREEN_WIDTH, SCREEN_HEIGHT - 16, SSD1306_BLACK);
    String doneText = "Calibration Done!";
    display.getTextBounds(doneText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, SCREEN_HEIGHT / 2);
    display.println(doneText);
    flushDisplay();
    
    delay(2000);  
    
    currentState = MAIN_MENU;
  }
  
  flushDisplay();
}

void runGSSmart2() {
  display.clearDisplay();
  display.setTextSize(1.5); 
  display.setTextColor(SSD1306_WHITE);
  
  drawTopBar("GS SMART 2");
  
  display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
  
  for (int i = 0; i < GS_SMART_2_COUNT; i++) {
    String menuText = TEXT_GS_SMART_2_EN[i];
    display.getTextBounds(menuText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    int xPos = (SCREEN_WIDTH - text_w) / 2;
    
    int yPos = 25 + (i * 20); 
    
    if (i == gsSmart2Selection) {
      display.fillRoundRect(10, yPos - 1, SCREEN_WIDTH - 20, 14, 4, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(xPos, yPos + 2);
      display.println(menuText);
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(xPos, yPos + 2);
      display.println(menuText);
    }
  }
  
  flushDisplay();
}

void runGSManual() {
  if (gsManualCalibrating && gsManualCalibrationStartTime == 0) {
    gsManualCalibrationStartTime = millis();
    calibrationCount = 0;
    calibrationAverage = 0;
    memset(calibrationValues, 0, sizeof(calibrationValues));
  }
  
  if (gsManualCalibrating) {
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(SSD1306_WHITE);
    
    String title = "GS MANUAL";
    display.getTextBounds(title.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 0);
    display.println(title);
    
    display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
    
    String calibText = "CALIBRATION";
    display.getTextBounds(calibText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 20);
    display.println(calibText);
    
    unsigned long elapsedTime = millis() - gsManualCalibrationStartTime;
    int progressPercent = (elapsedTime * 100) / CALIBRATION_TIME;
    progressPercent = constrain(progressPercent, 0, 100);
    
    int progressWidth = (progressPercent * 100) / 100;
    display.drawRect(14, 35, 100, 10, SSD1306_WHITE);
    display.fillRect(14, 35, progressWidth, 10, SSD1306_WHITE);
    
    String percentText = String(progressPercent) + "%";
    display.getTextBounds(percentText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 50);
    display.print(percentText);
    
    flushDisplay();
    
    if (elapsedTime < CALIBRATION_TIME) {
      int16_t difference = ads.readADC_Differential_0_1();
      
      calibrationValues[calibrationCount] = difference;
      calibrationAverage += difference;
      calibrationCount++;
    } else {
      calibrationAverage /= calibrationCount;
      gsManualCalibrating = false;
      delay(500); 
    }
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    String title = "GS MANUAL";
    display.getTextBounds(title.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 0);
    display.println(title);
    
    display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
    
    display.setTextSize(1.5);
    String smartDataText = "SMART DATA";
    display.getTextBounds(smartDataText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 20);
    display.println(smartDataText);
    
    display.setTextSize(1);
    String pressManualText = "PRESS MANUAL SCAN";
    display.getTextBounds(pressManualText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor((SCREEN_WIDTH - text_w) / 2, 35);
    display.println(pressManualText);
    
    display.drawRoundRect(34, 45, 60, 16, 4, SSD1306_WHITE);
    
    String sendText = "SEND DATA";
    display.getTextBounds(sendText.c_str(), 0, 0, &text_x1, &text_y1, &text_w, &text_h);
    display.setCursor(39, 49);
    display.println(sendText);
    
    if (gsManualSendButtonPressed) {
      display.drawBitmap(14, 45, bluetoothIcon, 16, 16, SSD1306_WHITE);
    }
    
    flushDisplay();
    
    if (digitalRead(SEND_BUTTON) == LOW && isBluetoothConnected) {
      gsManualSendButtonPressed = true;
      int16_t currentDifference = ads.readADC_Differential_0_1();  
      
      float deviation = currentDifference - calibrationAverage;
      
      int deviationPercent = constrain(abs(int((deviation / calibrationAverage) * 100)), 0, 100);
      
      int visualizer_data = map(currentDifference, mapMin, mapMax, 0, 1023);
      visualizer_data = constrain(visualizer_data, 0, 1023); 
      
      SerialBT.println(visualizer_data);
      
      Serial.println("Sent: " + String(visualizer_data));
      
      playConfirmTone();
      playButtonTone(); 
      
      display.fillRect(34, 45, 60, 16, SSD1306_BLACK);
      display.drawRoundRect(34, 45, 60, 16, 4, SSD1306_WHITE);
      display.setCursor(39, 49);
      display.println("SENT OK");
      flushDisplay();
      
      delay(1000); 
      
      display.fillRect(34, 45, 60, 16, SSD1306_BLACK);
      display.drawRoundRect(34, 45, 60, 16, 4, SSD1306_WHITE);
      display.setCursor(39, 49);
      display.println("SEND DATA");
      flushDisplay();
      
      delay(200);
    } else {
      gsManualSendButtonPressed = false;
    }
  }
}

void showCalibrationMessage(const char* levelName) {
  display.clearDisplay();
  
  int16_t x1, y1;
  uint16_t textWidth, textHeight;
  
  int rectWidth = SCREEN_WIDTH - 4;
  int rectHeight = 55; 
  int rectX = (SCREEN_WIDTH - rectWidth) / 2;
  int rectY = ((SCREEN_HEIGHT - rectHeight) / 2) + 5; 
  
  display.fillRoundRect(rectX, rectY, rectWidth, rectHeight, 8, WHITE);
  
  display.setTextSize(1); 
  display.setTextColor(BLACK); 
  
  display.getTextBounds("ACTIVATED", 0, 0, &x1, &y1, &textWidth, &textHeight);
  int activatedX = rectX + (rectWidth - textWidth) / 2;
  int activatedY = rectY + 15;
  
  display.setCursor(activatedX, activatedY);
  display.println("ACTIVATED");
  
  display.setTextSize(1); 
  
  display.getTextBounds(levelName, 0, 0, &x1, &y1, &textWidth, &textHeight);
  int levelX = rectX + (rectWidth - textWidth) / 2;
  int levelY = rectY + 30; 
  
  display.setCursor(levelX, levelY);
  display.println(levelName);
  
  display.getTextBounds("SENSITIVITY", 0, 0, &x1, &y1, &textWidth, &textHeight);
  int sensX = rectX + (rectWidth - textWidth) / 2;
  int sensY = rectY + 45; 
  
  display.setCursor(sensX, sensY);
  display.println("SENSITIVITY");
  
  flushDisplay();
  delay(1500); 
}

void showCalibrationProgress() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(35, 25);
  display.println("CALIBRATION");
  
  int barX = 10;
  int barY = 38; 
  int barWidth = 108;
  int barHeight = 12;
  
  display.drawRect(barX, barY, barWidth, barHeight, WHITE);
  
  for (int i = 0; i <= 100; i += 5) {
    display.fillRect(barX + 1, barY + 1, barWidth - 2, barHeight - 2, BLACK);
    
    int progressWidth = map(i, 0, 100, 0, barWidth - 2);
    display.fillRect(barX + 1, barY + 1, progressWidth, barHeight - 2, WHITE);
    
    String percentText = String(i) + "%";
    display.fillRect(45, 52, 40, 10, BLACK); 
    display.setCursor(50, 52);
    display.print(percentText);
    
    flushDisplay();
    delay(50); 
  }
  
  delay(500);
}

/**
 * التحقق من صحة الجهاز عبر مقارنة MAC المشفر
 * @return true إذا كان الجهاز أصلي، false إذا كان منسوخ
 */
bool validateDeviceAuthenticity() {
  Serial.println("🔐 Starting device authenticity check...");
  
  String storedEncryptedMac = mSpiffs.getFile(SPIFFS, "/gs_device_auth.dat");
  
  if (storedEncryptedMac == "File does not exist" || storedEncryptedMac.length() == 0) {
    Serial.println("❌ Protection file not found - Device not protected");
    return false;
  }
  
  uint8_t actualMac[6];
  esp_read_mac(actualMac, ESP_MAC_BT);
  
  String currentMacString = "";
  for (int i = 0; i < 6; i++) {
    currentMacString += String(actualMac[i], HEX);
  }
  
  Serial.println("📱 Current MAC: " + currentMacString);
  
  char *protectionKey = (char*)"GS_MAGNOVA_SECUR";
  cipher->setKey(protectionKey);
  
  String firstDecryption = cipher->decryptString(storedEncryptedMac);
  String originalMac = cipher->decryptString(firstDecryption);
  
  String cleanOriginalMac = "";
  for (int i = 0; i < originalMac.length(); i++) {
    char c = originalMac.charAt(i);
    if (c >= 32 && c <= 126) { 
      cleanOriginalMac += c;
    }
  }
  originalMac = cleanOriginalMac;
  
  Serial.println("🔓 Decrypted MAC: " + originalMac);
  
  Serial.println("=== DETAILED DEBUG ===");
  Serial.println("Original MAC length: " + String(originalMac.length()));
  Serial.println("Current MAC length: " + String(currentMacString.length()));
  
  Serial.print("Original MAC bytes: ");
  for (int i = 0; i < originalMac.length(); i++) {
    Serial.print(String((int)originalMac.charAt(i)) + " ");
  }
  Serial.println();
  
  Serial.print("Current MAC bytes: ");
  for (int i = 0; i < currentMacString.length(); i++) {
    Serial.print(String((int)currentMacString.charAt(i)) + " ");
  }
  Serial.println();
  
  originalMac.trim();
  currentMacString.trim();
  
  Serial.println("After trim - Original: '" + originalMac + "'");
  Serial.println("After trim - Current: '" + currentMacString + "'");
  
  bool isValid = (originalMac.equals(currentMacString));
  
  if (isValid) {
    Serial.println("✅ Device authenticity verified - Original device");
  } else {
    Serial.println("❌ Device authenticity failed - Cloned device detected");
    Serial.println("Expected: '" + originalMac + "'");
    Serial.println("Current:  '" + currentMacString + "'");
  }
  
  return isValid;
}

void showProtectionMessage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.drawRect(5, 15, SCREEN_WIDTH - 10, 35, SSD1306_WHITE);
  
  display.setCursor(15, 20);
  display.println("DEVICE PROTECTION");
  display.setCursor(25, 30);
  display.println("UNAUTHORIZED");
  display.setCursor(35, 40);
  display.println("ACCESS");
  
  flushDisplay();
}

void protectionLoop() {
  while (true) {
    showProtectionMessage();
    delay(1000);
    
    display.clearDisplay();
    flushDisplay();
    delay(500);
  }
}

