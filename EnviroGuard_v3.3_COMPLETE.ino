/*
================================================
  EnviroGuard Smart Gas Monitor
  ESP32 + MQ7 + Gas Sensor — v3.3 FINAL
================================================

  المميزات الأساسية:
  ✅ CO Detection (MQ7)
  ✅ Gas Meter Reading
  ✅ Blynk Integration
  ✅ LCD Display (16x2)
  ✅ Alerts (LEDs + Buzzer)
  ✅ WiFi Auto-Reconnect
  ✅ Auto Calibration

  بدون: SD Card, DHT, Battery Stats
================================================
*/

#include "secrets.h"

#include <WiFi.h>
#include <WiFiManager.h>
#include <BlynkSimpleEsp32.h>
#include <Preferences.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

/* ========================================= */
/*  DEBUG MODE                              */
/* ========================================= */

#define DEBUG_SERIAL 1

#if DEBUG_SERIAL
  #define LOG(x) Serial.println(x)
  #define LOGP(x) Serial.print(x)
#else
  #define LOG(x)
  #define LOGP(x)
#endif

/* ========================================= */
/*  HARDWARE PINS                           */
/* ========================================= */

#define MQ7_PIN          34    // CO Sensor Analog Input
#define GAS_SENSOR_PIN   35    // Gas Meter/Sensor Analog Input

#define LED_GREEN        16    // Safe Status
#define LED_YELLOW       17    // Warning Status
#define LED_RED          18    // Danger/Critical Status

#define BUZZER           19    // Audio Alert

/* ========================================= */
/*  CO SAFETY THRESHOLDS (PPM)             */
/* ========================================= */

#define SAFE_PPM          30

#define WARNING_ENTER     70
#define WARNING_EXIT      50
#define WARNING_MARGIN    5

#define DANGER_ENTER      120
#define DANGER_EXIT       90
#define DANGER_MARGIN     5

#define CRITICAL_ENTER    180
#define CRITICAL_EXIT     140
#define CRITICAL_MARGIN   5

/* ========================================= */
/*  MQ7 CALIBRATION CONSTANTS              */
/* ========================================= */

#define MQ7_RL           10.0f    // Load Resistance
#define MQ7_A            99.042f  // Curve Fit Coefficient
#define MQ7_B            -1.518f  // Curve Fit Exponent

#define CALIB_SAMPLES    100      // Samples to average
#define CALIB_TIME       60000UL  // 60 seconds
#define SPIKE_THRESHOLD  2.0f     // Spike detection multiplier
#define SPIKE_SMOOTHING  0.2f     // Smoothing factor

/* ========================================= */
/*  TIMING INTERVALS (milliseconds)        */
/* ========================================= */

#define SENSOR_INTERVAL      1000UL   // Read CO every 1 second
#define BLYNK_INTERVAL       5000UL   // Send to Blynk every 5 seconds
#define SCREEN_INTERVAL      3000UL   // Toggle display every 3 seconds
#define WARMUP_TIME          20000UL  // 20 second warmup
#define WiFi_CHECK_INTERVAL  30000UL  // Check WiFi every 30 seconds
#define BLYNK_RECONNECT_INT  10000UL  // Retry Blynk every 10 seconds

/* ========================================= */
/*  BLYNK VIRTUAL PINS (V0-V7)             */
/* ========================================= */

#define V_CO_PPM         V0   // Current CO Level (ppm)
#define V_CO_PREDICTED   V1   // Predicted CO (10 sec ahead)
#define V_CO_RATE        V2   // Rate of change (ppm/sec)
#define V_GAS_READING    V3   // Gas Sensor/Meter Reading
#define V_LED_GREEN      V4   // Green LED Status (0/255)
#define V_LED_YELLOW     V5   // Yellow LED Status (0/255)
#define V_LED_RED        V6   // Red LED Status (0/255)
#define V_WiFi_RSSI      V7   // WiFi Signal Strength (dBm)

/* ========================================= */
/*  HARDWARE OBJECTS                       */
/* ========================================= */

hd44780_I2Cexp lcd;        // I2C LCD display (16x2)
WiFiManager wm;            // WiFi configuration manager

/* ========================================= */
/*  LCD CUSTOM CHARACTERS (WiFi Icons)     */
/* ========================================= */

byte wifiWeak[8]   = {B00000, B00000, B00000, B00000, B00100, B00000, B00100, B00000};
byte wifiMedium[8] = {B00000, B00000, B00000, B00100, B01010, B00000, B01010, B00000};
byte wifiStrong[8] = {B00000, B00100, B01010, B10001, B00000, B10001, B01010, B00100};

/* ========================================= */
/*  CO SENSOR STATE STRUCT                 */
/* ========================================= */

struct {
  float filteredPPM = 0;       // Exponential moving average
  float previousPPM = 0;       // Last filtered value
  float rate = 0;              // PPM change per sample
  float previousRate = 0;      // Last rate of change
  float predicted10 = 0;       // Predicted value in 10 seconds
  float RO = 10.0f;            // Resistance at clean air (auto-calibrated)
  bool calibDone = false;      // Calibration complete flag
  uint8_t alertState = 0;      // 0=SAFE, 1=WARNING, 2=DANGER, 3=CRITICAL
  float alphaRise = 0.40f;     // EMA alpha for rising values
  float alphaFall = 0.55f;     // EMA alpha for falling values
} co;

/* ========================================= */
/*  GAS SENSOR STATE STRUCT                */
/* ========================================= */

struct {
  float reading = 0.0f;        // Current gas sensor reading
  float maxReading = 0.0f;     // Peak reading since startup
  unsigned long lastUpdateTime = 0;
} gas;

/* ========================================= */
/*  SYSTEM STATE STRUCT                    */
/* ========================================= */

struct {
  bool wifiConnected = false;
  bool warmupDone = false;
  bool screenToggle = false;   // Alternates between CO and GAS display
  bool criticalLock = false;   // Prevents screen clearing during CRITICAL
  bool buzzerState = false;    // Current buzzer state
  
  // Timing trackers
  unsigned long lastSensorRead = 0;
  unsigned long lastBlynkSend = 0;
  unsigned long lastScreenSwitch = 0;
  unsigned long lastWiFiCheck = 0;
  unsigned long lastBlynkReconnect = 0;
  unsigned long lastBuzzerToggle = 0;
  unsigned long startupTime = 0;
} sys;

/* ========================================= */
/*  LCD DISPLAY CACHE (Prevent Flicker)    */
/* ========================================= */

String lastLine0 = "";
String lastLine1 = "";

/* ========================================= */
/*  PREFERENCES (NVS STORAGE - EEPROM)     */
/* ========================================= */

void saveRO(float ro) {
  Preferences p;
  p.begin("enviro", false);  // false = read-write mode
  p.putFloat("ro", ro);
  p.end();
  LOG("[PREF] RO saved: " + String(ro));
}

float loadRO() {
  Preferences p;
  p.begin("enviro", true);   // true = read-only mode
  float ro = p.getFloat("ro", 0.0f);
  p.end();
  return ro;
}

/* ========================================= */
/*  LCD WRITE (Smart - Prevent Flicker)    */
/* ========================================= */

void lcdWrite(uint8_t col, uint8_t row, String text, uint8_t padTo = 16) {
  // Pad text to exact width
  while ((int)text.length() < padTo) {
    text += ' ';
  }
  text = text.substring(0, padTo);

  // During CRITICAL — always write (high priority display)
  if (co.alertState == 3) {
    lcd.setCursor(col, row);
    lcd.print(text);
    return;
  }

  // Normal mode — only write if changed (reduce I2C traffic)
  String& cache = (row == 0) ? lastLine0 : lastLine1;
  if (cache == text) {
    return;  // No change, skip write
  }

  cache = text;
  lcd.setCursor(col, row);
  lcd.print(text);
}

void lcdClearAll() {
  lcd.clear();
  lastLine0 = "";
  lastLine1 = "";
}

/* ========================================= */
/*  STARTUP ANIMATION                      */
/* ========================================= */

void startup() {
  lcdClearAll();
  String name = "EnviroGuard";
  
  // Character-by-character display
  for (int i = 0; i < (int)name.length(); i++) {
    lcd.setCursor(i, 0);
    lcd.print(name[i]);
    delay(80);
  }
  
  lcd.setCursor(0, 1);
  lcd.print("v3.3 Final");
  delay(1200);
}

/* ========================================= */
/*  DRAW WiFi ICON (Top-right corner)      */
/* ========================================= */

void drawWiFi() {
  lcd.setCursor(15, 0);
  
  if (WiFi.status() != WL_CONNECTED) {
    lcd.print(" ");
    return;
  }
  
  int rssi = WiFi.RSSI();
  if      (rssi > -50)  lcd.write(byte(2));  // Strong
  else if (rssi > -70)  lcd.write(byte(1));  // Medium
  else                  lcd.write(byte(0));  // Weak
}

/* ========================================= */
/*  MQ7: RAW ADC VALUE → RS (Sensor Resistance) */
/* ========================================= */

float rawToRS(int raw) {
  // Prevent division errors
  if (raw <= 0) {
    return MQ7_RL * 1000.0f;
  }
  
  float voltage = (float)raw * 3.3f / 4095.0f;
  
  if (voltage < 0.001f) {
    return MQ7_RL * 1000.0f;
  }
  
  // RS = ((VCC - Vout) / Vout) * RL
  return ((3.3f - voltage) / voltage) * MQ7_RL;
}

/* ========================================= */
/*  MQ7: RS → PPM (CO Concentration)      */
/* ========================================= */

float rsToPPM(float rs) {
  float ratio = rs / co.RO;
  
  if (ratio <= 0) {
    return 0;
  }
  
  // PPM = A * (RS/RO)^B
  float ppm = MQ7_A * pow(ratio, MQ7_B);
  
  return (ppm < 0) ? 0 : ppm;
}

/* ========================================= */
/*  SPIKE FILTER (Remove Sensor Noise)     */
/* ========================================= */

float spikeFilter(float newPPM) {
  // Skip filter if not initialized
  if (co.filteredPPM < 1.0f) {
    return newPPM;
  }

  // Detect impossible jumps
  if (newPPM > co.filteredPPM * SPIKE_THRESHOLD && newPPM > 20.0f) {
    float smoothed = co.filteredPPM * (1.0f - SPIKE_SMOOTHING) 
                   + newPPM * SPIKE_SMOOTHING;
    LOG("[SPIKE] " + String(newPPM, 1) + " → " + String(smoothed, 1));
    return smoothed;
  }

  return newPPM;
}

/* ========================================= */
/*  UPDATE PREDICTION (10 seconds ahead)   */
/* ========================================= */

void updatePrediction(float newReading) {
  // Apply spike filter
  newReading = spikeFilter(newReading);

  // Store previous for rate calculation
  co.previousPPM = co.filteredPPM;

  // Exponential Moving Average (faster response to rises, slower to falls)
  float alpha = (newReading > co.filteredPPM) ? co.alphaRise : co.alphaFall;
  co.filteredPPM = alpha * newReading + (1.0f - alpha) * co.filteredPPM;

  // Calculate rate of change
  co.previousRate = co.rate;
  co.rate = co.filteredPPM - co.previousPPM;

  // Acceleration (second derivative)
  float acceleration = co.rate - co.previousRate;

  // Weight for projection based on velocity
  float weight;
  if      (abs(co.rate) > 6)  weight = 1.3f;  // Fast change — high confidence
  else if (abs(co.rate) > 3)  weight = 1.1f;  // Moderate change
  else                        weight = 0.85f; // Slow change — conservative

  // Predict value 10 seconds ahead
  co.predicted10 = co.filteredPPM 
                 + (co.rate * 10.0f * weight)
                 + (acceleration * 5.0f);

  // Prevent impossible predictions
  if (co.rate < 0 && co.predicted10 > co.filteredPPM) {
    co.predicted10 = co.filteredPPM;
  }
  if (co.predicted10 < 0) {
    co.predicted10 = 0;
  }

  // Debug logging
  LOGP("[CO] raw=");
  LOGP(String(newReading, 1));
  LOGP(" filt=");
  LOGP(String(co.filteredPPM, 1));
  LOGP(" rate=");
  LOGP(String(co.rate, 2));
  LOGP(" p10=");
  LOG(String(co.predicted10, 1));
}

/* ========================================= */
/*  HYSTERESIS STATE MACHINE               */
/* ========================================= */

void updateAlertState() {
  switch (co.alertState) {
    
    case 0:  // SAFE
      if (co.filteredPPM >= CRITICAL_ENTER) {
        co.alertState = 3;
      } else if (co.filteredPPM >= DANGER_ENTER) {
        co.alertState = 2;
      } else if (co.filteredPPM >= WARNING_ENTER) {
        co.alertState = 1;
      }
      break;

    case 1:  // WARNING
      if (co.filteredPPM >= CRITICAL_ENTER) {
        co.alertState = 3;
      } else if (co.filteredPPM >= DANGER_ENTER) {
        co.alertState = 2;
      } else if (co.filteredPPM < (WARNING_EXIT - WARNING_MARGIN)) {
        co.alertState = 0;
      }
      break;

    case 2:  // DANGER
      if (co.filteredPPM >= CRITICAL_ENTER) {
        co.alertState = 3;
      } else if (co.filteredPPM <= (DANGER_EXIT + DANGER_MARGIN)) {
        co.alertState = 1;
      }
      break;

    case 3:  // CRITICAL
      if (co.filteredPPM < (CRITICAL_EXIT - CRITICAL_MARGIN)) {
        co.alertState = 2;
      }
      break;
  }
}

/* ========================================= */
/*  BUZZER CONTROL                         */
/* ========================================= */

void updateBuzzer(unsigned long interval) {
  unsigned long now = millis();
  
  if (now - sys.lastBuzzerToggle >= interval) {
    sys.buzzerState = !sys.buzzerState;
    digitalWrite(BUZZER, sys.buzzerState ? HIGH : LOW);
    sys.lastBuzzerToggle = now;
  }
}

void buzzerOff() {
  sys.buzzerState = false;
  digitalWrite(BUZZER, LOW);
}

/* ========================================= */
/*  ALERT SYSTEM (LEDs + Buzzer)           */
/* ========================================= */

void updateAlerts() {
  // Turn off all LEDs first
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED,    LOW);

  switch (co.alertState) {
    
    case 3:  // CRITICAL
      digitalWrite(LED_RED, HIGH);
      updateBuzzer(100);  // Fast beep (100ms toggle)
      break;

    case 2:  // DANGER
      digitalWrite(LED_RED, HIGH);
      updateBuzzer(400);  // Slower beep (400ms toggle)
      break;

    case 1:  // WARNING
      buzzerOff();
      digitalWrite(LED_YELLOW, HIGH);
      break;

    default:  // SAFE (0)
      buzzerOff();
      digitalWrite(LED_GREEN, HIGH);
      break;
  }
}

/* ========================================= */
/*  AUTO CALIBRATION (Clean Air)           */
/* ========================================= */

void runCalibration() {
  lcdClearAll();
  lcdWrite(0, 0, "Calibrating...");

  float rsSum = 0;
  int samples = 0;
  unsigned long calStart = millis();
  unsigned long lastSample = 0;

  // Collect samples over CALIB_TIME
  while (millis() - calStart < CALIB_TIME) {
    unsigned long now = millis();

    int secsLeft = (int)((CALIB_TIME - (now - calStart)) / 1000);
    lcdWrite(0, 1, "Clean air: " + String(secsLeft) + "s ");

    // Sample every 1 second
    if (now - lastSample >= 1000UL) {
      int raw = analogRead(MQ7_PIN);
      rsSum += rawToRS(raw);
      samples++;
      lastSample = now;
      LOG("[CAL] Sample " + String(samples) + " RS=" + String(rawToRS(raw)));
    }

    delay(50);
  }

  // Calculate average RO and save
  if (samples > 0) {
    float rsAvg = rsSum / samples;
    co.RO = rsAvg;

    saveRO(co.RO);

    LOG("[CAL] Done! RO=" + String(co.RO));

    lcdClearAll();
    lcdWrite(0, 0, "Cal Done!");
    lcdWrite(0, 1, "RO=" + String(co.RO, 2));
    delay(2000);
  }

  co.calibDone = true;
}

/* ========================================= */
/*  READ GAS SENSOR (FIXED)                */
/* ========================================= */

void readGasSensor() {
  unsigned long now = millis();
  
  // Rate limit: once per second
  if (now - gas.lastUpdateTime < 1000UL) {
    return;
  }

  gas.lastUpdateTime = now;

  // Read raw ADC value
  int rawGas = analogRead(GAS_SENSOR_PIN);
  
  // Convert to 0-1000 range (adjust based on your sensor datasheet)
  gas.reading = (rawGas / 4095.0f) * 1000.0f;
  
  // Track peak
  if (gas.reading > gas.maxReading) {
    gas.maxReading = gas.reading;
  }

  LOGP("[GAS] Raw=");
  LOGP(String(rawGas));
  LOGP(" Value=");
  LOG(String(gas.reading, 2));
}

/* ========================================= */
/*  WiFi CONNECTION CHECK                  */
/* ========================================= */

void checkWiFi() {
  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    sys.wifiConnected = true;
    sys.lastWiFiCheck = now;
    return;
  }

  sys.wifiConnected = false;

  // Reconnect attempt every 30 seconds
  if (now - sys.lastWiFiCheck >= WiFi_CHECK_INTERVAL) {
    LOG("[WiFi] Attempting reconnect...");
    WiFi.reconnect();
    sys.lastWiFiCheck = now;
  }
}

/* ========================================= */
/*  BLYNK CONNECTION CHECK                 */
/* ========================================= */

void checkBlynk() {
  if (!sys.wifiConnected) {
    return;  // No WiFi, can't connect to Blynk
  }
  
  if (Blynk.connected()) {
    return;  // Already connected
  }

  unsigned long now = millis();
  
  // Attempt reconnect every 10 seconds
  if (now - sys.lastBlynkReconnect < BLYNK_RECONNECT_INT) {
    return;
  }

  sys.lastBlynkReconnect = now;
  LOG("[Blynk] Attempting connect...");
  Blynk.connect(3000);
}

/* ========================================= */
/*  SEND DATA TO BLYNK                     */
/* ========================================= */

void sendToBlynk() {
  if (!sys.wifiConnected || !Blynk.connected()) {
    return;
  }

  // CO Data
  Blynk.virtualWrite(V_CO_PPM, co.filteredPPM);
  Blynk.virtualWrite(V_CO_PREDICTED, co.predicted10);
  Blynk.virtualWrite(V_CO_RATE, co.rate);

  // Gas Sensor
  Blynk.virtualWrite(V_GAS_READING, gas.reading);

  // WiFi Signal
  Blynk.virtualWrite(V_WiFi_RSSI, WiFi.RSSI());

  // LED States (0 = off, 255 = on)
  Blynk.virtualWrite(V_LED_GREEN,  (co.alertState == 0) ? 255 : 0);
  Blynk.virtualWrite(V_LED_YELLOW, (co.alertState == 1) ? 255 : 0);
  Blynk.virtualWrite(V_LED_RED,    (co.alertState >= 2) ? 255 : 0);

  // CRITICAL Alert (max once every 5 minutes)
  static unsigned long lastNotify = 0;
  unsigned long now = millis();

  if (co.alertState == 3 && now - lastNotify > 300000UL) {
    Blynk.logEvent("critical_co", 
      "CRITICAL CO: " + String((int)co.filteredPPM) + " ppm! EVACUATE!");
    lastNotify = now;
    LOG("[Blynk] CRITICAL alert sent");
  }

  LOG("[Blynk] Data sent");
}

/* ========================================= */
/*  BLYNK EVENT HANDLERS                   */
/* ========================================= */

BLYNK_WRITE(V_GAS_READING) {
  // Handle incoming gas reading from Blynk app (if needed)
  float value = param.asFloat();
  LOG("[Blynk] Gas value received: " + String(value, 2));
}

/* ========================================= */
/*  SETUP (Initialization)                 */
/* ========================================= */

void setup() {
  Serial.begin(115200);
  delay(500);

  LOG("\n\n[=== EnviroGuard v3.3 FINAL Starting ===]\n");

  /* --- Blynk Token Validation --- */
  #ifndef BLYNK_AUTH_TOKEN
    Serial.println("\n[ERROR] BLYNK_AUTH_TOKEN missing in secrets.h!");
    Serial.println("Define it as: #define BLYNK_AUTH_TOKEN \"your_token_here\"\n");
    while (1) {
      delay(1000);
    }
  #endif

  if (strlen(BLYNK_AUTH_TOKEN) < 10) {
    Serial.println("\n[ERROR] BLYNK_AUTH_TOKEN too short or invalid!\n");
    while (1) {
      delay(1000);
    }
  }

  LOG("[Setup] Blynk token verified");

  /* --- GPIO Setup --- */
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  pinMode(BUZZER,     OUTPUT);

  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED,    LOW);
  digitalWrite(BUZZER,     LOW);

  LOG("[Setup] GPIO pins configured");

  /* --- LCD Initialization --- */
  Wire.begin();
  lcd.begin(16, 2);
  
  // Define custom characters
  lcd.createChar(0, wifiWeak);
  lcd.createChar(1, wifiMedium);
  lcd.createChar(2, wifiStrong);

  LOG("[Setup] I2C LCD initialized");

  // Show startup animation
  startup();

  /* --- Load Calibration (RO) --- */
  float savedRO = loadRO();

  if (savedRO > 0.5f && savedRO < 500.0f) {
    // Valid RO found in storage
    co.RO = savedRO;
    co.calibDone = true;
    LOG("[CAL] Loaded from storage: RO=" + String(co.RO));

    lcdClearAll();
    lcdWrite(0, 0, "Cal loaded");
    lcdWrite(0, 1, "RO=" + String(co.RO, 2));
    delay(1500);
  } else {
    LOG("[CAL] No valid RO found — will calibrate at startup");
  }

  /* --- WiFi Setup --- */
  WiFi.mode(WIFI_STA);
  lcdClearAll();
  lcdWrite(0, 0, "Connecting WiFi");
  lcdWrite(0, 1, "...");

  wm.setConfigPortalTimeout(60);  // 60 sec portal timeout
  wm.setConnectTimeout(15);       // 15 sec connect attempt

  if (wm.autoConnect("EnviroGuard")) {
    // WiFi connected successfully
    sys.wifiConnected = true;
    sys.lastWiFiCheck = millis();

    LOG("[WiFi] Connected: " + WiFi.localIP().toString());

    lcdClearAll();
    lcdWrite(0, 0, "WiFi OK");
    lcdWrite(0, 1, WiFi.localIP().toString());
    delay(2000);

    /* --- Blynk Setup --- */
    lcdClearAll();
    lcdWrite(0, 0, "Blynk connect..");

    Blynk.config(BLYNK_AUTH_TOKEN);
    
    if (Blynk.connect(5000)) {
      LOG("[Blynk] Connected!");
      lcdClearAll();
      lcdWrite(0, 0, "Blynk OK");
      delay(1500);
    } else {
      LOG("[Blynk] Connection timeout — will retry in loop");
      lcdClearAll();
      lcdWrite(0, 0, "Blynk retry...");
      delay(1500);
    }
  } else {
    // WiFi connection failed
    sys.wifiConnected = false;
    lcdClearAll();
    lcdWrite(0, 0, "WiFi Failed");
    lcdWrite(0, 1, "Offline Mode");
    delay(2000);
    LOG("[WiFi] Offline mode active");
  }

  lcdClearAll();

  /* --- Warmup Timer --- */
  sys.startupTime = millis();
  sys.warmupDone = false;

  LOG("[Setup] Initialization complete!\n");
}

/* ========================================= */
/*  MAIN LOOP                               */
/* ========================================= */

void loop() {

  /* --- Blynk & WiFi Management --- */
  if (sys.wifiConnected) {
    Blynk.run();
    checkWiFi();
    checkBlynk();
  }

  unsigned long now = millis();

  /* ===== WARMUP PHASE (20 sec) ===== */
  if (!sys.warmupDone) {
    int secsLeft = (int)((WARMUP_TIME - (now - sys.startupTime)) / 1000);
    
    if (secsLeft > 0) {
      lcdWrite(0, 0, "Warming up...  ");
      lcdWrite(0, 1, "Wait: " + String(secsLeft) + "s    ");
      
      digitalWrite(LED_GREEN,  LOW);
      digitalWrite(LED_YELLOW, HIGH);  // Yellow during warmup
      digitalWrite(LED_RED,    LOW);
      
      delay(100);
      return;  // Exit loop, try again next iteration
    }

    // Warmup complete
    sys.warmupDone = true;
    lcdClearAll();

    /* --- Auto Calibration (if needed) --- */
    if (!co.calibDone) {
      runCalibration();
    }
  }

  /* ===== READ CO SENSOR ===== */
  if (now - sys.lastSensorRead >= SENSOR_INTERVAL) {
    int raw = analogRead(MQ7_PIN);
    float rs = rawToRS(raw);
    float ppm = rsToPPM(rs);

    updatePrediction(ppm);
    updateAlertState();
    updateAlerts();

    sys.lastSensorRead = now;
  }

  /* ===== READ GAS SENSOR ===== */
  readGasSensor();

  /* ===== SEND TO BLYNK ===== */
  if (now - sys.lastBlynkSend >= BLYNK_INTERVAL) {
    sendToBlynk();
    sys.lastBlynkSend = now;
  }

  /* ===== CRITICAL STATE DISPLAY ===== */
  if (co.alertState == 3) {
    if (!sys.criticalLock) {
      lcdClearAll();
      sys.criticalLock = true;
    }

    lcdWrite(0, 0, "!!! CRITICAL !!!");
    lcdWrite(0, 1, "CO:" + String((int)co.filteredPPM) + "ppm EVAC!");
    drawWiFi();
    
    delay(50);
    return;  // High priority display loop
  }

  /* ===== EXIT CRITICAL MODE ===== */
  if (sys.criticalLock) {
    lcdClearAll();
    sys.criticalLock = false;
  }

  /* ===== NORMAL DISPLAY (Alternating) ===== */
  if (now - sys.lastScreenSwitch >= SCREEN_INTERVAL) {
    sys.screenToggle = !sys.screenToggle;
    sys.lastScreenSwitch = now;
  }

  if (sys.screenToggle) {
    // Screen 1: CO Levels
    lcdWrite(0, 0, "CO:" + String((int)co.filteredPPM) + "ppm");
    lcdWrite(0, 1, "P10:" + String((int)co.predicted10) + "ppm");
  } else {
    // Screen 2: Gas Sensor + Status
    lcdWrite(0, 0, "Gas:" + String((int)gas.reading) + "ppm");
    
    String status;
    switch (co.alertState) {
      case 0:
        status = "SAFE";
        break;
      case 1:
        status = "WARNING";
        break;
      case 2:
        status = "DANGER";
        break;
      case 3:
        status = "CRITICAL";
        break;
      default:
        status = "?";
    }
    
    lcdWrite(0, 1, "Status:" + status);
  }

  drawWiFi();
  delay(50);
}

/* ================================================ */
/*  END OF SKETCH                                  */
/* ================================================ */
