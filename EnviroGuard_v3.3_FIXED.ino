/*
================================================
  EnviroGuard Smart Gas Monitor - v3.3 FIXED
  ESP32 + MQ7 + Gas Sensor
================================================

  ✅ CO Detection (MQ7)
  ✅ Gas Meter Reading
  ✅ LCD Display (I2C)
  ✅ Alerts (LEDs + Buzzer)
  ✅ WiFi Auto-Reconnect (standalone)
  ✅ Memory optimized
  ✅ No Blynk / No secrets.h

================================================
*/

#include <WiFi.h>
#include <WiFiManager.h>
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

#define MQ7_PIN          34    // CO Sensor (ADC1)
#define GAS_SENSOR_PIN   35    // Gas Meter (ADC1)

#define LED_GREEN        16
#define LED_YELLOW       17
#define LED_RED          18

#define BUZZER           19

/* ========================================= */
/*  CO SAFETY THRESHOLDS (PPM)              */
/* ========================================= */

#define SAFE_PPM          30
#define WARNING_ENTER     70
#define WARNING_EXIT      50
#define DANGER_ENTER      120
#define DANGER_EXIT       90
#define CRITICAL_ENTER    180
#define CRITICAL_EXIT     140

/* ========================================= */
/*  MQ7 CALIBRATION CONSTANTS               */
/* ========================================= */

#define MQ7_RL           10.0f   // Load resistance
#define MQ7_A            99.042f // Calibration constant A
#define MQ7_B            -1.518f // Calibration constant B
#define CALIB_TIME       60000UL // 60 seconds
#define SPIKE_THRESHOLD  2.0f
#define SPIKE_SMOOTHING  0.2f

/* ========================================= */
/*  TIMING (milliseconds)                   */
/* ========================================= */

#define SENSOR_INTERVAL      1000UL
#define SCREEN_INTERVAL      3000UL
#define WARMUP_TIME          20000UL
#define WiFi_CHECK_INTERVAL  30000UL  // FIXED: Was WiFi_Check_INTERVAL

/* ========================================= */
/*  LCD CUSTOM CHARACTERS                   */
/* ========================================= */

byte wifiWeak[8]   = {B00000,B00000,B00000,B00000,B00100,B00000,B00100,B00000};
byte wifiMedium[8] = {B00000,B00000,B00000,B00100,B01010,B00000,B01010,B00000};
byte wifiStrong[8] = {B00000,B00100,B01010,B10001,B00000,B10001,B01010,B00100};

/* ========================================= */
/*  CO SENSOR STATE                         */
/* ========================================= */

struct {
  float filteredPPM = 0;
  float previousPPM = 0;
  float rate = 0;
  float previousRate = 0;
  float predicted10 = 0;
  float RO = 10.0f;
  bool calibDone = false;
  uint8_t alertState = 0;  // 0=SAFE, 1=WARNING, 2=DANGER, 3=CRITICAL
  float alphaRise = 0.40f;
  float alphaFall = 0.55f;
} co;

/* ========================================= */
/*  GAS SENSOR STATE                        */
/* ========================================= */

struct {
  float reading = 0.0f;
  float maxReading = 0.0f;
  unsigned long lastUpdateTime = 0;
} gas;

/* ========================================= */
/*  SYSTEM STATE                            */
/* ========================================= */

struct {
  bool wifiConnected = false;
  bool warmupDone = false;
  bool screenToggle = false;
  bool criticalLock = false;
  bool buzzerState = false;
  
  unsigned long lastSensorRead = 0;
  unsigned long lastScreenSwitch = 0;
  unsigned long lastWiFiCheck = 0;
  unsigned long lastBuzzerToggle = 0;
  unsigned long startupTime = 0;
} sys;

/* ========================================= */
/*  LCD OBJECTS                             */
/* ========================================= */

hd44780_I2Cexp lcd;
WiFiManager wm;

/* ========================================= */
/*  PREFERENCES (EEPROM) - RO CALIBRATION   */
/* ========================================= */

void saveRO(float ro) {
  Preferences p;
  p.begin("enviro", false);
  p.putFloat("ro", ro);
  p.end();
  LOG("[PREF] RO saved: " + String(ro, 4));
}

float loadRO() {
  Preferences p;
  p.begin("enviro", true);
  float ro = p.getFloat("ro", 0.0f);
  p.end();
  if (ro > 0) {
    LOG("[PREF] RO loaded: " + String(ro, 4));
  }
  return ro;
}

/* ========================================= */
/*  LCD SAFE WRITE (Buffer to prevent flicker) */
/* ========================================= */

static String lastLine0 = "";
static String lastLine1 = "";

void lcdWrite(uint8_t col, uint8_t row, const char* text) {
  String& cache = (row == 0) ? lastLine0 : lastLine1;
  String newText = text;
  
  // Pad/trim to 16 chars
  while (newText.length() < 16) newText += ' ';
  newText = newText.substring(0, 16);
  
  // Only write if changed (or in CRITICAL mode)
  if (cache != newText || co.alertState == 3) {
    cache = newText;
    lcd.setCursor(col, row);
    lcd.print(newText.c_str());
  }
}

void lcdClearAll() {
  lcd.clear();
  lastLine0 = "";
  lastLine1 = "";
}

/* ========================================= */
/*  STARTUP ANIMATION                       */
/* ========================================= */

void startup() {
  lcdClearAll();
  const char* name = "EnviroGuard";
  for (int i = 0; i < 11; i++) {
    lcd.setCursor(i, 0);
    lcd.write(name[i]);
    delay(80);
  }
  lcd.setCursor(0, 1);
  lcd.print("v3.3 Fixed");
  delay(1500);
  lcdClearAll();
}

/* ========================================= */
/*  WIFI ICON ON LCD                        */
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
/*  MQ7: RAW ADC → RS RESISTANCE             */
/* ========================================= */

float rawToRS(int raw) {
  if (raw <= 0) return MQ7_RL * 1000.0f;
  
  float voltage = (float)raw * 3.3f / 4095.0f;
  if (voltage < 0.001f) return MQ7_RL * 1000.0f;
  
  // RS = RL * (Vcc - Vout) / Vout
  return ((3.3f - voltage) / voltage) * MQ7_RL;
}

/* ========================================= */
/*  MQ7: RS → PPM (CONVERSION FORMULA)      */
/* ========================================= */

float rsToPPM(float rs) {
  if (rs <= 0) return 0;
  
  float ratio = rs / co.RO;
  if (ratio <= 0) return 0;
  
  // PPM = a * (RS/RO)^b
  float ppm = MQ7_A * pow(ratio, MQ7_B);
  return (ppm < 0) ? 0 : ppm;
}

/* ========================================= */
/*  SPIKE FILTER - Remove false positives   */
/* ========================================= */

float spikeFilter(float newPPM) {
  if (co.filteredPPM < 1.0f) return newPPM;
  
  // If jump > 2x AND > 20ppm → smooth it
  if (newPPM > co.filteredPPM * SPIKE_THRESHOLD && newPPM > 20.0f) {
    float smoothed = co.filteredPPM * (1.0f - SPIKE_SMOOTHING) 
                   + newPPM * SPIKE_SMOOTHING;
    LOG("[SPIKE] Detected: " + String(newPPM, 1) + " → " + String(smoothed, 1));
    return smoothed;
  }
  
  return newPPM;
}

/* ========================================= */
/*  UPDATE PREDICTION (10s ahead)           */
/* ========================================= */

void updatePrediction(float newReading) {
  newReading = spikeFilter(newReading);
  
  co.previousPPM = co.filteredPPM;
  
  // Exponential smoothing (faster rise, slower fall)
  float alpha = (newReading > co.filteredPPM) ? co.alphaRise : co.alphaFall;
  co.filteredPPM = alpha * newReading + (1.0f - alpha) * co.filteredPPM;
  
  co.previousRate = co.rate;
  co.rate = co.filteredPPM - co.previousPPM;
  
  float acceleration = co.rate - co.previousRate;
  
  // Weight based on rate magnitude
  float weight;
  if      (fabs(co.rate) > 6.0f) weight = 1.3f;
  else if (fabs(co.rate) > 3.0f) weight = 1.1f;
  else                           weight = 0.85f;
  
  // Predict 10 seconds ahead
  co.predicted10 = co.filteredPPM 
                 + (co.rate * 10.0f * weight) 
                 + (acceleration * 5.0f);
  
  // Prevent going up when falling
  if (co.rate < 0 && co.predicted10 > co.filteredPPM) {
    co.predicted10 = co.filteredPPM;
  }
  
  if (co.predicted10 < 0) co.predicted10 = 0;
  
  // Debug log
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
/*  HYSTERESIS STATE MACHINE                */
/* ========================================= */

void updateAlertState() {
  switch (co.alertState) {
    
    case 0: // SAFE
      if (co.filteredPPM >= CRITICAL_ENTER) {
        co.alertState = 3;
      } else if (co.filteredPPM >= DANGER_ENTER) {
        co.alertState = 2;
      } else if (co.filteredPPM >= WARNING_ENTER) {
        co.alertState = 1;
      }
      break;
    
    case 1: // WARNING
      if (co.filteredPPM >= CRITICAL_ENTER) {
        co.alertState = 3;
      } else if (co.filteredPPM >= DANGER_ENTER) {
        co.alertState = 2;
      } else if (co.filteredPPM < (WARNING_EXIT - 5)) {
        co.alertState = 0;
      }
      break;
    
    case 2: // DANGER
      if (co.filteredPPM >= CRITICAL_ENTER) {
        co.alertState = 3;
      } else if (co.filteredPPM <= (DANGER_EXIT + 5)) {
        co.alertState = 1;
      }
      break;
    
    case 3: // CRITICAL
      if (co.filteredPPM < (CRITICAL_EXIT - 5)) {
        co.alertState = 2;
      }
      break;
  }
}

/* ========================================= */
/*  BUZZER CONTROL                          */
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
/*  UPDATE ALERTS (LEDs + Buzzer)           */
/* ========================================= */

void updateAlerts() {
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED,    LOW);
  
  switch (co.alertState) {
    case 3: // CRITICAL
      digitalWrite(LED_RED, HIGH);
      updateBuzzer(100);  // Fast beep
      LOG("[ALERT] CRITICAL!");
      break;
    
    case 2: // DANGER
      digitalWrite(LED_RED, HIGH);
      updateBuzzer(400);  // Slow beep
      LOG("[ALERT] DANGER!");
      break;
    
    case 1: // WARNING
      buzzerOff();
      digitalWrite(LED_YELLOW, HIGH);
      LOG("[ALERT] WARNING!");
      break;
    
    default: // SAFE
      buzzerOff();
      digitalWrite(LED_GREEN, HIGH);
      break;
  }
}

/* ========================================= */
/*  AUTO CALIBRATION (Clean Air)            */
/* ========================================= */

void runCalibration() {
  lcdClearAll();
  
  char buf[16];
  float rsSum = 0;
  int samples = 0;
  unsigned long calStart = millis();
  unsigned long lastSample = 0;
  
  LOG("[CAL] Starting 60-second calibration...");
  
  while (millis() - calStart < CALIB_TIME) {
    unsigned long now = millis();
    int secsLeft = (int)((CALIB_TIME - (now - calStart)) / 1000);
    
    sprintf(buf, "Cal: %ds left", secsLeft);
    lcdWrite(0, 0, "Calibrating...");
    lcdWrite(0, 1, buf);
    
    if (now - lastSample >= 1000UL) {
      int raw = analogRead(MQ7_PIN);
      float rs = rawToRS(raw);
      rsSum += rs;
      samples++;
      lastSample = now;
      LOG("[CAL] Sample " + String(samples) + " RS=" + String(rs, 2));
    }
    
    delay(50);
  }
  
  if (samples > 0) {
    float rsAvg = rsSum / samples;
    co.RO = rsAvg;
    saveRO(co.RO);
    
    LOG("[CAL] Done! RO=" + String(co.RO, 4));
    
    lcdClearAll();
    sprintf(buf, "RO=%.2f", co.RO);
    lcdWrite(0, 0, "Cal Done!");
    lcdWrite(0, 1, buf);
    delay(2000);
  }
  
  co.calibDone = true;
}

/* ========================================= */
/*  READ GAS SENSOR                         */
/* ========================================= */

void readGasSensor() {
  unsigned long now = millis();
  if (now - gas.lastUpdateTime < 1000UL) return;
  
  gas.lastUpdateTime = now;
  
  int rawGas = analogRead(GAS_SENSOR_PIN);
  
  // Convert to 0-1000 range (adjust based on your sensor datasheet)
  gas.reading = (rawGas / 4095.0f) * 1000.0f;
  
  if (gas.reading > gas.maxReading) {
    gas.maxReading = gas.reading;
  }
  
  LOGP("[GAS] Raw=");
  LOGP(String(rawGas));
  LOGP(" Value=");
  LOG(String(gas.reading, 2));
}

/* ========================================= */
/*  WIFI CHECK & RECONNECT                  */
/* ========================================= */

void checkWiFi() {
  unsigned long now = millis();
  
  if (WiFi.status() == WL_CONNECTED) {
    if (!sys.wifiConnected) {
      sys.wifiConnected = true;
      LOG("[WiFi] Connected!");
    }
    sys.lastWiFiCheck = now;
    return;
  }
  
  sys.wifiConnected = false;
  
  if (now - sys.lastWiFiCheck >= WiFi_CHECK_INTERVAL) {
    LOG("[WiFi] Attempting reconnect...");
    WiFi.reconnect();
    sys.lastWiFiCheck = now;
  }
}

/* ========================================= */
/*  SETUP                                   */
/* ========================================= */

void setup() {
  Serial.begin(115200);
  delay(500);
  
  LOG("\n\n[EnviroGuard v3.3 FIXED - Starting]\n");
  
  /* --- GPIO Setup --- */
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  pinMode(BUZZER,     OUTPUT);
  
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED,    LOW);
  digitalWrite(BUZZER,     LOW);
  
  LOG("[Setup] GPIO configured");
  
  /* --- LCD Setup --- */
  Wire.begin();
  delay(100);
  
  if (!lcd.begin(16, 2)) {
    LOG("[LCD] FAILED to initialize!");
    while (1) {
      digitalWrite(LED_RED, HIGH);
      delay(200);
      digitalWrite(LED_RED, LOW);
      delay(200);
    }
  }
  
  lcd.createChar(0, wifiWeak);
  lcd.createChar(1, wifiMedium);
  lcd.createChar(2, wifiStrong);
  
  LOG("[Setup] LCD initialized (16x2 I2C)");
  
  startup();
  
  /* --- Load Calibration --- */
  float savedRO = loadRO();
  
  if (savedRO > 0.5f && savedRO < 500.0f) {
    co.RO = savedRO;
    co.calibDone = true;
    
    char buf[16];
    sprintf(buf, "RO=%.2f", co.RO);
    
    lcdClearAll();
    lcdWrite(0, 0, "Cal loaded");
    lcdWrite(0, 1, buf);
    delay(1500);
  } else {
    LOG("[CAL] No valid RO found - will calibrate");
  }
  
  /* --- WiFi Setup --- */
  WiFi.mode(WIFI_STA);
  lcdClearAll();
  lcdWrite(0, 0, "WiFi setup...");
  
  wm.setConfigPortalTimeout(120);
  wm.setConnectTimeout(20);
  
  bool wifiConnected = wm.autoConnect("EnviroGuard");
  
  if (wifiConnected) {
    sys.wifiConnected = true;
    sys.lastWiFiCheck = millis();
    
    String ip = WiFi.localIP().toString();
    LOG("[WiFi] Connected! IP: " + ip);
    
    lcdClearAll();
    lcdWrite(0, 0, "WiFi OK");
    lcdWrite(0, 1, ip.c_str());
    delay(2000);
  } else {
    sys.wifiConnected = false;
    LOG("[WiFi] Offline mode activated");
    
    lcdClearAll();
    lcdWrite(0, 0, "WiFi Failed");
    lcdWrite(0, 1, "Offline Mode");
    delay(2000);
  }
  
  lcdClearAll();
  
  /* --- Ready --- */
  sys.startupTime = millis();
  sys.warmupDone = false;
  
  LOG("[Setup] Complete - Warming up for 20s\n");
}

/* ========================================= */
/*  MAIN LOOP                               */
/* ========================================= */

void loop() {
  
  unsigned long now = millis();
  
  /* --- WARMUP PHASE (20 seconds) --- */
  if (!sys.warmupDone) {
    int secsLeft = (int)((WARMUP_TIME - (now - sys.startupTime)) / 1000);
    
    if (secsLeft > 0) {
      char buf[16];
      sprintf(buf, "Wait: %ds", secsLeft);
      
      lcdWrite(0, 0, "Warming up...");
      lcdWrite(0, 1, buf);
      
      digitalWrite(LED_GREEN,  LOW);
      digitalWrite(LED_YELLOW, HIGH);
      digitalWrite(LED_RED,    LOW);
      
      delay(100);
      return;
    }
    
    sys.warmupDone = true;
    lcdClearAll();
    
    /* --- Auto Calibration if needed --- */
    if (!co.calibDone) {
      runCalibration();
    }
    
    LOG("[System] Warmup complete - Running normally\n");
  }
  
  /* --- READ CO SENSOR --- */
  if (now - sys.lastSensorRead >= SENSOR_INTERVAL) {
    int raw = analogRead(MQ7_PIN);
    float rs = rawToRS(raw);
    float ppm = rsToPPM(rs);
    
    updatePrediction(ppm);
    updateAlertState();
    updateAlerts();
    
    sys.lastSensorRead = now;
  }
  
  /* --- READ GAS SENSOR --- */
  readGasSensor();
  
  /* --- WiFi Management --- */
  checkWiFi();
  
  /* --- CRITICAL ALERT DISPLAY --- */
  if (co.alertState == 3) {
    if (!sys.criticalLock) {
      lcdClearAll();
      sys.criticalLock = true;
    }
    
    char buf[16];
    sprintf(buf, "CO:%dppm EVAC!", (int)co.filteredPPM);
    
    lcdWrite(0, 0, "!CRITICAL!");
    lcdWrite(0, 1, buf);
    drawWiFi();
    
    delay(50);
    return;  // Stay in critical display
  }
  
  /* --- EXIT CRITICAL MODE --- */
  if (sys.criticalLock) {
    lcdClearAll();
    sys.criticalLock = false;
  }
  
  /* --- NORMAL DISPLAY (Toggle every 3s) --- */
  if (now - sys.lastScreenSwitch >= SCREEN_INTERVAL) {
    sys.screenToggle = !sys.screenToggle;
    sys.lastScreenSwitch = now;
  }
  
  char line0[16], line1[16];
  
  if (sys.screenToggle) {
    // Screen 1: CO Status
    sprintf(line0, "CO:%dppm", (int)co.filteredPPM);
    sprintf(line1, "P10:%dppm", (int)co.predicted10);
  } else {
    // Screen 2: Gas + Status
    sprintf(line0, "Gas:%dppm", (int)gas.reading);
    
    const char* status;
    switch (co.alertState) {
      case 0: status = "SAFE"; break;
      case 1: status = "WARN"; break;
      case 2: status = "DANGER"; break;
      case 3: status = "CRITICAL"; break;
      default: status = "?"; break;
    }
    
    sprintf(line1, "%s %c", status, 
            WiFi.status() == WL_CONNECTED ? 'W' : '-');
  }
  
  lcdWrite(0, 0, line0);
  lcdWrite(0, 1, line1);
  drawWiFi();
  
  delay(50);  // Small delay to prevent overwhelming the loop
}
