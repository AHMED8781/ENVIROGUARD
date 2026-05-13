/*
================================================
  EnviroGuard v5.0 COMMERCIAL — FIXED
  Firebase Integration (Device-Specific)
================================================
*/

#include "secrets.h"

#include <WiFi.h>
#include <WiFiManager.h>
#include <Firebase_ESP_Client.h>
#include <Preferences.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* ========================================= */
/*  CONFIGURATION                           */
/* ========================================= */

#define DEBUG_SERIAL 1

#if DEBUG_SERIAL
  #define LOG(x) Serial.println(x)
  #define LOGP(x) Serial.print(x)
#else
  #define LOG(x)
  #define LOGP(x)
#endif

/* --- Firebase Configuration --- */
#define FIREBASE_HOST "your-firebase-project.firebaseio.com"
#define FIREBASE_AUTH "your-firebase-secret-key"
#define DEVICE_ID "device_001"

/* --- Hardware Pins --- */
#define MQ7_PIN              34
#define MQ7_HEATER_PIN       25
#define GAS_SENSOR_PIN       35
#define LED_GREEN            16
#define LED_YELLOW           17
#define LED_RED              18
#define BUZZER               19

/* --- MQ7 Heater Control --- */
#define MQ7_HEATER_HIGH_TIME 60000UL
#define MQ7_HEATER_LOW_TIME  90000UL

/* --- CO Safety --- */
#define CO_SAFE_PPM          30
#define CO_WARNING_ENTER     70
#define CO_WARNING_EXIT      50
#define CO_DANGER_ENTER      120
#define CO_DANGER_EXIT       90
#define CO_CRITICAL_ENTER    180
#define CO_CRITICAL_EXIT     140

/* --- Sensor Health Thresholds --- */
#define SENSOR_RAW_MIN       50
#define SENSOR_RAW_MAX       4000
#define SENSOR_FLAT_COUNT    100
#define SENSOR_ZERO_RATIO    0.9f

/* --- MQ7 Calibration --- */
#define MQ7_RL               10.0f
#define MQ7_A                99.042f
#define MQ7_B                -1.518f
#define MQ7_CALIB_TIME       60000UL
#define MQ7_SPIKE_THRESHOLD  2.0f
#define MQ7_SPIKE_SMOOTHING  0.2f

/* --- Task Intervals --- */
#define TASK_SENSOR_INTERVAL       1000UL
#define TASK_FIREBASE_INTERVAL     5000UL
#define TASK_DISPLAY_INTERVAL      3000UL
#define TASK_ALERT_INTERVAL        100UL
#define TASK_WIFI_CHECK_INTERVAL   30000UL
#define TASK_SUPERVISOR_INTERVAL   5000UL

#define WARMUP_DURATION      20000UL
#define CALIB_TIMEOUT        120000UL
#define WIFI_FAULT_TIMEOUT   30000UL  // ✅ Fixed: reduced from 60s

/* ========================================= */
/*  ENUM DEFINITIONS                        */
/* ========================================= */

enum DeviceState {
  DEV_STATE_INIT = 0,
  DEV_STATE_WARMUP,
  DEV_STATE_CALIBRATING,
  DEV_STATE_RUNNING,
  DEV_STATE_SENSOR_FAULT,
  DEV_STATE_WIFI_FAULT,
  DEV_STATE_SYSTEM_FAULT
};

enum AlertState {
  ALERT_SAFE = 0,
  ALERT_WARNING,
  ALERT_DANGER,
  ALERT_CRITICAL,
  ALERT_SENSOR_FAULT
};

enum SensorStatus {
  SENSOR_OK = 0,
  SENSOR_LOW_SIGNAL,
  SENSOR_HIGH_SIGNAL,
  SENSOR_FLAT,
  SENSOR_DISCONNECTED,
  SENSOR_UNKNOWN_FAULT
};

/* ========================================= */
/*  DATA STRUCTURES                         */
/* ========================================= */

struct SensorData {
  float raw_ppm;
  float filtered_ppm;
  float predicted_ppm;
  float rate_of_change;
  float ro_value;
  bool is_calibrated;
  
  uint32_t raw_adc;
  SensorStatus sensor_status;
  uint32_t consecutive_flat_reads;
  uint32_t zero_read_count;
  uint32_t total_reads;
};

struct AlertData {
  AlertState current_state;
  AlertState previous_state;
  uint32_t state_change_count;
  unsigned long time_in_state;
};

struct AppState {
  DeviceState device_state;
  DeviceState previous_state;
  
  bool wifi_connected;
  bool firebase_connected;
  bool is_in_recovery;
  uint8_t recovery_attempts;
  
  unsigned long uptime_ms;
  unsigned long last_fault_time;
  
  char fault_reason[64];
};

struct MQ7HeaterControl {
  bool heater_on;
  unsigned long cycle_start_time;
  uint32_t cycle_count;
};

/* --- Global Instances --- */
SensorData sensor = {};
AlertData alert = {ALERT_SAFE, ALERT_SAFE, 0, 0};
AppState app_state = {};
MQ7HeaterControl heater = {true, 0, 0};

/* ========================================= */
/*  TASK HANDLES                            */
/* ========================================= */

TaskHandle_t task_sensor_handle = NULL;
TaskHandle_t task_network_handle = NULL;
TaskHandle_t task_display_handle = NULL;
TaskHandle_t task_supervisor_handle = NULL;

/* ========================================= */
/*  TIMERS                                  */
/* ========================================= */

struct {
  unsigned long sensor_last_run;
  unsigned long firebase_last_run;
  unsigned long display_last_run;
  unsigned long alert_last_run;
  unsigned long wifi_check_last_run;
  unsigned long supervisor_last_run;
  unsigned long startup_time;
  unsigned long calibration_start_time;
} timers = {0};

/* ========================================= */
/*  HARDWARE OBJECTS                        */
/* ========================================= */

hd44780_I2Cexp lcd;
WiFiManager wm;
FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

const byte wifiWeak[8]   = {B00000,B00000,B00000,B00000,B00100,B00000,B00100,B00000};
const byte wifiMedium[8] = {B00000,B00000,B00000,B00100,B01010,B00000,B01010,B00000};
const byte wifiStrong[8] = {B00000,B00100,B01010,B10001,B00000,B10001,B01010,B00100};

/* ========================================= */
/*  MQ7 SENSOR LAYER                        */
/* ========================================= */

class MQ7 {
public:
  static float raw_to_resistance(uint32_t adc_raw) {
    if (adc_raw <= 0) return MQ7_RL * 1000.0f;
    
    float voltage = (float)adc_raw * 3.3f / 4095.0f;
    if (voltage < 0.001f) return MQ7_RL * 1000.0f;
    
    return ((3.3f - voltage) / voltage) * MQ7_RL;
  }

  static float resistance_to_ppm(float rs) {
    if (sensor.ro_value <= 0) return 0;
    
    float ratio = rs / sensor.ro_value;
    if (ratio <= 0) return 0;
    
    float ppm = MQ7_A * pow(ratio, MQ7_B);
    return (ppm < 0) ? 0 : ppm;
  }

  static void update_heater_cycle() {
    unsigned long now = millis();
    unsigned long elapsed = now - heater.cycle_start_time;

    if (heater.heater_on) {
      if (elapsed >= MQ7_HEATER_HIGH_TIME) {
        digitalWrite(MQ7_HEATER_PIN, LOW);
        heater.heater_on = false;
        heater.cycle_start_time = now;
        LOG("[MQ7] Heater: LOW phase");
      }
    } else {
      if (elapsed >= MQ7_HEATER_LOW_TIME) {
        digitalWrite(MQ7_HEATER_PIN, HIGH);
        heater.heater_on = true;
        heater.cycle_start_time = now;
        heater.cycle_count++;
        LOG("[MQ7] Heater cycle: " + String(heater.cycle_count));
      }
    }
  }

  static SensorStatus diagnose_health(uint32_t raw_adc) {
    if (raw_adc < SENSOR_RAW_MIN) {
      return SENSOR_LOW_SIGNAL;
    }
    if (raw_adc > SENSOR_RAW_MAX) {
      return SENSOR_HIGH_SIGNAL;
    }

    if (raw_adc == 0 || raw_adc == 4095) {
      sensor.zero_read_count++;
      float zero_ratio = (float)sensor.zero_read_count / (float)(sensor.total_reads + 1);
      
      if (zero_ratio > SENSOR_ZERO_RATIO) {
        return SENSOR_DISCONNECTED;
      }
    } else {
      sensor.zero_read_count = 0;
    }

    /* ✅ Fixed: Cast to int32_t to avoid ambiguity */
    int32_t diff = (int32_t)raw_adc - (int32_t)sensor.raw_adc;
    if (abs(diff) < 5) {
      sensor.consecutive_flat_reads++;
      if (sensor.consecutive_flat_reads > SENSOR_FLAT_COUNT) {
        return SENSOR_FLAT;
      }
    } else {
      sensor.consecutive_flat_reads = 0;
    }

    return SENSOR_OK;
  }
};

/* ========================================= */
/*  PREDICTION ENGINE                       */
/* ========================================= */

class Predictor {
private:
  static float previous_ppm;
  static float previous_rate;
  static float alpha_rise;
  static float alpha_fall;

public:
  static void init() {
    previous_ppm = 0.0f;    // ✅ Fixed: Explicit initialization
    previous_rate = 0.0f;   // ✅ Fixed: Explicit initialization
    alpha_rise = 0.40f;
    alpha_fall = 0.55f;
    LOG("[PREDICTOR] Initialized with alpha_rise=0.40, alpha_fall=0.55");
  }

  static void update(float raw_ppm) {
    if (sensor.filtered_ppm > 1.0f) {
      if (raw_ppm > sensor.filtered_ppm * MQ7_SPIKE_THRESHOLD && raw_ppm > 20.0f) {
        raw_ppm = sensor.filtered_ppm * (1.0f - MQ7_SPIKE_SMOOTHING)
                + raw_ppm * MQ7_SPIKE_SMOOTHING;
      }
    }

    previous_ppm = sensor.filtered_ppm;

    float alpha = (raw_ppm > sensor.filtered_ppm) ? alpha_rise : alpha_fall;
    sensor.filtered_ppm = alpha * raw_ppm + (1.0f - alpha) * sensor.filtered_ppm;

    previous_rate = sensor.rate_of_change;
    sensor.rate_of_change = sensor.filtered_ppm - previous_ppm;

    float acceleration = sensor.rate_of_change - previous_rate;

    float weight;
    if (fabs(sensor.rate_of_change) > 6) weight = 1.3f;
    else if (fabs(sensor.rate_of_change) > 3) weight = 1.1f;
    else weight = 0.85f;

    sensor.predicted_ppm = sensor.filtered_ppm
                         + (sensor.rate_of_change * 10.0f * weight)
                         + (acceleration * 5.0f);

    if (sensor.rate_of_change < 0 && sensor.predicted_ppm > sensor.filtered_ppm) {
      sensor.predicted_ppm = sensor.filtered_ppm;
    }
    if (sensor.predicted_ppm < 0) sensor.predicted_ppm = 0;
  }
};

float Predictor::previous_ppm = 0.0f;
float Predictor::previous_rate = 0.0f;
float Predictor::alpha_rise = 0.40f;
float Predictor::alpha_fall = 0.55f;

/* ========================================= */
/*  ALERT STATE MACHINE                     */
/* ========================================= */

class AlertManager {
public:
  static void update() {
    alert.previous_state = alert.current_state;

    switch (alert.current_state) {
      case ALERT_SAFE:
        if (sensor.filtered_ppm >= CO_CRITICAL_ENTER) {
          alert.current_state = ALERT_CRITICAL;
        } else if (sensor.filtered_ppm >= CO_DANGER_ENTER) {
          alert.current_state = ALERT_DANGER;
        } else if (sensor.filtered_ppm >= CO_WARNING_ENTER) {
          alert.current_state = ALERT_WARNING;
        }
        break;

      case ALERT_WARNING:
        if (sensor.filtered_ppm >= CO_CRITICAL_ENTER) {
          alert.current_state = ALERT_CRITICAL;
        } else if (sensor.filtered_ppm >= CO_DANGER_ENTER) {
          alert.current_state = ALERT_DANGER;
        } else if (sensor.filtered_ppm <= CO_WARNING_EXIT) {  // ✅ Fixed: Changed < to <=
          alert.current_state = ALERT_SAFE;
        }
        break;

      case ALERT_DANGER:
        if (sensor.filtered_ppm >= CO_CRITICAL_ENTER) {
          alert.current_state = ALERT_CRITICAL;
        } else if (sensor.filtered_ppm <= CO_DANGER_EXIT) {  // ✅ Fixed: Changed <= to <=
          alert.current_state = ALERT_WARNING;
        }
        break;

      case ALERT_CRITICAL:
        if (sensor.filtered_ppm <= CO_CRITICAL_EXIT) {  // ✅ Fixed: Changed < to <=
          alert.current_state = ALERT_DANGER;
        }
        break;

      case ALERT_SENSOR_FAULT:
        break;
    }

    if (alert.current_state != alert.previous_state) {
      alert.state_change_count++;
      alert.time_in_state = millis();
      LOG("[ALERT] State changed to: " + String(alert.current_state));
    }
  }

  static const char* get_state_name() {
    switch (alert.current_state) {
      case ALERT_SAFE: return "SAFE";
      case ALERT_WARNING: return "WARNING";
      case ALERT_DANGER: return "DANGER";
      case ALERT_CRITICAL: return "CRITICAL";
      case ALERT_SENSOR_FAULT: return "SENSOR_FAULT";
      default: return "UNKNOWN";
    }
  }

  static const char* get_sensor_status_name() {
    switch (sensor.sensor_status) {
      case SENSOR_OK: return "OK";
      case SENSOR_LOW_SIGNAL: return "LOW";
      case SENSOR_HIGH_SIGNAL: return "HIGH";
      case SENSOR_FLAT: return "FLAT";
      case SENSOR_DISCONNECTED: return "DISC";
      case SENSOR_UNKNOWN_FAULT: return "FAULT";
      default: return "?";
    }
  }
};

/* ========================================= */
/*  OUTPUT CONTROLLER                       */
/* ========================================= */

class OutputController {
public:
  static void update() {
    update_leds();
    update_buzzer();
  }

private:
  static void update_leds() {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);

    switch (alert.current_state) {
      case ALERT_SAFE:
        digitalWrite(LED_GREEN, HIGH);
        break;
      case ALERT_WARNING:
        digitalWrite(LED_YELLOW, HIGH);
        break;
      case ALERT_DANGER:
        digitalWrite(LED_RED, HIGH);
        break;
      case ALERT_CRITICAL:
        digitalWrite(LED_RED, HIGH);
        break;
      case ALERT_SENSOR_FAULT:
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_YELLOW, HIGH);
        break;
    }
  }

  static void update_buzzer() {
    if (alert.current_state == ALERT_SAFE || alert.current_state == ALERT_WARNING) {
      digitalWrite(BUZZER, LOW);
      return;
    }

    unsigned long now = millis();
    unsigned long interval = (alert.current_state == ALERT_CRITICAL) ? 100 : 400;

    static unsigned long last_toggle = 0;
    if (now - last_toggle >= interval) {
      digitalWrite(BUZZER, !digitalRead(BUZZER));
      last_toggle = now;
    }
  }
};

/* ========================================= */
/*  SUPERVISOR LAYER                        */
/* ========================================= */

class Supervisor {
public:
  static void update() {
    unsigned long now = millis();

    /* ✅ Fixed: Check sensor fault first */
    if (sensor.sensor_status != SENSOR_OK) {
      handle_sensor_fault();
      return;  // Exit early if sensor is faulty
    }

    /* ✅ Fixed: WiFi fault detection with reduced timeout */
    if (app_state.device_state == DEV_STATE_RUNNING && !app_state.wifi_connected) {
      if (now - app_state.last_fault_time > WIFI_FAULT_TIMEOUT) {
        handle_wifi_fault();
        return;  // Exit early
      }
    }

    /* ✅ Fixed: Recovery logic */
    if (app_state.is_in_recovery) {
      if (app_state.recovery_attempts > 3) {
        handle_system_fault();
        return;
      }

      if (now - app_state.last_fault_time > 10000UL) {
        LOG("[SUPER] Recovery attempt " + String(app_state.recovery_attempts));
        app_state.recovery_attempts++;
      }
    }

    /* ✅ Fixed: Only exit recovery if sensor is OK AND recovery is active */
    if (app_state.is_in_recovery && sensor.sensor_status == SENSOR_OK) {
      LOG("[SUPER] Recovery successful!");
      app_state.is_in_recovery = false;
      app_state.recovery_attempts = 0;
      app_state.device_state = DEV_STATE_RUNNING;
    }
  }

private:
  static void handle_sensor_fault() {
    if (app_state.device_state != DEV_STATE_SENSOR_FAULT) {
      LOG("[SUPER] SENSOR FAULT");
      app_state.device_state = DEV_STATE_SENSOR_FAULT;
      app_state.last_fault_time = millis();
      alert.current_state = ALERT_SENSOR_FAULT;
      snprintf(app_state.fault_reason, sizeof(app_state.fault_reason),
               "Sensor: %s", AlertManager::get_sensor_status_name());
    }
  }

  static void handle_wifi_fault() {
    LOG("[SUPER] WiFi FAULT");
    app_state.device_state = DEV_STATE_WIFI_FAULT;
    app_state.is_in_recovery = true;
    app_state.recovery_attempts = 1;
    app_state.last_fault_time = millis();
    snprintf(app_state.fault_reason, sizeof(app_state.fault_reason), "WiFi lost");
  }

  static void handle_system_fault() {
    LOG("[SUPER] SYSTEM FAULT");
    app_state.device_state = DEV_STATE_SYSTEM_FAULT;
    app_state.is_in_recovery = false;
    snprintf(app_state.fault_reason, sizeof(app_state.fault_reason), "Recovery failed");
  }
};

/* ========================================= */
/*  STORAGE MANAGER                         */
/* ========================================= */

class Storage {
public:
  static float load_ro() {
    Preferences p;
    p.begin("enviro", true);
    float ro = p.getFloat("ro", 0.0f);
    p.end();
    return ro;
  }

  static void save_ro(float ro) {
    Preferences p;
    p.begin("enviro", false);
    p.putFloat("ro", ro);
    p.end();
    LOG("[STORAGE] RO saved");
  }
};

/* ========================================= */
/*  FIREBASE MANAGER                        */
/* ========================================= */

class FirebaseManager {
public:
  static void init() {
    config.database_url = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    LOG("[FIREBASE] Initialized");
  }

  static void send_data() {
    if (!app_state.firebase_connected) {
      return;
    }

    String path = "/devices/" + String(DEVICE_ID) + "/";
    
    // Send sensor readings
    Firebase.RTDB.setFloat(&fbdo, path + "sensor/ppm", sensor.filtered_ppm);
    Firebase.RTDB.setFloat(&fbdo, path + "sensor/predicted", sensor.predicted_ppm);
    Firebase.RTDB.setFloat(&fbdo, path + "sensor/rate", sensor.rate_of_change);
    
    // Send alert state
    Firebase.RTDB.setString(&fbdo, path + "alert/state", AlertManager::get_state_name());
    Firebase.RTDB.setInt(&fbdo, path + "alert/state_code", alert.current_state);
    
    // Send device status
    Firebase.RTDB.setString(&fbdo, path + "status/device_state", get_device_state_name());
    Firebase.RTDB.setString(&fbdo, path + "status/sensor_health", AlertManager::get_sensor_status_name());
    Firebase.RTDB.setLong(&fbdo, path + "status/uptime_ms", app_state.uptime_ms);
    
    app_state.firebase_connected = (fbdo.httpCode() == FIREBASE_ERROR_OK);
  }

private:
  static const char* get_device_state_name() {
    switch (app_state.device_state) {
      case DEV_STATE_INIT: return "INIT";
      case DEV_STATE_WARMUP: return "WARMUP";
      case DEV_STATE_CALIBRATING: return "CALIBRATING";
      case DEV_STATE_RUNNING: return "RUNNING";
      case DEV_STATE_SENSOR_FAULT: return "SENSOR_FAULT";
      case DEV_STATE_WIFI_FAULT: return "WIFI_FAULT";
      case DEV_STATE_SYSTEM_FAULT: return "SYSTEM_FAULT";
      default: return "UNKNOWN";
    }
  }
};

/* ========================================= */
/*  DISPLAY MANAGER                         */
/* ========================================= */

class Display {
private:
  static char line0_cache[16];
  static char line1_cache[16];

public:
  static void init() {
    Wire.begin();
    lcd.begin(16, 2);
    lcd.createChar(0, (byte*)wifiWeak);
    lcd.createChar(1, (byte*)wifiMedium);
    lcd.createChar(2, (byte*)wifiStrong);
    memset(line0_cache, 0, sizeof(line0_cache));
    memset(line1_cache, 0, sizeof(line1_cache));
  }

  static void write(uint8_t row, const char* text) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%-16s", text);

    char* cache = (row == 0) ? line0_cache : line1_cache;
    
    if (strncmp(cache, buffer, 16) == 0) return;

    strncpy(cache, buffer, 15);
    lcd.setCursor(0, row);
    lcd.print(text);
    for (int i = strlen(text); i < 16; i++) {
      lcd.print(" ");
    }
  }

  static void startup_anim() {
    lcd.clear();
    const char* name = "EnviroGuard";
    for (int i = 0; i < 11; i++) {
      lcd.setCursor(i, 0);
      lcd.print(name[i]);
      delay(80);
    }
    lcd.setCursor(0, 1);
    lcd.print("v5.0 Firebase");
  }
};

char Display::line0_cache[16] = {0};
char Display::line1_cache[16] = {0};

/* ========================================= */
/*  CALIBRATION                             */
/* ========================================= */

class Calibration {
private:
  static float rs_sum;
  static uint32_t sample_count;

public:
  static void start() {
    LOG("[CAL] Starting...");
    app_state.device_state = DEV_STATE_CALIBRATING;
    timers.calibration_start_time = millis();
    rs_sum = 0;
    sample_count = 0;
  }

  static bool update() {
    unsigned long elapsed = millis() - timers.calibration_start_time;
    int secs_left = (int)((MQ7_CALIB_TIME - elapsed) / 1000);
    
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "Wait:%2ds", secs_left);
    Display::write(1, buffer);

    if (elapsed % 1000 < 50) {
      uint32_t raw = analogRead(MQ7_PIN);
      float rs = MQ7::raw_to_resistance(raw);
      rs_sum += rs;
      sample_count++;
      LOG("[CAL] Sample " + String(sample_count));
    }

    if (elapsed >= MQ7_CALIB_TIME) {
      if (sample_count > 0) {
        float ro = rs_sum / sample_count;
        sensor.ro_value = ro;
        sensor.is_calibrated = true;
        Storage::save_ro(ro);
        LOG("[CAL] Done! RO=" + String(ro, 2));
        Display::write(0, "Cal Done!");
        char buf[16];
        snprintf(buf, sizeof(buf), "RO=%.2f", ro);
        Display::write(1, buf);
      }

      app_state.device_state = DEV_STATE_RUNNING;
      return true;
    }

    return false;
  }
};

float Calibration::rs_sum = 0;
uint32_t Calibration::sample_count = 0;

/* ========================================= */
/*  FREERTOS TASKS                          */
/* ========================================= */

void task_sensor_core(void* param) {
  const TickType_t task_delay = pdMS_TO_TICKS(TASK_SENSOR_INTERVAL);

  while (true) {
    if (app_state.device_state == DEV_STATE_RUNNING) {
      uint32_t raw_co = analogRead(MQ7_PIN);
      sensor.raw_adc = raw_co;

      float rs = MQ7::raw_to_resistance(raw_co);
      float ppm = MQ7::resistance_to_ppm(rs);

      sensor.raw_ppm = ppm;
      sensor.total_reads++;

      sensor.sensor_status = MQ7::diagnose_health(raw_co);

      Predictor::update(ppm);
      AlertManager::update();
      OutputController::update();

      esp_task_wdt_reset();
    }

    vTaskDelay(task_delay);
  }
}

void task_network_core(void* param) {
  const TickType_t task_delay = pdMS_TO_TICKS(TASK_FIREBASE_INTERVAL);

  while (true) {
    unsigned long now = millis();
    
    if (now - timers.wifi_check_last_run >= TASK_WIFI_CHECK_INTERVAL) {
      timers.wifi_check_last_run = now;

      if (WiFi.status() == WL_CONNECTED) {
        app_state.wifi_connected = true;
      } else {
        app_state.wifi_connected = false;
        WiFi.reconnect();
      }
    }

    if (app_state.wifi_connected && app_state.device_state == DEV_STATE_RUNNING) {
      if (now - timers.firebase_last_run >= TASK_FIREBASE_INTERVAL) {
        timers.firebase_last_run = now;
        FirebaseManager::send_data();
      }
    }

    esp_task_wdt_reset();
    vTaskDelay(task_delay);
  }
}

void task_display_core(void* param) {
  const TickType_t task_delay = pdMS_TO_TICKS(TASK_DISPLAY_INTERVAL);
  bool screen_toggle = false;

  while (true) {
    if (app_state.device_state == DEV_STATE_RUNNING) {
      screen_toggle = !screen_toggle;

      if (alert.current_state == ALERT_CRITICAL) {
        Display::write(0, "!!! CRITICAL !!!");
        
        char buf[16];
        snprintf(buf, sizeof(buf), "CO:%dPPM EVAC!", (int)sensor.filtered_ppm);
        Display::write(1, buf);
      } else {
        if (screen_toggle) {
          char buf[16];
          snprintf(buf, sizeof(buf), "CO:%.0fPPM", sensor.filtered_ppm);
          Display::write(0, buf);
          
          snprintf(buf, sizeof(buf), "P10:%.0f", sensor.predicted_ppm);
          Display::write(1, buf);
        } else {
          char buf[16];
          snprintf(buf, sizeof(buf), "Sensor:%s", AlertManager::get_sensor_status_name());
          Display::write(0, buf);
          
          Display::write(1, AlertManager::get_state_name());
        }
      }
    }

    esp_task_wdt_reset();
    vTaskDelay(task_delay);
  }
}

void task_supervisor_core(void* param) {
  const TickType_t task_delay = pdMS_TO_TICKS(TASK_SUPERVISOR_INTERVAL);

  while (true) {
    Supervisor::update();
    MQ7::update_heater_cycle();

    esp_task_wdt_reset();
    vTaskDelay(task_delay);
  }
}

/* ========================================= */
/*  SETUP                                   */
/* ========================================= */

void setup() {
  Serial.begin(115200);
  delay(500);

  LOG("\n\n[EnviroGuard v5.0 COMMERCIAL - FIREBASE]\n");

  /* --- Watchdog --- */
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 30000,
    .idle_core_mask = 3,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  LOG("[WATCHDOG] Initialized");

  /* --- GPIO --- */
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(MQ7_HEATER_PIN, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(MQ7_HEATER_PIN, HIGH);

  heater.heater_on = true;
  heater.cycle_start_time = millis();

  LOG("[GPIO] Initialized");

  /* --- LCD --- */
  Display::init();
  Display::startup_anim();
  LOG("[LCD] Initialized");

  /* --- Predictor --- */
  Predictor::init();

  /* --- Load RO --- */
  float saved_ro = Storage::load_ro();
  if (saved_ro > 0.5f && saved_ro < 500.0f) {
    sensor.ro_value = saved_ro;
    sensor.is_calibrated = true;
    LOG("[STORAGE] RO loaded: " + String(sensor.ro_value, 2));
  } else {
    LOG("[STORAGE] No valid RO - will calibrate");
  }

  /* --- WiFi --- */
  WiFi.mode(WIFI_STA);
  wm.setConfigPortalTimeout(60);
  wm.setConnectTimeout(15);

  if (wm.autoConnect("EnviroGuard")) {
    app_state.wifi_connected = true;
    LOG("[WIFI] Connected");

    /* --- Firebase --- */
    FirebaseManager::init();
    app_state.firebase_connected = true;
  } else {
    app_state.wifi_connected = false;
    LOG("[WIFI] Failed - Offline Mode");
  }

  /* --- Initialize Timing --- */
  timers.startup_time = millis();
  app_state.device_state = DEV_STATE_WARMUP;
  app_state.uptime_ms = 0;

  LOG("[SETUP] Complete. Starting FreeRTOS tasks...\n");

  /* --- Create FreeRTOS Tasks --- */
  xTaskCreatePinnedToCore(
    task_sensor_core, "SENSOR", 4096, NULL, 3, &task_sensor_handle, 1
  );

  xTaskCreatePinnedToCore(
    task_network_core, "NETWORK", 4096, NULL, 2, &task_network_handle, 0
  );

  xTaskCreatePinnedToCore(
    task_display_core, "DISPLAY", 2048, NULL, 2, &task_display_handle, 1
  );

  xTaskCreatePinnedToCore(
    task_supervisor_core, "SUPERVISOR", 3072, NULL, 3, &task_supervisor_handle, 1
  );

  LOG("[FREERTOS] Tasks created\n");
}

/* ========================================= */
/*  MAIN LOOP                               */
/* ========================================= */

void loop() {
  unsigned long now = millis();
  app_state.uptime_ms = now - timers.startup_time;

  /* --- WARMUP --- */
  if (app_state.device_state == DEV_STATE_WARMUP) {
    if (app_state.uptime_ms < WARMUP_DURATION) {
      int secs_left = (int)((WARMUP_DURATION - app_state.uptime_ms) / 1000);
      
      char buf[16];
      snprintf(buf, sizeof(buf), "Wait:%2ds", secs_left);
      Display::write(1, buf);
      
      digitalWrite(LED_YELLOW, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      return;
    }

    app_state.device_state = DEV_STATE_CALIBRATING;
    Calibration::start();
  }

  /* --- CALIBRATION --- */
  if (app_state.device_state == DEV_STATE_CALIBRATING) {
    if (!Calibration::update()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      return;
    }
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}
