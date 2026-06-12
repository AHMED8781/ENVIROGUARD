/*
================================================
  EnviroGuard Secrets Configuration
  WiFi & Blynk Credentials
================================================
*/

#ifndef SECRETS_H
#define SECRETS_H

/* ========================================= */
/*  BLYNK CONFIGURATION                     */
/* ========================================= */

// Get your Blynk Auth Token from: https://blynk.cloud
// Create a template named "EnviroGUARD" with virtual pins V0-V7
#define BLYNK_TEMPLATE_NAME "EnviroGUARD"
#define BLYNK_AUTH_TOKEN "WEKEB8XOug-e5xVx8VMaVO_VKIo_-sO2"

/* ========================================= */
/*  BLYNK SERVER                            */
/* ========================================= */

#define BLYNK_SERVER "blynk.cloud"
#define BLYNK_PORT 80

/* ========================================= */
/*  OPTIONAL: WiFi SSID & Password          */
/* ========================================= */

// If you want hardcoded WiFi (instead of WiFiManager portal):
// Uncomment these and comment out the WiFiManager code in setup()

// #define WIFI_SSID "Your_SSID"
// #define WIFI_PASSWORD "Your_Password"

#endif  // SECRETS_H

/*
================================================
  HOW TO USE:
================================================

1. Go to https://blynk.cloud and create a new project
2. Select "EnviroGUARD" as template (or create it)
3. Copy your Auth Token and paste it above
4. Upload this file to your Arduino sketch directory
5. The main .ino file will include this automatically

Template Setup:
- Create Virtual Pins (V0-V7) as per the main sketch
- V0: CO PPM (Value)
- V1: CO Predicted (Value)
- V2: CO Rate (Value)
- V3: Gas Reading (Value)
- V4-V6: LED Status (0-255)
- V7: WiFi RSSI (Value)

Optional Automations in Blynk:
- Alert when CO > 120 ppm
- Send notification when state changes to CRITICAL
- Log events with timestamps

================================================
*/
