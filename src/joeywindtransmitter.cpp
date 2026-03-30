#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <AsyncMqttClient.h>
#include "driver/pulse_cnt.h"
#include <FastLED.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <ADS1115_WE.h>
#include <ArduinoOTA.h>
#include "esp_sntp.h"
#include "time.h"
// ==================== CONFIGURATION ====================
#define DEVICE_ID 1010

// WiFi credentials
const char* ssid     = "mikesnet";
const char* password = "springchicken";

// Blynk — neoAuth, local server
char auth[] = "19oL8t8mImCdoUqYhfhk6DADL7540f8s";
char authHubert[] = "8_-CN2rm4ki9P3i_NkPhxIbCiKd5RXhK";  //hubert
// Blynk virtual pins (matching relay's bridgeNeo / direct writes)
#define VPIN_AVG_WIND   V31
#define VPIN_WIND_GUST  V32
#define VPIN_WIND_ANGLE V33
#define VPIN_WIND_ANGLE_HOURLY V34

// ==================== MQTT ====================
// Same IP as Blynk server
#define MQTT_HOST    IPAddress(192, 168, 50, 197)
#define MQTT_PORT    1883
const char* mqttUser     = "moeburn";
const char* mqttPassword = "minimi";
const char* mqttClientId = "joey_wind";

AsyncMqttClient mqttClient;
TimerHandle_t   mqttReconnectTimer;

void connectToMqtt() {
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
    Serial.println("MQTT connected.");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("MQTT disconnected.");
    // Always start the reconnect timer — even if WiFi is currently down.
    // If WiFi is still down when the timer fires, connectToMqtt() will fail
    // gracefully and onMqttDisconnect will re-arm the timer for another retry.
    // This ensures MQTT recovers after a WiFi dropout without any extra logic.
    xTimerStart(mqttReconnectTimer, 0);
}

// ==================== HARDWARE ====================
#define ADS1115_I2C_ADDR 0x48
ADS1115_WE ads(ADS1115_I2C_ADDR);

#define NUM_LEDS  1
#define PIN_DATA  21
CRGB leds[NUM_LEDS];
WidgetBridge bridgeHubert(V60);
const int ANEMOMETER_PIN = 11;
const int BUTTON_PIN     = 1;
const int LED_PIN        = 17;

// ==================== WIND VANE CALIBRATION ====================
enum SFEWeatherMeterKitAnemometerAngles {
    WMK_ANGLE_0_0 = 0,
    WMK_ANGLE_22_5,
    WMK_ANGLE_45_0,
    WMK_ANGLE_67_5,
    WMK_ANGLE_90_0,
    WMK_ANGLE_112_5,
    WMK_ANGLE_135_0,
    WMK_ANGLE_157_5,
    WMK_ANGLE_180_0,
    WMK_ANGLE_202_5,
    WMK_ANGLE_225_0,
    WMK_ANGLE_247_5,
    WMK_ANGLE_270_0,
    WMK_ANGLE_292_5,
    WMK_ANGLE_315_0,
    WMK_ANGLE_337_5,
    WMK_NUM_ANGLES
};

#define SFE_WIND_VANE_DEGREES_PER_INDEX  (360.0f / 16.0f)
#define SFE_WMK_ADC_ANGLE_0_0    20264
#define SFE_WMK_ADC_ANGLE_22_5   10539
#define SFE_WMK_ADC_ANGLE_45_0   11966
#define SFE_WMK_ADC_ANGLE_67_5    2188
#define SFE_WMK_ADC_ANGLE_90_0    2428
#define SFE_WMK_ADC_ANGLE_112_5   1721
#define SFE_WMK_ADC_ANGLE_135_0   4797
#define SFE_WMK_ADC_ANGLE_157_5   3287
#define SFE_WMK_ADC_ANGLE_180_0   7466
#define SFE_WMK_ADC_ANGLE_202_5   6363
#define SFE_WMK_ADC_ANGLE_225_0  16314
#define SFE_WMK_ADC_ANGLE_247_5  15526
#define SFE_WMK_ADC_ANGLE_270_0  24322
#define SFE_WMK_ADC_ANGLE_292_5  21326
#define SFE_WMK_ADC_ANGLE_315_0  22844
#define SFE_WMK_ADC_ANGLE_337_5  18149

struct SFEWeatherMeterKitCalibrationParams {
    uint16_t vaneADCValues[WMK_NUM_ANGLES];
};
SFEWeatherMeterKitCalibrationParams _calibrationParams;

// ==================== ANEMOMETER / WIND STATE ====================
const float CALIBRATION_FACTOR  = 2.4;
const int   GLITCH_FILTER_NS    = 4000;

const int AVG_WINDOW_MAX   = 600;   // 10-minute ceiling at 1 sample/sec
int       avgWindowSize    = 600;    // runtime-adjustable, default 60 s
int       gustWindowSize   = 3;     // runtime-adjustable, default 3 s

float avgSamples[AVG_WINDOW_MAX];
int   avgIndex  = 0;
int   avgCount  = 0;

float currentWindSpeed = 0.0;
float averageWindSpeed = 0.0;
float windGust         = 0.0;
float windAngle        = 0.0;
int16_t   windVaneReading  = 0;

// Direction sample buffer — one sample every 3 s → 20 samples per minute.
// Direction reporting always uses the last 60 s regardless of avg window size.
#define DIR_SAMPLE_MAX 20
float dirSamples[DIR_SAMPLE_MAX];
int   dirSampleCount = 0;

// Hourly direction tracking — stores the 60 minutely mode values
#define HOURLY_DIR_SAMPLE_MAX 60
float hourlyDirSamples[HOURLY_DIR_SAMPLE_MAX];
int   hourlyDirSampleCount = 0;
float hourlyWindAngle = 0.0;

pcnt_unit_handle_t pcnt_unit = NULL;

// ==================== LED / PULSE BLINK ====================
volatile uint32_t ledOnTime = 0;   // millis() timestamp set by ISR, 0 = off

// ==================== TIMING ====================
#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

unsigned long reconnectTime = 0;
bool isNtpSynced = false;

// ==================== FUNCTION PROTOTYPES ====================
void    initCalibrationParams();
float   getWindDirection(int16_t adcValue);
void    initPulseCounter();
void    updateWindData();
float   getInstantWindSpeed();
int16_t readWindVaneADC();
void IRAM_ATTR handlePulseInterrupt();
void IRAM_ATTR handleChangeInterrupt();
WidgetTerminal terminal(V10);
void printLocalTime();
BLYNK_WRITE(V10) {
  String cmd = param.asStr();
  if (cmd == "help") {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("a<seconds>  — set avg window (1-600), e.g. a180");
    terminal.println("g<seconds>  — set gust window (1-avg), e.g. g6");
    terminal.println("==End of list.==");
  }
  else if (cmd == "wifi") {
    terminal.print("Connected to: ");
    terminal.println(ssid);
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    terminal.println(WiFi.RSSI());
    printLocalTime();
  }
  else if (cmd.length() >= 2 && cmd[0] == 'a') {
    int val = cmd.substring(1).toInt();
    if (val >= 1 && val <= AVG_WINDOW_MAX) {
      avgWindowSize = val;
      // Reset buffer so stale samples from a different window don't pollute avg
      avgCount = 0;
      avgIndex = 0;
      // Clamp gust window in case it's now larger than the new avg window
      if (gustWindowSize > avgWindowSize) gustWindowSize = avgWindowSize;
      terminal.print("Avg window set to ");
      terminal.print(avgWindowSize);
      terminal.println(" s");
    } else {
      terminal.println("Invalid: avg window must be 1-600");
    }
  }
  else if (cmd.length() >= 2 && cmd[0] == 'g') {
    int val = cmd.substring(1).toInt();
    if (val >= 1 && val <= avgWindowSize) {
      gustWindowSize = val;
      terminal.print("Gust window set to ");
      terminal.print(gustWindowSize);
      terminal.println(" s");
    } else {
      terminal.print("Invalid: gust window must be 1-");
      terminal.println(avgWindowSize);
    }
  }
  terminal.flush();
}

void printLocalTime() {
  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  terminal.print(asctime(timeinfo));
}
// ==================== WIND VANE ====================
void initCalibrationParams() {
    _calibrationParams.vaneADCValues[WMK_ANGLE_0_0]   = SFE_WMK_ADC_ANGLE_0_0;
    _calibrationParams.vaneADCValues[WMK_ANGLE_22_5]  = SFE_WMK_ADC_ANGLE_22_5;
    _calibrationParams.vaneADCValues[WMK_ANGLE_45_0]  = SFE_WMK_ADC_ANGLE_45_0;
    _calibrationParams.vaneADCValues[WMK_ANGLE_67_5]  = SFE_WMK_ADC_ANGLE_67_5;
    _calibrationParams.vaneADCValues[WMK_ANGLE_90_0]  = SFE_WMK_ADC_ANGLE_90_0;
    _calibrationParams.vaneADCValues[WMK_ANGLE_112_5] = SFE_WMK_ADC_ANGLE_112_5;
    _calibrationParams.vaneADCValues[WMK_ANGLE_135_0] = SFE_WMK_ADC_ANGLE_135_0;
    _calibrationParams.vaneADCValues[WMK_ANGLE_157_5] = SFE_WMK_ADC_ANGLE_157_5;
    _calibrationParams.vaneADCValues[WMK_ANGLE_180_0] = SFE_WMK_ADC_ANGLE_180_0;
    _calibrationParams.vaneADCValues[WMK_ANGLE_202_5] = SFE_WMK_ADC_ANGLE_202_5;
    _calibrationParams.vaneADCValues[WMK_ANGLE_225_0] = SFE_WMK_ADC_ANGLE_225_0;
    _calibrationParams.vaneADCValues[WMK_ANGLE_247_5] = SFE_WMK_ADC_ANGLE_247_5;
    _calibrationParams.vaneADCValues[WMK_ANGLE_270_0] = SFE_WMK_ADC_ANGLE_270_0;
    _calibrationParams.vaneADCValues[WMK_ANGLE_292_5] = SFE_WMK_ADC_ANGLE_292_5;
    _calibrationParams.vaneADCValues[WMK_ANGLE_315_0] = SFE_WMK_ADC_ANGLE_315_0;
    _calibrationParams.vaneADCValues[WMK_ANGLE_337_5] = SFE_WMK_ADC_ANGLE_337_5;
}

float getWindDirection(int16_t adcValue) {
    int32_t closestDifference = INT32_MAX;
    uint8_t closestIndex = 0;
    for (uint8_t i = 0; i < WMK_NUM_ANGLES; i++) {
        int32_t diff = abs((int32_t)_calibrationParams.vaneADCValues[i] - (int32_t)adcValue);
        if (diff < closestDifference) {
            closestDifference = diff;
            closestIndex = i;
        }
    }
    return (float)closestIndex * SFE_WIND_VANE_DEGREES_PER_INDEX;
}

// ==================== ADS1115 ====================
int16_t readWindVaneADC() {
    // Continuous mode — just return the most recent conversion, no waiting
    return ads.getRawResult();
}

// ==================== PULSE COUNTER ====================
void initPulseCounter() {
    pcnt_unit_config_t unit_config = {
        .low_limit  = -32768,
        .high_limit =  32767,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = GLITCH_FILTER_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num  = ANEMOMETER_PIN,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_HOLD,      // rising  → ignore
        PCNT_CHANNEL_EDGE_ACTION_INCREASE   // falling → count
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

float getInstantWindSpeed() {
    static uint32_t lastReadMs = 0;
    uint32_t now = millis();
    uint32_t elapsedMs = (lastReadMs == 0) ? 1000 : (now - lastReadMs);
    if (elapsedMs == 0) elapsedMs = 1;  // guard against zero division
    lastReadMs = now;

    int pulse_count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

    // Normalise to a 1-second rate so WiFi stalls don't inflate readings
    float elapsed_s = elapsedMs / 1000.0f;
    return ((float)pulse_count / elapsed_s) * CALIBRATION_FACTOR;
}

// Physical upper limit of the anemometer (~80 m/s → ~288 kph).
// Any sample above this is a measurement artefact (stall, glitch, etc.)
// and is clamped rather than rejected so the buffer always advances.
#define MAX_PLAUSIBLE_KPH  200.0f

void updateWindData() {
    currentWindSpeed = getInstantWindSpeed();
    if (currentWindSpeed > MAX_PLAUSIBLE_KPH) currentWindSpeed = MAX_PLAUSIBLE_KPH;

    // --- rolling average (respects avgWindowSize) ---
    avgSamples[avgIndex] = currentWindSpeed;
    avgIndex = (avgIndex + 1) % avgWindowSize;
    if (avgCount < avgWindowSize) avgCount++;

    float sum = 0.0;
    for (int i = 0; i < avgCount; i++) sum += avgSamples[i];
    averageWindSpeed = sum / avgCount;

    // --- Gust = highest average over any gustWindowSize-wide sub-window
    //     within the full avgWindowSize history ---
    windGust = 0.0;
    if (avgCount >= gustWindowSize) {
        for (int start = 0; start <= avgCount - gustWindowSize; start++) {
            float sum = 0.0;
            for (int j = 0; j < gustWindowSize; j++) {
                // Walk the ring buffer from oldest-available sample forward
                int idx = (avgIndex - avgCount + start + j + avgWindowSize) % avgWindowSize;
                sum += avgSamples[idx];
            }
            float windowAvg = sum / gustWindowSize;
            if (windowAvg > windGust) windGust = windowAvg;
        }
    } else {
        // Not enough samples yet — gust is just the current peak
        for (int i = 0; i < avgCount; i++) {
            if (avgSamples[i] > windGust) windGust = avgSamples[i];
        }
    }
}

BLYNK_CONNECTED() {
  bridgeHubert.setAuthToken (authHubert);
}
// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // ── 1. FastLED FIRST ── WS2812B disables interrupts during write;
    //    must run before WiFi/Blynk init to avoid IPC1 stack corruption.
    FastLED.addLeds<WS2812B, PIN_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(10);
    FastLED.clear();
    FastLED.show();
    leds[0] = CRGB::White; FastLED.show(); delay(500);
    leds[0] = CRGB::Black; FastLED.show(); delay(500);
    leds[0] = CRGB::White; FastLED.show(); delay(500);
    leds[0] = CRGB::Black; FastLED.show();

    Serial.println("ESP32-S3 Wind Transmitter (WiFi + Blynk + MQTT)");

    // ── 2. Wind vane calibration params ──
    initCalibrationParams();

    // ── 3. I2C + ADS1115 ──
    Wire.begin(5, 6);
    ads.init();
    ads.setVoltageRange_mV(ADS1115_RANGE_4096);
    ads.setCompareChannels(ADS1115_COMP_0_GND);
    ads.setConvRate(ADS1115_8_SPS);
    
    ads.setMeasureMode(ADS1115_CONTINUOUS);  // free-running; getRawResult() always has fresh data
    //ads.startSingleMeasurement();            // kick off continuous conversions

    // ── 4. Hardware pulse counter ──
    Serial.println("Initializing pulse counter...");
    initPulseCounter();
    Serial.println("Pulse counter initialized!");

    // ── 5. WiFi ──
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());

    // ── 6. Blynk (local server) ──
    Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
    Blynk.connect();
    Serial.print("Connecting to Blynk");
    unsigned long blynkStart = millis();
    while (!Blynk.connected() && millis() - blynkStart < 10000) {
        Serial.print(".");
        delay(500);
    }
    if (Blynk.connected()) {
        Serial.println("\nBlynk connected!");
    } else {
        Serial.println("\nBlynk timeout — will retry in loop.");
    }

    // ── 7. MQTT ──
    mqttReconnectTimer = xTimerCreate("mqttReconnect", pdMS_TO_TICKS(5000),
                                      pdFALSE, NULL,
                                      [](TimerHandle_t){ connectToMqtt(); });
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCredentials(mqttUser, mqttPassword);
    mqttClient.setClientId(mqttClientId);
    connectToMqtt();

    // ── 8. NTP ──
    sntp_set_time_sync_notification_cb([](struct timeval*) {
        isNtpSynced = true;
        Serial.println("NTP synced.");
    });
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "192.168.50.197");
    esp_sntp_init();
    setenv("TZ", "EST5EDT,M3.2.0,M11.1.0", 1);
    tzset();

    // ── 9. Attach anemometer interrupt LAST ──
    attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), handleChangeInterrupt, CHANGE);
    Serial.println("Setup complete. Transmitting wind data.");
    ArduinoOTA.setHostname("JoeyWindTransmitter");
    ArduinoOTA.begin();
  terminal.println("***WIND SERVER STARTED***");
  terminal.print("Connected to ");
  terminal.println(ssid);
  terminal.print("IP address: ");
  terminal.println(WiFi.localIP());
  printLocalTime();
  terminal.print("Compiled on: ");
  terminal.print(__DATE__);
  terminal.print(" at ");
  terminal.println(__TIME__);
  terminal.flush();
}

// ==================== INTERRUPTS ====================
void IRAM_ATTR handlePulseInterrupt() {
    // kept for pulse-counter compatibility (not attached, logic preserved)
    static uint32_t lastTrigger = 0;
    uint32_t now = millis();
    if (now - lastTrigger > 20) {
        lastTrigger = now;
    }
}

void IRAM_ATTR handleChangeInterrupt() {
    if (digitalRead(ANEMOMETER_PIN) == LOW) {
        digitalWrite(LED_PIN, HIGH);  // falling edge → LED ON
        ledOnTime = millis();
    } else { 
        digitalWrite(LED_PIN, LOW);   // rising edge → LED OFF immediately
        ledOnTime = 0;
    }
}


// ==================== LOOP ====================
void loop() {
    // Turn LED off after 10 ms, or immediately if the switch has already opened
    if (ledOnTime && (millis() - ledOnTime >= 2)) {
        digitalWrite(LED_PIN, LOW);
        ledOnTime = 0;
    }

    // WiFi watchdog — reconnect if dropped
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - reconnectTime > 30000) {
            Serial.println("WiFi lost — reconnecting...");
            WiFi.disconnect();
            WiFi.reconnect();
            reconnectTime = millis();
        }
    } else if (!mqttClient.connected() && !xTimerIsTimerActive(mqttReconnectTimer)) {
        // WiFi is up but MQTT is not connected and no reconnect is pending —
        // this can happen if WiFi dropped before the MQTT disconnect callback
        // had a chance to arm the timer. Kick off a reconnect now.
        Serial.println("WiFi up but MQTT disconnected — scheduling reconnect...");
        xTimerStart(mqttReconnectTimer, 0);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Blynk.run();
        ArduinoOTA.handle();
    }
    if ((WiFi.status() == WL_CONNECTED) && !Blynk.connected()) {
        Blynk.connect();
    }


    // Sample wind speed every second
    every(1000) {
        updateWindData();
    }

    // Sample wind vane every 3 seconds and publish to MQTT
    every(3000) {
        windVaneReading = readWindVaneADC();
        float angle = getWindDirection(windVaneReading);

        if (dirSampleCount < DIR_SAMPLE_MAX) {
            dirSamples[dirSampleCount++] = angle;
        }

        if (mqttClient.connected()) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.2f", averageWindSpeed);
            mqttClient.publish("home/joeywind/avgwind", 0, false, buf);

            snprintf(buf, sizeof(buf), "%.2f", windGust);
            mqttClient.publish("home/joeywind/windgust", 0, false, buf);

            snprintf(buf, sizeof(buf), "%.1f", angle);
            mqttClient.publish("home/joeywind/angle", 0, false, buf);
        }
    }

    // Push to Blynk every 60 seconds using mode direction from accumulated samples
    every(60000) {
        // Compute mode direction from accumulated samples.
        // The vane snaps to one of 16 fixed angles, so we count occurrences
        // of each and pick the most frequent.
        if (dirSampleCount > 0) {
            int votes[WMK_NUM_ANGLES] = {};
            for (int i = 0; i < dirSampleCount; i++) {
                int idx = (int)roundf(dirSamples[i] / SFE_WIND_VANE_DEGREES_PER_INDEX)
                          % WMK_NUM_ANGLES;
                votes[idx]++;
            }
            int bestIdx = 0;
            for (int i = 1; i < WMK_NUM_ANGLES; i++) {
                if (votes[i] > votes[bestIdx]) bestIdx = i;
            }
            windAngle = bestIdx * SFE_WIND_VANE_DEGREES_PER_INDEX;
        }
        // Reset direction sample buffer for next minute
        dirSampleCount = 0;

        // Store this minute's mode direction for hourly calculation
        if (hourlyDirSampleCount < HOURLY_DIR_SAMPLE_MAX) {
            hourlyDirSamples[hourlyDirSampleCount++] = windAngle;
        }

        if (Blynk.connected()) {
            Blynk.virtualWrite(VPIN_AVG_WIND,  averageWindSpeed);
            Blynk.virtualWrite(VPIN_WIND_GUST, windGust);
            Blynk.virtualWrite(VPIN_WIND_ANGLE, windAngle);
        }
        // FIX: Reset gust so it reflects the peak of the *next* reporting
        // window rather than accumulating indefinitely.
        // Do NOT reset avgCount/avgIndex — the rolling window is continuous.
        windGust = 0.0;
    }

    // Calculate and push hourly mode direction every 3600 seconds (1 hour)
    every(3600000) {
        // Compute mode direction from the 60 minutely mode values
        if (hourlyDirSampleCount > 0) {
            int votes[WMK_NUM_ANGLES] = {};
            for (int i = 0; i < hourlyDirSampleCount; i++) {
                int idx = (int)roundf(hourlyDirSamples[i] / SFE_WIND_VANE_DEGREES_PER_INDEX)
                          % WMK_NUM_ANGLES;
                votes[idx]++;
            }
            int bestIdx = 0;
            for (int i = 1; i < WMK_NUM_ANGLES; i++) {
                if (votes[i] > votes[bestIdx]) bestIdx = i;
            }
            hourlyWindAngle = bestIdx * SFE_WIND_VANE_DEGREES_PER_INDEX;
        }
        // Reset hourly direction sample buffer for next hour
        hourlyDirSampleCount = 0;

        if (Blynk.connected()) {
            Blynk.virtualWrite(VPIN_WIND_ANGLE_HOURLY, hourlyWindAngle);
        }
    }
}