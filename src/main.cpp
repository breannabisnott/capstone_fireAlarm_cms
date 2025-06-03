#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "env.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

const int BUTTON_PIN1 = 32;
const int BUTTON_PIN2 = 33;
const int siren = 27;

int displayMode = 0;
bool lastButtonState = HIGH;

// Siren control variables
unsigned long sirenStartTime = 0;
unsigned long sirenDuration = 3000;
unsigned long lastTriggerTime = 0;
bool sirenActive = false;
bool sirenSilenced = false;

// Button variables for debouncing stuff
volatile bool buttonPressed = false;
volatile bool silenceButtonPressed = false; 
unsigned long lastDebounceTime = 0;
unsigned long lastSilenceDebounceTime = 0; 
const unsigned long debounceDelay = 200;

// Communication modes
typedef enum {
  MODE_ESPNOW,
  MODE_WIFI
} comm_mode_t;

comm_mode_t currentMode = MODE_ESPNOW;
unsigned long lastModeSwitch = 0;
const unsigned long ESPNOW_DURATION = 10000;  // 10 seconds for ESP-NOW
const unsigned long WIFI_DURATION = 5000;     // 5 seconds for WiFi operations

typedef struct sensor_data {
  char device_id[50];
  float temperature;
  bool fire_status;
  float fire_level;
  bool threshold;
  float sensorValue;
  float oxygen_level;
  float humidity;
  float lat;
  float lng;
  float accuracy;
} sensor_data;


sensor_data sensorData;
sensor_data pendingData[5];  // Buffer for multiple readings
int pendingDataCount = 0;
bool hasNewData = false;


float latitude = 0.0;
float longitude = 0.0;
float accuracy = 0.0;
bool locationObtained = false;

String endpoint = "https://api.fyahalarm.com";
const char* Host = "www.googleapis.com";
String thisPage = "/geolocation/v1/geolocate?key=";
String key = "AIzaSyDidYXtk9kzBG_sDo1bVP4U89PblOW6Ocs";

// Task handles
TaskHandle_t communicationTaskHandle = NULL;

void initESPNOW();
void initWiFi();
void switchToESPNOW();
void switchToWiFi();
void updateLCD();
void scanWiFiNetworks();
void sendRequest(String jsonString);
void post_sensor_data();
void checkSirenTrigger();
void communicationTask(void *parameter);
void silenceSiren();

// interrupt handler for button 1 (display button)
void IRAM_ATTR handleButtonInterrupt() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = currentTime;
  }
}

// interrupt handler for button 2 (silence button)
void IRAM_ATTR handleSilenceButtonInterrupt() {
  unsigned long currentTime = millis();
  if ((currentTime - lastSilenceDebounceTime) > debounceDelay) {
    silenceButtonPressed = true;
    lastSilenceDebounceTime = currentTime;
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Store data in buffer to prevent blocking
  if (pendingDataCount < 5) {
    memcpy(&pendingData[pendingDataCount], incomingData, sizeof(sensor_data));
    pendingDataCount++;
    hasNewData = true;
    
    Serial.println("Data received via ESP-NOW");
    Serial.print("Device ID: ");
    Serial.println(pendingData[pendingDataCount-1].device_id);
    Serial.print("Temperature: ");
    Serial.println(pendingData[pendingDataCount-1].temperature);
  }
}

void updateLCD() {
  lcd.clear();

  switch (displayMode) {
    case 0:  // Temperature & Humidity
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(sensorData.temperature);
      lcd.print("C");
      
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(sensorData.humidity);
      lcd.print("%");
      break;

    case 1:  // Gas & O2 Levels
      lcd.setCursor(0, 0);
      lcd.print("Gas: ");
      lcd.print(sensorData.sensorValue);
      
      lcd.setCursor(0, 1);
      lcd.print("O2: ");
      lcd.print(sensorData.oxygen_level);
      lcd.print("%");
      break;

    case 2:  // Flame & Gas Threshold
      lcd.setCursor(0, 0);
      lcd.print("Flame: ");
      lcd.print(sensorData.fire_status ? "YES" : "NO");
      
      lcd.setCursor(0, 1);
      lcd.print("Gas Thresh: ");
      lcd.print(sensorData.threshold ? "HIGH" : "LOW");
      break;

    case 3:  // Status & Location
      lcd.setCursor(0, 0);
      lcd.print("Mode: ");
      lcd.print(currentMode == MODE_ESPNOW ? "ESP-NOW" : "WiFi");
      
      lcd.setCursor(0, 1);
      if (locationObtained) {
        lcd.print("Location: OK");
      } else {
        lcd.print("Getting location...");
      }
      break;

    default:
      displayMode = 0;
      updateLCD();
      break;
  }
}

void switchToESPNOW() {
  if (currentMode == MODE_ESPNOW) return;
  
  Serial.println("Switching to ESP-NOW mode");
  
  // PROPERLY disconnect WiFi
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500); //cleanup again
  }
  
  WiFi.mode(WIFI_STA);
  delay(100);
  
  int channel = 6;
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  currentMode = MODE_ESPNOW;
  lastModeSwitch = millis();
  Serial.println("ESP-NOW mode active");
}

void switchToWiFi() {
  if (currentMode == MODE_WIFI) return;
  
  Serial.println("Switching to WiFi mode");
  
  // Properly deinitialize ESP-NOW
  esp_now_unregister_recv_cb();
  esp_now_deinit();
  delay(500); // Give time for cleanup to prevent those random characters...
  
  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  delay(100);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    currentMode = MODE_WIFI;
  } else {
    Serial.println("\nWiFi connection failed - staying in current mode");
    // Don't switch back to ESP-NOW immediately, let it retry
  }
  lastModeSwitch = millis();
}

void scanWiFiNetworks() {
  Serial.println("Scanning WiFi...");
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("No networks found");
    return;
  }

  String jsonString;
  jsonString = "{\n\"homeMobileCountryCode\": 234,\n";
  jsonString += "\"homeMobileNetworkCode\": 27,\n";
  jsonString += "\"radioType\": \"gsm\",\n";
  jsonString += "\"carrier\": \"Vodafone\",\n";
  jsonString += "\"wifiAccessPoints\": [\n";
  
  for (int i = 0; i < n; i++) {
    jsonString += "{\n";
    jsonString += "\"macAddress\" : \"" + WiFi.BSSIDstr(i) + "\",\n";
    jsonString += "\"signalStrength\": " + String(WiFi.RSSI(i)) + "\n";
    jsonString += (i < n - 1) ? "},\n" : "}\n";
  }

  jsonString += "]\n}";
  sendRequest(jsonString);
}

void sendRequest(String jsonString) {
  HTTPClient http;
  String url = "https://" + String(Host) + thisPage + key;

  Serial.println("Getting device location...");
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0) {
    String response = http.getString();
    
    JsonDocument docloc;
    DeserializationError error = deserializeJson(docloc, response);

    if (!error) {
      latitude = docloc["location"]["lat"];
      longitude = docloc["location"]["lng"];
      accuracy = docloc["accuracy"];
      locationObtained = true;
      
      Serial.println("Location obtained successfully:");
      Serial.print("Latitude: ");
      Serial.println(latitude, 6);
      Serial.print("Longitude: ");
      Serial.println(longitude, 6);
      Serial.print("Accuracy: ");
      Serial.println(accuracy);
    } else {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.print("Location request failed: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void post_sensor_data() {
  if (pendingDataCount == 0) return;
  
  // Skip posting if location not obtained yet
  if (!locationObtained) {
    Serial.println("Location not available yet, skipping data post");
    return;
  }

  HTTPClient http;
  String newEndpoint = endpoint + "/data";
  
  http.begin(newEndpoint);
  http.addHeader("Content-Type", "application/json");

  // process the pending data jsut received
  for (int i = 0; i < pendingDataCount; i++) {
    JsonDocument doc;
    
    // using the fixed location obtained during setup
    pendingData[i].lat = latitude;
    pendingData[i].lng = longitude;
    pendingData[i].accuracy = accuracy;

    doc["device_id"] = pendingData[i].device_id;
    doc["temperature"] = pendingData[i].temperature;
    doc["flame"] = pendingData[i].fire_status;
    doc["flame_level"] = pendingData[i].fire_level;
    doc["gas"] = pendingData[i].threshold;
    doc["gas_concentration"] = pendingData[i].sensorValue;
    doc["oxygen_concentration"] = pendingData[i].oxygen_level;
    doc["humidity"] = pendingData[i].humidity;
    doc["lat"] = pendingData[i].lat;
    doc["lng"] = pendingData[i].lng;
    doc["accuracy"] = pendingData[i].accuracy;

    String requestBody;
    serializeJson(doc, requestBody);

    int httpResponseCode = http.POST(requestBody);
    
    if (httpResponseCode == 200) {
      Serial.println("Data posted successfully");
      // Update latest sensor data for display
      memcpy(&sensorData, &pendingData[i], sizeof(sensor_data));
    } else {
      Serial.print("Post failed: ");
      Serial.println(httpResponseCode);
    }
    
    delay(100); // Small delay between posts
  }
  
  pendingDataCount = 0; // Clear buffer
  hasNewData = false;
  http.end();
}

void checkSirenTrigger() {
  WiFiClientSecure client;
  HTTPClient http;
  client.setInsecure();

  String fullUrl = endpoint + "/siren-triggers/" + "AC:15:18:D7:B0:80";
  
  http.begin(client, fullUrl);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      bool fireTrigger = doc["fire_trigger"] | false;
      bool tempTrigger = doc["temp_trigger"] | false;
      bool gasTrigger = doc["gas_trigger"] | false;

      // Only activate siren if it's not already active and not silenced
      if ((fireTrigger || tempTrigger || gasTrigger) && !sirenActive && !sirenSilenced) {
        sirenStartTime = millis();
        lastTriggerTime = millis();
        sirenActive = true;
        
        if (fireTrigger) digitalWrite(siren, HIGH);

        lcd.clear();
        delay(50);
        lcd.setCursor(0, 0);
        lcd.print("ALERT!");
        if (fireTrigger) lcd.print(" Siren ON");

        lcd.setCursor(0, 1);
        String triggerMsg = "";
        if (fireTrigger) triggerMsg += "Fire ";
        if (tempTrigger) triggerMsg += "Temp ";
        if (gasTrigger) triggerMsg += "Gas ";
        
        // Clear the line first, then print
        lcd.print("                    ");
        lcd.setCursor(0, 1);
        lcd.print(triggerMsg);
        
        
        Serial.println("SIREN ACTIVATED!");
      }
      
      // Reset silenced flag if no triggers are active
      if (!fireTrigger) {
        sirenSilenced = false;
      }
    }
  }
  
  http.end();
}

void silenceSiren() {
  if (sirenActive) {
    digitalWrite(siren, LOW);
    sirenActive = false;
    sirenSilenced = true;  // Mark as manually silenced
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SIREN SILENCED");
    lcd.setCursor(0, 1);
    lcd.print("by user");
    
    Serial.println("Siren manually silenced by user");
    
    // Show silenced message for 2 seconds, then return to normal display
    delay(2000);
    updateLCD();
  }
}

void communicationTask(void *parameter) {
  while (true) {
    unsigned long currentTime = millis();
    
    if (currentMode == MODE_ESPNOW) {
      // ESP-NOW mode - listen for sensor data
      if (currentTime - lastModeSwitch >= ESPNOW_DURATION) {
        switchToWiFi();
      }
    } else if (currentMode == MODE_WIFI) {
      // wifi mode - for API requests
      if (WiFi.status() == WL_CONNECTED) {
        
        // Get location first if not obtained yet but this shouldn't happen after the inital setup
        if (!locationObtained) {
          scanWiFiNetworks();
        } else {
          // Post any pending sensor data
          if (hasNewData) {
            post_sensor_data();
            updateLCD(); // Update display with latest data
          }
          
          checkSirenTrigger();
        }
      } else {
        Serial.println("WiFi disconnected in task");
      }
      
      // Switch back to ESP-NOW mode
      if (currentTime - lastModeSwitch >= WIFI_DURATION) {
        switchToESPNOW();
      }
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(9600);
  delay(2000);

  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  pinMode(siren, OUTPUT);
  
  // interrupts for buttons
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN1), handleButtonInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN2), handleSilenceButtonInterrupt, FALLING);

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Welcome to");
  lcd.setCursor(3, 1);
  lcd.print("Fyah Alarm");
  delay(2000);

  // Initialize communication and get location
  Serial.println("Initializing WiFi for location...");
  
  // Start with a clean WiFi initialization
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(100);
  
  WiFi.begin(ssid, password);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi...");
  
  // wait for wifi
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(1000);
    Serial.print(".");
    wifiAttempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected for setup");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    currentMode = MODE_WIFI;
    
    Serial.println("Getting device location...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Getting location...");
    
    // Get location
    int locationAttempts = 0;
    while (!locationObtained && locationAttempts < 3) {
      scanWiFiNetworks();
      if (!locationObtained) {
        delay(2000);
      }
      locationAttempts++;
    }
    
    if (locationObtained) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Location");
      lcd.setCursor(0, 1);
      lcd.print("obtained!");
      delay(2000);
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Location failed");
      lcd.setCursor(0, 1);
      lcd.print("Using default");
      delay(2000);
      // Set default location if geolocation fails
      latitude = 0.0;
      longitude = 0.0;
      accuracy = 1000.0;
      locationObtained = true;
    }
  } else {
    Serial.println("\nWiFi connection failed during setup");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi failed");
    lcd.setCursor(0, 1);
    lcd.print("Using default loc");
    delay(2000);
    
    // Set default location
    latitude = 0.0;
    longitude = 0.0;
    accuracy = 1000.0;
    locationObtained = true;
  }
  
  // Now switch to ESP-NOW for normal operation
  switchToESPNOW();
  
  // Create communication task
  xTaskCreate(communicationTask, "CommunicationTask", 10000, NULL, 1, &communicationTaskHandle);
  
  Serial.println("System initialized");
}

void loop() {
  // display mode button presses (Button 1)
  if (buttonPressed) {
    buttonPressed = false;
    displayMode = (displayMode + 1) % 4;
    updateLCD();
    Serial.println("Display mode changed");
  }

  // silence button presses (Button 2)
  if (silenceButtonPressed) {
    silenceButtonPressed = false;
    silenceSiren();
    Serial.println("Silence button pressed");
  }

  // siren timeout (only if not manually silenced)
  if (sirenActive && (millis() - sirenStartTime >= sirenDuration)) {
    digitalWrite(siren, LOW);
    sirenActive = false;
    Serial.println("Siren deactivated by timeout");
    updateLCD(); // Refresh display
  }

  delay(100);
}