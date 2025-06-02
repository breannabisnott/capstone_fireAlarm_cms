#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "env.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

const int BUTTON_PIN1 = 32;  // Butt1
const int BUTTON_PIN2 = 33;  // Butt2
int displayMode = 0;         // Tracks which data to display
bool lastButtonState = HIGH;

unsigned long sirenStartTime = 0;
unsigned long sirenDuration = 3000;  // Duration in milliseconds
const unsigned long COOLDOWN_PERIOD = 0; // 5 mins before allowing retrigger
unsigned long lastTriggerTime = 0;
bool sirenActive = false;
bool dataUpdated = false;

unsigned long lastPressTime = 0;
const int siren = 27;

String endpoint = "https://api.fyahalarm.com";

void scanWiFiNetworks();
void sendRequest(String jsonString);
void fetchDataFromAPI();
void processDataIfReady();
void scanWiFiTask(void * parameter);

TaskHandle_t wifiScanTaskHandle = NULL;

volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;  // 200 ms debounce
volatile int pressCount = 0;

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long currentTime = millis();
  // Check if enough time has passed since the last press
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = currentTime;
  }
}

const char* Host = "www.googleapis.com";
String thisPage = "/geolocation/v1/geolocate?key=";
String key = "AIzaSyDidYXtk9kzBG_sDo1bVP4U89PblOW6Ocs";

// Structure example to receive data
// Must match the sender structure
typedef struct sensor_data {
  char device_id[50];
  float temperature;
  bool fire_status; //digital flame
  float fire_level; //analog flame
  bool threshold;  //gas thresh
  float sensorValue; // gas conc
  float oxygen_level; // o2 conc
  float humidity;
  float lat;
  float lng;
  float accuracy;
} sensor_data;

sensor_data sensorData;
sensor_data latestData;
volatile bool dataReceived = false;

unsigned long lastDataFetchTime = 0;
const unsigned long DATA_FETCH_INTERVAL = 5000; // 5 seconds

unsigned long lastApiUpdate = 0;
const unsigned long API_UPDATE_INTERVAL = 5000; // Update every 5 seconds

void updateLCD() {
  
  lcd.clear();

  switch (displayMode) {
    case 0:  // Temperature & Humidity
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(sensorData.temperature);
      Serial.print(latestData.temperature);
      lcd.print("C");
      
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(latestData.humidity);
      lcd.print("%");
      break;

    case 1:  // Gas & O2 Levels
      lcd.setCursor(0, 0);
      lcd.print("Gas: ");
      lcd.print(sensorData.sensorValue);
      
      lcd.setCursor(0, 1);
      lcd.print("O2: ");
      lcd.print(latestData.oxygen_level);
      lcd.print("%");
      break;

    case 2:  // Flame & Gas Threshold
      lcd.setCursor(0, 0);
      lcd.print("Flame: ");
      lcd.print(latestData.fire_status ? "YES" : "NO");
      
      lcd.setCursor(0, 1);
      lcd.print("Gas Thresh: ");
      lcd.print(latestData.threshold ? "HIGH" : "LOW");
      break;

    case 3:  // Flame Level
      lcd.setCursor(0, 0);
      lcd.print("Flame Lvl: ");
      lcd.print(latestData.fire_level);
      break;

    default:
      displayMode = 0;  // Reset to first mode
      updateLCD();
      break;
  }
}
    
   

void fetchDataFromAPI() {
  if ((millis() - lastApiUpdate) < API_UPDATE_INTERVAL) {
    return;
  }

  WiFiClientSecure client;
  HTTPClient http;
  client.setInsecure(); // For testing only

  String fullUrl = endpoint + "/latestData/" + "AC:15:18:D7:B0:80";
  http.begin(client, fullUrl);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println(payload);
  
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      JsonArray arr = doc.as<JsonArray>();  // ✅ Define it here

      if (arr.size() > 0) {
        JsonObject data = arr[0];

        latestData.temperature = data["temperature"] | 0;
        latestData.fire_status = data["flame"] | false;
        latestData.fire_level = data["flame_level"] | 0;
        latestData.threshold = data["gas"] | false;
        latestData.sensorValue = data["gas_concentration"] | 0;
        latestData.oxygen_level = data["oxygen_concentration"] | 0;
        latestData.humidity = data["humidity"] | 0;

        updateLCD();  // ✅ Immediately update LCD with new values
      }
    }else {
      Serial.print("JSON deserialization failed: ");
      Serial.println(error.c_str());
    }
  }else {
    Serial.print("HTTP Error: ");
    Serial.println(httpCode);
    Serial.println(http.getString());
  }
  http.end();
  lastApiUpdate = millis();
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));

  Serial.print("Device ID: ");
  Serial.println(sensorData.device_id);
  Serial.print("Temperature: ");
  Serial.println(sensorData.temperature);
  Serial.print("Humidity: ");
  Serial.println(sensorData.humidity);
  Serial.print("Flame Detection: ");
  Serial.println(sensorData.fire_status);
  Serial.print("Flame Level: ");
  Serial.println(sensorData.fire_level);
  Serial.print("Gas Threshold: ");
  Serial.println(sensorData.threshold);
  Serial.print("Gas Concentration: ");
  Serial.println(sensorData.sensorValue);
  Serial.print("O2 Concentration: ");
  Serial.println(sensorData.oxygen_level);
  Serial.println();

  dataReceived = true;

}


void post_sensor_data(float latitude, float longitude, float accuracy){
  if (!dataReceived) return;

  HTTPClient http;
  String requestBody;

  String newEndpoint;
  String path = "/data";
  newEndpoint = endpoint + path;    //change path

  http.begin(newEndpoint);
  http.addHeader("Content-Type", "application/json");

  JsonDocument doc;

  sensorData.lat = latitude;
  sensorData.lng = longitude;
  sensorData.accuracy = accuracy;

  doc["device_id"] = sensorData.device_id;
  doc["temperature"] = sensorData.temperature;
  doc["flame"] = sensorData.fire_status;
  doc["flame_level"]  = sensorData.fire_level;
  doc["gas"]  = sensorData.threshold;
  doc["gas_concentration"]  = sensorData.sensorValue;
  doc["oxygen_concentration"] = sensorData.oxygen_level;
  doc["humidity"] = sensorData.humidity;
  doc["lat"] = sensorData.lat;
  doc["lng"] = sensorData.lng;
  doc["accuracy"] = sensorData.accuracy;

  Serial.println("--------------------------------------");
  Serial.print("Latitude: ");
  Serial.println(sensorData.lat, 6);
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);
  Serial.print("Accuracy: ");
  Serial.println(accuracy);
  Serial.println("--------------------------------------\n");

  doc.shrinkToFit();

  serializeJson(doc, requestBody);

  int httpResponseCode = http.POST(requestBody);

  Serial.print("HERE IS THE RESPONSE: ");
  Serial.println(requestBody);
  Serial.println(http.getString());
  Serial.println();

  http.end();

  dataReceived = false;
}

void scanWiFiNetworks() {
    Serial.println("Scanning WiFi...");
    int n = WiFi.scanNetworks();
    
    if (n == 0) {
        Serial.println("No networks found");
        return;
    }

    Serial.println(String(n) + " networks found.");
    
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

  Serial.println("Connecting to Google...");
  http.begin(url);  // Start the HTTP request
  http.addHeader("Content-Type", "application/json");  // Set the content type

  Serial.println("Sending request...");
  int httpResponseCode = http.POST(jsonString);  // Send the POST request

  if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);

      String response = http.getString();  // Get the response payload
      //Serial.println("Response: " + response);

      // Parse JSON response
      JsonDocument docloc;
      DeserializationError error = deserializeJson(docloc, response);

      if (!error) {
          float latitude = docloc["location"]["lat"];
          float longitude = docloc["location"]["lng"];
          float accuracy = docloc["accuracy"];

          post_sensor_data(latitude, longitude, accuracy);

      } else {
          Serial.println("JSON parsing failed!");
      }
  } else {
      Serial.print("Error on HTTP request: ");
      Serial.println(httpResponseCode);
  }

  http.end();  // Close the connection
}

void checkSirenTrigger() {
  if (lastTriggerTime > 0 && millis() - lastTriggerTime < COOLDOWN_PERIOD && !sirenActive) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - skipping siren check");
    return;
  }

  WiFiClientSecure client;
  HTTPClient http;
  
  // Configure SSL (important for HTTPS)
  client.setInsecure(); // Bypass SSL verification (for testing)
  // For production, add root CA certificate instead with:
  // client.setCACert(root_ca);

  // Construct the complete URL
  String fullUrl = endpoint + "/siren-triggers/" + "AC:15:18:D7:B0:80";
  
  // Serial.print("Connecting to: ");
  // Serial.println(fullUrl);

  // Begin connection with HTTPS client
  http.begin(client, fullUrl);
  
  // Add headers if needed (remove if not required)
  http.addHeader("Content-Type", "application/json");
  // http.addHeader("Authorization", "Bearer your_token"); // If using auth

  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.print("API Response: ");
    Serial.println(payload);

    // Parse JSON response
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      bool fireTrigger = doc["fire_trigger"] | false;
      bool tempTrigger = doc["temp_trigger"] | false;
      bool gasTrigger = doc["gas_trigger"] | false;

      if ((fireTrigger | tempTrigger | gasTrigger) && !sirenActive) {
        sirenStartTime = millis();
        lastTriggerTime = millis();
        sirenActive = true;
        if (fireTrigger) digitalWrite(siren, HIGH);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ALERT!");
        if (fireTrigger) lcd.print(" Siren ON");
        
        lcd.setCursor(0, 1);
        if (fireTrigger) lcd.print("Fire ");
        if (tempTrigger) lcd.print("Temp ");
        if (gasTrigger) lcd.print("Gas ");
        
        //Serial.println("Siren ACTIVATED due to:");
        
        if (fireTrigger) Serial.println("- Fire detected");
        if (tempTrigger) Serial.println("- High temperature");
        if (gasTrigger) Serial.println("- Gas leak detected");
      }
    } else {
      Serial.print("JSON parse failed: ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.print("HTTP Error: ");
    Serial.println(httpCode);
    Serial.print("Response: ");
    Serial.println(http.getString()); // Print error response if available
  }
  
  http.end();
}


void scanWiFiTask(void * parameter) {
  for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      scanWiFiNetworks();
      vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void sirenCheckTask(void *parameter) {
  while (true) {
    checkSirenTrigger();
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Check every 5 seconds
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  delay(5000);

  pinMode(BUTTON_PIN2, INPUT_PULLUP);

  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN1), handleButtonInterrupt, FALLING);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Welcome to");
  lcd.setCursor(3, 1);
  lcd.print("Fyah Alarm");

  pinMode(siren, OUTPUT);

  // WiFi_SSID and WIFI_PASS should be stored in the env.h
  WiFi.begin(ssid, password);

  // Connect to wifi
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Set the Wi-Fi channel (1-14, usually 1-11 in most regions)
  int channel = 6; // Choose a channel with least interference
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA); // Works even without WiFi connection

  esp_now_register_recv_cb(OnDataRecv);

  xTaskCreatePinnedToCore(
    scanWiFiTask,           // Function to run
    "WiFiScanTask",         // Name of task
    8192,                   // Stack size (bytes)
    NULL,                   // Parameter
    1,                      // Priority
    &wifiScanTaskHandle,    // Task handle
    1                       // Core to run on (usually 1 for networking)
  );
  xTaskCreate(sirenCheckTask, "SirenCheckTask", 8000, NULL, 1, NULL);

}

void loop() {

  if (buttonPressed) {
    buttonPressed = false;

    displayMode = (displayMode + 1) % 4;
    updateLCD();
    Serial.println("Button was pressed once.");
  }

  if (sirenActive && (millis() - sirenStartTime >= sirenDuration)) {
    digitalWrite(siren, LOW);
    sirenActive = false;
    Serial.println("Siren deactivated after duration");
  }

}

