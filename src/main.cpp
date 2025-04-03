#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "env.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

String endpoint = "https://api.fyahalarm.com";

void scanWiFiNetworks();
void sendRequest(String jsonString);

const char* Host = "www.googleapis.com";
String thisPage = "/geolocation/v1/geolocate?key=";
String key = "AIzaSyArfQH4roSbTQfg1d9qHydtbF1hCvg_A8c";

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
volatile bool dataReceived = false;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  Serial.print("Device ID: ");
  Serial.println(sensorData.device_id);
  Serial.print("Temperature: ");
  Serial.println(sensorData.temperature);
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


void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

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

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  
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

          // Serial.println("--------------------------------------");
          // Serial.print("Latitude: ");
          // Serial.println(latitude, 6);
          // Serial.print("Longitude: ");
          // Serial.println(longitude, 6);
          // Serial.print("Accuracy: ");
          // Serial.println(accuracy);
          // Serial.println("--------------------------------------\n");
      } else {
          Serial.println("JSON parsing failed!");
      }
  } else {
      Serial.print("Error on HTTP request: ");
      Serial.println(httpResponseCode);
  }

  http.end();  // Close the connection
}

void loop() {
  scanWiFiNetworks();

  // if (WiFi.status() == WL_CONNECTED && dataReceived) {
  //   scanWiFiNetworks();
  // } else {
  //   Serial.println("Waiting for new sensor data...");
  //   delay(2000);
  // }
  //--------------------------------------------------------

}
