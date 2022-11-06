
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <WiFiClientSecure.h>
//#include <Husarnet ESP32.h>
// definicja pinów
float dana = 1.09;
float dana2 = 2.09;
float dana3 = 3.09;
float dana4 = 4.09;
const char* ssid = "NiePodejrzaneWIFI";
const char* password = "zaq1@WSX";
String message = "";
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create a WebSocket object

AsyncWebSocket ws("/ws");
// Inicjowanie SPIFFS
void initFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  else{
   Serial.println("SPIFFS mounted successfully");
  }
}

// Inicjowanie WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}
//Tworzenie JSona
JSONVar SensorData; // zmienna do trzymania danyhc w Jsonie
String GetData(){
  Serial.println("O KURNA");
  SensorData["dana"] = String(dana);
  SensorData["dana2"] = String(dana2);
  SensorData["dana3"] = String(dana3);
  SensorData["dana4"] = String(dana4);
  Serial.println("O KURNA2");
  String jsonString = JSON.stringify(SensorData);
  Serial.println("O KURNA3");
  return jsonString;
}

//Wysyłanie danych
void notifyClients(String DataValues) {
  ws.textAll(DataValues);
}
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  Serial.printf("??");
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    
    message = (char*)data;
 
    if (strcmp((char*)data, "getValues") == 0) {
       notifyClients(GetData());
    }
  }
}
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
 // switch(type){
    // case WS_EVT_CONNECT:
    // notifyClients(GetData());
    // Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    // break;
    // case WS_EVT_PONG:
    // notifyClients(GetData());
    // break;
    switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      Serial.printf("req");
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// Inicjowanie Socketa 
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup(){
  Serial.begin(115200);
  initFS();
  initWiFi();
  // Husarnet.join(
  // "fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/DGPYXCsZTxPBQcL6uJwUDg", 
  // "hydros");
  // Husarnet.start();
  initWebSocket();
  
  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });
  
  server.serveStatic("/", SPIFFS, "/");

  // Start server
  server.begin();
}
void loop(){
 // ws.cleanupClients();
}