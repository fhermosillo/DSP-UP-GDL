#include <WiFi.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "yourNetworkName";
const char* password =  "yourNetworkPass";

AsyncWebServer server(80);

void setup(){
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println(WiFi.localIP());

  // on header callback
  server.on("/header", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Ok");
    // Variable: Test-Header, Value: My header value
    response->addHeader("Test-Header", "My header value");
    request->send(response);
  });

  server.begin();
}

void loop(){}
