#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = " faraj_xA";
const char* password = "sawqst231";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  ArduinoOTA.setHostname("esp32");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("Start updating: " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nUpdate complete");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error [%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();
  Serial.println("OTA ready");


  Serial2.begin(9600); 
}

void loop() {
  ArduinoOTA.handle();

  
  if (Serial2.available()) {
    String dataFromArduino = Serial2.readString();
    Serial.print("Received from Arduino: ");
    Serial.println(dataFromArduino);
  }


  if (Serial.available()) {
    String dataToArduino = Serial.readString();
    Serial2.print(dataToArduino);
    Serial.print("Sent to Arduino: ");
    Serial.println(dataToArduino);
  }


}
