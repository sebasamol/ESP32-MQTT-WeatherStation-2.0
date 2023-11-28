#include <Arduino.h>
#include <WiFi.h>


const char* ssid = "INEA-0082_2.4G";
const char* pwd = "4YC3Hbce";


// put function declarations here:
int myFunction(int, int);

void setup() {
    Serial.begin(9600);
    delay(1000);

    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, pwd);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
