#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <SPI.h>

#define SDApin 21
#define SCLpin 22

const char* ssid = "INEA-0082_2.4G";
const char* pwd = "4YC3Hbce";

// int tempBME = 0;
// int humBME = 0;
// int pressBME = 0;



IPAddress local_IP(192,168,1,10);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,1,1);

Adafruit_BME280 bme;
bool statusBME;

void setup() {

    Serial.begin(9600);
    Wire.begin(SDApin,SCLpin);
    statusBME = bme.begin(0x76);
    delay(1000);

    WiFi.mode(WIFI_STA); 
    WiFi.begin(ssid, pwd);
    Serial.println("\nConnecting");

    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("STA Failed to configure");
    }

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());

    if (!statusBME) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        delay(5000);
    }else{
        Serial.println("BME280 connected");
    }
}

void measureBME(){
    
    
    

    Serial.print("Temperatura = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Wilgotność = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.print("Ciśnienie = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");


   
    delay(6000);

}



void loop() { 
    measureBME();
  
}


