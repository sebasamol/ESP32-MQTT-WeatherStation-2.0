#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <SPI.h>
#include <PubSubClient.h>

#define SDApin 21
#define SCLpin 22

const char* ssid = "INEA-0082_2.4G";
const char* pwd = "4YC3Hbce";

const char* mqtt_broker = "192.168.1.13";
const char* mqtt_user = "bastulon";
const char* mqtt_pwd = "pt7T4RoqdPnzri7";
const char* topic = "testTopic";
const char* tempTopic = "tempTopicPoznan";
const char* humTopic = "humTopicPoznan";
const char* pressTopic = "pressTopicPoznan";
const int mqtt_port = 1883;

unsigned long previousMillis = 0;
unsigned long interval = 10000;

// int tempBME = 0;
// int humBME = 0;
// int pressBME = 0;


IPAddress local_IP(192,168,1,15);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,1,1);

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;
bool statusBME;


void initWiFi() {
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
}

void initMQTT() {

    client.setServer(mqtt_broker, mqtt_port);
        while (!client.connected()) {
            String client_id = "esp32-client-";
            client_id += String(WiFi.macAddress());
            Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
            if (client.connect(client_id.c_str(), mqtt_user, mqtt_pwd)) {
                Serial.println("Public EMQX MQTT broker connected");
            } else {
                Serial.print("failed with state ");
                Serial.print(client.state());
                delay(2000);
            }
    }
}

void setup() {

    Serial.begin(9600);
    Wire.begin(SDApin,SCLpin);
    statusBME = bme.begin(0x76);
    delay(1000);

    initWiFi();
    initMQTT();

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


   
    delay(10000);

}



void loop() { 
    unsigned long currentMillis = millis();
    //WiFi Reconnecting
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
        Serial.print(millis());
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        previousMillis = currentMillis;
    }

    measureBME();
    client.publish(topic, "Hi from ESP");
    delay(5000);
    client.loop();
  
}


