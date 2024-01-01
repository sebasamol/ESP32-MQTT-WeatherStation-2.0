#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <DFRobot_ENS160.h>
#include <ArduinoJson.h>

#define SDApin_1 21
#define SCLpin_1 22
#define SDApin_2 33
#define SCLpin_2 32



const char* ssid = "INEA-0082_2.4G";
const char* pwd = "4YC3Hbce";

const char* mqtt_broker = "192.168.1.13";
const char* mqtt_user = "bastulon";
const char* mqtt_pwd = "pt7T4RoqdPnzri7";

const char* topic = "testTopic";
const char* dataPoznanInside = "dataPoznanInside";
const char* dataPoznanOutside = "dataPoznanOutside";
const char* service = "service";


const int mqtt_port = 1883;

unsigned long previousMillisWiFi = 0;
unsigned long intervalWiFi = 5000;

unsigned long previousMillisMQTT = 0;
unsigned long intervalMQTT = 5000;

unsigned long currentMillis_outside = 0;
unsigned long previousMillis_outside = 0;
unsigned long interval_outside = 3000;

unsigned long currentMillis_inside = 0;
unsigned long previousMillis_inside = 0;
unsigned long interval_inside = 3000;

unsigned long currentMillis_service = 0;
unsigned long previousMillis_service = 0;
unsigned long interval_service = 3000;



IPAddress local_IP(192,168,1,15);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,1,1);

WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_BME280 bme_outside;
Adafruit_BME280 bme_inside;

DFRobot_ENS160_I2C ENS160(&Wire, 0x53);

bool statusBME_outside;
bool statusBME_inside;


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

void connectMQTT() {

    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());

        if (client.connect(client_id.c_str(), mqtt_user, mqtt_pwd)) {
            Serial.println("MQTT broker connected!");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void dataInside() {
    DynamicJsonDocument doc(1024);
    char buffer [200];
    currentMillis_inside = millis();
    
    JsonArray tempBME = doc.createNestedArray("temp");
    JsonArray humBME = doc.createNestedArray("hum");


    if(currentMillis_inside - previousMillis_inside >= interval_inside){
        previousMillis_inside = currentMillis_inside;
    
        tempBME.add(bme_inside.readTemperature());
        humBME.add(bme_inside.readHumidity());
    
        serializeJsonPretty(doc,buffer);
        client.publish(dataPoznanInside,buffer);
        doc.clear();
        memset(buffer,0,sizeof(buffer));   
    }
}

void dataOutside() {
    DynamicJsonDocument doc(1024);
    char buffer [300];
    currentMillis_outside = millis();

    JsonArray tempBME = doc.createNestedArray("temp");
    JsonArray humBME = doc.createNestedArray("hum");
    JsonArray pressBME = doc.createNestedArray("press");
    JsonArray aqi = doc.createNestedArray("aqi");
    JsonArray eco2 = doc.createNestedArray("eco2");
    JsonArray tvoc = doc.createNestedArray("tvoc");


    if(currentMillis_outside - previousMillis_outside >= interval_outside){
        previousMillis_outside = currentMillis_outside;
        

        tempBME.add(bme_outside.readTemperature());
        humBME.add(bme_outside.readHumidity());
        pressBME.add(bme_outside.readPressure() / 100.0F);

        aqi.add(ENS160.getAQI());
        eco2.add(ENS160.getTVOC());
        tvoc.add(ENS160.getECO2());

        serializeJsonPretty(doc,buffer);
        client.publish(dataPoznanOutside,buffer);
        doc.clear();
        memset(buffer,0,sizeof(buffer));  

    }
}

void serviceFunction() {

    currentMillis_service = millis();

    if(currentMillis_service - previousMillis_outside >= interval_service) { 
        previousMillis_service = currentMillis_service;
        while (!statusBME_outside) {
            client.publish(service,"Lost connection with outside BME280");
            delay(5000);
        } 
        while (!statusBME_inside) {
            client.publish(service,"Lost connection with inside BME280");
            delay(5000);
        } 
        while ( NO_ERR != ENS160.begin() ){
            Serial.println("Communication with ENS160 failed, please check connection");
            delay(5000);
        }
    
    }
}

void setup() {

    Serial.begin(9600);

    Wire.begin(SDApin_1,SCLpin_1);
    Wire1.begin(SDApin_2,SCLpin_2);

    statusBME_outside = bme_outside.begin(0x76);
    statusBME_inside = bme_inside.begin(0x76, &Wire1); 

    ENS160.setPWRMode(ENS160_STANDARD_MODE);
    ENS160.setTempAndHum(25.0, 50.0);
    delay(1000);

    client.setServer(mqtt_broker, mqtt_port);
    initWiFi();
    connectMQTT();
}

void loop() { 

    unsigned long currentMillisWiFi = millis();
    unsigned long currentMillisMQTT = millis();

    if ((WiFi.status() != WL_CONNECTED) && (currentMillisWiFi - previousMillisWiFi >= intervalWiFi)) {
        
        client.publish(service,"Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        previousMillisWiFi = currentMillisWiFi;
    }
    if (WiFi.status() == WL_CONNECTED && !client.connected() && (currentMillisMQTT - previousMillisMQTT >= intervalMQTT)) {
        client.publish(service,"Reconnection to MQTT Broker");
        connectMQTT();
        previousMillisMQTT = currentMillisMQTT;

    } else if (WiFi.status() == WL_CONNECTED && client.connected()) { 
        dataInside();
        dataOutside();
        serviceFunction();
    }
    client.loop();
    
} 
    
    
  



