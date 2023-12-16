#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <DFRobot_ENS160.h>

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
const char* tempOutTopic = "tempOutPoznan";
const char* humOutTopic = "humOutPoznan";
const char* pressOutTopic = "pressOutPoznan";
const int mqtt_port = 1883;

unsigned long previousMillisWiFi = 0;
unsigned long intervalWiFi = 5000;

unsigned long previousMillisMQTT = 0;
unsigned long intervalMQTT = 5000;

unsigned long currentMillisBME = 0;
unsigned long previousMillisBME = 0;
unsigned long intervalBME = 3000;

float tempBME = 0;
float humBME = 0;
float pressBME = 0;


IPAddress local_IP(192,168,1,15);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,1,1);

WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_BME280 bme1;
Adafruit_BME280 bme2;

DFRobot_ENS160_I2C ENS160(&Wire, 0x53);

bool statusBME1;
bool statusBME2;


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
void measureBME(){
    

    currentMillisBME = millis();

    if(currentMillisBME - previousMillisBME >= intervalBME){
        previousMillisBME = currentMillisBME;
            
        tempBME = bme2.readTemperature();
        humBME = bme2.readHumidity();
        pressBME = bme2.readPressure() / 100.0F;

        char tempString[8];
        char humString[8];
        char pressString[9];


        dtostrf(tempBME, 1, 2, tempString);
        dtostrf(humBME, 1, 2, humString);
        dtostrf(pressBME, 1, 2, pressString);
                    

        client.publish(tempOutTopic, tempString);
        client.publish(humOutTopic, humString);
        client.publish(pressOutTopic, pressString);
    
    }
    
}
void measureENS160(){
    
    uint8_t Status = ENS160.getENS160Status();
    
    Serial.print("Sensor operating status : ");
    Serial.println(Status);

    /**
     * Get the air quality index
     * Return value: 1-Excellent, 2-Good, 3-Moderate, 4-Poor, 5-Unhealthy
     */
    uint8_t AQI = ENS160.getAQI();
    Serial.print("Air quality index : ");
    Serial.println(AQI);

    /**
     * Get TVOC concentration
     * Return value range: 0–65000, unit: ppb
     */
    uint16_t TVOC = ENS160.getTVOC();
    Serial.print("Concentration of total volatile organic compounds : ");
    Serial.print(TVOC);
    Serial.println(" ppb");

    /**
     * Get CO2 equivalent concentration calculated according to the detected data of VOCs and hydrogen (eCO2 – Equivalent CO2)
     * Return value range: 400–65000, unit: ppm
     * Five levels: Excellent(400 - 600), Good(600 - 800), Moderate(800 - 1000), 
     *               Poor(1000 - 1500), Unhealthy(> 1500)
     */
    uint16_t ECO2 = ENS160.getECO2();
    Serial.print("Carbon dioxide equivalent concentration : ");
    Serial.print(ECO2);
    Serial.println(" ppm");

    Serial.println();
    
    delay(5000);
}

void setup() {

    Serial.begin(9600);

    Wire.begin(SDApin_1,SCLpin_1);
    Wire1.begin(SDApin_2,SCLpin_2);

    statusBME1 = bme1.begin(0x76);
    statusBME2 = bme2.begin(0x76, &Wire1); 

    ENS160.setPWRMode(ENS160_STANDARD_MODE);
    ENS160.setTempAndHum(25.0, 50.0);


    delay(1000);

    if (!statusBME1) {
        Serial.println("Could not find a first BME280 sensor, check wiring!");
        delay(5000);
    }else{
        Serial.println("First BME280 connected");
        delay(1000);
    }
    if (!statusBME2) {
        Serial.println("Could not find a second BME280 sensor, check wiring!");
        delay(5000);
    }else{
        Serial.println("Second BME280 connected");
        delay(1000);
    }

     while( NO_ERR != ENS160.begin() ){
        Serial.println("Communication with ENS160 failed, please check connection");
        delay(3000);
    }
    Serial.println("Begin ENS160 ok!");

    client.setServer(mqtt_broker, mqtt_port);
    initWiFi();
    connectMQTT();
}





void loop() { 

    unsigned long currentMillisWiFi = millis();
    unsigned long currentMillisMQTT = millis();
    //WiFi Reconnecting
    if ((WiFi.status() != WL_CONNECTED) && (currentMillisWiFi - previousMillisWiFi >= intervalWiFi)) {
        
        //Serial.print(millis());
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        previousMillisWiFi = currentMillisWiFi;
    }
    if (WiFi.status() == WL_CONNECTED && !client.connected() && (currentMillisMQTT - previousMillisMQTT >= intervalMQTT)) {
        Serial.println("Reconnection to MQTT Broker");
        connectMQTT();
        previousMillisMQTT = currentMillisMQTT;

    } else if (WiFi.status() == WL_CONNECTED && client.connected()) { 
        measureBME();
        measureENS160();

    }
    client.loop();
    
} 
    
    
  



