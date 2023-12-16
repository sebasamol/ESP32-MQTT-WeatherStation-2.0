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
const char* tempInTopic = "tempInPoznan";
const char* humInTopic = "humInTopic";
const char* aqiTopic = "aqiPoznan";
const char* tvocTopic = "tvocPoznan";
const char* eco2Topic = "eco2Poznan";




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

float tempBME_outside = 0;
float humBME_outside = 0;
float pressBME_outside = 0;
float tempBME_inside = 0;
float humBME_inside = 0; 


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
void measurementsOutside(){
    

    currentMillis_outside = millis();

    if(currentMillis_outside - previousMillis_outside >= interval_outside){
        previousMillis_outside = currentMillis_outside;
        
        uint8_t AQI = ENS160.getAQI();
        uint16_t TVOC = ENS160.getTVOC();
        uint16_t ECO2 = ENS160.getECO2();

        tempBME_outside = bme_outside.readTemperature();
        humBME_outside = bme_outside.readHumidity();
        pressBME_outside = bme_outside.readPressure() / 100.0F;

        char tempString[8];
        char humString[8];
        char pressString[9];
        char aqiString[8];
        char tvocString[8];
        char eco2String[8];


        dtostrf(tempBME_outside, 1, 2, tempString);
        dtostrf(humBME_outside, 1, 2, humString);
        dtostrf(pressBME_outside, 1, 2, pressString);
        dtostrf(AQI, 1, 2, aqiString);
        dtostrf(TVOC, 1, 2, tvocString);
        dtostrf(ECO2, 1, 2, eco2String);
                    

        client.publish(tempOutTopic, tempString);
        client.publish(humOutTopic, humString);
        client.publish(pressOutTopic, pressString);
        client.publish(aqiTopic, aqiString);
        client.publish(tvocTopic, tvocString);
        client.publish(eco2Topic, eco2String);
    
    }
    
}

void measurementsInside(){

    currentMillis_inside = millis();

    if(currentMillis_inside - previousMillis_inside >= interval_inside){
        previousMillis_inside = currentMillis_inside;
            
        tempBME_inside = bme_inside.readTemperature();
        humBME_inside = bme_inside.readHumidity();
        
        char tempString[8];
        char humString[8];
        
        dtostrf(tempBME_inside, 1, 2, tempString);
        dtostrf(humBME_inside, 1, 2, humString);
            

        client.publish(tempInTopic, tempString);
        client.publish(humInTopic, humString);
        
    
    }

}

void measureENS160(){
    /**
   * Get the sensor operating status
   * Return value: 0-Normal operation, 
   *         1-Warm-Up phase, first 3 minutes after power-on.
   *         2-Initial Start-Up phase, first full hour of operation after initial power-on. Only once in the sensor’s lifetime.
   * note: Note that the status will only be stored in the non-volatile memory after an initial 24h of continuous
   *       operation. If unpowered before conclusion of said period, the ENS160 will resume "Initial Start-up" mode
   *       after re-powering.
   */
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

    statusBME_outside = bme_outside.begin(0x76);
    statusBME_inside = bme_inside.begin(0x76, &Wire1); 

    ENS160.setPWRMode(ENS160_STANDARD_MODE);
    ENS160.setTempAndHum(25.0, 50.0);


    delay(1000);

    if (!statusBME_outside) {
        Serial.println("Could not find a first BME280 sensor, check wiring!");
        delay(5000);
    }else{
        Serial.println("First BME280 connected");
        delay(1000);
    }
    if (!statusBME_inside) {
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
        measurementsOutside();
        measurementsInside();

    }
    client.loop();
    
} 
    
    
  



