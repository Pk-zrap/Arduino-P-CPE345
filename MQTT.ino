#include <WiFi.h>
#include <PubSubClient.h>
#include "Adafruit_SHT31.h"


const char* ssid     = "Pk";
const char* password = "Pk343276";

#define mqttServer    "broker.emqx.io"
#define mqttPort      1883
#define mqttUser      ""
#define mqttPassword  ""

#define LED_R 27
#define RelayPin01 32
#define RelayPin02 14

float T;
float H;

String msg;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
unsigned long myTime = 0;

WiFiClient wificlient;
PubSubClient mqttClient(wificlient);

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  pinMode(LED_R,OUTPUT);
  digitalWrite(LED_R,HIGH);

  //Relay
  pinMode(32,OUTPUT);
  pinMode(14,OUTPUT);

  digitalWrite(RelayPin01,HIGH);
  digitalWrite(RelayPin02,HIGH);

  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //ตั้งค่าเซนเซอร์ SHT31 โดยตรวจสอบการเชื่อมต่อ
  Serial.println("SHT31 test");
  if (!sht31.begin(0x44)) {  // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

    Serial.print("Heater Enabled State: ");
  if (sht31.isHeaterEnabled())
    Serial.println("ENABLED");
  else
    Serial.println("DISABLED");


  setupMQTT();
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();  
  }
  
  /*//
  if(millis()- myTime > 10000 )
  {//10 s =10000. 10000-0 >10000 =
    mqttClient.publish("CPE345IoT/65057316/msg/Time",String(millis()/1000).c_str());
    //readSHT31();
      myTime = millis();
  }
  */
  //

  if(millis()- myTime > 10000 )
  {
   // mqttClient.publish("CPE345IoT/65057316/msg/Time",String(millis()/1000).c_str());
    readSHT31();
    myTime = millis();
  }

  mqttClient.loop();
}

void readSHT31(){
    T = sht31.readTemperature();
    H = sht31.readHumidity();

    Serial.print("Temperature: ");
    Serial.print(T);
    Serial.print("°C, Humidity: ");
    Serial.print(H);
    Serial.println("%");

    String DATA = "Temperature: "+ String(T)+"Humidity: "+ String(H);
    mqttClient.publish("CPE345IoT/65057316/msg/Time",DATA.c_str());

}

void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  // set the callback function
  mqttClient.setCallback(callback);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
      Serial.println("Reconnecting to MQTT Broker..");
      String clientId = "ESP32Client-";
      clientId += String(random(0xffff), HEX);
      
      if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
        Serial.println("Connected.");
        Serial.println(clientId);
        // subscribe to topic
        mqttClient.subscribe("CPE345IoT/65057316/msg/#");
      }
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  String sTopic = String(topic);
  Serial.print("Message arrived ["); Serial.print(topic); Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg += (char)payload[i];
  }

  if(sTopic == "CPE345IoT/65057316/msg/LED"){
    if(msg == "ON"){
        digitalWrite(LED_R,LOW);
    }
    else{
        digitalWrite(LED_R,HIGH);
    }
  }
  

  if(sTopic == "CPE345IoT/65057316/msg/Relay_01"){
    if(msg == "ON"){
        digitalWrite(RelayPin01,LOW);
    }
    else{
        digitalWrite(RelayPin01,HIGH);
    }
      Serial.println("Relay02: "+ msg);

  }

  if(sTopic == "CPE345IoT/65057316/msg/Relay_02"){
    if(msg == "ON"){
        digitalWrite(RelayPin02,LOW);
    }
    else{
        digitalWrite(RelayPin02,HIGH);
    }
      Serial.println("Relay02: "+ msg);

  }
  
  msg = "";
  Serial.println("\n======================================");
}