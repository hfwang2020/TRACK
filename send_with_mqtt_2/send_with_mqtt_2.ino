
#include <Vector.h>
#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90641_API.h"
#include "MLX9064X_I2C_Driver.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <sstream>
#include <string>
using namespace std;


#ifndef STASSID
#define STASSID "TP-LINK_E925"
#define STAPSK  "12345678"
#endif

#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 1024
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;
const char* mqtt_server = "192.168.1.120";



#define debug  Serial

const byte MLX90641_address = 0x33; //Default 7-bit unshifted address of the MLX90641
#define TA_SHIFT 8 //Default shift for MLX90641 in open air

uint16_t eeMLX90641[832];
float MLX90641To[192];
uint16_t MLX90641Frame[242];
paramsMLX90641 MLX90641;
int errorno = 0;

WiFiClient espClient;
PubSubClient client(espClient);
const char* msg;



void setup_wifi() {

    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

//mqtt 回调函数
void callback(char* topic, byte* payload ,unsigned int length){
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]");
    for (int i = 0; i < length;i++){
        Serial.print((char)payload[i]);
    }
    Serial.println();

    if ((char)payload[0] == '1'){
        digitalWrite(BUILTIN_LED,LOW);
    }else{
        digitalWrite(BUILTIN_LED,HIGH);
    }
}

//mqtt 连接函数
void reconnect(){
    while (!client.connected()){
        Serial.print("Attempting MQTT connection...");
        String clientID = "ESP8266Client-hfwang";
        //connect
        if(client.connect(clientID.c_str())){
            Serial.println("connected successfully");
        }else{
            Serial.print("failed,rc=");
            Serial.print(client.state());
            Serial.println("try again in 5 seconds");
            delay(5000);
        }
    }
}


void setup() {

    setup_wifi();
    client.setServer(mqtt_server,1883);
    client.setCallback(callback);
  
    Wire.begin();
    Wire.setClock(800000); //Increase I2C clock speed to 400kHz

    debug.begin(115200); //Fast debug as possible

    while (!debug); //Wait for user to open terminal
    //debug.println("MLX90640 IR Array Example");

    if (isConnected() == false) {
        debug.println("MLX90641 not detected at default I2C address. Please check wiring. Freezing.");
        while (1);
    }
    //Get device parameters - We only have to do this once
    int status;
    status = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
    errorno = status;//MLX90641_CheckEEPROMValid(eeMLX90641);//eeMLX90641[10] & 0x0040;//
    
    if (status != 0) {
        debug.println("Failed to load system parameters");
       while(1);
    }
    
    status = MLX90641_ExtractParameters(eeMLX90641, &MLX90641);
    //errorno = status;
    if (status != 0) {
        debug.println("Parameter extraction failed");
        while(1);
    }

    //Once params are extracted, we can release eeMLX90641 array

    //MLX90641_SetRefreshRate(MLX90641_address, 0x02); //Set rate to 2Hz
    MLX90641_SetRefreshRate(MLX90641_address, 0x05); //Set rate to 4Hz
    //MLX90641_SetRefreshRate(MLX90641_address, 0x07); //Set rate to 64Hz    
    delay(200);

}

void loop() {
    getPiexls();    
    String str = "0,";
    for (int x = 1 ; x < 193 ; x++) {
        str.concat(MLX90641To[x-1]);
        str += ",";
    }
    str += "255";
    msg = str.c_str();
    if(!client.connected()){
        reconnect();
    }
    mqttSendlongString(msg);
   
}

void getPiexls(){
    long startTime = millis();
    int status = MLX90641_GetFrameData(MLX90641_address, MLX90641Frame);
    float vdd = MLX90641_GetVdd(MLX90641Frame, &MLX90641);
    float Ta = MLX90641_GetTa(MLX90641Frame, &MLX90641);
    long time3 = millis();
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;
    MLX90641_CalculateTo(MLX90641Frame, &MLX90641, emissivity, tr, MLX90641To);
    long stopTime = millis();
    debug.print(1000/(stopTime-startTime));
}

void mqttSendlongString(String json_str){
    //拆分字符串发送
    int cut = 128;  //要拆分发送的实际大小
    int json_str_len = json_str.length();  //总数据长度

    if (json_str_len > cut)
    {
        //开始发送长文件参数分别为  主题，长度，是否持续
        client.beginPublish("test", json_str_len, true);
        int count = json_str_len / cut;
        for (int i = 0; i < (count-1); i++)
        {
            client.print(json_str.substring(i * cut, (i * cut + cut)));
        }
        client.print(json_str.substring(cut * (count - 1)));
        //结束发送文本
        client.endPublish();
    }
    else
    {
        client.publish("test", json_str.c_str());
    }
    client.loop();
}


//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected() {
    Wire.beginTransmission((uint8_t)MLX90641_address);
    if (Wire.endTransmission() != 0) {
        return (false);    //Sensor did not ACK
    }
    return (true);
}

void array(){
    
}
