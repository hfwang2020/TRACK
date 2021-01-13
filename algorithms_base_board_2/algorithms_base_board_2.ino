#include <Wire.h>
#include <Vector.h>
#include <stdlib.h>
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
const char* mqtt_server = "42.192.171.165";


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

class Frame {
public:
    float piexls[192];
    float piexls_mean;
    float col_mean[16];
    float col_var[16];
    float col_diff[16];
    float col_final[16];
    vector<float> index;
    float index_mean;
    //构造函数
    Frame(float a[192]) {
        for(int i=0;i<=191;i++){
          piexls[i]=a[i];
        }
        piexls_mean = cal0();
        cal_mean();
        cal_var();
        cal_diff();
        cal_final();
        indexCal();
        indexmean();
    }

    // 计算帧平均温度
    float cal0() {
        float sum = 0;
        for (int i = 0; i <= 191; i++) {
            sum += piexls[i];
        }
        return sum / 192;
    }

    // 计算列平均温度
    void cal_mean() {
        for (int i = 0; i <= 15; i++) {
            float sum = 0;
            for (int j = 0; j <= 11; j++) {
                sum += piexls[16 * j + i];
            }
            col_mean[i] = sum / 12;
        }
    }

    // 计算列方差
    void cal_var() {
        for (int i = 0; i <= 15; i++) {
            float sum = 0;
            for (int j = 0; j <= 11; j++) {
                sum += pow(col_mean[i] - piexls[16 * j + i], 2);
            }
            col_var[i] = sum / 12;
        }
    }

    // 计算列与帧的温差
    void cal_diff() {
        for (int i = 0; i <= 15; i++) {
            col_diff[i] = col_mean[i] - piexls_mean;
        }
    }

    // 最终用来判断有无人的col
    void cal_final() {
        for (int i = 0; i <= 15; i++) {
            col_final[i] = 4 * col_diff[i] + 10 * col_var[i];
        }
    }

    void indexCal() {
        int count = 0;
        for (int i = 0; i <= 15; i++) {
            if (col_final[i] >= 20) {
                count += 1;
            } else {
                col_final[i] = -1;
            }
        }
        if (count <= 1) {
            index.push_back(-1);
            return;
        }
        for (int i = 0; i <= 15; i++) {
            if (col_final[i] < 0) {
                continue;
            }
            int track_point = 0;
            double sum_i = 0;
            double sum_col_i = 0;
            while ((i <= 15) && (col_final[i] > 0)) {
                track_point += 1;
                sum_i += col_final[i];
                sum_col_i += i * col_final[i];
                i++;
            }
            double a = sum_col_i / sum_i;
            if ((track_point > 1) && (a > 2) && (a < 13)) {
                index.push_back(a);
            }
        }
        if (index.size() == 0) {
            index.push_back(-1);
        }
    }

    void indexmean() {
        float sum = 0;
        for (int i = 0; i <= index.size() - 1; i++) {
            sum += index[i];
        }
        index_mean = sum/index.size();
    }
};
 


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


class Track {
public:
    int flag = 0;
    vector<vector<float>> pointList = {{0}};
    int num = 5;
    int time = 0;

    void judge() {
        int flag = 0;
        vector<vector<float>> track_list;
        if (pointList[1][0] > 7) {
            flag = -1;
        }
        if (pointList[1][0] < 7) {
            flag = 1;
        }
        if (flag == -1) {
            for (int i = 1; i <= pointList.size() - 1; i++) {
                for (int j = 0; j <= pointList[i].size() - 1; j++) {
                    if (track_list.size() == 0) {
                        track_list.push_back({pointList[i][j]});
                    } else if (track_list.size() == 1) {
                        if (pointList[i][j] > track_list[0][track_list.size() - 1]) {
                            track_list.push_back({pointList[i][j]});
                        } else {
                            track_list[0].push_back(pointList[i][j]);
                        }
                    } else {
                        float a = track_list[track_list.size() - 2][track_list[track_list.size() - 2].size() - 1];
                        float b = track_list[track_list.size() - 1][track_list[track_list.size() - 1].size() - 1];
                        if (pointList[i][j] < (a + 0.5)) {
                            track_list[track_list.size() - 2].push_back(pointList[i][j]);
                        } else if (pointList[i][j] < (b + 0.5)) {
                            track_list[track_list.size() - 1].push_back(pointList[i][j]);
                        } else {
                            track_list.push_back({pointList[i][j]});
                        }

                    }
                }
            }
            int count = 0;
            for (int i = 0; i <= track_list.size() - 1; i++) {
                if (track_list[i][0] > 8 && track_list[i][track_list[i].size() - 1] < 8 && track_list[i].size() >= 3) {
                    count += 1;
                    //debug
//                    String msg = "track:[";
//                    for(float point:track_list[i]){
//                        msg+=String(point);
//                        msg+=" ";
//                    }
//                    msg+="]";
//                    if(!client.connected()){
//                        reconnect();
//                    }
//                    client.publish("track", msg.c_str());
                    //debug
                }
            }
//            if (count>5){
//              count = 2;
//            }
            num = num + count;
            //debug
            //client.publish("count",String(count).c_str());
            //debug
        }
        if (flag == 1) {
            for (int i = 1; i <= pointList.size() - 1; i++) {
                for (int j = 0; j <= pointList[i].size() - 1; j++) {
                    if (track_list.size() == 0) {
                        track_list.push_back({pointList[i][j]});
                    } else if (track_list.size() == 1) {
                        if (pointList[i][j] > track_list[0][track_list.size() - 1]) {
                            
                            track_list.push_back({pointList[i][j]});
                        } else {
                            track_list[0].push_back(pointList[i][j]);
                        }
                    } else {
                        float a = track_list[track_list.size() - 2][track_list[track_list.size() - 2].size() - 1];
                        float b = track_list[track_list.size() - 1][track_list[track_list.size() - 1].size() - 1];
                        if (pointList[i][j] > (a - 0.5)) {
                            track_list[track_list.size() - 2].push_back(pointList[i][j]);
                        } else if (pointList[i][j] > (b - 0.5)) {
                            track_list[track_list.size() - 1].push_back(pointList[i][j]);
                        } else {
                            track_list.push_back({pointList[i][j]});
                        }

                    }
                }
            }
            int count = 0;
            for (int i = 0; i <= track_list.size() - 1; i++) {
                if (track_list[i][0] < 8 && track_list[i][track_list[i].size() - 1] > 7 && track_list[i].size() >= 3) {
                    count += 1;
                    //debug
//                    String msg = "track:[";
//                    for(float point:track_list[i]){
//                        msg+=String(point);
//                        msg+=" ";
//                    }
//                    msg+="]";
//                    if(!client.connected()){
//                        reconnect();
//                    }
//                    client.publish("track", msg.c_str());
                    //debug
                }
            }
//            if (count > 5){
//              count = 2;
//            }
            num = num - count;
            //debug
            //client.publish("count",String(count).c_str()); 
            //debug
        }
    }

};


void setup() {
    setup_wifi();
    client.setServer(mqtt_server,1883);
    client.setCallback(callback);
    Wire.begin();
    Wire.setClock(800000); //Increase I2C clock speed to 400kHz
    Serial.begin(115200); //Fast debug as possible
    while (!Serial);
    if (isConnected() == false) {
        Serial.println("MLX90641 not detected at default I2C address. Please check wiring. Freezing.");
        while (1);
    }
    //Get device parameters - We only have to do this once
    int status;
    status = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
    errorno = status;//MLX90641_CheckEEPROMValid(eeMLX90641);//eeMLX90641[10] & 0x0040;//
    
    if (status != 0) {
        Serial.println("Failed to load system parameters");
       while(1);
    }
    
    status = MLX90641_ExtractParameters(eeMLX90641, &MLX90641);
    //errorno = status;
    if (status != 0) {
        Serial.println("Parameter extraction failed");
        while(1);
    }

    //Once params are extracted, we can release eeMLX90641 array

    //MLX90641_SetRefreshRate(MLX90641_address, 0x02); //Set rate to 2Hz
    MLX90641_SetRefreshRate(MLX90641_address, 0x05); //Set rate to 4Hz
    //MLX90641_SetRefreshRate(MLX90641_address, 0x07); //Set rate to 64Hz    
    delay(200);

}


Track track;

float piexls_past[192];
float piexls_diff[192];
void loop() {
    long startTime = millis();
    getPiexls();
    
    
//    String str = "0,";
//    for (int x = 1 ; x < 193 ; x++) {
//        str.concat(MLX90641To[x-1]);
//        str += ",";
//    }
//    str += "255";
//    msg = str.c_str();
//    if(!client.connected()){
//        reconnect();
//    }
//    mqttSendlongString(msg);

    Serial.println("----------running----------");
    
    Frame frame(MLX90641To);
    if (frame.index_mean > 0) {
        track.time = 0;
        if (frame.index != track.pointList.back()) {
            track.pointList.push_back(frame.index);
            //debug
//            String msg = "index:[";
//            for(float point:frame.index){
//                msg+=String(point);
//                msg+=" ";
//            }
//            msg+="]";
//            if(!client.connected()){
//                reconnect();
//            }
//            client.publish("debug", msg.c_str());
            //debug 
               
        } 
    }else{
      track.time += 1;
    }

    if (track.time >= 20 && track.pointList.size() >= 3) {
        track.judge();
        Serial.println("-----judge running-----");
        vector<vector<float>> v = {{0}};
        track.pointList = v;
        track.time = 0;
//        String a = String(track.num);
//        String b = "#";
//        if(!client.connected()){
//            reconnect();
//        }
//        client.publish("count", (a+b).c_str());
    }
    
    if (track.time >= 100 && track.pointList.size() <= 2) {
        vector<vector<float>> v = {{0}};
        track.pointList = v;
//        if(!client.connected()){
//            reconnect();
//        }
//        client.publish("num", "----------clear----------");
        track.time = 0;
    }
    long stopTime = millis();
    
    if(!client.connected()){
        reconnect();
    }
    client.publish("hz",String(1000/(stopTime-startTime)).c_str());
    String a = String(track.num);
//    String b = "#";
    client.publish("count",a.c_str());
    
    for(int i=0;i<=191;i++){
        piexls_past[i]=MLX90641To[i];
    }

}


void getPiexls(){
    
    int status = MLX90641_GetFrameData(MLX90641_address, MLX90641Frame);
    float vdd = MLX90641_GetVdd(MLX90641Frame, &MLX90641);
    float Ta = MLX90641_GetTa(MLX90641Frame, &MLX90641);
    long time3 = millis();
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;
    MLX90641_CalculateTo(MLX90641Frame, &MLX90641, emissivity, tr, MLX90641To);
    
    
//    Serial.print("帧率：");
//    Serial.println(1000/(stopTime-startTime));
//    Serial.println("");
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
        return (false);    
    }
    return (true);
}
