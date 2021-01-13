#include <Wire.h>
#include <Vector.h>
#include <stdlib.h>
#include "MLX90640_API.h"
#include "MLX90641_API.h"
#include "MLX9064X_I2C_Driver.h"
#include <math.h>

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

float threshold = 1.25;

//对右下角坏点进行中值处理
float* filter(float arr[]) {
  float al[4] = { arr[174], arr[175], arr[190], arr[191] };
  sort(al, al + 4);
  arr[191] = (al[1] + al[2]) / 2;

  return arr;
}

float maxALL(float arr[]) {
  float max = arr[0];
  for (int i = 0; i < sizeof(arr); i++) {
    if (arr[i] > max)
      max = arr[i];
  }

  return max;
}

float* maxCOL(float arr[]) {
  float* ans = new float[16];

  for (int i = 0; i < 16; i++) {
    float max = arr[i];
    for (int j = 0; j < 12; j++) {
      if (arr[16 * j + i] > max)
        max = arr[16 * j + i];
    }
    ans[i] = max;
  }

  return ans;

}

float* meanCOL(float arr[]) {
  float* ans = new float[16];

  for (int i = 0; i < 16; i++) {
    float sum = 0;
    for (int j = 0; j < 12; j++) {
      sum += arr[16 * j + i];
    }
    ans[i] = sum / 12;
  }

  return ans;
}

float* varCOL(float arr[]) {
  float* mean = meanCOL(arr);
  float* varCOL = new float[16];

  for (int i = 0; i <= 15; i++) {
    float sum = 0;
    for (int j = 0; j <= 11; j++) {
      sum += pow(mean[i] - arr[16 * j + i], 2);
    }
    varCOL[i] = sum / 12;
  }

  return varCOL;

}

bool isPassing(float col[]) {
  if (maxALL(col) < threshold)
    return false;

  int i = 0, j = 0;
  while (j < sizeof(col)) {
    if (col[j] >= threshold)
      j++;
    else {
      if (i == j)
        i++, j++;
      else {
        if (j - i >= 2)
          return true;
        i = j;
      }
    }
  }
  if (col[j - 1] >= threshold && j - i >= 2)
    return true;

  return false;
}

vector<float> judgePoints(float col[]) {
  int i = 0, j = 0;
  vector<float> points;
  while (j < sizeof(col)) {
    if (col[j] >= threshold)
      j++;
    else {
      if (i == j)
        i++, j++;
      else {
        if (j - i >= 2)
          points.push_back((i + j - 1) / 2);
        i = j;
      }
    }
  }
  if (col[j - 1] >= threshold && j - i >= 2)
    points.push_back((i + j) / 2);

  return points;
}


//前一帧传感器数据
float* pre = filter(MLX90641To);
//当前帧传感器数据
float* cur;
//当前传感器数据差分
float* diff;
//当前差分列最大值
float* max_col;
//当前差分列方差
float* var_col;
//当前差分最终列值
float* col;
//在一次有人循环中的所有可能轨迹
vector<vector<float>> tracks;
//每条轨迹的检测状态，当检测状态值大于3时，表明该轨迹已结束跟踪
vector<int> tracks_status;
//记录每条轨迹的方向，flag=1表示进入，flag=-1表示离开
vector<int> flags;
//每一帧中的检测点位置
vector<float> points;
//检测点跟踪状态，0表示对应该点未被跟踪，1表示已被跟踪
vector<int> visited;


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


void loop() {
    getPiexls();
    cur = filter(MLX90641To);

  //差分计算
  for (int i = 0; i < 192; i++) {
    diff[i] = cur[i] - pre[i];
  }

  //计算列最大值和列方差
  max_col = maxCOL(diff);
  var_col = varCOL(diff);

  //计算用于判断的列值
  for (int i = 0; i < 16; i++)
    col[i] = pow(max_col[i], 2) + pow(var_col[i], 2);

  //若判断有人，则进入有人循环
  if (isPassing(col)) {

    int breaks = 0; //若连续三帧以上无人经过，则停止有人循环，每次检测到有人帧时，该值重置为0

    points = judgePoints(col); //判断该帧的检测点

    for (float point : points) {
      tracks.push_back(vector<float>(point));
      tracks_status.push_back(0);
      if (point >= 8)
        flags.push_back(1);
      else
        flags.push_back(-1);
    }

    pre = cur;
    points.clear();

    while (true) {
      getPiexls();
      cur = filter(MLX90641To);

      for (int i = 0; i < 192; i++)
        diff[i] = cur[i] - pre[i];


      max_col = maxCOL(diff);
      var_col = varCOL(diff);

      for (int i = 0; i < 16; i++)
        col[i] = pow(max_col[i], 2) + pow(var_col[i], 2);

      if (!isPassing(col))
        if (breaks == 3)
          break;
        else {
          breaks++;
          pre = cur;
          continue;
        }

      breaks = 0;

      points = judgePoints(col);

      for (int i = 0; i < points.size(); i++)
        visited.push_back(0);

      for (int i = tracks.size() - 1; i >= 0; i--) { //从后往前遍历tracks，优先为最晚出现的轨迹匹配跟踪点
        if (tracks_status.at(i) == 3) //#该轨迹历史已连续三次没有跟踪到点，跳过
          continue;

        float pre_point = tracks.at(i).back();//该轨迹最后一个点
        int flag = flags.at(i);
        tracks_status.at(i) = tracks_status.at(i) + 1; //先假设该轨迹无法跟踪到点

        for (int j = 0; j < points.size(); j++) { //遍历所有检测点，若该点已加入某轨迹，则跳过
          if (visited.at(j) == 1)
            continue;
          else {
            float point = points.at(j);

            //当前遍历跟踪点与轨迹方向不符，但与最后一个点距离接近，视为该轨迹的点，但不加入轨迹
            if (flag * point >= flag * pre_point && abs(point - pre_point) <= 2) {
              visited.at(j) = 1;
              tracks_status.at(i) = 0; //该轨迹实际跟踪到点，更新状态
            }

            //当前遍历点与轨迹方向相符且距离接近
            if (flag * point <= flag * pre_point && abs(point - pre_point) <= 3.5) {
              tracks.at(i).push_back(point);
              visited.at(j) = 1;
              tracks_status.at(i) = 0; //该轨迹实际跟踪到点，更新状态
            }
          }
        }
      }

      //遍历完所有轨迹后，为没有完成跟踪的监测点创建新的轨迹
      for (int i = 0; i < points.size(); i++)
        if (visited.at(i) == 0) {
          tracks.push_back(vector<float>(points.at(i)));
          tracks_status.push_back(0);
          if (points.at(i) >= 8)
            flags.push_back(1);
          else
            flags.push_back(-1);
        }


      pre = cur;
      points.clear();
      visited.clear();

    }

    //对tracks进行分析
    int count = 0;
    for (int i = 0; i < tracks.size(); i++) {
      vector<float> track = tracks.at(i);
      int flag = flags.at(i);
      if ((8 - track.front()) * flag < 0 && (8 - track.back()) * flag > 0 && track.size() >= 5)
        cout << "当前进出：" << flag << endl;
    }

    tracks.clear();
    tracks_status.clear();
    flags.clear();
  }

  pre = cur;


    
    long stopTime = millis();
    
    if(!client.connected()){
        reconnect();
    }
    client.publish("hz",String(1000/(stopTime-startTime)).c_str());
    String a = String(track.num);
//    String b = "#";
    client.publish("count",a.c_str());
    
    
}


void getPiexls(){
    
    int status = MLX90641_GetFrameData(MLX90641_address, MLX90641Frame);
    float vdd = MLX90641_GetVdd(MLX90641Frame, &MLX90641);
    float Ta = MLX90641_GetTa(MLX90641Frame, &MLX90641);
    long time3 = millis();
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;
    MLX90641_CalculateTo(MLX90641Frame, &MLX90641, emissivity, tr, MLX90641To);
    
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
