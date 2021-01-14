// 改algorithms2 -> 帧差法

#include <Wire.h>
#include <Vector.h>
#include <stdlib.h>
#include "MLX90640_API.h"
#include "MLX90641_API.h"
#include "MLX9064X_I2C_Driver.h"

#include <ESP8266WiFi.h>
#include <sstream>
#include <string>
using namespace std;

#ifndef STASSID
#define STASSID "TP-LINK_E925"
#define STAPSK "12345678"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

const byte MLX90641_address = 0x33; //Default 7-bit unshifted address of the MLX90641
#define TA_SHIFT 8                  //Default shift for MLX90641 in open air

uint16_t eeMLX90641[832];
float MLX90641To[192];
uint16_t MLX90641Frame[242];
paramsMLX90641 MLX90641;
int errorno = 0;

float piexls_past[192] = {0};
// float piexls_past_sum[100][192];
// int queue = 0;

WiFiClient espClient;

const char *msg;

class Frame
{
public:
    float piexls[192];
    float piexls_past[192];
    float piexls_diff[192];
    float col_diff_max[16];
    float col_diff_mean[16];
    float col_diff_var[16];
    float col_diff_final[16];
    vector<float> diff_index;
    float index_mean;

    //构造函数 a now b past
    Frame(float a[192], float b[192])
    {
        for (int i = 0; i <= 191; i++)
        {
            piexls[i] = a[i];
        }
        for (int i = 0; i <= 191; i++)
        {
            piexls_past[i] = b[i];
        }
        for (int i = 0; i <= 191; i++)
        {
            piexls_diff[i] = piexls[i] - piexls_past[i];
        }

        cal_diff_max();
        cal_diff_mean();
        cal_diff_var();
        cal_diff_final();
        cal_diff_index();

        diff_indexmean();
    }

    // 计算帧差图像的列最大值
    void cal_diff_max()
    {
        for (int i = 0; i <= 15; i++)
        {
            float max = 0;
            for (int j = 0; j <= 11; j++)
            {
                if (piexls_diff[16 * j + i] > max)
                {
                    max = piexls_diff[16 * j + i];
                }
                col_diff_max[i] = max;
            }
        }
    }

    void cal_diff_mean()
    {
        for (int i = 0; i <= 15; i++)
        {
            float sum = 0;
            for (int j = 0; j <= 11; j++)
            {
                sum += piexls_diff[16 * j + i];
            }
            col_diff_mean[i] = sum / 12;
        }
    }

    void cal_diff_var()
    {
        for (int i = 0; i <= 15; i++)
        {
            float sum = 0;
            for (int j = 0; j <= 11; j++)
            {
                sum += pow(col_diff_mean[i] - piexls_diff[16 * j + i], 2);
            }
            col_diff_var[i] = sum / 12;
        }
    }

    void cal_diff_final()
    {
        for (int i = 0; i <= 15; i++)
        {
            col_diff_final[i] = col_diff_var[i] + col_diff_max[i];
        }
    }

    void cal_diff_index()
    {
        int count = 0;
        for (int i = 0; i <= 15; i++)
        {
            if (col_diff_final[i] >= 1.25)
            {
                count += 1;
            }
            else
            {
                col_diff_final[i] = -1;
            }
        }
        if (count <= 1)
        {
            diff_index.push_back(-1);
            return;
        }
        for (int i = 0; i <= 15; i++)
        {
            if (col_diff_final[i] < 0)
            {
                continue;
            }
            int track_point = 0;
            double sum_i = 0;
            double sum_col_i = 0;
            while ((i <= 15) && (col_diff_final[i] > 0))
            {
                track_point += 1;
                sum_i += col_diff_final[i];
                sum_col_i += i * col_diff_final[i];
                i++;
            }
            double a = sum_col_i / sum_i;
            if ((track_point > 1) && (a > 2) && (a < 13))
            {
                diff_index.push_back(a);
            }
        }
        if (diff_index.size() == 0)
        {
            diff_index.push_back(-1);
        }
    }

    void diff_indexmean()
    {
        float sum = 0;
        for (int i = 0; i <= diff_index.size() - 1; i++)
        {
            sum += diff_index[i];
        }
        index_mean = sum / diff_index.size();
    }
};

void setup_wifi()
{

    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
    Wire.beginTransmission((uint8_t)MLX90641_address);
    if (Wire.endTransmission() != 0)
    {
        return (false);
    }
    return (true);
}

void getPiexls()
{
    int status = MLX90641_GetFrameData(MLX90641_address, MLX90641Frame);
    float vdd = MLX90641_GetVdd(MLX90641Frame, &MLX90641);
    float Ta = MLX90641_GetTa(MLX90641Frame, &MLX90641);
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;
    MLX90641_CalculateTo(MLX90641Frame, &MLX90641, emissivity, tr, MLX90641To);
}

class Track
{
public:
    int flag = 0;
    vector<vector<float>> pointList = {{0}};
    int num = 5;
    int time = 0;

    void judge()
    {
        int flag = 0;
        vector<vector<float>> track_list;
        if (pointList[1][0] > 7)
        {
            flag = -1;
        }
        if (pointList[1][0] < 7)
        {
            flag = 1;
        }
        if (flag == -1)
        {
            for (int i = 1; i <= pointList.size() - 1; i++)
            {
                for (int j = 0; j <= pointList[i].size() - 1; j++)
                {
                    if (track_list.size() == 0)
                    {
                        track_list.push_back({pointList[i][j]});
                    }
                    else if (track_list.size() == 1)
                    {
                        if (pointList[i][j] > track_list[0][track_list.size() - 1])
                        {
                            track_list.push_back({pointList[i][j]});
                        }
                        else
                        {
                            track_list[0].push_back(pointList[i][j]);
                        }
                    }
                    else
                    {
                        float a = track_list[track_list.size() - 2][track_list[track_list.size() - 2].size() - 1];
                        float b = track_list[track_list.size() - 1][track_list[track_list.size() - 1].size() - 1];
                        if (pointList[i][j] < (a + 0.5))
                        {
                            track_list[track_list.size() - 2].push_back(pointList[i][j]);
                        }
                        else if (pointList[i][j] < (b + 0.5))
                        {
                            track_list[track_list.size() - 1].push_back(pointList[i][j]);
                        }
                        else
                        {
                            track_list.push_back({pointList[i][j]});
                        }
                    }
                }
            }
            int count = 0;
            for (int i = 0; i <= track_list.size() - 1; i++)
            {
                if (track_list[i][0] > 8 && track_list[i][track_list[i].size() - 1] < 8 && track_list[i].size() >= 3)
                {
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
        if (flag == 1)
        {
            for (int i = 1; i <= pointList.size() - 1; i++)
            {
                for (int j = 0; j <= pointList[i].size() - 1; j++)
                {
                    if (track_list.size() == 0)
                    {
                        track_list.push_back({pointList[i][j]});
                    }
                    else if (track_list.size() == 1)
                    {
                        if (pointList[i][j] > track_list[0][track_list.size() - 1])
                        {

                            track_list.push_back({pointList[i][j]});
                        }
                        else
                        {
                            track_list[0].push_back(pointList[i][j]);
                        }
                    }
                    else
                    {
                        float a = track_list[track_list.size() - 2][track_list[track_list.size() - 2].size() - 1];
                        float b = track_list[track_list.size() - 1][track_list[track_list.size() - 1].size() - 1];
                        if (pointList[i][j] > (a - 0.5))
                        {
                            track_list[track_list.size() - 2].push_back(pointList[i][j]);
                        }
                        else if (pointList[i][j] > (b - 0.5))
                        {
                            track_list[track_list.size() - 1].push_back(pointList[i][j]);
                        }
                        else
                        {
                            track_list.push_back({pointList[i][j]});
                        }
                    }
                }
            }
            int count = 0;
            for (int i = 0; i <= track_list.size() - 1; i++)
            {
                if (track_list[i][0] < 8 && track_list[i][track_list[i].size() - 1] > 7 && track_list[i].size() >= 3)
                {
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
            num = num - count;
            //debug
            //client.publish("count",String(count).c_str());
            //debug
        }
    }
};

Track track;

void setup()
{
    setup_wifi();
    Wire.begin();
    Wire.setClock(800000); //Increase I2C clock speed to 400kHz
    Serial.begin(115200);  //Fast debug as possible
    while (!Serial)
        ;
    if (isConnected() == false)
    {
        Serial.println("MLX90641 not detected at default I2C address. Please check wiring. Freezing.");
        while (1)
            ;
    }
    //Get device parameters - We only have to do this once
    int status;
    status = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
    errorno = status; //MLX90641_CheckEEPROMValid(eeMLX90641);//eeMLX90641[10] & 0x0040;//
    if (status != 0)
    {
        Serial.println("Failed to load system parameters");
        while (1)
            ;
    }
    status = MLX90641_ExtractParameters(eeMLX90641, &MLX90641);
    //errorno = status;
    if (status != 0)
    {
        Serial.println("Parameter extraction failed");
        while (1)
            ;
    }

    //Once params are extracted, we can release eeMLX90641 array

    //MLX90641_SetRefreshRate(MLX90641_address, 0x02); //Set rate to 2Hz
    MLX90641_SetRefreshRate(MLX90641_address, 0x03); //Set rate to 4Hz
    //MLX90641_SetRefreshRate(MLX90641_address, 0x07); //Set rate to 64Hz
    delay(200);

    // Serial.println("----------初始化背景----------");
    // for (int i = 0; i < 100; i++)
    // {
    //     getPiexls;
    //     for (int j = 0; j < 192; j++)
    //     {
    //         piexls_past_sum[i][j] = MLX90641To[j];
    //     }
    // }
    // for (int i = 0; i < 192; i++)
    // {
    //     float sum = 0;
    //     for (int j = 0; j < 100; j++)
    //     {
    //         sum += piexls_past_sum[j][i];
    //     }
    //     piexls_past[i] = sum / 100;
    // }
    // Serial.println("--------背景初始化完成--------");
}

void loop()
{
    Serial.println("----------loop-running----------");
    long startTime = millis();
    getPiexls();
    Frame frame(MLX90641To, piexls_past);

    //debug 串口输出
    //frame.piexls_diff
    //frame.index
    //track.time
    // Serial.print("Piexls_diff:");
    // for (int i = 0; i < 16; i++)
    // {
    //     Serial.print(frame.piexls_diff[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();
    // Serial.print("Diff_var:");
    // for (int i = 0; i < 16; i++){
    //     Serial.print(frame.col_diff_var[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();
    // Serial.print("Diff_final:");
    // for (int i = 0; i < 16; i++){
    //     Serial.print(frame.col_diff_final[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();

    Serial.print("Index:");
    for (float i : frame.diff_index)
    {
        Serial.print(i);
    }
    Serial.println();

    Serial.print("TRACK-TIME:");
    Serial.print(track.time);

    if (frame.index_mean > 0)
    {
        track.time = 0;
        if (frame.diff_index != track.pointList.back())
        {
            track.pointList.push_back(frame.diff_index);
        }
    }
    else
    {
        track.time += 1;
    }
    if (track.time >= 20 && track.pointList.size() >= 3)
    {
        track.judge();
        Serial.println("-----judge running-----");
        vector<vector<float>> v = {{0}};
        track.pointList = v;
        track.time = 0;
    }

    if (track.time >= 100 && track.pointList.size() <= 2)
    {
        vector<vector<float>> v = {{0}};
        track.pointList = v;
        track.time = 0;
    }

    long stopTime = millis();
    Serial.print("HZ:");
    Serial.print(1000 / (stopTime - startTime));
    Serial.print("NUM:");
    Serial.println(track.num);

    for (int i = 0; i <= 191; i++)
    {
        piexls_past[i] = MLX90641To[i];
    }

    // for (int i = 0; i <= 191; i++)
    // {
    //     piexls_past_sum[queue][i] = MLX90641To[i];
    // }
    // queue = queue + 1;
    // if (queue == 100)
    // {
    //     queue = 0;
    // }
    // for (int i = 0; i < 192; i++)
    // {
    //     float sum = 0;
    //     for (int j = 0; j < 100; j++)
    //     {
    //         sum += piexls_past_sum[j][i];
    //     }
    //     piexls_past[i] = sum / 100;
    // }
    
}
