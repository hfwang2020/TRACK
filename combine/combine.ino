#include <Adafruit_BME280.h>
#include <Wire.h>
#include <math.h>
#include <ccs811.h> 
CCS811 ccs811(D3);  
#include <ESP8266WiFi.h>
#include <PubSubClient.h>


#define BME280_ADDRESS 0x76
int BH1750address = 0x23; 
unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;

byte buff[2];

uint16_t dig_T1;
 int16_t dig_T2;
 int16_t dig_T3;
uint16_t dig_P1;
 int16_t dig_P2;
 int16_t dig_P3;
 int16_t dig_P4;
 int16_t dig_P5;
 int16_t dig_P6;
 int16_t dig_P7;
 int16_t dig_P8;
 int16_t dig_P9;
 int8_t  dig_H1;
 int16_t dig_H2;
 int8_t  dig_H3;
 int16_t dig_H4;
 int16_t dig_H5;
 int8_t  dig_H6;



#ifndef STASSID
#define STASSID "TP-LINK_E925"
#define STAPSK  "12345678"
#endif

const char* ssid     = STASSID;//WiFi名称
const char* password = STAPSK;//wifi密码
const char* mqtt_server = "42.192.171.165";// mqtt服务器地址
const int mqtt_server_port = 1883;// mqtt的端口号
//#define ID_MQTT  "6b4deb575dc7e791470fb2c72a5432c0"//Client ID  巴法云用户私钥


WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)

char msg[MSG_BUFFER_SIZE];
int value = 0;

//WiFi设置
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

char po[3];
void callback(char* topic, byte* payload ,unsigned int length){
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]");
    for (int i = 0; i < length;i++){
        Serial.print((char)payload[i]);
    }
    if(length==1){
      sprintf(po,"%c",(char)payload[0]);
    }else{
      sprintf(po,"%c%c",(char)payload[0],(char)payload[1]);
    }
    Serial.println();

    if ((char)payload[0] == '1'){
        digitalWrite(BUILTIN_LED,LOW);
    }else{
        digitalWrite(BUILTIN_LED,HIGH);
    }
}

void reconnect(){
    while (!client.connected()){
        Serial.print("Attempting MQTT connection...");
        String clientID = "ESP8266Client-hfwang";
        //connect
        if(client.connect(clientID.c_str())){
            Serial.println("connected successfully");
            client.subscribe("data");
        }else{
            Serial.print("failed,rc=");
            Serial.print(client.state());
            Serial.println("try again in 5 seconds");
            delay(5000);
        }
    }
}


void setup()
{
    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 3;               //Normal mode
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off 
    uint8_t spi3w_en = 0;           //3-wire SPI Disable
    
    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;
    
    Serial.begin(9600);//启动频率
    setup_wifi();
    client.setServer(mqtt_server,1883);//mqtt的服务器地址和端口号
    client.setCallback(callback); 
    Wire.begin();
    
    ccs811.set_i2cdelay(50);
    ccs811.begin();
    ccs811.start(CCS811_MODE_1SEC);
    
    writeReg(0xF2,ctrl_hum_reg);
    writeReg(0xF4,ctrl_meas_reg);
    writeReg(0xF5,config_reg);
    readTrim();                    
}


void loop()
{   int i;
    uint16_t val=0;
    uint16_t eco2, etvoc, errstat, raw;
    ccs811.read(&eco2,&etvoc,&errstat,&raw);
    if( errstat==CCS811_ERRSTAT_OK ) { 
    //Serial.print("CCS811: ");
    Serial.print("eco2=");  Serial.print(eco2);     Serial.print(" ppm  ");
    Serial.print("etvoc="); Serial.print(etvoc);    Serial.print(" ppb  ");
    Serial.println();
  } else if( errstat==CCS811_ERRSTAT_OK_NODATA ) {
    Serial.println("CCS811: waiting for (new) data");
  } else if( errstat & CCS811_ERRSTAT_I2CFAIL ) { 
    Serial.println("CCS811: I2C error");
  } else {
    Serial.print("CCS811: errstat="); Serial.print(errstat,HEX); 
    Serial.print("="); Serial.println( ccs811.errstat_str(errstat) ); 
  }
  
  delay(1000);
    BH1750_Init(BH1750address);
    delay(200);
    if(2==BH1750_Read(BH1750address))
    {
       val=((buff[0]<<8)|buff[1])/1.2;
      Serial.print("Light strength : ");
      Serial.print(val);   
    }
    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;
    
    readData();
    
    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / 100.0;
    hum_act = (double)hum_cal / 1024.0;
    Serial.print(" lx  TEMP : ");
    Serial.print(temp_act);
    Serial.print(" DegC  PRESS : ");
    Serial.print(press_act);
    Serial.print(" hPa  HUM : ");
    Serial.print(hum_act);
    Serial.println(" %");  

    char output[90];
    sprintf(output,"%d#%d#%d#%.2f#%.2f#%.2f#%s",eco2,etvoc,val,temp_act,press_act,hum_act,po);    //传感器传输到mqtt时的格式
 // sprintf(output,"%d#%.2f#%.2f#%.2f",val,temp_act,press_act,hum_act); 

   if(!client.connected()){
        reconnect();
    }
    client.loop();
    
    //millis() andruino从运行开始的毫秒数
    unsigned long now = millis();
    if(now - lastMsg >2000){
        lastMsg = now;
        ++value;
        //按照format的格式格式化为字符串，然后再将其拷贝至msg中
        snprintf(msg,MSG_BUFFER_SIZE,output,value);
        client.publish("data",msg);//mqtt的订阅主题
    }

    delay(1000);
}
void readTrim()
{
    uint8_t data[32],i=0;
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,24);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xA1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,1);
    data[i] = Wire.read();
    i++;
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,7);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;    
    }
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];   
}
void writeReg(uint8_t reg_address, uint8_t data)
{
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg_address);
    Wire.write(data);
    Wire.endTransmission();    
}

int BH1750_Read(int address) //

{

  int i=0;

  Wire.beginTransmission(address);

  Wire.requestFrom(address, 2);

  while(Wire.available()) //

  {

    buff[i] = Wire.read();  // receive one byte

    i++;

  }

  Wire.endTransmission();  

  return i;

}


void BH1750_Init(int address) 

{

  Wire.beginTransmission(address);

  Wire.write(0x10);//1lx reolution 120ms

  Wire.endTransmission();

}

void readData()
{
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,8);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T)
{
    
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;
    
    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) + 
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) * 
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);   
}

