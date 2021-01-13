/*
    Output the temperature readings to all pixels to be read by a Processing visualizer
*/

#include <Wire.h>


#include "MLX90640_API.h"

#include "MLX90641_API.h"

#include "MLX9064X_I2C_Driver.h"
#define debug  Serial

const byte MLX90641_address = 0x33; //Default 7-bit unshifted address of the MLX90641
#define TA_SHIFT 8 //Default shift for MLX90641 in open air

uint16_t eeMLX90641[832];
float MLX90641To[192];
uint16_t MLX90641Frame[242];
paramsMLX90641 MLX90641;
int errorno = 0;

void setup() {
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

    
   
//    for (int x = 0 ; x < 192; x++) {
//        debug.print(MLX90641To[x], 2);
//        debug.print(x);
//        debug.print(",");
//    }
//    debug.println("");    

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




//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected() {
    Wire.beginTransmission((uint8_t)MLX90641_address);
    if (Wire.endTransmission() != 0) {
        return (false);    //Sensor did not ACK
    }
    return (true);
}
