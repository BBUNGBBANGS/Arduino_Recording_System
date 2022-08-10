
#include <SD.h>
#include <SPI.h>
#include <wave_header.h>

#define MIC_PIN     (A0) //A0

uint16_t Sound_Raw;
Wave_Header_t Wave_Header;

void setup() 
{
    analogReadResolution(16);
    pinMode(MIC_PIN,INPUT);
    Serial.begin(115200);
}

void loop() 
{
    Sound_Raw = analogRead(MIC_PIN);
    Serial.print("Analog : ");
    Serial.println(Sound_Raw);
    delay(5);
}

void Create_WaveHeader(void)
{
    memcpy(Wave_Header.Riff.ChunkID,"RIFF",4);
}
