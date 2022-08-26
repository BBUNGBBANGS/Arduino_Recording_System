#include "main.h"
#include "wave_header.h"
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include "SDRAM.h"
#include <stdint.h>

#define SOUND_DATA_SIZE           (500)

#define SOUND_RECORD_INIT           (0)
#define SOUND_RECORD_START          (1)
#define SOUND_RECORD_COPY           (2)
#define SOUND_RECORD_HEADER         (3)
#define SOUND_RECORD_FINISH         (4)

#define RECORD_TIME_SECOND          (80)  //Setting Record time <Max : 80[s]>
#define RECORD_TIME                 (RECORD_TIME_SECOND * 1000000 / 22)

Wave_Header_t WavFile;
uint32_t WavFile_length;
uint32_t Sound_Data_Length;
uint16_t Sound_Data[SOUND_DATA_SIZE];
uint8_t Sound_Record_Step;
uint8_t Sound_Data_Copy_Flag;
uint8_t *WavFile_Data;
uint16_t WavFile_Count;

uint8_t Serial_data;

SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");
FILE *myFile;
char myFileName[] = "fs/0000.wav";  

SDRAMClass mySDRAM;

uint32_t milis_pre,milis_new;

void setup() 
{
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();

    HAL_TIM_Base_Start(&htim6);
    Serial.begin(115200);
    
    Serial.println("SD Card Mount Start");
    int err =  fs.mount(&block_device);
    Serial.println("SD Card Mount Finished");

    Serial.println("SDRam Allocation Start");
    mySDRAM.begin(SDRAM_START_ADDRESS);
    WavFile_Data = (uint8_t *)mySDRAM.malloc(7 * 1024 * 1024);
    Serial.println("SDRam Allocation Finished");
}


void loop() 
{
    if(Serial.available() > 0)
    {
        Serial_data = Serial.read();
        if(Serial_data == 's')
        {
            Sound_Record_Step = SOUND_RECORD_START;
            Serial.println("Sound Record Start");
            milis_pre = millis();
        }
    }
    Create_WaveFile();
    Save_WavFile_SDCard();
}

void Adc_Sampling(void)
{
    Sound_Data[Sound_Data_Length%SOUND_DATA_SIZE] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    Sound_Data_Length++;

    if(((Sound_Data_Length%SOUND_DATA_SIZE) == 0)&&(Sound_Data_Length>0))
    {
        Sound_Data_Copy_Flag = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        Adc_Sampling();
    }
}

void Create_WaveFile_Header(void)
{
    WavFile_length = Sound_Data_Length*2 + 36;
    /* Riff Header */
    WavFile.Riff.ChunkID[0] = 'R';
    WavFile.Riff.ChunkID[1] = 'I';
    WavFile.Riff.ChunkID[2] = 'F';
    WavFile.Riff.ChunkID[3] = 'F';   
    WavFile.Riff.ChunkSize = WavFile_length - 8;//Convert_Little_Endian_32bit(WavFile_length - 8);
    WavFile.Riff.Format[0] = 'W';
    WavFile.Riff.Format[1] = 'A';
    WavFile.Riff.Format[2] = 'V';
    WavFile.Riff.Format[3] = 'E';
    /* Fmt Header */
    WavFile.Fmt.ChunkID[0] = 'f';
    WavFile.Fmt.ChunkID[1] = 'm';
    WavFile.Fmt.ChunkID[2] = 't';
    WavFile.Fmt.ChunkID[3] = ' ';
    WavFile.Fmt.ChunkSize = FMT_SIZE;//Convert_Little_Endian_32bit(FMT_SIZE);
    WavFile.Fmt.AudioFormat = AUDIO_FORMAT;//Convert_Little_Endian_16bit(AUDIO_FORMAT);
    WavFile.Fmt.NumChannels = NUMBER_OF_CHANNEL;//Convert_Little_Endian_16bit(NUMBER_OF_CHANNEL);
    WavFile.Fmt.SampleRate = SAMPLE_RATE;//Convert_Little_Endian_32bit(SAMPLE_RATE);
    WavFile.Fmt.ByteRate = BYTE_RATE;//Convert_Little_Endian_32bit(BYTE_RATE);
    WavFile.Fmt.BlockAlign = BLOCK_ALIGN;//Convert_Little_Endian_16bit(BLOCK_ALIGN);
    WavFile.Fmt.BitPerSample = BPS;//Convert_Little_Endian_16bit(BPS);
    /* Data Header */
    WavFile.Data.ChunkID[0] = 'd';
    WavFile.Data.ChunkID[1] = 'a';
    WavFile.Data.ChunkID[2] = 't';
    WavFile.Data.ChunkID[3] = 'a';
    WavFile.Data.ChunkSize = Sound_Data_Length*2;
}

void Create_WaveFile(void)
{
    uint8_t *Ptr = NULL;
    uint8_t temp;

    switch(Sound_Record_Step)
    {
        case SOUND_RECORD_START : 
            HAL_ADC_Start_IT(&hadc1);
            Sound_Data_Length = 0;
            Sound_Record_Step = SOUND_RECORD_COPY;
            Serial.println("Sound Record Step : START");
        break;
        case SOUND_RECORD_COPY :
            if(Sound_Data_Copy_Flag == 1)
            {
                for(uint32_t i = 0; i < SOUND_DATA_SIZE; i++)
                {
                    WavFile_Data[WAVFILE_HEADER_LENGTH+((Sound_Data_Length - SOUND_DATA_SIZE)*2)+(2*i)] = (uint8_t)(Sound_Data[i]&0x00FF);
                    WavFile_Data[WAVFILE_HEADER_LENGTH+((Sound_Data_Length - SOUND_DATA_SIZE)*2)+(2*i)+1] = (uint8_t)((Sound_Data[i]>>8)&0x00FF);             
                    Sound_Data[i] = 0;
                }
                Sound_Data_Copy_Flag = 0;
                
                //Serial.print("Sound Record Step : COPY , Count : ");
                //Serial.println(Sound_Data_Length);
            }

            if(Sound_Data_Length == 3636364)
            {
                Sound_Record_Step = SOUND_RECORD_HEADER;
                HAL_ADC_Stop_IT(&hadc1);
            }
        break;
        case SOUND_RECORD_HEADER :
            Serial.println("Sound Record Step : HEADER");
            Create_WaveFile_Header();
            Ptr = &WavFile.Riff.ChunkID[0];
            for(uint8_t i = 0; i<WAVFILE_HEADER_LENGTH; i++)
            {
                WavFile_Data[i] = Ptr[i];        
            }
            WavFile_Count++;
            Sound_Record_Step = SOUND_RECORD_FINISH;
            Serial.println("Sound Record Step : FINISH");
            milis_new = millis();
            Serial.print("Record total time : ");
            Serial.print(milis_new - milis_pre);
            Serial.println("[ms]");
        break;
        case SOUND_RECORD_FINISH : 
            Sound_Record_Step = SOUND_RECORD_INIT;
        break;
        default :   
            /* do nothing */
        break;
    }
}

void Save_WavFile_SDCard(void)
{
    uint32_t millis_pre,millis_new;
    switch(Sound_Record_Step)
    {
        case SOUND_RECORD_FINISH : 

            myFileName[3] = (WavFile_Count/1000) % 10 + '0';
            myFileName[4] = (WavFile_Count/100) % 10 + '0';
            myFileName[5] = (WavFile_Count/10) % 10 + '0';
            myFileName[6] = (WavFile_Count) % 10 + '0';

            myFile = fopen(myFileName, "a");
            Serial.println("SD Card Record Start");
            Serial.println(myFileName);
            millis_pre = millis();
            fwrite(WavFile_Data, 1, WavFile_length, myFile);
            millis_new = millis();
            fclose(myFile);
            Serial.println("SD Card Record Finished");
            Serial.print("SD Card Data Write time : ");
            Serial.print(millis_new - millis_pre);
            Serial.println("[ms]");
        break;
        default : 
        break;
    }
}
