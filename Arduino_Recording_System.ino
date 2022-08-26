#include "main.h"
#include "wave_header.h"
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include "SDRAM.h"
#include <stdint.h>
//#include <SensirionI2CSfm3000.h>
//#include <Wire.h>

#define SOUND_DATA_SIZE             (500)
#define FLOW_DATA_SIZE              (100000)

#define SOUND_RECORD_INIT           (0)
#define SOUND_RECORD_START          (1)
#define SOUND_RECORD_COPY           (2)
#define SOUND_RECORD_HEADER         (3)
#define SOUND_RECORD_FINISH         (4)

#define RECORD_TIME_SECOND          (60)  //Setting Record time <Max : 80[s]>
#define RECORD_TIME                 (RECORD_TIME_SECOND * 1000000 / 22)

#define SFM3000_I2C_ADDRESS         (0x40)

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
char WavFileName[] = "fs/0000.wav";  
char FlowFileName[] = "fs/0000.txt";  

SDRAMClass mySDRAM;
uint16_t Flow_Raw_Data;
uint16_t Flow_Data[FLOW_DATA_SIZE];
uint16_t Flow_Data_Length;

uint32_t time_pre,time_new;

void setup() 
{
    uint8_t tx_buf[2] = {0x10,0x00};
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();
    MX_TIM16_Init();
    MX_I2C3_Init();

    HAL_TIM_Base_Start(&htim6);
    HAL_TIM_Base_Start_IT(&htim16);
    /*****************************************/

    /* Initialize Serial for debugging */
    Serial.begin(115200);
    while(!Serial);
    /***********************************/

    /* SD Card Mount and Read file list */
    Serial.println("SD Card Mount Start");
    int err =  fs.mount(&block_device);
    Serial.println("SD Card Mount Finished");
    Read_WavFile_SdCard();
    /************************************/

    /* External SDRAM Allocation for Record wavFile */
    Serial.println("SDRam Allocation Start");
    mySDRAM.begin(SDRAM_START_ADDRESS);
    WavFile_Data = (uint8_t *)mySDRAM.malloc(7 * 1024 * 1024);
    Serial.println("SDRam Allocation Finished");
    /************************************************/

    /* Delay 100[ms] for SFM3000 Safe Start */
    delay(100);
    /* start Continuous Measurement */
    HAL_I2C_Master_Transmit(&hi2c3,SFM3000_I2C_ADDRESS<<1,tx_buf,2,10);
}

void loop() 
{
    uint16_t status;
    if(Serial.available() > 0)
    {
        Serial_data = Serial.read();
        if(Serial_data == 's')
        {
            Sound_Record_Step = SOUND_RECORD_START;
            Serial.println("Sound Record Start");
            time_pre = millis();
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t rx_buf[3] = {0,};

    if(htim->Instance == TIM16)
    {
        HAL_I2C_Master_Receive(&hi2c3,SFM3000_I2C_ADDRESS<<1,rx_buf,3,1);
        Flow_Raw_Data = rx_buf[1] | (rx_buf[0] << 8);
        if(SOUND_RECORD_COPY == Sound_Record_Step)
        {
            Flow_Data[Flow_Data_Length] = Flow_Raw_Data;
            Flow_Data_Length++;
        }
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
            Flow_Data_Length = 0;
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
            }

            if(Sound_Data_Length == RECORD_TIME)
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
            time_new = millis();
            Serial.print("Record total time : ");
            Serial.print(time_new - time_pre);
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
            Serial.println("SD Card Record Start");

            WavFileName[3] = (WavFile_Count/1000) % 10 + '0';
            WavFileName[4] = (WavFile_Count/100) % 10 + '0';
            WavFileName[5] = (WavFile_Count/10) % 10 + '0';
            WavFileName[6] = (WavFile_Count) % 10 + '0';
            myFile = fopen(WavFileName, "a");
            Serial.println(WavFileName);
            fwrite(WavFile_Data, 1, WavFile_length, myFile);
            fclose(myFile);

            FlowFileName[3] = (WavFile_Count/1000) % 10 + '0';
            FlowFileName[4] = (WavFile_Count/100) % 10 + '0';
            FlowFileName[5] = (WavFile_Count/10) % 10 + '0';
            FlowFileName[6] = (WavFile_Count) % 10 + '0';
            myFile = fopen(FlowFileName, "a");
            Serial.println(FlowFileName);
            fprintf(myFile,"< SFM3000 Flow Sensor Raw Data >\n");
            for(uint16_t i = 0; i < Flow_Data_Length; i++)
            {
                fprintf(myFile,"%d\n",Flow_Data[i]);
            }
            fclose(myFile);

            Serial.println("SD Card Record Finished");
        break;
        default : 
        break;
    }
}

void Read_WavFile_SdCard(void)
{
    DIR *dir;
    struct dirent *ent;
  
    Serial.println("SDCard File List : ");
    if ((dir = opendir("/fs")) != NULL) 
    {
        // Print all the files and directories within directory (not recursively)
        while ((ent = readdir (dir)) != NULL)
        {
            Serial.println(ent->d_name);
            WavFile_Count++;
        };
        
        if(WavFile_Count>0)
        {
            WavFile_Count = WavFile_Count - 1;
        }

        closedir (dir);
    } 
    else 
    {
        // Could not open directory
        Serial.println("SDCard Open Error - Please Check SDCard And Arduino\n");
        while(1);
    }
}
