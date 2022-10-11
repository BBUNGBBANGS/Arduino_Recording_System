

#define _TEST

#include <stdint.h>
#include "main.h"
/* 아래 값 조정해서 측정 민감도 도절 가능 */
#define SFM3000_FLOW_THRESHOLD      (40000)//Raw Data
#define SFM3000_SAMPLING_TIME       (10)//[ms]
#define LOWPASS_FILTER_FREQUENCY    (1000)//[Hz]
#define WIFI_SSID                   "WIFI_SSID명"
#define WIFI_PASSWORD               "WIFI비밀번호_입력"

#define SOUND_DATA_SIZE             (100)
#define FLOW_DATA_SIZE              (100000)

#define SOUND_RECORD_INIT           (0)
#define SOUND_RECORD_START          (1)
#define SOUND_RECORD_COPY           (2)
#define SOUND_RECORD_HEADER         (3)
#define SOUND_RECORD_FINISH         (4)

#define SRAM3_START_ADDRESS       ((uint32_t) 0x30040000)
struct shared_data
{
    uint8_t M4_status;
    uint8_t M4_Semapore;
    uint8_t M7_status;
    uint8_t M7_Semapore;
    uint16_t Flow_Raw_Data;
    uint16_t Flow_Data_Length;
    uint16_t Flow_Data[FLOW_DATA_SIZE];
};

#ifdef CORE_CM7
#include "wave_header.h"
#include <SPI.h>
#include <SD.h>
#include "SDRAM.h"
#include <WiFi.h>


#define RECORD_MAX_SECOND           (60)  //Setting Record time <Max : 80[s]>
#define RECORD_MAX_TIME             (RECORD_MAX_SECOND * 1000000 / 22)

/************************/

#define FILTER_PI (3.141592f)
#define FILTER_TS (0.000022f)
#define FILTER_TAU ((float)(1.0f/LOWPASS_FILTER_FREQUENCY/2.0f/FILTER_PI))

Wave_Header_t WavFile;
uint32_t WavFile_length;
uint32_t Sound_Data_Length;
int16_t Sound_Data[SOUND_DATA_SIZE];
uint8_t Sound_Record_Step;
uint8_t Sound_Data_Copy_Flag;
uint8_t *WavFile_Data;
uint16_t WavFile_Count;

uint8_t Serial_data;

File myFile;
char WavFileName[] = "0001.wav";  
char FlowFileName[] = "0001.txt";  

SDRAMClass mySDRAM;

uint32_t time_pre,time_new;

char ssid[] = WIFI_SSID;        // your network SSID (name)
char pass[] = WIFI_PASSWORD;        // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;

static struct shared_data * const Shared_Ptr = (struct shared_data *)SRAM3_START_ADDRESS;

void setup() 
{    
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);    
    MPU_Config();
    bootM4();
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();
    /*****************************************/

    /* Initialize Serial for debugging */
    Serial.begin(115200);
    while(!Serial);
    /***********************************/

    /* SD Card Mount and Read file list */
    SD_Card_Init();
    /************************************/

    /* External SDRAM Allocation for Record wavFile */
    SD_Ram_Init();
    /************************************************/

    /* WiFi Initialize */
    //WiFi_Init();
    /*******************/

    HAL_TIM_Base_Start(&htim6);
    Shared_Ptr->M7_status = SOUND_RECORD_START;
}

void loop() 
{
    #ifdef _TEST
    if(Serial.available() > 0)
    {
        Serial_data = Serial.read();
        if(Serial_data == 's')
        {
            Shared_Ptr->Flow_Raw_Data = SFM3000_FLOW_THRESHOLD + 1;
        }
        if(Serial_data == 'p')
        {
            Shared_Ptr->Flow_Raw_Data = 0;
        }
    }
    #endif
    Create_WaveFile();
    Save_WavFile_SDCard();
}

void SD_Card_Init(void)
{
    Serial.println("SD Card Mount Start");
    if (!SD.begin(7)) 
    {
        Serial.println("SD Card initialization failed!");
        while (1);
    }

    Serial.println("SD Card Mount Finished");
    Read_WavFile_SdCard();
}

void SD_Ram_Init(void)
{
    Serial.println("SDRam Allocation Start");
    mySDRAM.begin(SDRAM_START_ADDRESS);
    WavFile_Data = (uint8_t *)mySDRAM.malloc(7 * 1024 * 1024);
    Serial.println("SDRam Allocation Finished");
}

void WiFi_Init(void)
{
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_SHIELD) 
    {
        Serial.println("Communication with WiFi module failed!");
        // don't continue
        while (true);
    }
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED) 
    {
        Serial.print("..");
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(ssid, pass);
        delay(100);
    }
    Serial.println("Connected to WiFi");
    printWifiStatus();
}

void Adc_Sampling(void)
{
    uint32_t idx;
    uint16_t AdcValue;
    int16_t ConvertVoltage;
    int16_t diffVoltage;
    static int16_t Sound_Data_Old;
    static int16_t ConvertVoltage_Old;

    idx = Sound_Data_Length%SOUND_DATA_SIZE;
    AdcValue = (uint16_t)HAL_ADC_GetValue(&hadc1);
    /* Max 2.0V for MAX9814 */
    /* 2.0(Vpp) / 3.0(Aref) * 65536(Adc Resolution) */
    if(AdcValue>43691)
    {
        AdcValue = 43691;
    }
    /* Max 0.4V for MAX9814 */
    /* 0.4(Vpp) / 3.0(Aref) * 65536(Adc Resolution) */
    if(AdcValue<8738)
    {
        AdcValue = 8738;
    }
    /* Convert 1.2(Vpp) DC Bias + 0.06(vpp) Bias*/
    ConvertVoltage = (int16_t)(AdcValue - 27525);
    /* Prevent Urgent gradient difference */
    diffVoltage = ConvertVoltage - ConvertVoltage_Old;
    if((diffVoltage > 15000)||(diffVoltage < -15000))
    {
        ConvertVoltage = ConvertVoltage_Old;
    }
    ConvertVoltage_Old = ConvertVoltage;
   
    Sound_Data[idx] = (int16_t)LowPassFilter((float)ConvertVoltage,(float)Sound_Data_Old,FILTER_TS,FILTER_TAU);
    Sound_Data_Old = Sound_Data[idx];
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

    switch(Sound_Record_Step)
    {
        case SOUND_RECORD_INIT:
            if (Shared_Ptr->Flow_Raw_Data >= SFM3000_FLOW_THRESHOLD)
            {
                Sound_Record_Step = SOUND_RECORD_START;
                Shared_Ptr->M7_status = SOUND_RECORD_START;
                Serial.println("Sound Record Start");
                time_pre = millis();       
            }
        break;
        case SOUND_RECORD_START : 
            HAL_ADC_Start_IT(&hadc1);
            Sound_Data_Length = 0;
            Shared_Ptr->Flow_Data_Length = 0;
            Sound_Record_Step = SOUND_RECORD_COPY;
            Shared_Ptr->M7_status = SOUND_RECORD_COPY;
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

            if((RECORD_MAX_TIME <= Sound_Data_Length) || (Shared_Ptr->Flow_Raw_Data <= SFM3000_FLOW_THRESHOLD))
            {
                #ifdef _TEST
                Shared_Ptr->Flow_Raw_Data = 0;
                #endif
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
            Shared_Ptr->M7_status = SOUND_RECORD_FINISH;
            Serial.println("Sound Record Step : FINISH");
            time_new = millis();
            Serial.print("Record total time : ");
            Serial.print(time_new - time_pre);
            Serial.println("[ms]");
        break;
        case SOUND_RECORD_FINISH : 
            Sound_Record_Step = SOUND_RECORD_INIT;
            Shared_Ptr->M7_status = SOUND_RECORD_INIT;
        break;
        default :   
            /* do nothing */
        break;
    }
}

void Save_WavFile_SDCard(void)
{
    String temp = "";
    switch(Sound_Record_Step)
    {
        case SOUND_RECORD_FINISH : 
            Serial.println("SD Card Record Start");
            SPI.beginTransaction(SPISettings(18000000, MSBFIRST, SPI_MODE0));
            WavFileName[0] = (WavFile_Count/1000) % 10 + '0';
            WavFileName[1] = (WavFile_Count/100) % 10 + '0';
            WavFileName[2] = (WavFile_Count/10) % 10 + '0';
            WavFileName[3] = (WavFile_Count) % 10 + '0';
            myFile = SD.open(WavFileName,FILE_WRITE);
            Serial.println(WavFileName);
            for(uint32_t i = 0; i<WavFile_length;i++)
            {
                myFile.write(WavFile_Data[i]);
            }
            myFile.close();
            
            FlowFileName[0] = (WavFile_Count/1000) % 10 + '0';
            FlowFileName[1] = (WavFile_Count/100) % 10 + '0';
            FlowFileName[2] = (WavFile_Count/10) % 10 + '0';
            FlowFileName[3] = (WavFile_Count) % 10 + '0';

            myFile = SD.open(FlowFileName,FILE_WRITE);
            Serial.println(FlowFileName);
            myFile.print("< SFM3000 Flow Sensor Raw Data >\n");
            for(uint16_t i = 0; i < Shared_Ptr->Flow_Data_Length; i++)
            {
                temp = String(Shared_Ptr->Flow_Data[i]);
                myFile.println(temp);
            }
            myFile.close();
            SPI.endTransaction();
            Serial.println("SD Card Record Finished");
        break;
        default : 
        break;
    }
}

void Read_WavFile_SdCard(void)
{
    char wavfile_name[] = "0001.wav";
    char flowfile_name[] = "0001.txt";
    Serial.println("/***************************************************/");
    Serial.println("SD Card File List : ");
    while (true) 
    {
        WavFile_Count++;
        wavfile_name[0] = (WavFile_Count/1000) % 10 + '0';
        wavfile_name[1] = (WavFile_Count/100) % 10 + '0';
        wavfile_name[2] = (WavFile_Count/10) % 10 + '0';
        wavfile_name[3] = (WavFile_Count) % 10 + '0';
        flowfile_name[0] = (WavFile_Count/1000) % 10 + '0';
        flowfile_name[1] = (WavFile_Count/100) % 10 + '0';
        flowfile_name[2] = (WavFile_Count/10) % 10 + '0';
        flowfile_name[3] = (WavFile_Count) % 10 + '0';
        if(!(SD.exists(wavfile_name)))
        {
            if(WavFile_Count == 1)
            {
                Serial.println("No File in SD Card");
            }
            Serial.println("/***************************************************/");
            break;
        }
        Serial.println(wavfile_name);
        Serial.println(flowfile_name);
    }

    if(WavFile_Count>0)
    {
        WavFile_Count = WavFile_Count-1;
    }
    
    myFile.close();
}

float LowPassFilter(float x_k,float y_kml,float Ts,float tau)
{
    float y_k;
    y_k = ((tau * y_kml) + (Ts * x_k))/(Ts + tau);
    return y_k;
}

void printWifiStatus() 
{
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

#endif // End all M7 core programming

/////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CORE_CM4 // Start M4 programming

#define SFM3000_I2C_ADDRESS         (0x40)

static struct shared_data * const Shared_Ptr = (struct shared_data *)SRAM3_START_ADDRESS;

void setup() 
{    
    uint8_t tx_buf[2] = {0x10,0x00};
    MX_I2C3_Init();
    delay(100);
    HAL_I2C_Master_Transmit(&hi2c3,SFM3000_I2C_ADDRESS<<1,tx_buf,2,10);
}

void loop()
{
    SFM3000_Read_Data();
    delay(SFM3000_SAMPLING_TIME);
}

void SFM3000_Read_Data(void)
{
    uint8_t rx_buf[3] = {0,};

    if(Shared_Ptr->M7_status == SOUND_RECORD_COPY)
    {
        HAL_I2C_Master_Receive(&hi2c3,SFM3000_I2C_ADDRESS<<1,rx_buf,3,1);
        Shared_Ptr->M4_Semapore = 1;
        #ifndef _TEST
        Shared_Ptr->Flow_Raw_Data = rx_buf[1] | (rx_buf[0] << 8);
        #endif
        Shared_Ptr->M4_Semapore = 0;
        if(Shared_Ptr->M7_status)
        {
            Shared_Ptr->Flow_Data[Shared_Ptr->Flow_Data_Length] = Shared_Ptr->Flow_Raw_Data;
            Shared_Ptr->Flow_Data_Length++;
        }    
    }
}

#endif

