#include "main.h"
#include "wave_header.h"
#include <stdint.h>

uint32_t Adc1Data;
uint8_t Adc_Isr_Flag;

Wave_Header_t WavFile;
uint32_t WavFile_length;
uint32_t Sound_Data_Length;
uint16_t Sound_Data[500000];
uint8_t WavFile_Data[100000];
void setup() 
{
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();

    HAL_ADC_Start_IT(&hadc1);
    HAL_TIM_Base_Start(&htim6);

    Serial.begin(115200);
}


void loop() 
{
    Create_WaveFile_Header();
    Create_WaveFile();
    delay(50);
    Serial.print("ADC DMA Value : ");
    Serial.println(Adc1Data);
}

void Adc_Sampling(void)
{
    static uint8_t state;
    Adc1Data = HAL_ADC_GetValue(&hadc1);
    state = (~state)&0x01;
    if(state == 1)
    {
         HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, GPIO_PIN_RESET);  
    }
    else
    {
        HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, GPIO_PIN_SET);  
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
    WavFile_length = Sound_Data_Length + 36;
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
    WavFile.Data.ChunkSize = Sound_Data_Length;
}

void Create_WaveFile(void)
{
    uint8_t *Ptr = NULL;
    Ptr = &WavFile.Riff.ChunkID[0];
    for(uint8_t i = 0; i<WAVFILE_HEADER_LENGTH; i++)
    {
        WavFile_Data[i] = Ptr[i];
    }

    for(uint32_t j = WAVFILE_HEADER_LENGTH;j < (Sound_Data_Length + WAVFILE_HEADER_LENGTH);j++)
    {
        WavFile_Data[j] = 
    }

}

uint32_t Convert_Little_Endian_32bit(uint32_t data)
{
    uint8_t data1,data2,data3,data4;
    uint32_t retval = 0;

    data1 = (uint8_t)(data & 0x000000FF);
    data2 = (uint8_t)((data & 0x0000FF00)>>8);
    data3 = (uint8_t)((data & 0x00FF0000)>>16);
    data4 = (uint8_t)((data & 0xFF000000)>>24);

    retval = (uint32_t)((data1 << 24) | (data2 << 16) | (data3 << 8) | (data4));

    return (retval);
}

uint16_t Convert_Little_Endian_16bit(uint16_t data)
{
    uint8_t data1,data2;
    uint16_t retval = 0;

    data1 = (uint8_t)(data & 0x000000FF);
    data2 = (uint8_t)((data & 0x0000FF00)>>8);

    retval = (uint16_t)((data1 << 24) | (data2 << 16));

    return (retval);
}