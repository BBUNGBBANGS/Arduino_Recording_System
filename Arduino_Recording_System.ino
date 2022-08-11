#include "main.h"
#include "wave_header.h"
#include <stdint.h>

uint32_t Adc1Data;
uint8_t Adc_Isr_Flag;

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
