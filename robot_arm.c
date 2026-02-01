#include "stm32f3xx_hal.h"

// --- HASSASİYET VE HIZ AYARLARI ---
#define DEADZONE 180       
#define AVG_SAMPLES 64     
#define HYSTERESIS 15      
#define SERVO_SPEED 5      // S1, S2, S3 için hız ayarı (1: Yavaş, 20: Hızlı)

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;

// Mevcut konumları saklayan dizi (Sadece S1, S2, S3 için yavaşlatma kullanılacak)
uint32_t current_pulse[4] = {1500, 1500, 1500, 1500};
uint32_t last_pulse[4] = {1500, 1500, 1500, 1500};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
uint32_t ADC_Read_Filtered(uint32_t channel);
void Update_Servo_Slow(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t adc_val, uint8_t index);
void Update_Servo_Fast(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t adc_val, uint8_t index);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Servo 1 (Yavaş)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Servo 2 (Yavaş)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Servo 3 (Yavaş)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Servo 4 (Hızlı - Orijinal Hız)

    while (1) {
        // İlk 3 Servo Yavaşlatılmış Fonksiyonu Kullanıyor
        Update_Servo_Slow(&htim3, TIM_CHANNEL_2, ADC_Read_Filtered(ADC_CHANNEL_1), 0);
        Update_Servo_Slow(&htim3, TIM_CHANNEL_3, ADC_Read_Filtered(ADC_CHANNEL_2), 1);
        Update_Servo_Slow(&htim3, TIM_CHANNEL_1, ADC_Read_Filtered(ADC_CHANNEL_6), 2);
        
        // Servo 4 Orijinal Hızlı Fonksiyonu Kullanıyor
        Update_Servo_Fast(&htim2, TIM_CHANNEL_1, ADC_Read_Filtered(ADC_CHANNEL_7), 3);

        HAL_Delay(5); 
    }
}

// --- Yavaşlatılmış Servo Fonksiyonu (S1, S2, S3 için) ---
void Update_Servo_Slow(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t adc_val, uint8_t index) {
    uint32_t target_pulse;

    if (adc_val > (2048 - DEADZONE) && adc_val < (2048 + DEADZONE)) {
        target_pulse = 1500;
    } else {
        target_pulse = 500 + (adc_val * 2000 / 4095);
    }

    if (current_pulse[index] < target_pulse) {
        current_pulse[index] += SERVO_SPEED;
        if (current_pulse[index] > target_pulse) current_pulse[index] = target_pulse;
    } else if (current_pulse[index] > target_pulse) {
        if (current_pulse[index] > SERVO_SPEED) current_pulse[index] -= SERVO_SPEED;
        if (current_pulse[index] < target_pulse) current_pulse[index] = target_pulse;
    }

    int32_t diff = (int32_t)current_pulse[index] - (int32_t)last_pulse[index];
    if (diff > HYSTERESIS || diff < -HYSTERESIS) {
        __HAL_TIM_SET_COMPARE(htim, channel, current_pulse[index]);
        last_pulse[index] = current_pulse[index];
    }
}

// --- Hızlı Servo Fonksiyonu (S4 için - Orijinal kodun yapısı) ---
void Update_Servo_Fast(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t adc_val, uint8_t index) {
    uint32_t target_pulse;

    if (adc_val > (2048 - DEADZONE) && adc_val < (2048 + DEADZONE)) {
        target_pulse = 1500;
    } else {
        target_pulse = 500 + (adc_val * 2000 / 4095);
    }

    int32_t diff = (int32_t)target_pulse - (int32_t)last_pulse[index];
    if (diff > HYSTERESIS || diff < -HYSTERESIS) {
        __HAL_TIM_SET_COMPARE(htim, channel, target_pulse);
        last_pulse[index] = target_pulse;
        current_pulse[index] = target_pulse; // Senkronizasyon için
    }
}

uint32_t ADC_Read_Filtered(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    uint64_t total = 0;
    for(int i = 0; i < AVG_SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            total += HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }
    return (uint32_t)(total / AVG_SAMPLES);
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef G = {0};
    G.Mode = GPIO_MODE_AF_PP; G.Pull = GPIO_NOPULL; G.Speed = GPIO_SPEED_FREQ_HIGH;
    G.Pin = GPIO_PIN_5; G.Alternate = GPIO_AF1_TIM2; HAL_GPIO_Init(GPIOA, &G);
    G.Pin = GPIO_PIN_4; G.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOB, &G);
    G.Pin = GPIO_PIN_7; G.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOC, &G);
    G.Pin = GPIO_PIN_0; G.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOB, &G);
    G.Mode = GPIO_MODE_ANALOG;
    G.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4; HAL_GPIO_Init(GPIOA, &G);
    G.Pin = GPIO_PIN_0; HAL_GPIO_Init(GPIOC, &G);
}

static void MX_TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.Period = 19999;
    HAL_TIM_PWM_Init(&htim2);
    TIM_OC_InitTypeDef s = {0};
    s.OCMode = TIM_OCMODE_PWM1; s.OCPolarity = TIM_OCPOLARITY_HIGH; s.Pulse = 1500;
    HAL_TIM_PWM_ConfigChannel(&htim2, &s, TIM_CHANNEL_1);
}

static void MX_TIM3_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.Period = 19999;
    HAL_TIM_PWM_Init(&htim3);
    TIM_OC_InitTypeDef s = {0};
    s.OCMode = TIM_OCMODE_PWM1; s.OCPolarity = TIM_OCPOLARITY_HIGH; s.Pulse = 1500;
    HAL_TIM_PWM_ConfigChannel(&htim3, &s, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &s, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim3, &s, TIM_CHANNEL_3);
}

static void MX_ADC1_Init(void) {
    __HAL_RCC_ADC12_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    HAL_ADC_Init(&hadc1);
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef o = {0}; RCC_ClkInitTypeDef c = {0};
    o.OscillatorType = RCC_OSCILLATORTYPE_HSI; o.HSIState = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState = RCC_PLL_ON; o.PLL.PLLSource = RCC_PLLSOURCE_HSI; o.PLL.PLLMUL = RCC_PLL_MUL12;
    HAL_RCC_OscConfig(&o);
    c.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1; c.APB1CLKDivider = RCC_HCLK_DIV2; c.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2);
}

void SysTick_Handler(void) { HAL_IncTick(); }
