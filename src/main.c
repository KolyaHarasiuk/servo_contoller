/**
 * Servo Controller with CRSF for STM32G431CBU6
 * Controls 3 servos and 3 GPIO outputs based on CRSF channel 12
 */

#include "stm32g4xx_hal.h"
#include <string.h>

/* Pin definitions */
// Inputs
#define SAFETY_PIN      GPIO_PIN_0  // PA0 - Safety switch

// Outputs
#define OUTPUT0_PIN     GPIO_PIN_4  // PB4
#define OUTPUT1_PIN     GPIO_PIN_6  // PB6
#define OUTPUT2_PIN     GPIO_PIN_8  // PB8
#define SAFETY_OUT_PIN  GPIO_PIN_11 // PC11 - Safety mirror

// CRSF on PA3 (USART2_RX)

/* Servo PWM values */
#define SERVO_PERIOD    20000  // 20ms period (50Hz)
#define SERVO_MIN       1000   // 1ms - servo open
#define SERVO_MAX       2000   // 2ms - servo closed

/* CRSF Protocol definitions */
#define CRSF_BAUDRATE   420000
#define CRSF_MAX_PACKET_SIZE 64
#define CRSF_SYNC_BYTE  0xC8
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_PAYLOAD_SIZE_MAX 60

// CRSF frame types
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

// CRSF channel values
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MAX 1811
#define CRSF_CHANNEL_MID 992

/* Handle declarations */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* CRSF variables */
uint8_t crsf_rx_buffer[CRSF_MAX_PACKET_SIZE];
uint8_t crsf_rx_index = 0;
uint16_t crsf_channels[16];
uint32_t crsf_last_packet_time = 0;
uint8_t crsf_rx_byte;

/* Function prototypes */
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM1_Init(void);
void TIM3_Init(void);
void USART2_Init(void);
void Error_Handler(void);
void UpdateOutputs(void);
void ProcessCRSFPacket(void);
uint8_t CalculateCRC8(const uint8_t *data, uint8_t len);
void ParseCRSFChannels(const uint8_t *data);

int main(void)
{
    /* Reset peripherals, Initialize Flash and Systick */
    HAL_Init();
    
    /* Configure system clock */
    SystemClock_Config();
    
    /* Initialize GPIO */
    GPIO_Init();
    
    /* Initialize Timers for PWM */
    TIM1_Init();
    TIM3_Init();
    
    /* Initialize USART for CRSF */
    USART2_Init();
    
    /* Start PWM channels */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PC6 - Servo 0
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // PA9 - Servo 1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // PA11 - Servo 2
    
    /* Start UART reception with interrupt */
    HAL_UART_Receive_IT(&huart2, &crsf_rx_byte, 1);
    
    /* Initialize channels to center position */
    for (int i = 0; i < 16; i++) {
        crsf_channels[i] = CRSF_CHANNEL_MID;
    }
    
    /* Main loop */
    while (1)
    {
        UpdateOutputs();
        
       
        
        HAL_Delay(5);  // 5ms update rate
    }
}

void UpdateOutputs(void)
{
    /* Read safety input */
    uint8_t safety = HAL_GPIO_ReadPin(GPIOA, SAFETY_PIN);
    
    /* Mirror safety to PC11 */
    HAL_GPIO_WritePin(GPIOC, SAFETY_OUT_PIN, safety);
    
    /* Get channel 12 value (index 11) */
    uint16_t ch12 = crsf_channels[11];
    
    /* Servo 0 logic: open when ch12 = 172-992 */
    uint8_t servo0_open = (ch12 <= 992);
    if (servo0_open) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MIN);  // 1ms - open
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MAX);  // 2ms - closed
    }
    
    /* Servo 1 logic: open when ch12 = 337-1320 */
    uint8_t servo1_open = (ch12 >= 337 && ch12 <= 1320);
    if (servo1_open) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SERVO_MIN);  // 1ms - open
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SERVO_MAX);  // 2ms - closed
    }
    
    /* Servo 2 logic: open when ch12 = 665-1648 */
    uint8_t servo2_open = (ch12 >= 665 && ch12 <= 1648);
    if (servo2_open) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_MIN);  // 1ms - open
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_MAX);  // 2ms - closed
    }
    
    /* Update GPIO outputs based on safety switch */
    if (safety) {
        /* Safety OFF - outputs follow servo states */
        HAL_GPIO_WritePin(GPIOB, OUTPUT0_PIN, servo0_open);
        HAL_GPIO_WritePin(GPIOB, OUTPUT1_PIN, servo1_open);
        HAL_GPIO_WritePin(GPIOB, OUTPUT2_PIN, servo2_open);
    } else {
        /* Safety ON - all outputs LOW */
        HAL_GPIO_WritePin(GPIOB, OUTPUT0_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, OUTPUT1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, OUTPUT2_PIN, GPIO_PIN_RESET);
    }
}

/* UART interrupt handler */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        /* Add byte to buffer */
        if (crsf_rx_index == 0 && crsf_rx_byte != CRSF_SYNC_BYTE) {
            /* Wait for sync byte */
            HAL_UART_Receive_IT(&huart2, &crsf_rx_byte, 1);
            return;
        }
        
        crsf_rx_buffer[crsf_rx_index++] = crsf_rx_byte;
        
        /* Check if we have enough bytes for a header */
        if (crsf_rx_index >= 3) {
            uint8_t frame_size = crsf_rx_buffer[1];
            uint8_t total_size = frame_size + 2; // sync + size + payload + crc
            
            if (crsf_rx_index >= total_size) {
                /* We have a complete packet */
                ProcessCRSFPacket();
                crsf_rx_index = 0;
            } else if (crsf_rx_index >= CRSF_MAX_PACKET_SIZE) {
                /* Buffer overflow, reset */
                crsf_rx_index = 0;
            }
        }
        
        /* Continue receiving */
        HAL_UART_Receive_IT(&huart2, &crsf_rx_byte, 1);
    }
}

void ProcessCRSFPacket(void)
{
    uint8_t frame_size = crsf_rx_buffer[1];
    uint8_t frame_type = crsf_rx_buffer[2];
    
    /* Calculate CRC */
    uint8_t calculated_crc = CalculateCRC8(&crsf_rx_buffer[2], frame_size - 1);
    uint8_t received_crc = crsf_rx_buffer[frame_size + 1];
    
    if (calculated_crc != received_crc) {
        return; // CRC error
    }
    
    /* Process RC channels packet */
    if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        ParseCRSFChannels(&crsf_rx_buffer[3]);
        crsf_last_packet_time = HAL_GetTick();
    }
}

void ParseCRSFChannels(const uint8_t *data)
{
    /* CRSF channels are packed 11 bits each */
    crsf_channels[0]  = ((data[0]    | data[1] << 8) & 0x07FF);
    crsf_channels[1]  = ((data[1]>>3 | data[2] << 5) & 0x07FF);
    crsf_channels[2]  = ((data[2]>>6 | data[3] << 2 | data[4] << 10) & 0x07FF);
    crsf_channels[3]  = ((data[4]>>1 | data[5] << 7) & 0x07FF);
    crsf_channels[4]  = ((data[5]>>4 | data[6] << 4) & 0x07FF);
    crsf_channels[5]  = ((data[6]>>7 | data[7] << 1 | data[8] << 9) & 0x07FF);
    crsf_channels[6]  = ((data[8]>>2 | data[9] << 6) & 0x07FF);
    crsf_channels[7]  = ((data[9]>>5 | data[10] << 3) & 0x07FF);
    crsf_channels[8]  = ((data[11]   | data[12] << 8) & 0x07FF);
    crsf_channels[9]  = ((data[12]>>3| data[13] << 5) & 0x07FF);
    crsf_channels[10] = ((data[13]>>6| data[14] << 2 | data[15] << 10) & 0x07FF);
    crsf_channels[11] = ((data[15]>>1| data[16] << 7) & 0x07FF);
    crsf_channels[12] = ((data[16]>>4| data[17] << 4) & 0x07FF);
    crsf_channels[13] = ((data[17]>>7| data[18] << 1 | data[19] << 9) & 0x07FF);
    crsf_channels[14] = ((data[19]>>2| data[20] << 6) & 0x07FF);
    crsf_channels[15] = ((data[20]>>5| data[21] << 3) & 0x07FF);
}

uint8_t CalculateCRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure safety input pin (PA0) */
    GPIO_InitStruct.Pin = SAFETY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure output pins (PB4, PB6, PB8) */
    HAL_GPIO_WritePin(GPIOB, OUTPUT0_PIN | OUTPUT1_PIN | OUTPUT2_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = OUTPUT0_PIN | OUTPUT1_PIN | OUTPUT2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Configure safety mirror output (PC11) */
    HAL_GPIO_WritePin(GPIOC, SAFETY_OUT_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = SAFETY_OUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    /* Configure PWM pins */
    // PC6 - TIM3_CH1
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // PA9 - TIM1_CH2
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // PA11 - TIM1_CH4
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF11_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure USART2_RX pin (PA3) */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    
    huart2.Instance = USART2;
    huart2.Init.BaudRate = CRSF_BAUDRATE;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    
    /* Enable USART2 interrupt */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    /* Assuming 170MHz system clock
     * Prescaler = 170-1 gives 1MHz timer clock
     * Period = 20000-1 gives 50Hz PWM (20ms) */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 170 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = SERVO_PERIOD - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);
    
    /* Configure Channel 2 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = SERVO_MAX;  // Start with servo closed
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    
    /* Configure Channel 4 */
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
}

void TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    /* Assuming 170MHz system clock
     * Prescaler = 170-1 gives 1MHz timer clock
     * Period = 20000-1 gives 50Hz PWM (20ms) */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 170 - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = SERVO_PERIOD - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);
    
    /* Configure Channel 1 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = SERVO_MAX;  // Start with servo closed
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    /* Configure voltage scaling */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    
    /* Configure HSI oscillator and PLL for 170MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    /* Configure system clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    /* Error occurred, stop here */
    __disable_irq();
    while (1) {
    }
}

/* Required HAL callbacks */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}