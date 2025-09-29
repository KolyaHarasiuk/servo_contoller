/**
 * Servo Controller for STM32G431CBU6
 * Controls 3 servos and 3 GPIO outputs based on 4 GPIO inputs
 */

#include "stm32g4xx_hal.h"

/* Pin definitions */
// Inputs
#define SAFETY_PIN      GPIO_PIN_0  // PA0 - Safety switch
#define INPUT1_PIN      GPIO_PIN_2  // PA2 - Control servo 0
#define INPUT2_PIN      GPIO_PIN_4  // PA4 - Control servo 1  
#define INPUT3_PIN      GPIO_PIN_6  // PA6 - Control servo 2

// Outputs
#define OUTPUT0_PIN     GPIO_PIN_4  // PB4
#define OUTPUT1_PIN     GPIO_PIN_6  // PB6
#define OUTPUT2_PIN     GPIO_PIN_8  // PB8
#define SAFETY_OUT_PIN  GPIO_PIN_11 // PC11 - Safety mirror

// Servo PWM pins
// PC6 - TIM3_CH1 (Servo 0)
// PA9 - TIM1_CH2 (Servo 1)
// PA11 - TIM1_CH4 (Servo 2)

/* Servo PWM values */
#define SERVO_PERIOD    20000  // 20ms period (50Hz)
#define SERVO_MIN       1000   // 1ms - servo open
#define SERVO_MAX       2000   // 2ms - servo closed

/* Handle declarations */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* Function prototypes */
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM1_Init(void);
void TIM3_Init(void);
void Error_Handler(void);
void UpdateOutputs(void);

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
    
    /* Start PWM channels */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PC6 - Servo 0
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // PA9 - Servo 1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // PA11 - Servo 2
    
    /* Main loop */
    while (1)
    {
        UpdateOutputs();
        HAL_Delay(10);  // 10ms update rate
    }
}

void UpdateOutputs(void)
{
    /* Read inputs */
    uint8_t safety = HAL_GPIO_ReadPin(GPIOA, SAFETY_PIN);
    uint8_t input1 = HAL_GPIO_ReadPin(GPIOA, INPUT1_PIN);
    uint8_t input2 = HAL_GPIO_ReadPin(GPIOA, INPUT2_PIN);
    uint8_t input3 = HAL_GPIO_ReadPin(GPIOA, INPUT3_PIN);
    
    /* Mirror safety to PC11 */
    HAL_GPIO_WritePin(GPIOC, SAFETY_OUT_PIN, safety);
    
    /* Update Servo 0 (PC6 - TIM3_CH1) */
    if (input1) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MIN);  // 1ms - open
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MAX);  // 2ms - closed
    }
    
    /* Update Servo 1 (PA9 - TIM1_CH2) */
    if (input2) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SERVO_MIN);  // 1ms - open
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SERVO_MAX);  // 2ms - closed
    }
    
    /* Update Servo 2 (PA11 - TIM1_CH4) */
    if (input3) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_MIN);  // 1ms - open
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_MAX);  // 2ms - closed
    }
    
    /* Update GPIO outputs based on safety switch */
    if (safety) {
        /* Safety ON - outputs follow inputs */
        HAL_GPIO_WritePin(GPIOB, OUTPUT0_PIN, input1);
        HAL_GPIO_WritePin(GPIOB, OUTPUT1_PIN, input2);
        HAL_GPIO_WritePin(GPIOB, OUTPUT2_PIN, input3);
    } else {
        /* Safety OFF - all outputs LOW */
        HAL_GPIO_WritePin(GPIOB, OUTPUT0_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, OUTPUT1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, OUTPUT2_PIN, GPIO_PIN_RESET);
    }
}

void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure input pins (PA0, PA2, PA4, PA6) */
    GPIO_InitStruct.Pin = SAFETY_PIN | INPUT1_PIN | INPUT2_PIN | INPUT3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // Pull-down for defined state
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Configure system clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
    {
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