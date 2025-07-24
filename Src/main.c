/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/*2025年2月119日10:36:20  V1.0.1  开机前2秒，不检测火焰信号， 火焰丢失0.5s后，再给输出信号*/

/*2025年3月31日17:05:11 V1.0.2,修改火焰判定的值，由2500改为2680，熄火判定由0.5秒，改为1.5秒*/

/*2025年7月10日 V1.1 实现自适应火焰检测算法，自动学习环境基线，动态调整检测阈值*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tm1650.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 自适应火焰检测参数定义
#define BASELINE_CALIBRATION_SAMPLES    50      // 基线校准采样数
#define BASELINE_MIN_MV                1000     // 最小基线值(mV) - 提高以检测传感器连接
#define BASELINE_MAX_MV               3000      // 最大基线值(mV)
#define DYNAMIC_THRESHOLD_OFFSET       300      // 动态阈值偏移(mV)
#define GRADIENT_THRESHOLD              50      // 梯度变化阈值(mV/采样)
#define RELATIVE_CHANGE_THRESHOLD       50      // 相对变化阈值(%)
#define CONFIRMATION_COUNT               3      // 确认计数要求
#define BASELINE_UPDATE_INTERVAL      5000      // 基线更新间隔(ms)
#define FLAME_OFF_DELAY_MS            1500      // 火焰熄灭延迟时间(ms)
#define STARTUP_DELAY_MS              2000      // 开机延迟时间(ms)
#define ADC_BUFFER_SIZE                128      // ADC采样缓冲区大小
#define ADC_REFERENCE_VOLTAGE_MV      3250      // ADC参考电压(mV)
#define ADC_RESOLUTION                4095      // ADC分辨率(12位)
#define SENSOR_DISCONNECT_THRESHOLD    800      // 传感器断开检测阈值(mV)

// 新增：智能初始状态检测参数
#define EXPECTED_NO_FLAME_MIN_MV      2200      // 预期无火焰最小值(mV)
#define EXPECTED_NO_FLAME_MAX_MV      2800      // 预期无火焰最大值(mV)
#define INITIAL_CHECK_SAMPLES          10      // 初始状态检查采样数
#define CALIBRATION_RETRY_DELAY      3000      // 校准重试延迟(ms)
#define MAX_CALIBRATION_RETRIES         5      // 最大校准重试次数

// GPIO控制宏定义 
#define LED_ON()                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)  // 低电平点亮
#define LED_OFF()                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)    // 高电平熄灭
#define RELAY_LED_ON()              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)  //    高电平熄灭
#define RELAY_LED_OFF()             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)    // 低电平点亮

// 火焰状态枚举
typedef enum {
    FLAME_NOT_DETECTED = 0,     // 未检测到火焰
    FLAME_DETECTED = 1,         // 检测到火焰  
    SENSOR_DISCONNECTED = 2,    // 传感器断开
    CALIBRATION_ERROR = 3       // 校准错误（上电时有火焰）
} FlameStatus_t;

// 自适应检测状态
typedef struct {
    uint32_t baseline;                  // 环境基线值(mV)
    uint32_t dynamicThreshold;          // 动态检测阈值(mV)
    uint32_t previousValue;             // 上次测量值(mV)
    uint8_t  confirmationCounter;       // 确认计数器
    uint8_t  isCalibrated;              // 是否已校准标志
    uint32_t lastBaselineUpdate;        // 上次基线更新时间
    uint32_t calibrationSampleCount;    // 校准采样计数
    uint32_t calibrationSum;            // 校准累积和
    uint8_t  sensorConnected;           // 传感器连接状态
    uint8_t  calibrationRetryCount;     // 校准重试计数
    uint32_t lastCalibrationRetry;      // 上次校准重试时间
    uint8_t  initialStateValid;         // 初始状态是否有效
} AdaptiveDetector_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

// 全局变量
uint32_t Value = 0;                     // 当前ADC转换值(mV)
static uint16_t FlameOffCount = 0;      // 火焰熄灭计数
static uint16_t FirstStartCount = 0;    // 启动计数
static uint32_t lastDisplayTime = 0;    // 上次显示时间
static AdaptiveDetector_t detector = {0}; // 自适应检测器
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_IWDG_Init(void);
static FlameStatus_t getFlameStatus(void);
static void handleFlameControl(FlameStatus_t flameStatus);
static void setOutputState(uint8_t state);
static void initializeAdaptiveDetector(void);
static void updateBaseline(uint32_t currentValue);
static uint8_t performMultiCriteriaDetection(uint32_t currentValue);
uint32_t adc_convert_single(void);
// 新增：智能初始状态检测函数
static uint8_t checkInitialState(void);
static uint8_t performInitialStateCheck(void);
static void displayCalibrationStatus(uint8_t isValid);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief 初始化自适应检测器
 */
static void initializeAdaptiveDetector(void)
{
    detector.baseline = 0;
    detector.dynamicThreshold = 0;
    detector.previousValue = 0;
    detector.confirmationCounter = 0;
    detector.isCalibrated = 0;
    detector.lastBaselineUpdate = 0;
    detector.calibrationSampleCount = 0;
    detector.calibrationSum = 0;
    detector.sensorConnected = 0; // 初始化传感器连接状态
    detector.calibrationRetryCount = 0;
    detector.lastCalibrationRetry = 0;
    detector.initialStateValid = 0;
}

/**
 * @brief 智能初始状态检测
 * @retval 1-初始状态有效, 0-初始状态无效(可能有火焰)
 */
static uint8_t checkInitialState(void)
{
    uint32_t sum = 0;
    uint32_t averageValue = 0;
    
    // 进行初始状态检查采样
    for(int i = 0; i < INITIAL_CHECK_SAMPLES; i++) {
        sum += adc_convert_single();
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(10); // 短暂延时确保采样稳定
    }
    
    // 计算平均值
    averageValue = (sum * ADC_REFERENCE_VOLTAGE_MV) / (ADC_RESOLUTION * INITIAL_CHECK_SAMPLES);
    
    // 检查传感器连接状态
    if(averageValue < SENSOR_DISCONNECT_THRESHOLD) {
        detector.sensorConnected = 0;
        return 0; // 传感器断开
    }
    
    detector.sensorConnected = 1;
    
    // 检查是否在预期的无火焰范围内
    if(averageValue >= EXPECTED_NO_FLAME_MIN_MV && averageValue <= EXPECTED_NO_FLAME_MAX_MV) {
        detector.initialStateValid = 1;
        return 1; // 初始状态有效
    }
    
    // 如果ADC值过低，可能有火焰
    if(averageValue < EXPECTED_NO_FLAME_MIN_MV) {
        detector.initialStateValid = 0;
        return 0; // 可能有火焰
    }
    
    // 如果ADC值过高，可能传感器异常，但仍可尝试校准
    detector.initialStateValid = 1;
    return 1;
}

/**
 * @brief 执行初始状态检查和校准重试逻辑
 * @retval 1-可以进行校准, 0-需要等待或重试
 */
static uint8_t performInitialStateCheck(void)
{
    uint32_t currentTime = HAL_GetTick();
    
    // 如果已经通过初始状态检查
    if(detector.initialStateValid) {
        return 1;
    }
    
    // 检查是否需要重试
    if(currentTime - detector.lastCalibrationRetry > CALIBRATION_RETRY_DELAY) {
        detector.calibrationRetryCount++;
        detector.lastCalibrationRetry = currentTime;
        
        // 进行初始状态检查
        if(checkInitialState()) {
            return 1; // 可以开始校准
        }
        
        // 如果重试次数超过限制，使用安全基线
        if(detector.calibrationRetryCount >= MAX_CALIBRATION_RETRIES) {
            // 使用安全的固定基线值
            detector.baseline = (EXPECTED_NO_FLAME_MIN_MV + EXPECTED_NO_FLAME_MAX_MV) / 2;
            detector.dynamicThreshold = detector.baseline - DYNAMIC_THRESHOLD_OFFSET;
            detector.isCalibrated = 1;
            detector.initialStateValid = 1;
            detector.lastBaselineUpdate = currentTime;
            return 1;
        }
    }
    
    return 0; // 需要继续等待
}

/**
 * @brief 显示校准状态
 * @param isValid: 校准状态是否有效
 */
static void displayCalibrationStatus(uint8_t isValid)
{
    static uint32_t lastFlashTime = 0;
    static uint8_t flashState = 0;
    uint32_t currentTime = HAL_GetTick();
    
    if(isValid) {
        // 正常显示当前值
        dis_value(Value);
    } else {
        // 闪烁显示错误状态
        if(currentTime - lastFlashTime > 500) { // 500ms闪烁间隔
            flashState = !flashState;
            lastFlashTime = currentTime;
            
            if(flashState) {
                dis_value(8888); // 显示错误代码
            } else {
                dis_value(0); // 空显示
            }
        }
    }
}

/**
 * @brief 更新环境基线（无火焰时）
 * @param currentValue 当前测量值
 */
static void updateBaseline(uint32_t currentValue)
{
    uint32_t currentTime = HAL_GetTick();
    
    // 定期更新基线（仅在无火焰状态下）
    if(currentTime - detector.lastBaselineUpdate > BASELINE_UPDATE_INTERVAL) {
        // 简单的基线更新：当前值与基线差异不大时更新
        if(detector.isCalibrated && detector.sensorConnected) {
            uint32_t diff = (currentValue > detector.baseline) ? 
                           (currentValue - detector.baseline) : 
                           (detector.baseline - currentValue);
            
            // 如果变化不大，说明环境稳定，可以更新基线
            if(diff < DYNAMIC_THRESHOLD_OFFSET / 2) {
                detector.baseline = (detector.baseline * 7 + currentValue) / 8; // 慢速更新
                
                // 限制基线范围
                if(detector.baseline < BASELINE_MIN_MV) detector.baseline = BASELINE_MIN_MV;
                if(detector.baseline > BASELINE_MAX_MV) detector.baseline = BASELINE_MAX_MV;
                
                // 更新动态阈值
                detector.dynamicThreshold = detector.baseline - DYNAMIC_THRESHOLD_OFFSET;
            }
        }
        detector.lastBaselineUpdate = currentTime;
    }
}

/**
 * @brief 执行多条件火焰检测
 * @param currentValue 当前测量值
 * @retval 1-检测到火焰, 0-未检测到火焰
 */
static uint8_t performMultiCriteriaDetection(uint32_t currentValue)
{
    uint8_t detectionResult = 0;
    
    // 只有在传感器连接时才进行检测
    if(!detector.sensorConnected) {
        return 0;
    }
    
    // 条件1: 阈值检测
    if(currentValue < detector.dynamicThreshold) {
        detectionResult = 1;
    }
    
    // 条件2: 梯度检测（大幅下降）
    // 只有在已校准且有有效previousValue时才进行梯度检测
    if(detector.isCalibrated && detector.previousValue > 0 && 
       detector.previousValue > SENSOR_DISCONNECT_THRESHOLD) {
        int32_t gradient = (int32_t)detector.previousValue - (int32_t)currentValue;
        if(gradient > GRADIENT_THRESHOLD) {
            detectionResult = 1;
        }
    }
    
    // 条件3: 相对变化检测（只检测下降，不检测上升）
    if(detector.baseline > 0 && currentValue < detector.baseline) {
        uint32_t relativeChange = ((detector.baseline - currentValue) * 100) / detector.baseline;
        if(relativeChange > RELATIVE_CHANGE_THRESHOLD) {
            detectionResult = 1;
        }
    }
    
    // 防误触发：需要连续确认
    if(detectionResult) {
        detector.confirmationCounter++;
        if(detector.confirmationCounter >= CONFIRMATION_COUNT) {
            detector.confirmationCounter = CONFIRMATION_COUNT; // 防止溢出
            return 1; // 确认检测到火焰
        }
    } else {
        detector.confirmationCounter = 0; // 重置确认计数
    }
    
    return 0; // 未确认火焰
}

/**
 * @brief ADC单次转换函数
 * @retval ADC转换值
 */
uint32_t adc_convert_single(void)
{
    uint32_t adcValue;
    
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    adcValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return adcValue;
}

/**
 * @brief 获取火焰检测状态（自适应算法）
 * @retval 火焰状态
 */
static FlameStatus_t getFlameStatus(void)
{
    uint32_t currentTime = HAL_GetTick();
    
    // ADC采样并计算平均值
    uint32_t sum = 0;
    for(int i = 0; i < ADC_BUFFER_SIZE; i++) 
    {
        sum += adc_convert_single();
        HAL_IWDG_Refresh(&hiwdg);
        
        // 优化延时：每4次采样延时1ms
        if((i & 0x03) == 0x03) {
            HAL_Delay(1);
        }
    }
    
    // 使用整数运算计算电压值
    Value = (sum * ADC_REFERENCE_VOLTAGE_MV) / (ADC_RESOLUTION * ADC_BUFFER_SIZE);
    
    // 传感器断开检测
    if(Value < SENSOR_DISCONNECT_THRESHOLD) {
        detector.sensorConnected = 0; // 断开
        
        // 显示断开状态（显示---或000）
        if(currentTime - lastDisplayTime >= 100) {
            dis_value(0); // 显示000表示传感器断开
            lastDisplayTime = currentTime;
        }
        
        return SENSOR_DISCONNECTED;
    } else {
        detector.sensorConnected = 1; // 连接
    }
    
    // 基线校准阶段
    if(!detector.isCalibrated) {
        // 执行智能初始状态检查
        if(!performInitialStateCheck()) {
            // 初始状态检查失败，显示错误状态
            if(currentTime - lastDisplayTime >= 100) {
                displayCalibrationStatus(0); // 显示错误状态
                lastDisplayTime = currentTime;
            }
            return CALIBRATION_ERROR; // 返回校准错误状态
        }
        
        detector.calibrationSum += Value;
        detector.calibrationSampleCount++;
        
        if(detector.calibrationSampleCount >= BASELINE_CALIBRATION_SAMPLES) {
            // 计算基线
            detector.baseline = detector.calibrationSum / detector.calibrationSampleCount;
            
            // 限制基线范围
            if(detector.baseline < BASELINE_MIN_MV) detector.baseline = BASELINE_MIN_MV;
            if(detector.baseline > BASELINE_MAX_MV) detector.baseline = BASELINE_MAX_MV;
            
            // 设置动态阈值
            detector.dynamicThreshold = detector.baseline - DYNAMIC_THRESHOLD_OFFSET;
            
            // 安全检查：确保动态阈值不会太低
            if(detector.dynamicThreshold < SENSOR_DISCONNECT_THRESHOLD) {
                detector.dynamicThreshold = SENSOR_DISCONNECT_THRESHOLD + 100;
            }
            
            detector.isCalibrated = 1;
            detector.lastBaselineUpdate = currentTime;
        }
        
        // 校准期间显示当前值，不进行火焰检测
        if(currentTime - lastDisplayTime >= 100) {
            displayCalibrationStatus(1); // 显示正常校准状态
            lastDisplayTime = currentTime;
        }
        
        // 更新previousValue，为校准完成后的检测做准备
        detector.previousValue = Value;
        
        return FLAME_NOT_DETECTED; // 校准期间不检测火焰
    }
    
    // 自适应检测
    uint8_t flameDetected = performMultiCriteriaDetection(Value);
    
    // 更新基线（仅在无火焰时）
    if(!flameDetected) {
        updateBaseline(Value);
    }
    
    // 更新显示
    if(currentTime - lastDisplayTime >= 100) {
        // 显示当前值，每5秒短暂显示基线值
        static uint32_t baselineDisplayTime = 0;
        if(currentTime - baselineDisplayTime > 5000) {
            dis_value(detector.baseline); // 显示基线1秒
            baselineDisplayTime = currentTime;
        } else if(currentTime - baselineDisplayTime > 1000) {
            dis_value(Value); // 正常显示当前值
        }
        lastDisplayTime = currentTime;
    }
    
    // 更新上次测量值
    detector.previousValue = Value;
    
    return flameDetected ? FLAME_DETECTED : FLAME_NOT_DETECTED;
}

/**
 * @brief 设置输出状态
 * @param state: 1-有火焰输出, 0-无火焰输出
 */
static void setOutputState(uint8_t state)
{
    if(state) {
        LED_ON();        // 火焰指示LED点亮
        RELAY_LED_ON();  // 继电器工作指示LED点亮
    } else {
        LED_OFF();       // 火焰指示LED熄灭
        RELAY_LED_OFF(); // 继电器工作指示LED熄灭
    }
}

/**
 * @brief 处理火焰控制逻辑
 * @param flameStatus: 火焰检测状态
 */
static void handleFlameControl(FlameStatus_t flameStatus)
{
    if(flameStatus == FLAME_DETECTED) {
        // 检测到火焰
        FlameOffCount = 0;
        
        // 开机延迟期间不输出
        if(FirstStartCount >= STARTUP_DELAY_MS) {
            setOutputState(1);  // 有火焰输出
        }
    } else if(flameStatus == SENSOR_DISCONNECTED) {
        // 传感器断开 - 安全起见关闭输出
        setOutputState(0); // 无火焰输出
        FlameOffCount = 0; // 重置计数
    } else if(flameStatus == CALIBRATION_ERROR) {
        // 校准错误（上电时有火焰）- 安全起见关闭输出
        setOutputState(0); // 无火焰输出
        FlameOffCount = 0; // 重置计数
    } else {
        // 未检测到火焰
        if(FlameOffCount > FLAME_OFF_DELAY_MS) {
            setOutputState(0);  // 无火焰输出
        }
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
	
	FlameOffCount++;
	
	FirstStartCount++;
	
	if(FirstStartCount > STARTUP_DELAY_MS) // 启动延迟限制
			FirstStartCount = STARTUP_DELAY_MS;

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_ADC1_Init();
  tm1650_gpio_init();
  MX_IWDG_Init();
  
  // 初始化自适应检测器
  initializeAdaptiveDetector();
  
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_IWDG_Refresh(&hiwdg);
		
		// 获取火焰检测状态（自适应算法）
		FlameStatus_t flameStatus = getFlameStatus();
	 
		// 处理火焰控制逻辑
		handleFlameControl(flameStatus);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
