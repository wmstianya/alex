# 多LED控制逻辑建议

## 🎯 建议的三LED设计

### LED功能分配
- **LED1 (电源指示灯)** - 绿色，通电后常亮
- **LED2 (运行指示灯)** - 蓝色，正常运行时闪烁，故障时常亮
- **LED3 (火焰指示灯)** - 红色，检测到火焰时点亮

### GPIO引脚建议
```c
// 建议的GPIO引脚分配
#define LED1_PIN     GPIO_PIN_8   // PA8 - 电源指示灯
#define LED2_PIN     GPIO_PIN_9   // PA9 - 运行指示灯  
#define LED3_PIN     GPIO_PIN_10  // PA10 - 火焰指示灯（当前已有）

// LED控制宏（假设都是低电平点亮）
#define LED1_ON()    HAL_GPIO_WritePin(GPIOA, LED1_PIN, GPIO_PIN_RESET)
#define LED1_OFF()   HAL_GPIO_WritePin(GPIOA, LED1_PIN, GPIO_PIN_SET)
#define LED2_ON()    HAL_GPIO_WritePin(GPIOA, LED2_PIN, GPIO_PIN_RESET)
#define LED2_OFF()   HAL_GPIO_WritePin(GPIOA, LED2_PIN, GPIO_PIN_SET)
#define LED3_ON()    HAL_GPIO_WritePin(GPIOA, LED3_PIN, GPIO_PIN_RESET)
#define LED3_OFF()   HAL_GPIO_WritePin(GPIOA, LED3_PIN, GPIO_PIN_SET)
```

### 控制逻辑
```c
typedef enum {
    LED_STATUS_POWER_ON = 0,      // 电源指示
    LED_STATUS_RUNNING,           // 运行指示
    LED_STATUS_FLAME_DETECTED,    // 火焰检测
    LED_STATUS_SENSOR_ERROR,      // 传感器故障
    LED_STATUS_CALIBRATING        // 校准状态
} LEDStatus_t;

void ledControl(LEDStatus_t status) {
    switch(status) {
        case LED_STATUS_POWER_ON:
            LED1_ON();   // 电源指示灯常亮
            LED2_OFF();  // 运行指示灯熄灭
            LED3_OFF();  // 火焰指示灯熄灭
            break;
            
        case LED_STATUS_RUNNING:
            LED1_ON();   // 电源指示灯常亮
            // LED2闪烁逻辑在主循环中处理
            LED3_OFF();  // 火焰指示灯熄灭
            break;
            
        case LED_STATUS_FLAME_DETECTED:
            LED1_ON();   // 电源指示灯常亮
            LED2_ON();   // 运行指示灯常亮
            LED3_ON();   // 火焰指示灯点亮
            break;
            
        case LED_STATUS_SENSOR_ERROR:
            LED1_ON();   // 电源指示灯常亮
            LED2_ON();   // 运行指示灯常亮（故障）
            LED3_OFF();  // 火焰指示灯熄灭
            break;
            
        case LED_STATUS_CALIBRATING:
            LED1_ON();   // 电源指示灯常亮
            // LED2和LED3交替闪烁表示校准
            break;
    }
}
```

## 📋 当前系统状态指示

### 现有单LED状态
| 系统状态 | LED状态 | 继电器状态 | 显示屏 |
|----------|---------|------------|---------|
| 启动校准 | 熄灭 | 断开 | 实时电压 |
| 正常运行 | 熄灭 | 断开 | 当前电压 |
| 检测到火焰 | 点亮 | 吸合 | 当前电压 |
| 传感器断开 | 熄灭 | 断开 | 显示000 |

### 建议的三LED状态
| 系统状态 | LED1(电源) | LED2(运行) | LED3(火焰) | 说明 |
|----------|------------|------------|------------|------|
| 上电启动 | 常亮 | 熄灭 | 熄灭 | 系统初始化 |
| 基线校准 | 常亮 | 闪烁 | 闪烁 | 校准进行中 |
| 正常运行 | 常亮 | 慢闪烁 | 熄灭 | 心跳指示 |
| 检测到火焰 | 常亮 | 常亮 | 常亮 | 火焰确认 |
| 传感器故障 | 常亮 | 快闪烁 | 熄灭 | 故障指示 |
| 传感器断开 | 常亮 | 常亮 | 熄灭 | 断开指示 |

## 🔧 实现建议

如果您需要实现多LED控制，我可以帮您：

1. **修改GPIO配置**：添加额外的LED引脚
2. **更新控制逻辑**：实现状态机管理
3. **添加闪烁功能**：定时器驱动的闪烁逻辑
4. **状态指示优化**：清晰的视觉反馈

请告诉我您的具体需求，我来帮您实现完整的多LED控制系统！ 