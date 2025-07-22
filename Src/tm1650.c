#include "tm1650.h"
#include "main.h"

// TM1650寄存器地址和命令定义
#define TM1650_CMD_MODE     0x48    // 模式设置命令
#define TM1650_BRIGHTNESS   0x71    // 亮度设置(最亮)
#define TM1650_DIG1_ADDR    0x68    // 数码管1地址
#define TM1650_DIG2_ADDR    0x6A    // 数码管2地址
#define TM1650_DIG3_ADDR    0x6C    // 数码管3地址
#define TM1650_DIG4_ADDR    0x6E    // 数码管4地址

// I2C时序参数
#define I2C_DELAY_US        1       // I2C时序延时
#define I2C_ACK_TIMEOUT     100     // ACK超时计数

// GPIO控制宏
#define TM1650_SCL_HIGH()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET)
#define TM1650_SCL_LOW()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET)
#define TM1650_SDA_HIGH()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET)
#define TM1650_SDA_LOW()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
#define TM1650_SDA_READ()   HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2)

// 数码管字形编码表(0-9)
static const uint8_t digitCode[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// 显示缓冲区
static uint8_t displayBuffer[4] = {0, 0, 0, 0};

/**
 * @brief 微秒级延时函数
 * @param delayUs 延时微秒数
 */
static void delayMicroseconds(uint16_t delayUs)
{
    if(delayUs == 0) return;
    
    // 基于系统时钟的精确延时
    uint32_t cycles = delayUs * (SystemCoreClock / 1000000);
    while(cycles--) {
        __NOP();
    }
}

/**
 * @brief I2C起始信号
 */
static void i2cStart(void)
{
    TM1650_SCL_HIGH();
    delayMicroseconds(I2C_DELAY_US);
    TM1650_SDA_HIGH();
    delayMicroseconds(I2C_DELAY_US);
    
    TM1650_SDA_LOW();
    delayMicroseconds(I2C_DELAY_US);
    TM1650_SCL_LOW();
    delayMicroseconds(I2C_DELAY_US);
}

/**
 * @brief I2C停止信号
 */
static void i2cStop(void)
{
    TM1650_SDA_LOW();
    delayMicroseconds(I2C_DELAY_US);
    TM1650_SCL_HIGH();
    delayMicroseconds(I2C_DELAY_US);
    TM1650_SDA_HIGH();
    delayMicroseconds(I2C_DELAY_US);
}

/**
 * @brief 等待I2C应答信号
 * @retval 1-收到应答, 0-超时无应答
 */
static uint8_t i2cWaitAck(void)
{
    uint8_t timeout = 0;
    
    TM1650_SCL_LOW();
    delayMicroseconds(I2C_DELAY_US);
    TM1650_SCL_HIGH();
    TM1650_SDA_HIGH();
    
    while(TM1650_SDA_READ()) {
        timeout++;
        delayMicroseconds(I2C_DELAY_US);
        if(timeout > I2C_ACK_TIMEOUT) {
            return 0;
        }
    }
    
    TM1650_SCL_LOW();
    TM1650_SDA_LOW();
    return 1;
}

/**
 * @brief I2C发送一个字节
 * @param data 要发送的数据
 */
static void i2cSendByte(uint8_t data)
{
    for(uint8_t i = 0; i < 8; i++) {
        TM1650_SCL_LOW();
        
        if(data & 0x80) {
            TM1650_SDA_HIGH();
        } else {
            TM1650_SDA_LOW();
        }
        
        delayMicroseconds(I2C_DELAY_US);
        TM1650_SCL_HIGH();
        delayMicroseconds(I2C_DELAY_US);
        data <<= 1;
    }
}

/**
 * @brief 向TM1650写入数据
 * @param addr 寄存器地址
 * @param data 数据
 * @retval 1-成功, 0-失败
 */
static uint8_t tm1650WriteData(uint8_t addr, uint8_t data)
{
    // 发送模式设置命令
    i2cStart();
    i2cSendByte(TM1650_CMD_MODE);
    if(!i2cWaitAck()) {
        i2cStop();
        return 0;
    }
    
    i2cSendByte(TM1650_BRIGHTNESS);
    if(!i2cWaitAck()) {
        i2cStop();
        return 0;
    }
    i2cStop();
    
    // 发送显示数据
    i2cStart();
    i2cSendByte(addr);
    if(!i2cWaitAck()) {
        i2cStop();
        return 0;
    }
    
    i2cSendByte(data);
    if(!i2cWaitAck()) {
        i2cStop();
        return 0;
    }
    i2cStop();
    
    return 1;
}

/**
 * @brief TM1650 GPIO初始化
 */
void tm1650GpioInit(void)
{
    GPIO_InitTypeDef gpioInit = {0};

    // 使能GPIO时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 配置TM1650 I2C引脚 (PD1-SCL, PD2-SDA)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);
    
    gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    gpioInit.Mode = GPIO_MODE_OUTPUT_OD;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpioInit);

    // 配置LED控制引脚 (PA10-火焰指示LED, PA11-继电器工作指示LED)
    // 初始状态：两个LED都熄灭(高电平)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);    // 火焰指示LED熄灭
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);    // 继电器工作指示LED熄灭
    
    gpioInit.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    
    // 确保初始状态正确
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);    // LED OFF (高电平熄灭)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);  // Relay OFF (低电平断开)
}

/**
 * @brief 更新显示缓冲区到TM1650
 */
void tm1650UpdateDisplay(void)
{
    const uint8_t addresses[4] = {
        TM1650_DIG1_ADDR,
        TM1650_DIG2_ADDR,
        TM1650_DIG3_ADDR,
        TM1650_DIG4_ADDR
    };
    
    for(uint8_t i = 0; i < 4; i++) {
        tm1650WriteData(addresses[i], displayBuffer[i]);
    }
}

/**
 * @brief 显示4位数字值
 * @param value 要显示的数值(0-9999)
 * @param leadingZero 是否显示前导零
 */
void tm1650DisplayValue(uint16_t value, uint8_t leadingZero)
{
    // 限制数值范围
    if(value > 9999) value = 9999;
    
    // 提取各位数字
    uint8_t digit[4];
    digit[0] = value / 1000;           // 千位
    digit[1] = (value % 1000) / 100;   // 百位
    digit[2] = (value % 100) / 10;     // 十位
    digit[3] = value % 10;             // 个位
    
    // 生成显示编码
    for(uint8_t i = 0; i < 4; i++) {
        if(!leadingZero && i < 3) {
            // 不显示前导零的情况
            uint8_t hasHigherDigit = 0;
            for(uint8_t j = 0; j < i; j++) {
                if(digit[j] != 0) {
                    hasHigherDigit = 1;
                    break;
                }
            }
            
            if(digit[i] == 0 && !hasHigherDigit) {
                displayBuffer[i] = 0x00;  // 熄灭
            } else {
                displayBuffer[i] = digitCode[digit[i]];
            }
        } else {
            displayBuffer[i] = digitCode[digit[i]];
        }
    }
    
    // 更新显示
    tm1650UpdateDisplay();
}

/**
 * @brief 清空显示
 */
void tm1650Clear(void)
{
    for(uint8_t i = 0; i < 4; i++) {
        displayBuffer[i] = 0x00;
    }
    tm1650UpdateDisplay();
}

// 兼容性函数 - 保持与原代码的兼容性
void tm1650_gpio_init(void) { tm1650GpioInit(); }
void dis_value(uint16_t data) { tm1650DisplayValue(data, 0); }




