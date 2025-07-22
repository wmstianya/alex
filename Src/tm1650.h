#ifndef __TM1650_H
#define __TM1650_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void tm1650GpioInit(void);
void tm1650DisplayValue(uint16_t value, uint8_t leadingZero);
void tm1650UpdateDisplay(void);
void tm1650Clear(void);

// 兼容性函数声明
void tm1650_gpio_init(void);
void dis_value(uint16_t data);

#ifdef __cplusplus
}
#endif

#endif /* __TM1650_H */

