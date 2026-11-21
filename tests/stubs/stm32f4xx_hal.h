/* 电脑端测试的 HAL 替身。只向测试提供所需定义，上板构建使用 Drivers 中的 STM32 HAL。 */

#ifndef HOST_TEST_STM32_HAL_H
#define HOST_TEST_STM32_HAL_H
#include <stdint.h>
/* 仅用于电脑模拟，不加入Keil编译，也不操作真实硬件。 */
uint32_t HAL_GetTick(void);
#endif
