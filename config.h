#ifndef CONFIG_H
#define CONFIG_H

#define I2C_IRQ
#define SYSTICK

#define OLED_ENABLE  //ericyang 20151123
//#define DEBUG_ENABLE //ericyang 20151111 for use printf(),be sure: DEBUG_ENABLE_SEMIHOST enable in KEIL DEFINE
#define I2C_TIMEOUT_ENABLE //ericynag 20151111
#ifdef I2C_TIMEOUT_ENABLE
#define I2C_TIMEOUT_NOVALIDACK_COUNT 0x20
#define I2C_TIMEOUT_UNREACHABLE_COUNT 0x100
#endif



#endif //end of config.h






