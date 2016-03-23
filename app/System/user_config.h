#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

//#define MakeUSER2
#ifndef MakeUSER2
#define USE_OPTIMIZE_PRINTF
#define SYSINFOR_PRINTF
#define PHONE_PRINTF
//#define SAVECLOUD_PRINTF
#define RF_WORK_PRINTF


//#define RF_WORK_DEBUG_PRINTF
//#define Flash_Test

#define Network_Test
#ifndef Network_Test
#define ATCmd
#endif

#define Cloud
#ifdef Cloud
//#define CloudKey
#ifndef CloudKey
#define SaveCloud
#define AddSecurity
#endif
#endif

//#define Sec_Test
#define Flash_Test
#define OTA_Test


#ifdef ICACHE_FLASH
#define ICACHE_FLASH_ATTR __attribute__((section(".irom0.text")))
#define ICACHE_RODATA_ATTR __attribute__((section(".irom.text")))
#else
#define ICACHE_FLASH_ATTR
#define ICACHE_RODATA_ATTR
#endif /* ICACHE_FLASH */



//#define ESP_PLATFORM 1
#define LIGHT_DEVICE 0
//#define USE_US_TIMER 1



//#define         SYS_VERSION      "IAP-STM32F103C8V100D20150923"
//#define         SYS_VERSION      "uV103D151029M20"
#define         SYS_VERSION      "uV102D151029M20"




#endif



#endif

