/******************************************************************************
 * Copyright 2013-2014 Espressif Systems (Wuxi)
 *
 * FileName: user_main.c
 *
 * Description: entry file of user application
 *
 * Modification history:
 *     2014/1/1, v1.0 create this file.
*******************************************************************************/
#include "ets_sys.h"
//#include "osapi.h"
#include "uart.h"
//#include "user_interface.h"
#include "smartlink.h"
#include "SysLayerInit.h"
#include "zc_hf_adpter.h"
#include "esp_wifi.h"

volatile unsigned long  g_ulStatus = 0;

extern u32 Update_Flash_addr;
extern int ESP_Init(void);

/*************************************************
* Function: wifi_handle_event_cb
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void wifi_handle_event_cb(System_Event_t *evt) 
{  
    os_printf("event %x\n", evt->event_id);  
    switch (evt->event_id) 
    {      
        case EVENT_STAMODE_CONNECTED:
            os_printf("connect to ssid %s, channel %d\n", 
                                evt->event_info.connected.ssid, 
                                evt->event_info.connected.channel);
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            break;      
        case EVENT_STAMODE_DISCONNECTED:
            os_printf("disconnect from ssid %s, reason %d\n",
                                evt->event_info.disconnected.ssid,
                                evt->event_info.disconnected.reason);
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_DISCONNECTED);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
            break;      
        case EVENT_STAMODE_AUTHMODE_CHANGE:
            os_printf("mode: %d -> %d\n",
                            evt->event_info.auth_change.old_mode,
                            evt->event_info.auth_change.new_mode);
            break;      
        case EVENT_STAMODE_GOT_IP:
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
            break;
        default:
            break;
    } 
} 
/*************************************************
* Function: user_pre_init
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR 
user_pre_init(void)
{
	Sys_LayerInit();
    ESP_Init();
}
/*************************************************
* Function: user_init
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void user_init(void)
{
    UartInit();
	uart_init_new();
    os_printf("\r\n%s from 0x%x \r\n", SYS_VERSION, system_get_userbin_addr());
    os_delay_us(100);
    wifi_set_event_handler_cb(wifi_handle_event_cb);
    espconn_init();
    user_pre_init();
}
