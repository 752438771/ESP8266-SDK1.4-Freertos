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

#include "user_config.h"
#include "ets_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_sta.h"
#include "esp8266.h"
#include "smartconfig.h"
#include "smartlink.h"
#include "SysLayerInit.h"
#include "esp_libc.h"

extern void ESP_ChangeToNormalState(void);

LOCAL void //ICACHE_FLASH_ATTR
smartlink_done(sc_status status, void *pdata)
{
    switch (status) {
    case SC_STATUS_WAIT:
        os_printf("SC_STATUS_WAIT\n");
        break;
    case SC_STATUS_FIND_CHANNEL:
        os_printf("SC_STATUS_FIND_CHANNEL\n");
        break;
    case SC_STATUS_GETTING_SSID_PSWD:
        os_printf("SC_STATUS_GETTING_SSID_PSWD\n");
        break;
    case SC_STATUS_LINK:
        os_printf("SC_STATUS_LINK\n");
        struct station_config *sta_conf = pdata;

        wifi_station_set_config(sta_conf);
        wifi_station_disconnect();
        wifi_station_connect();
        break;
    case SC_STATUS_LINK_OVER:
        os_printf("SC_STATUS_LINK_OVER\n");

        if (pdata != NULL)
        {
            uint8 phone_ip[4] = {0};

            memcpy(phone_ip, (uint8*)pdata, 4);
            os_printf("Phone ip: %d.%d.%d.%d\n", phone_ip[0], phone_ip[1], phone_ip[2], phone_ip[3]);

        }
        ESP_ChangeToNormalState();
        smartconfig_stop();
        break;
    }

}

/******************************************************************************
 * FunctionName : SmartLink
 * Description  : ÷«ƒ‹≈‰÷√
 * Parameters   :
 * Returns      :
*******************************************************************************/
void //ICACHE_FLASH_ATTR
SmartLink(void)
{
	wifi_set_opmode(STATION_MODE);
    //smartconfig_stop();
	smartconfig_start(smartlink_done);
}

void //ICACHE_FLASH_ATTR
SmartLinkInter(void)
{ 
	ESP_Rest();
}

void ICACHE_FLASH_ATTR
smartconfig_task(void *pvParameters)
{
    smartconfig_start(smartlink_done);
    
    vTaskDelete(NULL);
}
/******************************************************************************
 * FunctionName : WifiInit
 * Description  : wifi≥ı ºªØ
 * Parameters   :
 * Returns      :
*******************************************************************************/
void //ICACHE_FLASH_ATTR
WifiInit(void)
{
	struct station_config s_staconf;

	if(STATION_MODE != wifi_get_opmode_default())
	{
		os_printf("not STATION_MODE \r\n");
		wifi_set_opmode(STATION_MODE);//STATION_MODE
	}

	wifi_station_get_config_default(&s_staconf);
	if (strlen(s_staconf.ssid))
	{
		wifi_station_set_config(&s_staconf);
		wifi_station_connect();
	}
    else
	{
		SmartLinkInter();
	}
}

