/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Simple user application.
 *
 * Copyright (c) 2016-2018, LEAPS. All rights reserved.
 *
 */

#include "dwm.h"
#include <stdio.h>

/* Thread priority */
#ifndef THREAD_APP_PRIO
#define THREAD_APP_PRIO	20
#endif /* THREAD_APP_PRIO */

/* Thread stack size */
#ifndef THREAD_APP_STACK_SIZE
#define THREAD_APP_STACK_SIZE	(3 * 1024)
#endif /* THREAD_APP_STACK_SIZE */

#define APP_ERR_CHECK(err_code)	\
do {							\
	if ((err_code) != DWM_OK)	\
		printf("err: line(%u) code(%u)", __LINE__, (err_code));\
} while (0)						\

#define MSG_INIT	\
	"\n\n"	\
	"App   :  dwm-range-iot\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"

//uint16_t range_request = 0 ;
//uint16_t cnt_data_out = 0 ;

/**
 * Event callback
 *
 * @param[in] p_evt  Pointer to event structure
 */
void on_dwm_evt(dwm_evt_t *p_evt)
{
	uint8_t len;
	uint8_t i;
        uint16_t static range_request = 0;
        uint16_t static cnt_data_out = 0;
        uint16_t reg;

        extern void * uwb_get_if(int);

        //Counter used for IoT_uplink test
        

        // Data to send in IoT data 
        uint8_t data_out[34] = {0};
        // Data to send in IoT data 
        uint8_t data_in[34] = {0};
        // Current Idc to store in data
        uint8_t idx = 0;

	switch (p_evt->header.id) {
	/* New location data */
	case DWM_EVT_LOC_READY:
        
           
        #if 1
        if (range_request == 1 && cnt_data_out < 1000) {
                 
                 memcpy(data_out+idx,&(p_evt->loc.anchors.dist.cnt),sizeof(p_evt->loc.anchors.dist.cnt));
                 idx = idx + sizeof(p_evt->loc.anchors.dist.cnt);

		for (i = 0; i < p_evt->loc.anchors.dist.cnt; ++i) 
                {
                        memcpy(data_out+idx,&(p_evt->loc.anchors.dist.addr[i]),sizeof(p_evt->loc.anchors.dist.addr[i]));
                        idx = idx + sizeof(p_evt->loc.anchors.dist.addr[i]);
                        memcpy(data_out+idx,&(p_evt->loc.anchors.dist.dist[i]),sizeof(p_evt->loc.anchors.dist.dist[i]));
                        idx = idx + sizeof(p_evt->loc.anchors.dist.dist[i]);               
		}

                memcpy(data_out+idx,&cnt_data_out,sizeof(cnt_data_out));
                idx = idx + sizeof(cnt_data_out);
          
                // Send Ranges in IoT data
                cnt_data_out++;
                dwm_evt_listener_register(
			DWM_EVT_LOC_READY | DWM_EVT_USR_DATA_SENT |
			DWM_EVT_BH_INITIALIZED_CHANGED |
			DWM_EVT_UWBMAC_JOINED_CHANGED, NULL);
          } else if (cnt_data_out >= 10000)
          {
                range_request = 0 ;
                cnt_data_out  = 0 ;
                dwm_evt_listener_register(
			DWM_EVT_LOC_READY | DWM_EVT_USR_DATA_READY | DWM_EVT_USR_DATA_SENT |
			DWM_EVT_BH_INITIALIZED_CHANGED |
			DWM_EVT_UWBMAC_JOINED_CHANGED, NULL);
          }
        #endif
        #if 0
        if (range_request == 1 && cnt_data_out < 1000) {
          cnt_data_out++;
          memcpy(data_out+idx,&cnt_data_out,sizeof(cnt_data_out));
                  idx = idx + sizeof(cnt_data_out);
          dwm_usr_data_write(data_out,34,false);
        }
        else if (cnt_data_out >= 1000)
        { range_request = 0 ;
          cnt_data_out  = 0 ;
        }
        #endif
	break;    

	case DWM_EVT_USR_DATA_READY:
		len = p_evt->header.len - sizeof(dwm_evt_hdr_t);
		if (len <= 0)
                        break;
                dwm_usr_data_read(data_in,sizeof(p_evt->header.len));
                /*If 0x0f sent on broker, will send range within iot_data*/
                if (data_in[0]==15)
                {
                  range_request = 1 ;
                }
                break;

	case DWM_EVT_USR_DATA_SENT:
		printf("%d\n",cnt_data_out);
		break;

	case DWM_EVT_BH_INITIALIZED_CHANGED:
		printf("uwbmac: backhaul = %d\n", p_evt->bh_initialized);
		break;

	case DWM_EVT_UWBMAC_JOINED_CHANGED:
		printf("uwbmac: joined = %d\n", p_evt->uwbmac_joined);
		break;

	default:
		break;
	}
}

/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */
void app_thread_entry(uint32_t data)
{
	dwm_cfg_t cfg;
	uint8_t i2cbyte;
	dwm_evt_t evt;
	int rv;
	uint8_t label[DWM_LABEL_LEN_MAX];
	uint8_t label_len = DWM_LABEL_LEN_MAX;
        uint8_t buf[DWM_NVM_USR_DATA_LEN_MAX];
        uint8_t len;
        dwm_uwb_cfg_t uwb_cfg ;

        config_tag();

	/* Initial message */
	printf(MSG_INIT);

	/* Get node configuration */
	APP_ERR_CHECK(dwm_cfg_get(&cfg));

	/* Update rate set to 1 second, stationary update rate set to 5 seconds */
	APP_ERR_CHECK(dwm_upd_rate_set(1, 1));

	/* Sensitivity for switching between stationary and normal update rate */
	APP_ERR_CHECK(dwm_stnry_cfg_set(DWM_STNRY_SENSITIVITY_NORMAL));

	/* Register event callback */
	dwm_evt_listener_register(
			DWM_EVT_LOC_READY | DWM_EVT_USR_DATA_READY | DWM_EVT_USR_DATA_SENT |
			DWM_EVT_BH_INITIALIZED_CHANGED |
			DWM_EVT_UWBMAC_JOINED_CHANGED, NULL);


       //Read User NVM     
       len = sizeof(buf);
       rv = dwm_nvm_usr_data_get(buf, &len);

	/* Test the accelerometer */
	i2cbyte = 0x0f;
	rv = dwm_i2c_write(0x33 >> 1, &i2cbyte, 1, true);

	if (rv == DWM_OK) {
		rv = dwm_i2c_read(0x33 >> 1, &i2cbyte, 1);

		if (rv == DWM_OK) {
			printf("Accelerometer chip ID: %u\n", i2cbyte);
		} else {
			printf("i2c: read failed (%d)\n", rv);
		}
	} else {
		printf("i2c: write failed (%d)\n", rv);
	}

	rv = dwm_label_read(label, &label_len);

	if (rv == DWM_OK) {
		printf("LABEL(len=%d):", label_len);
		for (rv = 0; rv < label_len; ++rv) {
			printf(" %02x", label[rv]);
		}
		printf("\n");
	} else {
		printf("can't read label len=%d, error %d\n", label_len, rv);
	}

      
       if (rv == DWM_ERR_OVERRUN)
       {
          printf("expected length=%d\n", len);

       }
      

	while (1) {
		/* Thread loop */
		rv = dwm_evt_wait(&evt);

		if (rv != DWM_OK) {
			printf("dwm_evt_wait, error %d\n", rv);
		} else {
			on_dwm_evt(&evt);
		}
	}
}}}}

/**
 * Application entry point. Initialize application thread.
 *
 * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
 * USER THREADS CAN BE DONE IN THIS FUNCTION
 */
void dwm_user_start(void)
  {
	uint8_t hndl;
	int rv;

    dwm_shell_compile();
	//Disabling ble by default as softdevice prevents debugging with breakpoints (due to priority)
    //dwm_ble_compile();
	dwm_le_compile();
	dwm_serial_spi_compile();
    dwm_serial_uart_compile();

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}

void config_tag(void){
	int rv;
	dwm_cfg_t cfg;
	dwm_cfg_tag_t  cfg_tag;

	printf("trying to configure node as  tag\n");

	/* Get node configuration */
	APP_ERR_CHECK(dwm_cfg_get(&cfg));

	/* Configure device as TAG */
	cfg_tag.stnry_en = true;
	cfg_tag.loc_engine_en = true;
	cfg_tag.low_power_en = false;
	cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
	cfg_tag.common.fw_update_en = false;
	cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
	cfg_tag.common.ble_en = true;
	cfg_tag.common.led_en = true;
	cfg_tag.common.enc_en = false;

	if ((cfg.mode != DWM_MODE_TAG) ||
	(cfg.stnry_en != cfg_tag.stnry_en) ||
	(cfg.loc_engine_en != cfg_tag.loc_engine_en) ||
	(cfg.low_power_en != cfg_tag.low_power_en) ||
	(cfg.meas_mode != cfg_tag.meas_mode) ||
	(cfg.common.fw_update_en != cfg_tag.common.fw_update_en) ||
	(cfg.common.uwb_mode != cfg_tag.common.uwb_mode) ||
	(cfg.common.ble_en != cfg_tag.common.ble_en) ||
	(cfg.common.led_en != cfg_tag.common.led_en)) {

		if(cfg.mode 			!= DWM_MODE_TAG) 		printf("mode: get = %d, set = %d\n", cfg.mode, 		DWM_MODE_ANCHOR);
		if(cfg.stnry_en     		!= cfg_tag.stnry_en)  		printf("acce: get = %d, set = %d\n", cfg.stnry_en, 	cfg_tag.stnry_en);
		if(cfg.loc_engine_en 		!= cfg_tag.loc_engine_en) 	printf("le  : get = %d, set = %d\n", cfg.loc_engine_en, cfg_tag.loc_engine_en);
		if(cfg.low_power_en		!= cfg_tag.low_power_en)	printf("lp  : get = %d, set = %d\n", cfg.low_power_en, 	cfg_tag.low_power_en);
		if(cfg.meas_mode 		!= cfg_tag.meas_mode) 		printf("meas: get = %d, set = %d\n", cfg.meas_mode, 	cfg_tag.meas_mode);
		if(cfg.common.fw_update_en 	!= cfg_tag.common.fw_update_en)	
										printf("fwup: get = %d, set = %d\n", cfg.common.fw_update_en, cfg_tag.common.fw_update_en);
		if(cfg.common.uwb_mode		!= cfg_tag.common.uwb_mode)	printf("uwb : get = %d, set = %d\n", cfg.common.uwb_mode, cfg_tag.common.uwb_mode);
		if(cfg.common.ble_en 		!= cfg_tag.common.ble_en)	printf("ble : get = %d, set = %d\n", cfg.common.ble_en, cfg_tag.common.ble_en);
		if(cfg.common.enc_en 		!= cfg_tag.common.enc_en)	printf("enc : get = %d, set = %d\n", cfg.common.enc_en, cfg_tag.common.enc_en);
		if(cfg.common.led_en 		!= cfg_tag.common.led_en)	printf("led : get = %d, set = %d\n", cfg.common.led_en, cfg_tag.common.led_en);

		APP_ERR_CHECK(rv = dwm_cfg_tag_set(&cfg_tag));

		printf("dwm_cfg_tag_set(&cfg_tag): %d \n", rv);
		dwm_reset();
	}
   	printf("dwm_cfg_tag_set(&cfg_tag):\t\t\t%s\n","pass");
}



