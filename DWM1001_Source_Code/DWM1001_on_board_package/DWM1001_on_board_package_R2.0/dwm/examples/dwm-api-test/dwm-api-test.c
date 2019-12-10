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
#include <stdlib.h>
#include <string.h>

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
	"App   :  dwm-simple\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"


//  =====================================================================
//  =================== below are test utilities ========================
//  =====================================================================

static int total = 0;
static int total_ok = 0;
static int total_fail = 0;

// rv == 0 : ok
// rv == 1 : fail
int Test_Check(int rv)
{   
   if (rv == 0)
   {
      total_ok++;
      printf("OK\n"); 
   }
   else
   {
      total_fail++;
      printf("Fail\n");       
   }
   total++;

   return (rv != 0);
}

void Test_End(void)
{
   printf("\n");
   printf("Total = %d \n", total);   
   printf("   OK = %d \n", total_ok);   
   printf(" Fail = %d \n", total_fail); 
}

#define DECA_LOGO   0xDECA
#define TEST_MAX_FN 30
typedef struct{
   uint16_t logo;
   uint16_t curr;
   uint16_t total;
   void (*test_fn[TEST_MAX_FN+1])(void);
}test_nvr;
test_nvr nvram;
uint8_t nvram_len;

void test_ramLoad(void){
   memset((uint8_t*)&nvram, 0, sizeof(test_nvr));
   dwm_nvm_usr_data_get((uint8_t*)&nvram, &nvram_len);   
}

test_nvr* test_ramGet(void){
   return &nvram;
}

void test_ramSync(void){
   dwm_nvm_usr_data_set((uint8_t*)&nvram, sizeof(test_nvr));
}

uint8_t test_ramValid(test_nvr* ram){
   if (ram->logo == DECA_LOGO)
   {
      return 1;
   }
   else
   {
      printf("RAM not valid\n");  
      return 0;
   }
}

int test_ramInit(void){
   test_nvr* ram = test_ramGet();
   if(!test_ramValid(ram))
   {
      printf("Initializing RAM\n");  
      memset((uint8_t*)ram, 0, sizeof(test_nvr));
      ram->logo = DECA_LOGO;
      ram->curr = 0;
      ram->total = 0;
      test_ramSync();
      return 1;
   }
   return 0;
}

int test_fnPush(void (*func)(void)){
   test_nvr* ram = test_ramGet();
   if(test_ramValid(ram) && (ram->total < TEST_MAX_FN))
   {
      printf("Pushing function: %d\n", ram->total);  
      ram->test_fn[ram->total++] = func;
      return 0;
   }
   return -1;
}

void* test_fnCurr(void){
   test_nvr* ram = test_ramGet();
   if(test_ramValid(ram))
   {
      return ram->test_fn[ram->curr];
   }else
   {
      return NULL;
   }
}

int test_fnNext(void){
   test_nvr* ram = test_ramGet();
   if(test_ramValid(ram) && (ram->curr < ram->total))
   {
      ram->curr++;
      test_ramSync();
      return 0;
   }else
   {
      return -1;
   }
}

void test_rollTheBall(void){
   void (*testfunc)(void);
   test_nvr* ram = test_ramGet();
	
   testfunc = test_fnCurr();
   while(testfunc != NULL)
   {
      printf("Testing function: %d\n", ram->curr);      
      testfunc();
      test_fnNext();
      testfunc = test_fnCurr();
      dwm_thread_delay(10);
   }
}

//  =====================================================================
//  =================== above are test utilities ========================
//  =====================================================================

void delayedStart(uint8_t s2wait){
   int i;
   uint32_t sys_time;

   for(i=0; i<s2wait; i++)
   {      
      sys_time = dwm_systime_us_get();
      printf("%ds to start: dwm_systime_us_get() = %lu\n", s2wait-i, sys_time);  
 
      dwm_thread_delay(100);
   }   
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
	cfg_tag.common.ble_en = false;
	cfg_tag.common.led_en = false;
	cfg_tag.common.enc_en = false;

	if ((cfg.mode != DWM_MODE_TAG) ||
	(cfg.stnry_en != cfg_tag.stnry_en) ||
	(cfg.loc_engine_en != cfg_tag.loc_engine_en) ||
	(cfg.low_power_en != cfg_tag.low_power_en) ||
	(cfg.meas_mode != cfg_tag.meas_mode) ||
	(cfg.common.fw_update_en != cfg_tag.common.fw_update_en) ||
	(cfg.common.uwb_mode != cfg_tag.common.uwb_mode) ||
	(cfg.common.ble_en != cfg_tag.common.ble_en) ||
	//(cfg.common.enc_en != cfg_tag.common.enc_en) ||
	(cfg.common.led_en != cfg_tag.common.led_en)) {

		if(cfg.mode 			!= DWM_MODE_TAG) 		
         printf("mode: get = %d, set = %d\n", cfg.mode, 		DWM_MODE_ANCHOR);
		if(cfg.stnry_en     		!= cfg_tag.stnry_en)  		
         printf("acce: get = %d, set = %d\n", cfg.stnry_en, 	cfg_tag.stnry_en);
		if(cfg.loc_engine_en 		!= cfg_tag.loc_engine_en) 	
         printf("le  : get = %d, set = %d\n", cfg.loc_engine_en, cfg_tag.loc_engine_en);
		if(cfg.low_power_en		!= cfg_tag.low_power_en)	
         printf("lp  : get = %d, set = %d\n", cfg.low_power_en, 	cfg_tag.low_power_en);
		if(cfg.meas_mode 		!= cfg_tag.meas_mode) 		
         printf("meas: get = %d, set = %d\n", cfg.meas_mode, 	cfg_tag.meas_mode);
		if(cfg.common.fw_update_en 	!= cfg_tag.common.fw_update_en)	
			printf("fwup: get = %d, set = %d\n", cfg.common.fw_update_en, cfg_tag.common.fw_update_en);
		if(cfg.common.uwb_mode		!= cfg_tag.common.uwb_mode)	
         printf("uwb : get = %d, set = %d\n", cfg.common.uwb_mode, cfg_tag.common.uwb_mode);
		if(cfg.common.ble_en 		!= cfg_tag.common.ble_en)	
         printf("ble : get = %d, set = %d\n", cfg.common.ble_en, cfg_tag.common.ble_en);
		if(cfg.common.enc_en 		!= cfg_tag.common.enc_en)	
         printf("enc : get = %d, set = %d\n", cfg.common.enc_en, cfg_tag.common.enc_en);
		if(cfg.common.led_en 		!= cfg_tag.common.led_en)	
         printf("led : get = %d, set = %d\n", cfg.common.led_en, cfg_tag.common.led_en);

		APP_ERR_CHECK(rv = Test_Check(dwm_cfg_tag_set(&cfg_tag)));

		printf("dwm_cfg_tag_set(&cfg_tag): %d \n", rv);
		dwm_reset();
	}
   	printf("dwm_cfg_tag_set(&cfg_tag):\t\t\t%s\n","pass");
}
   
void config_anchor(void){
	//int rv;
	dwm_cfg_t cfg;
	dwm_cfg_anchor_t cfg_an;

	/* Get node configuration */
	APP_ERR_CHECK(dwm_cfg_get(&cfg));

	/* Configure device as Anchor */
	cfg_an.initiator = 1;
	cfg_an.bridge = 0;
	cfg_an.common.enc_en = 0;
	cfg_an.common.led_en = 1;
	cfg_an.common.ble_en = 0;
	cfg_an.common.fw_update_en = 0;
	cfg_an.common.uwb_mode = DWM_UWB_MODE_ACTIVE;


	if ((cfg.mode != DWM_MODE_ANCHOR) ||
		(cfg.initiator     != cfg_an.initiator) ||
		(cfg.bridge 	   != cfg_an.bridge) ||
		(cfg.common.fw_update_en != cfg_an.common.fw_update_en) ||
		(cfg.common.uwb_mode     != cfg_an.common.uwb_mode) ||
		(cfg.common.ble_en != cfg_an.common.ble_en) ||
		//(cfg.common.enc_en != cfg_an.common.enc_en) ||
		(cfg.common.led_en != cfg_an.common.led_en)
		) {

		if(cfg.mode != DWM_MODE_ANCHOR) 		
         printf("mode: get = %d, set = %d\n", cfg.mode, DWM_MODE_ANCHOR);
		if(cfg.initiator != cfg_an.initiator)  	
         printf("init: get = %d, set = %d\n", cfg.initiator, cfg_an.initiator);
		if(cfg.bridge != cfg_an.bridge) 		
         printf("brdg: get = %d, set = %d\n", cfg.bridge, cfg_an.bridge);
		if(cfg.common.fw_update_en != cfg_an.common.fw_update_en)	
			printf("fwup: get = %d, set = %d\n", cfg.common.fw_update_en, cfg_an.common.fw_update_en);
		if(cfg.common.uwb_mode!= cfg_an.common.uwb_mode)
         printf("uwb : get = %d, set = %d\n", cfg.common.uwb_mode, cfg_an.common.uwb_mode);
		if(cfg.common.ble_en != cfg_an.common.ble_en)	
         printf("ble : get = %d, set = %d\n", cfg.common.ble_en, cfg_an.common.ble_en);
		if(cfg.common.led_en != cfg_an.common.led_en)	
         printf("led : get = %d, set = %d\n", cfg.common.led_en, cfg_an.common.led_en);
		if(cfg.common.enc_en != cfg_an.common.enc_en)	
         printf("enc : get = %d, set = %d\n", cfg.common.enc_en, cfg_an.common.enc_en);

		APP_ERR_CHECK(Test_Check(dwm_cfg_anchor_set(&cfg_an)));

		dwm_reset();
	}
   	printf("dwm_cfg_anchor_set(&cfg_an):\t\t\t%s\n","pass");
}

#define GPIO_PAIR1_A    DWM_GPIO_IDX_10 // This GPIO should be wired to GPIO31
#define GPIO_PAIR1_B    DWM_GPIO_IDX_31 // This GPIO should be wired to GPIO10
#define GPIO_LVL_HI   	1
#define GPIO_LVL_LO   	0

#define USER_BTN    	DWM_GPIO_IDX_12

#define LED_RED    	DWM_GPIO_IDX_22
#define LED_RED1    	DWM_GPIO_IDX_14
#define LED_BLUE   	DWM_GPIO_IDX_31
#define LED_GREEN  	DWM_GPIO_IDX_30
#define LED_ON(x)  	dwm_gpio_cfg_output(x, 0)
#define LED_OFF(x) 	dwm_gpio_cfg_output(x, 1)
#define LED_TOGGLE(x) 	dwm_gpio_value_toggle(x)
static uint8_t test_irq_success;
static bool gpio_set_value;
void gpio_cb(void* p_data)
{
   bool value;
   /* callback routine */

   printf("gpio_cb: \n");

   dwm_gpio_value_get(GPIO_PAIR1_B, &value);
   test_irq_success = (value == gpio_set_value);
}

void test_irq_rising(void)
{
   int rv, err_cnt = 0;
   int timeout = 10;
   test_irq_success = 0;

   gpio_set_value = GPIO_LVL_LO;
   err_cnt += rv = Test_Check(dwm_gpio_cfg_output(GPIO_PAIR1_A, gpio_set_value));
   printf("dwm_gpio_cfg_output\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");

   err_cnt += rv = Test_Check(dwm_gpio_cfg_input(GPIO_PAIR1_B, DWM_GPIO_PIN_PULLDOWN));
   printf("dwm_gpio_cfg_input\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");

   dwm_gpio_irq_dis(GPIO_PAIR1_B);// don't need to check because error is acceptable here.
   
   err_cnt += rv = Test_Check(dwm_gpio_irq_cfg(GPIO_PAIR1_B, DWM_IRQ_TYPE_EDGE_RISING, &gpio_cb, NULL));
   printf("dwm_gpio_irq_cfg\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");

   dwm_thread_delay(1);

   gpio_set_value = GPIO_LVL_HI;
   err_cnt += rv = Test_Check(dwm_gpio_cfg_output(GPIO_PAIR1_A, gpio_set_value));
   printf("dwm_gpio_cfg_output\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");

   dwm_thread_delay(timeout);
   err_cnt += Test_Check(!test_irq_success);
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);  
}

void test_irq_falling(void)
{
   int rv, err_cnt = 0;
   int timeout = 10;
   test_irq_success = 0;

   gpio_set_value = GPIO_LVL_HI;
   err_cnt += rv = Test_Check(dwm_gpio_cfg_output(GPIO_PAIR1_A, gpio_set_value));
   printf("dwm_gpio_cfg_output\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");

   err_cnt += rv = Test_Check(dwm_gpio_cfg_input(GPIO_PAIR1_B, DWM_GPIO_PIN_PULLUP));
   printf("dwm_gpio_cfg_input\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");

   dwm_gpio_irq_dis(GPIO_PAIR1_B);// don't need to check because error is acceptable here.
   
   err_cnt += rv = Test_Check(dwm_gpio_irq_cfg(GPIO_PAIR1_B, DWM_IRQ_TYPE_EDGE_FALLING, &gpio_cb, NULL));
   printf("dwm_gpio_irq_cfg\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");

   dwm_thread_delay(1);

   gpio_set_value = GPIO_LVL_LO;
   err_cnt += rv = Test_Check(dwm_gpio_cfg_output(GPIO_PAIR1_A, gpio_set_value));
   printf("dwm_gpio_cfg_output\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");

   dwm_thread_delay(timeout);
   err_cnt += Test_Check(!test_irq_success);
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);  
}
 
void test_pos(void)
{
   int rv;
   dwm_pos_t pos_set;
   dwm_pos_t pos_get;
   
   
   printf("dwm_pos_set(pos_set)\n");
   pos_set.qf = 100;
   pos_set.x = 123;
   pos_set.y = 50;
   pos_set.z = 234;
   rv = Test_Check(dwm_pos_set(&pos_set)); 
   printf("dwm_pos_set(&pos_set):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
      
   
   printf("dwm_pos_get(&pos_get)\n");
   rv = Test_Check(dwm_pos_get(&pos_get));
   if(rv == DWM_OK)
   {
      printf("\t\tpos_get.x = %d\n", pos_get.x);
      printf("\t\tpos_get.y = %d\n", pos_get.y);
      printf("\t\tpos_get.z = %d\n", pos_get.z);
      printf("\t\tpos_get.qf = %d\n", pos_get.qf);
   }
   printf("dwm_pos_get(&pos_get):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");

}
 
void test_loc(void)
{
   int rv, i;
   dwm_loc_data_t loc;
   printf("dwm_loc_get(&loc)\n");
   rv = Test_Check(dwm_loc_get(&loc));
   if(rv == DWM_OK)
   {
      printf("[%d,%d,%d,%u]\n", loc.pos.x, loc.pos.y, loc.pos.z,
            loc.pos.qf);

      for (i = 0; i < loc.anchors.dist.cnt; ++i) 
      {
         printf("%u)", i);
         printf("0x%llx", loc.anchors.dist.addr[i]);
         if (i < loc.anchors.an_pos.cnt) 
         {
            printf("[%d,%d,%d,%u]", loc.anchors.an_pos.pos[i].x,
                  loc.anchors.an_pos.pos[i].y,
                  loc.anchors.an_pos.pos[i].z,
                  loc.anchors.an_pos.pos[i].qf);
         }
         printf("=%u,%u\n", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);
      }
   }
   printf("dwm_loc_get(&loc):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** "); 
   
}
 
void test_ur(void)
{
   int rv;
   uint16_t ur_set = 22;
   uint16_t ur_s_set = 50;
   uint16_t ur_get;
   uint16_t ur_s_get;
   
   printf("dwm_upd_rate_set(%d, %d)\n", ur_set, ur_s_set);
   rv = Test_Check(dwm_upd_rate_set(ur_set, ur_s_set));
   printf("dwm_upd_rate_set(ur, ur_s):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
    
   
   printf("dwm_upd_rate_get(&ur, &ur_s)\n");
   rv = Test_Check(dwm_upd_rate_get(&ur_get, &ur_s_get));
   if(rv == DWM_OK)
   {
      printf("\t\tur  =%d \n", ur_get);
      printf("\t\tur_s=%d \n", ur_s_get);
   }
   printf("dwm_upd_rate_get(&ur, &ur_s):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");    

}
void test_cfg_tag(void)
{
   config_tag();

}

void test_cfg_an(void)
{
   config_anchor();

}

void test_ver(void)
{
   int rv; 
   dwm_ver_t ver; 
   printf("dwm_ver_get(&ver)\n");
   rv = Test_Check(dwm_ver_get(&ver));
   if(rv == DWM_OK)
   {
      printf("\t\tver.fw.maj  = %d\n", ver.fw.maj);
      printf("\t\tver.fw.min  = %d\n", ver.fw.min);
      printf("\t\tver.fw.patch= %d\n", ver.fw.patch);
      printf("\t\tver.fw.res  = %d\n", ver.fw.res);
      printf("\t\tver.fw.var  = %d\n", ver.fw.var);
      printf("\t\tver.cfg     = %08x\n", ver.cfg);
      printf("\t\tver.hw      = %08x\n", ver.hw);
   }
   printf("dwm_ver_get(&ver):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
}

void test_gpio(void)
{
   int rv = 0, i, j; 
   uint8_t gpio_idx_list[]={
      DWM_GPIO_IDX_2,DWM_GPIO_IDX_8,DWM_GPIO_IDX_9,
      DWM_GPIO_IDX_10,DWM_GPIO_IDX_12,DWM_GPIO_IDX_13,
      DWM_GPIO_IDX_14,DWM_GPIO_IDX_15,DWM_GPIO_IDX_22,
      DWM_GPIO_IDX_23,DWM_GPIO_IDX_27,
      DWM_GPIO_IDX_30,DWM_GPIO_IDX_31 };
   bool gpio_val_list[]={false, true};
   uint8_t gpio_pull_list[]={DWM_GPIO_PIN_NOPULL,DWM_GPIO_PIN_PULLDOWN,DWM_GPIO_PIN_PULLUP};

   config_tag();

   rv = 0;
   printf("dwm_gpio_cfg_input:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      for(j = 0; j < sizeof(gpio_pull_list); j++)
      {     
         printf("dwm_gpio_cfg_input(%d, %d)\n", gpio_idx_list[i], gpio_pull_list[j]);
         rv += Test_Check(dwm_gpio_cfg_input(gpio_idx_list[i], gpio_pull_list[j]));
      }
   }   
   printf("dwm_gpio_cfg_input:\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
    
   
   rv = 0;
   printf("dwm_gpio_cfg_output:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      for(j = 0; j < sizeof(gpio_val_list); j++)
      {      
         printf("dwm_gpio_cfg_output(%d, %d)\n", gpio_idx_list[i], gpio_val_list[j]);
         rv += Test_Check(dwm_gpio_cfg_output(gpio_idx_list[i], gpio_val_list[j]));
      }
   }
   printf("dwm_gpio_cfg_output:\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
    
      
   rv = 0;
   printf("dwm_gpio_value_set:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      for(j = 0; j < sizeof(gpio_val_list); j++)
      {     
         printf("dwm_gpio_value_set(%d, %d)\n", gpio_idx_list[i], gpio_val_list[j]);
         rv += Test_Check(dwm_gpio_value_set(gpio_idx_list[i], gpio_val_list[j]));   
      }
   }
   printf("dwm_gpio_value_set:\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
    
   
   rv = 0;
   static bool value_gpio_value_get;
   printf("dwm_gpio_value_get:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      printf("dwm_gpio_value_get(%d, &val)\n", gpio_idx_list[i]);
      rv += Test_Check(dwm_gpio_value_get(gpio_idx_list[i], &value_gpio_value_get));
   }
   printf("dwm_gpio_value_get:\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
    
   
   rv = 0;
   printf("dwm_gpio_value_toggle\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      printf("dwm_gpio_value_toggle(%d, &val)\n", gpio_idx_list[i]);
      rv += Test_Check(dwm_gpio_value_toggle(gpio_idx_list[i])); 
   }
   printf("dwm_gpio_value_toggle:\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
}

void test_status(void)
{
   int rv;
   dwm_status_t status;  
   
   printf("dwm_status_get(&status)\n");
   rv = Test_Check(dwm_status_get(&status));
   if(rv == DWM_OK)
   {
      printf("\t\tstatus.loc_data      = %d\n", status.loc_data);
      printf("\t\tstatus.uwbmac_joined = %d\n", status.uwbmac_joined);
   }
   printf("dwm_status_get(&status):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");   
}

void test_enc(void)
{
   int rv, i;
   dwm_cfg_tag_t cfg_tag;
    
   cfg_tag.stnry_en = 1;
   cfg_tag.low_power_en = 0; 
   cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
   cfg_tag.loc_engine_en = 1;
   cfg_tag.common.led_en = 0;
   cfg_tag.common.ble_en = 0;
   cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
   cfg_tag.common.fw_update_en = 0;
   printf("dwm_cfg_tag_set(&cfg_tag)\n");
   rv = Test_Check(dwm_cfg_tag_set(&cfg_tag));
   printf("dwm_cfg_tag_set(&cfg_tag):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
   
   // ========= dwm_enc_key_set ===========
   dwm_enc_key_t key;
   for(i = 0; i < DWM_ENC_KEY_LEN; i++)
   {
      key.byte[i] = i;
   }
   
   rv = Test_Check(dwm_enc_key_set(&key));
   printf("dwm_enc_key_set(&key):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
   
   rv = Test_Check(dwm_enc_key_clear());
   printf("dwm_enc_key_clear():\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
}
   

void test_usr_data(void)
{
   int rv, i;

   // ========= dwm_usr_data_read/write =========   
   uint8_t user_data[DWM_USR_DATA_LEN_MAX];
   uint8_t len, overwrite;
   
   printf("dwm_usr_data_read(user_data, &len);\n");
   rv = Test_Check(dwm_usr_data_read(user_data, &len));
   if(rv == DWM_OK)
   {
      printf("\t\tuser_data_len  = %d\n", len);
      printf("\t\tuser_data_get  = 0x");
      for (i = 0; i < len; i++)
      {
         printf(" %02x", user_data[i]);
      }      
      printf("\n");
   }
   printf("dwm_usr_data_read(user_data, &len):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
      
   
   printf("dwm_usr_data_write(user_data, len, overwrite);\n");
   len = DWM_USR_DATA_LEN_MAX;
   overwrite = 0;
   for(i=0; i< DWM_USR_DATA_LEN_MAX; i++)
   {
      user_data[i] = i;
   }
   rv = Test_Check(dwm_usr_data_write(user_data, len, overwrite));
   printf("dwm_usr_data_write(user_data, len, overwrite):\t\t\t%s\n", rv==0 ? "pass":"\t\t *** fail *** ");
}

void test_stnry(void)
{
   int rv, err_cnt = 0;

   dwm_stnry_sensitivity_t stnry_get, stnry_get2, stnry_set;
   
   printf("dwm_stnry_cfg_get:\n");
   rv = Test_Check(dwm_stnry_cfg_get(&stnry_get));
   if(rv == DWM_OK)
   {
      printf("stnry_get  = %d\n", stnry_get);
   }
   printf("dwm_stnry_cfg_get(&stnry_get):\t\t\t%s\n", rv==DWM_OK ? "pass":"fail");
   err_cnt += rv; 
   
   stnry_set = (stnry_get+1) % (DWM_STNRY_SENSITIVITY_HIGH+1);   
   
   printf("dwm_stnry_cfg_set: %d\n", stnry_set);
   rv = Test_Check(dwm_stnry_cfg_set(stnry_set));
   printf("dwm_stnry_cfg_set(stnry_set):\t\t\t%s\n", rv==DWM_OK ? "pass":"fail");
   err_cnt += rv; 
   
   printf("dwm_stnry_cfg_get:\n");
   rv = Test_Check(dwm_stnry_cfg_get(&stnry_get2));
   if(rv == DWM_OK)
   {
      printf("stnry_get2 = %d\n", stnry_get2);
   }
   printf("dwm_stnry_cfg_get(&stnry_get2):\t\t\t%s\n", rv==DWM_OK ? "pass":"fail");
   err_cnt += rv; 
   
   rv = stnry_get2!=stnry_set;
   printf("dwm_stnry_cfg_get/set:\t\t\t%s\n", rv==DWM_OK ? "pass":"fail");
   err_cnt += rv; 
   
   printf("dwm_stnry_cfg_set: %d\n", stnry_get);
   rv = Test_Check(dwm_stnry_cfg_set(stnry_get));
   printf("dwm_stnry_cfg_set(stnry_get):\t\t\t%s\n", rv==DWM_OK ? "pass":"fail");
   err_cnt += rv; 

   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);  
}

void test_uwb_cfg(void)
{   
   int rv, err_cnt = 0;
   dwm_uwb_cfg_t cfg, cfg_get;

   cfg.pg_delay = 197;
   cfg.tx_power = 0xD0252525;
   printf("dwm_uwb_cfg_set:\n");
   rv = Test_Check(dwm_uwb_cfg_set(&cfg));
   printf("dwm_uwb_cfg_set  = %s\n", rv==DWM_OK ? "pass":"fail");
   err_cnt += rv; 

   printf("dwm_uwb_cfg_get:\n");
   rv = Test_Check(dwm_uwb_cfg_get(&cfg_get));
   printf("delay=%x, power=%lx\n",
		cfg_get.pg_delay,
		cfg_get.tx_power);
   printf("dwm_uwb_cfg_get  = %s\n", rv==DWM_OK ? "pass":"fail");
   err_cnt += rv; 

   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);  
}

#define LIS2DX_SLAVE_ADDR       0x19   /* I2C slave address */
#define LIS2DX_WHO_AM_I         0x0F   /* Who I am ID */
#define LIS2DX_WHO_AM_I_VAL     0x33   /* ID value */
void test_i2c(void)
{
   int rv, err_cnt = 0;

   uint8_t data[2];
   const uint8_t addr = LIS2DX_SLAVE_ADDR;
   data[0] = LIS2DX_WHO_AM_I;

   printf("dwm_i2c_write:\n");
   rv = Test_Check(dwm_i2c_write(addr ,data, 1, true));
   printf("dwm_i2c_write  = %s\n", rv==DWM_OK ? "pass":"fail");

   printf("dwm_i2c_read:\n");
   rv = Test_Check(dwm_i2c_read(addr ,data, 1));
   printf("dwm_i2c_read  = %s\n", rv==DWM_OK ? "pass":"fail");
   err_cnt += rv; 

   printf("id_get = %d\n", data[0]);
   printf("id     = %d\n", LIS2DX_WHO_AM_I_VAL);
   rv = Test_Check(data[0] != LIS2DX_WHO_AM_I_VAL);
   printf("id_get vs id = %s\n", rv==DWM_OK ? "pass":"fail");
   err_cnt += rv; 
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);  
}

/**
 * @brief Test BLE address get/set
 *
 */
void test_baddr(void)
{
   int rv, err_cnt = 0;  
   dwm_baddr_t baddr;
   int i = 0;
   
   //dwm_baddr_get(&baddr);
   printf("dwm_baddr_get(&baddr):\n");
   err_cnt += Test_Check(dwm_baddr_get(&baddr)); 
   
   printf("sizeof(dwm_baddr_t) = %d\n", sizeof(dwm_baddr_t));   
   
   dwm_baddr_t temp_baddr;
   memcpy((uint8_t*)&temp_baddr, (uint8_t*)&baddr, sizeof(dwm_baddr_t));
   baddr.byte[0] += 1;
   printf("dwm_baddr_set(&baddr + 1):\n");
   err_cnt += Test_Check(dwm_baddr_set(&baddr));

   printf("dwm_baddr_get(&baddr + 1):\n");
   err_cnt += Test_Check(dwm_baddr_get(&baddr)); 
   
   printf("Comparing read and write BLE addresses.\n");
   rv = Test_Check(! ((baddr.byte[0] == temp_baddr.byte[0]+1)
                        && (baddr.byte[1] == temp_baddr.byte[1])
                        && (baddr.byte[2] == temp_baddr.byte[2])
                        && (baddr.byte[3] == temp_baddr.byte[3])
                        && (baddr.byte[4] == temp_baddr.byte[4])
                        && (baddr.byte[5] == temp_baddr.byte[5])));
   if(rv != 0)
   {
      printf("Comparison failed:\n");
      i = 0;
      printf("\t baddr.byte[%d]=0x%02x : temp_baddr.byte[%d]+1=0x%02x\n", i, baddr.byte[i], i, temp_baddr.byte[i]+1);
      for (i = 1; i < 6; i++)
      {
         printf("\t baddr.byte[%d]=0x%02x : temp_baddr.byte[%d]=0x%02x\n", i, baddr.byte[i], i, temp_baddr.byte[i]);
      }
      err_cnt += rv; 
   }
                     
   baddr.byte[0] -= 1;
   printf("dwm_baddr_set(&baddr):\n");
   err_cnt += Test_Check(dwm_baddr_set(&baddr));

   printf("dwm_baddr_get(&baddr):\n");
   err_cnt += Test_Check(dwm_baddr_get(&baddr)); 
   
   printf("Comparing read and write BLE addresses.\n");
   rv = Test_Check(! ((baddr.byte[0] == temp_baddr.byte[0])
                        && (baddr.byte[1] == temp_baddr.byte[1])
                        && (baddr.byte[2] == temp_baddr.byte[2])
                        && (baddr.byte[3] == temp_baddr.byte[3])
                        && (baddr.byte[4] == temp_baddr.byte[4])
                        && (baddr.byte[5] == temp_baddr.byte[5])));
   if(rv != 0)
   {
      printf("Comparison failed:\n");
      for (i = 0; i < 6; i++)
      {
         printf("\t baddr.byte[%d]=0x%02x : temp_baddr.byte[%d]=0x%02x\n", i, baddr.byte[i], i, temp_baddr.byte[i]);
      }      
      err_cnt += rv; 
   }
   
   printf("BLE address get/set part test:\t\t\t%s\n", err_cnt==0 ? "pass":"fail");
   
   printf("%s %s: err_cnt = %d\n", err_cnt? "ERR" : "   ", __FUNCTION__, err_cnt);
}

void test_la(void)
{
   int err_cnt = 0;

   dwm_anchor_list_t list;
   int i;

   err_cnt += Test_Check(dwm_anchor_list_get(&list));

   for (i = 0; i < list.cnt; ++i) {
      printf("%d. id=0x%04X pos=[%ld,%ld,%ld] rssi=%d seat=%u neighbor=%d\n", i,
      list.v[i].node_id,
      list.v[i].x,
      list.v[i].y,
      list.v[i].z,
      list.v[i].rssi,
      list.v[i].seat,
      list.v[i].neighbor_network);
   }
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);  
}

void test_nid(void)
{
   int err_cnt = 0;
   uint64_t node_id;

   err_cnt += Test_Check(dwm_node_id_get(&node_id));

   printf("node id:0x%llx \n", node_id);
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);  
}

void test_frst(void)
{
   printf("dwm_factory_reset()\n");
   dwm_factory_reset();
}



/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */
void app_thread_entry(uint32_t data)
{
   delayedStart(3); 

   test_ramLoad();

   if(test_ramInit())
   {
      printf("Pushing test functions...\n");

      test_fnPush(test_pos);
      test_fnPush(test_loc);
      test_fnPush(test_ur);
      test_fnPush(test_cfg_tag);
      test_fnPush(test_i2c);
      test_fnPush(test_cfg_an);
      test_fnPush(test_ver);
      test_fnPush(test_gpio);
      test_fnPush(test_status);
      test_fnPush(test_enc);
      test_fnPush(test_usr_data);
      test_fnPush(test_stnry);
      test_fnPush(test_uwb_cfg);
      test_fnPush(test_baddr);
      test_fnPush(test_la);
      test_fnPush(test_nid);
      test_fnPush(test_irq_rising);    
      test_fnPush(test_irq_falling);  

      test_ramSync();
   }

   test_rollTheBall();

   Test_End();

   printf("All test done\n");

   delayedStart(10); 
   test_frst();

	while (1) {
		/* Thread loop */
		dwm_thread_delay(100);
	}
}

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
	dwm_ble_compile();
	dwm_le_compile();
	dwm_serial_spi_compile();
	//dwm_serial_uart_compile();

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}


