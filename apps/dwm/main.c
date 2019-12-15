// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"

#define DWM_CS   NRF_GPIO_PIN_MAP(0,18)
#define DWM_SCLK NRF_GPIO_PIN_MAP(0,17)
#define DWM_MOSI NRF_GPIO_PIN_MAP(0,16)
#define DWM_MISO NRF_GPIO_PIN_MAP(0,15)

nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);


#define DWM1001_TLV_TYPE_POS_XYZ                   0x41  /* position coordinates x,y,z*/

#define RV_OK           (0)      /*  ret value OK */
#define RV_ERR          (1)      /*  ret value ERROR: unknown command or broken tlv frame */

#define RESP_ERRNO_LEN           3
#define RESP_DAT_TYPE_OFFSET     RESP_ERRNO_LEN
#define RESP_DAT_LEN_OFFSET      RESP_DAT_TYPE_OFFSET+1
#define RESP_DAT_VALUE_OFFSET    RESP_DAT_LEN_OFFSET+1

#define RESP_DATA_LOC_LOC_SIZE     15
#define RESP_DATA_LOC_DIST_OFFSET  RESP_DAT_TYPE_OFFSET + RESP_DATA_LOC_LOC_SIZE
#define RESP_DATA_LOC_DIST_LEN_MIN 3
#define DWM1001_TLV_TYPE_RNG_AN_POS_DIST           0x49  /* ranging anchor distances and positions*/
#define DWM1001_TLV_TYPE_RNG_AN_DIST               0x48  /* ranging anchor distances*/
/* Maximum number of neighbor anchors */
#define DWM_RANGING_ANCHOR_CNT_MAX	14

/**
 * @brief Position coordinates in millimeters + quality factor
 */
typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
	uint8_t qf;
} dwm_pos_t;


/**
 * @brief Position of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	dwm_pos_t pos[DWM_RANGING_ANCHOR_CNT_MAX];
}dwm_anchor_pos_t;

/**
 * @brief Distances of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	uint16_t addr[DWM_RANGING_ANCHOR_CNT_MAX];
	uint32_t dist[DWM_RANGING_ANCHOR_CNT_MAX];
	uint8_t qf[DWM_RANGING_ANCHOR_CNT_MAX];
}dwm_distance_t;


/**
 * @brief Distances and position of ranging anchors
 */
typedef struct {
	dwm_distance_t dist;
	dwm_anchor_pos_t an_pos;
}dwm_ranging_anchors_t;


/**
 * @brief Location data (position of current node and list of positions
 * and distances of ranging anchors)
 */
typedef struct dwm_loc_data_t {
	dwm_pos_t* p_pos;
	dwm_ranging_anchors_t anchors;
} dwm_loc_data_t;

int dwm_loc_get(dwm_loc_data_t* loc, uint8_t* rx_data, uint16_t rx_len) 
{

  uint8_t data_cnt, i, j;

  if(rx_len<RESP_ERRNO_LEN+RESP_DATA_LOC_LOC_SIZE + RESP_DATA_LOC_DIST_LEN_MIN)// ok + pos + distance/range
  {
     return RV_ERR;
  }
  
  if(rx_data[RESP_DAT_TYPE_OFFSET]==DWM1001_TLV_TYPE_POS_XYZ)//0x41
  {
     // node self position.
     data_cnt = RESP_DAT_VALUE_OFFSET;// jump Type and Length, goto data
     loc->p_pos->x = rx_data[data_cnt] 
                  + (rx_data[data_cnt+1]<<8) 
                  + (rx_data[data_cnt+2]<<16) 
                  + (rx_data[data_cnt+3]<<24); 
     data_cnt += 4;
     loc->p_pos->y = rx_data[data_cnt] 
                  + (rx_data[data_cnt+1]<<8) 
                  + (rx_data[data_cnt+2]<<16) 
                  + (rx_data[data_cnt+3]<<24); 
     data_cnt += 4;
     loc->p_pos->z = rx_data[data_cnt] 
                  + (rx_data[data_cnt+1]<<8) 
                  + (rx_data[data_cnt+2]<<16) 
                  + (rx_data[data_cnt+3]<<24); 
     data_cnt += 4;
     loc->p_pos->qf = rx_data[data_cnt++];
  }
  
  if(rx_data[RESP_DATA_LOC_DIST_OFFSET]==DWM1001_TLV_TYPE_RNG_AN_DIST)//0x48
  {
     // node is Anchor, recording Tag ID, distances and qf
     loc->anchors.dist.cnt = rx_data[RESP_DATA_LOC_DIST_OFFSET+2];
     loc->anchors.an_pos.cnt = 0;
     data_cnt = RESP_DATA_LOC_DIST_OFFSET + 3; // jump Type, Length and cnt, goto data
     for (i = 0; i < loc->anchors.dist.cnt; i++)
     {
        // Tag ID
        loc->anchors.dist.addr[i] = 0;
        for (j = 0; j < 8; j++)
        {
           loc->anchors.dist.addr[i] += rx_data[data_cnt++]<<(j*8);
        }
        // Tag distance
        loc->anchors.dist.dist[i] = 0;
        for (j = 0; j < 4; j++)
        {
           loc->anchors.dist.dist[i] += rx_data[data_cnt++]<<(j*8);
        }
        // Tag qf
        loc->anchors.dist.qf[i] = rx_data[data_cnt++];
     }
  }
  else if (rx_data[RESP_DATA_LOC_DIST_OFFSET]==DWM1001_TLV_TYPE_RNG_AN_POS_DIST)//0x49
  {
     // node is Tag, recording Anchor ID, distances, qf and positions
     loc->anchors.dist.cnt = rx_data[RESP_DATA_LOC_DIST_OFFSET+2];
     loc->anchors.an_pos.cnt = rx_data[RESP_DATA_LOC_DIST_OFFSET+2];
     data_cnt = RESP_DATA_LOC_DIST_OFFSET + 3; // jump Type, Length and cnt, goto data
     for (i = 0; i < loc->anchors.dist.cnt; i++) {
        // anchor ID
        loc->anchors.dist.addr[i] = 0;
        for (j = 0; j < 2; j++)
        {
           loc->anchors.dist.addr[i] += ((uint64_t)rx_data[data_cnt++])<<(j*8);
        }
        // anchor distance
        loc->anchors.dist.dist[i] = 0;
        for (j = 0; j < 4; j++)
        {
           loc->anchors.dist.dist[i] += ((uint32_t)rx_data[data_cnt++])<<(j*8);
        }
        // anchor qf
        loc->anchors.dist.qf[i] = rx_data[data_cnt++];
        // anchor position
        loc->anchors.an_pos.pos[i].x  = rx_data[data_cnt] 
                                     + (rx_data[data_cnt+1]<<8) 
                                     + (rx_data[data_cnt+2]<<16) 
                                     + (rx_data[data_cnt+3]<<24); 
        data_cnt += 4;
        loc->anchors.an_pos.pos[i].y = rx_data[data_cnt] 
                                     + (rx_data[data_cnt+1]<<8) 
                                     + (rx_data[data_cnt+2]<<16) 
                                     + (rx_data[data_cnt+3]<<24); 
        data_cnt += 4;
        loc->anchors.an_pos.pos[i].z = rx_data[data_cnt] 
                                     + (rx_data[data_cnt+1]<<8) 
                                     + (rx_data[data_cnt+2]<<16) 
                                     + (rx_data[data_cnt+3]<<24); 
        data_cnt += 4;
        loc->anchors.an_pos.pos[i].qf = rx_data[data_cnt++];
     }
  } else {
    return RV_ERR; 
  }
  return RV_OK;
}






int main(void) {
    ret_code_t error_code = NRF_SUCCESS;

    // initialize RTT library
    error_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(error_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    printf("Log initialized!\n");

    nrf_drv_spi_config_t spi_config = {
        .sck_pin = DWM_SCLK,
        .mosi_pin = DWM_MOSI,
        .miso_pin = DWM_MISO,
        .ss_pin = DWM_CS,
        .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
        .orc = 0,
        .frequency = NRF_DRV_SPI_FREQ_4M,
        .mode = NRF_DRV_SPI_MODE_0,
        .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };

    error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
    APP_ERROR_CHECK(error_code);

    uint8_t reset_buf[1];
    reset_buf[0] = 0xff;

    uint8_t tx_buf[4];
    tx_buf[0] = 0x28;
    tx_buf[1] = 0x02;
    tx_buf[2] = 0x0D;
    tx_buf[3] = 0x01;

    uint8_t rx_buf[3];
    rx_buf[0] = 0;
    rx_buf[1] = 0;
    rx_buf[2] = 0;

    ret_code_t err_code = nrf_drv_spi_transfer(&spi_instance, reset_buf, 1, NULL, 0);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_spi_transfer(&spi_instance, reset_buf, 1, NULL, 0);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_spi_transfer(&spi_instance, reset_buf, 1, NULL, 0);
    APP_ERROR_CHECK(err_code);

    printf("reset\n");
    while (rx_buf[0] == 0 && rx_buf[1] == 0) {
        err_code = nrf_drv_spi_transfer(&spi_instance, NULL, 0, rx_buf, 2);
        APP_ERROR_CHECK(err_code);
        if (err_code != NRF_SUCCESS) {
            printf("continuing spi error code: %d\n", (int) err_code);
        }
        else {
            printf("rx_buf: %x %x %x\n", rx_buf[0], rx_buf[1], rx_buf[2]);
        }
        nrf_delay_ms(100);
    }

    nrf_delay_ms(100);

    printf("Trying to send command\n");
    err_code = nrf_drv_spi_transfer(&spi_instance, tx_buf, 4, NULL, 0);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_spi_transfer(&spi_instance, NULL, 0, rx_buf, 2);
    APP_ERROR_CHECK(err_code);
    if (err_code != NRF_SUCCESS) {
        printf("spi error code: %d\n", (int) err_code);
    }

    while (rx_buf[0] == 0 && rx_buf[1] == 0) {
        err_code = nrf_drv_spi_transfer(&spi_instance, NULL, 0, rx_buf, 2);
        APP_ERROR_CHECK(err_code);
        if (err_code != NRF_SUCCESS) {
            printf("continuing spi error code: %d\n", (int) err_code);
        }
        else {
            printf("rx_buf: %x %x %x\n", rx_buf[0], rx_buf[1], rx_buf[2]);
        }
    }

    printf("received: %x %x\n", rx_buf[0], rx_buf[1]);

    while (rx_buf[0] == 0 || rx_buf[0] == 0xff) {
        err_code = nrf_drv_spi_transfer(&spi_instance, NULL, 0, rx_buf, 3);
        APP_ERROR_CHECK(err_code);
        if (err_code != NRF_SUCCESS) {
            printf("continuing spi error code: %d\n", (int) err_code);
        }
        else {
            printf("rx_buf: %x %x %x\n", rx_buf[0], rx_buf[1], rx_buf[2]);
        }
    }

    // loop forever, running state machine
    while (1) {

    	// nrf_drv_spi_uninit(&spi_instance);
    	// if (error_code != 0 )
    	// 	printf("err_code %d\n", error_code);

    	// error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
    	// if (error_code != 0 )
    	// 	printf("err_code %d\n", error_code);

    	nrf_delay_ms(100);

        printf("looping\n");
        nrf_delay_ms(1);
        uint8_t tx_data[2];
        tx_data[0] = 0x0c;
        tx_data[1] = 0x00;
        uint8_t rx_data[400];
        uint16_t rx_len;

        // send tlv request
        err_code = nrf_drv_spi_transfer(&spi_instance, tx_data, 2, NULL, 0);
    	if (error_code != 0 )
    		printf("err_code %d\n", error_code);

    	// check for response
        err_code = nrf_drv_spi_transfer(&spi_instance, NULL, 0, rx_data, 2);
    	if (error_code != 0 )
    		printf("err_code %d\n", error_code);

        while(rx_data[0] == 0x00) {
            err_code = nrf_drv_spi_transfer(&spi_instance, NULL, 0, rx_data, 2);
        	if (error_code != 0 )
    			printf("err_code %d\n", error_code);
        }

	    printf("size: %x num: %x \n",rx_data[0],rx_data[1]);

        // if error, send reset
        if (rx_data[0] == 0x40) {
        	printf("reset\n");
			error_code = nrf_drv_spi_transfer(&spi_instance, reset_buf, 1, NULL, 0);
			if (error_code != 0 )
				printf("err_code %d\n", error_code);
		} else {
	        int num = rx_data[0];

	        // recieve response
	        err_code = nrf_drv_spi_transfer(&spi_instance, NULL, 0, rx_data, rx_data[0]);
	        if (error_code != 0 )
    			printf("err_code %d\n", error_code);

	        //printf("received: %s", rx_data);
	        dwm_loc_data_t loc;
	        err_code = dwm_loc_get(&loc, rx_data, rx_len);
	        printf("distance: %u", loc.anchors.dist.dist[0]);

    		if (error_code != 0 )
	    		printf("err_code %d\n", error_code);


	        for(int i=0;i<num;i++){
	            printf("%x ",rx_data[i]);
	        }
	        printf("\n");

    	}
    }
}

