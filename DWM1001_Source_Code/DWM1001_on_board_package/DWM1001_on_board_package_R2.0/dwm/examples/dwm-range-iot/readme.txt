dwm-range-iot.c

Purpose: When tag receives 0x0f from gateway (downlink data), it will send ranges over uplink data for the next 1000 trilateration performed. 

Entry function: app_thread_entry

Descriptions:
Firstly, on the user_data_ready event, checking if 0x0f is received. 
If received, then for the next 1000 location_ready_event, the ranges are stored in the uplink data and sent back to gateway. 
After 1000 echange, the sequence is reproduced if 0x0f is received.

For more information of the APIs, please refer to DWM1001_API_Guide