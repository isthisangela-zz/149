dwm-api-test.c

Purpose: to test all provided on-board c language APIs. 

Entry function: app_thread_entry

Descriptions:
The example uses the NVM read/write (dwm_nvm_usr_data_get/set)as part of the test utilities and stores the testing progress. On reset, this test information is reserved. 
Each set of APIs are wrapped in different functions, e.g. test_pos, test_loc, etc. 

for more information of the APIs, please refer to DWM1001_API_Guide