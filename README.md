### **CAT CALLER**

##### EECS 149/249A Project Proposal, Fall 2019

##### Angela Dong, Francis Leary, Wendi Luan, Qingrong Zhou



#### **Goals**
Use several wireless communication technologies (UWB, BLE, LoRa) and sensor readings (GPS and camera) from a collar to track a cat.

#### **Approach**
The collar will be turned on/off via BLE. We’ll use different sets of modules based on whether the sensed distance exceeds a constant threshold d.
Short range (distance <  d): UWB (localization), AV receiver + camera (high-quality video)
Long range (distance > d): LoRa + GPS (location), BLE + camera (low-quality video)
Surplus time could be spent switching to an infrared/night vision camera after a certain time of day, or developing a communication protocol to transmit video over LoRaWAN.


#### **Resources**

In this project we plan to use [DWM1001C](https://www.decawave.com/product/dwm1001-module/) Module for distance measurement and BLE on buckler and nRF52832 in DWM1001C. We’ll build a collar containing a battery (), a BLE-enabled breakout board (nRF52832), the GPS modules provided in lab () and our own camera (), and UWB (DWM1001C) and LoRa (RFM95W) transceivers. Separate from the collar, we’ll also use an AV receiver (RC832) and a screen ().

#### **Schedule**
• November 8, Milestone 1:  We received most of our parts. Sarah and Angela are working on LoRa. Wendy and Francis made a switching circuit for the camera. Video.

• November 17, Milestone 2:  Sarah and Angela work toward adapting LoRa Arduino library to nRF52832. Wendy will work on switch for UWB and GPS and implement switching logic on Sparkfun nRF52832. Francis will work on UWB ranging and displaying output on Buckler.

• November 22:  Sarah and Angela continue adapting LoRa Arduino library to nRF52832. Wendy will make a compact circuit for tracking device and test switching the camera on and off over BLE or LoRa. Francis will continue on UWB ranging and displaying output on Buckler, possibly start GPS as time allows.

• November 29: If there is time, implement GPS and/or WiFi positioning system with Mozilla Location Service. 

• December 6: Testing: find someone with a cat and see if the device is usable/useful. Determine which technologies are helpful, what radio protocol offers the best performance, if the battery life is realistic, and if the device can be made small enough.

• December 17: Demonstration video and poster made, powerpoint prepared. Project Expo.

• December 20: Final report

#### **Risks/Feasibility**

While we may not be able to get reliable wireless communication with all the physical obstructions in the real world, we are minimizing our risk of project failure by having redundancy in wireless communication technologies. BLE and LoRa both allow for two-way communication. BLE we are familiar with but has shorter range. LoRa has a longer range but we have no experience using it and current libraries are written for Arduino, not nRF52832. We could also use UWB for the two-way communication as well as ranging, however the connection distance is similar to BLE. We expect adapting existing Arduino libraries for the nRF will make up the bulk of the project, but in the event it is easy, we have other features we can add: GPS, WiFi positioning system, camera power and range optimization, or night vision with infrared LED. The Cat Caller device may even be useful with just one of the proposed features. GPS or WiFi positioning could be useful for determining the approximate location of a pet, or UWB or a live video stream alone could be enough to locate and ensure the safety of a feline friend. In addition, LoRa should provide adequate range but BLE may be sufficient for this use case. 
