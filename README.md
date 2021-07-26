# BLE Weather Monitor
**Author:** MrMarshy

**Category:** Embedded Systems - Bluetooth Low Energy (IoT)

**Date:** 23rd July 2021

----------------------------------------------------------------------

### Project Description



----------------------------------------------------------------------

### Aim of the Project
The aim of the project is to produce an IoT weather station using BMP180 temperature and pressure sensor using
Bluetooth Low Energy to deliver the weather data to interested clients. 


----------------------------------------------------------------------

### Project Diagrams


#### Project Block Diagram(s)
![Block Diagram](Docs/Img/BlockDiagram.png)

#### Project UML Diagrams
TBC
#### Project Circuit Diagram (Optional)
![MCP2515 Example System](Docs/Img/MCP2515-ExampleSystem.png)


----------------------------------------------------------------------

### Project Construction
TBC
----------------------------------------------------------------------

### Suggestions for Additional Work
TBC

----------------------------------------------------------------------
### Installation/Tutorials

#### Step 1 - Create a Custom Base UUID
Use [UUID Generator](https://www.uuidgenerator.net/version4) to generate a 128-bit UUID.
In my case this was:
```
d810a115-6a86-4de3-b88d-7bf1443af9fd
```
The UUID is given as the sixteen octets of a UUID are represented as 32 hexadecimal (base 16) digits, displayed in five groups separated by hyphens, in the form 8-4-4-4-12. The 16 octets are given in big-endian, while nordic use the small-endian representation in their SDK. Thus, you must reverse the byte-ordering when you define your UUID base in the ble_custom_weather_service.h, as shown below.
```
/* This code belongs in ble_custom_weather_service.h*/
#define CUSTOM_SERVICE_UUID_BASE         {0xD8, 0x10, 0xA1, 0x15, 0x6A, 0x86, 0x4D, 0xE3, \
                                          0xB8, 0x8D, 0x7B, 0xF1, 0x44, 0x3A, 0xF9, 0xFD}

```

Now you need to define a 16-bit UUID for the Custom Service and a 16-bit UUID for a Custom Value Characteristic.
```
/* This code belongs in ble_custom_weather_service.h*/
#define CUSTOM_SERVICE_UUID               0x1400
#define CUSTOM_VALUE_CHAR_UUID            0x1401
```
The values for the 16-bit UUIDs that will be inserted into the base UUID can be choosen by random.

#### Step 2 - Adding a Custom Value Characteristic to the Custom Weather Service
A service is without a characteristic is next to nothing, so one is needed.
----------------------------------------------------------------------

## References
1. [IoT Projects with Arduino Nano 33 BLE Sense](https://www.apress.com/gp/book/9781484264577)
2. [nRF5x-custom-ble-service-tutorial](https://github.com/NordicPlayground/nRF5x-custom-ble-service-tutorial)
3. [nRF52-Bluetooth-Course](https://github.com/NordicPlayground/nRF52-Bluetooth-Course)