# DANSA
Drone Assisted Network Spectrum Analyzer (DANSA)

Senior Design Project at Purdue University utilizing microcontroller stm32F0 and various extrnal modules such as WiFi, GPS/GSM and SD card storage.
The DANSA is designed to conduct autonomous wireless site surveying for large buildings. The product works by gathering GPS/GSM location data and matching it wit WiFi RSID at certain locations and storing them on an SD card. The data is then run through a Python script which generates a WiFi HeatMap on on Google Maps.

The microcontroller communicates with the GPS/GSM module and WiFi module via UART, the LED strip through PWM and FreeRTOS, and the SD card with SPI and FatFs. 

Microcontroller: STM32F051R8T6\
GPS/GSM module: SIMCom SIM808\
WiFi Module: Redpine rs9113-n00-d1w\
SD Card: SanDisk 32GB\
LED Strip: NeoPixel ws2812b\
