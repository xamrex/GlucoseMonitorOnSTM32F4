
# Glucose Monitor

This project uses STM32F429I Discovery board with ESP-01 to shows blood Glucose level on 2.4" display. To develop this project I used:
- TouchGFX 4.22.0 Designer
- STM32CubeIDE 1.11.0
 

## What is needed to run this project
- HARDWARE:
  - You need FreeStyle Libre, Dexcom or other CGMs to monitor your glucose level.
  - ESP-01 wifi module (be sure that its working on 119200 Baud rate)
  - STM32F429I discovery boards
- SOFTWARE:
  - You need nightscout server, where data is uploaded form your CGM. (Web server needs to be available via HTTP, not HTTPS because of ESP-01 limitations)
  - You need other web server, that parse data from nightscout
## How to Run
1. Configure your phone app to upload data form CGM to Nightscout server.
You can use Juggluco or XDrip to upload data to nightscout.  
More information about nightscout server you can find [here](https://nightscout.github.io/).

2. Edit `index.php` file form `1.WEBSITE FOLDER`. Change `$Webadress` to your Nightscout server.   
3. Open project form `STM32CubeIDE`
 a) Fill SSID and Password in `ESP_Init("SSID","PASSWORD");` function  
 b) Fill website address in `ConnectToWebsite("Website_Address_where_index.php_Is_Uploaded");` function  

3. Flash the boardd
4. Connect ESP-01 to STM32F429I disco board.    
## Connecting ESP-01 to STM32F429

| ESP-01 Pin number            |STM32F429                                                                |
| ----------------- | ------------------------------------------------------------------ |
| 1.GND |GND  |
| 2.Tx | PD2 |
| 3.GPIO-2 | N/A|
| 4.CH_EN | Vcc|
| 5.GPIO-0 | N/A |
| 6.Reset | Vcc |
| 7.Rx |  PC12|
| 8.Vcc | Vcc |


## Screenshots

![App Screenshot](https://github.com/xamrex/GlucoseMonitorOnSTM32F4/blob/master/0.Screens/1.jpg)


## Contributing

Contributions are always welcome!  
If you have any problem don't hesitate to contact me ;-)
