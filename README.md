# _Tornadoedge display server_


## How to use example

### Hardware Required

* A development board with ESP32 SoC (e.g., ESP32-DevKitC, ESP-WROVER-KIT, etc.)
* A USB cable for power supply and programming

Connect Uart to display module:
#### Tornadoedge Gateway for Aerobox settings pin define
|Define|GPIO|
|:---:|:---:|
|Module Tx|GPIO17|
|Module Rx|GPIO16|
|Display Tx|GPIO15|
|Display Rx|GPIO14|

|ledc channel|GPIO|Color|
|:---:|:---:|:---:|
|channel 0|GPIO25|Green|
|channel 1|GPIO26|Red|

### Configure the project

```
idf.py menuconfig
```

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Troubleshooting

* Programming fail

    * Hardware connection is not correct: run `idf.py -p PORT monitor`, and reboot your board to see if there are any output logs.
    * The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.

For any technical queries, please open an [issue] (https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.

## V2.0.0 feature
* correct sensor unit and display roles for tornadoedge gateway

## bug fix
V1.0.9
add aerobox heartbeat -> using same time resolution in tornadoedge gateway
V1.0.8
change namespace ideasky -> tornadoedge
aerobox-raw Client -> DeviceId