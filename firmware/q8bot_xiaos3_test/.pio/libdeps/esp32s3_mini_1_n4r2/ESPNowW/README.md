# ESPNowW

The ESPNowW is a ESPNow wrapper library designed for Arduino/Platformio

## About
There are small differences between the ESPNow libraries for the esp8266 and esp32. This library unifies and simplifies the access in an arduino manner. 

## How to use
### WiFi mode
You have to use a WiFi mode, otherwise the esp is likely to crash. 
Depending on your platform its either of the options below.
```
WiFi.mode(
#ifdef ESP8266
    WIFI_STA
#elif ESP32
    WIFI_MODE_STA
#endif
); // MUST NOT BE WIFI_MODE_NULL
```
*NOTE*: It can be any mode but WIFI_MODE_NULL. 

### init
Next is to init the ESPNow library. The examples from espressif always made a disconnect before, so just keep it to be safe.
You can check the return value of ESPNow.init() to see if the init was successful and maybe retry with a delay if it wasn't.

```
WiFi.disconnect();
ESPNow.init();
```
*NOTE*: The return value of any function is zero (ESP_OK) if it is successful, check esp_err.h or esp_now.h.

### set mac
This lib also includes a unified way to change the macaddress of the esp:
```
ESPNow.set_mac(uint8_t *new_mac);
```

Adding peers or callbacks should be self explanatory.


## Contribution
If you feel like something should be changed feel free to submit a pullrequest.

## License

This project is licensed under the "THE BEER-WARE LICENSE" (Revision 42).  See the [LICENSE.md](LICENSE.md) file for details.


