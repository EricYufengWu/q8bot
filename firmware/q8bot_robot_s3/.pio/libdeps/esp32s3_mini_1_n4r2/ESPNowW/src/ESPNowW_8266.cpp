/*
 * "THE BEER-WARE LICENSE" (Revision 42):
 * regenbogencode@gmail.com wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return
 */
#include "ESPNowW.h"
#ifdef ESP8266
#include <cstring>
#include <list>
#include <memory>
#include <user_interface.h>
class ESPNowW8266 : public IESPNowW {
  public:
    virtual int init();
    virtual int add_peer(uint8_t *mac, int channel);
    virtual int remove_peer(uint8_t *mac);
    virtual int send_message(uint8_t *mac, uint8_t *data, size_t datalen);
    virtual int set_mac(uint8_t *mac);
};
int ESPNowW8266::init() {
    auto success = IESPNowW::init();
    return esp_now_set_self_role(ESP_NOW_ROLE_COMBO) + success;
}
int ESPNowW8266::add_peer(uint8_t *mac, int channel) {

    auto success = esp_now_add_peer(mac, ESP_NOW_ROLE_COMBO, channel, 0, 0);

    if (!success)
        remove_peer(mac);
    return success;
}
int ESPNowW8266::remove_peer(uint8_t *mac) { return esp_now_del_peer(mac); }
int ESPNowW8266::send_message(uint8_t *mac, uint8_t *data, size_t datalen) {
    return esp_now_send(mac, data, datalen);
}
int ESPNowW8266::set_mac(uint8_t *mac) {
    return wifi_set_macaddr(STATION_IF, mac);
}
ESPNowW8266 esp8266 = ESPNowW8266();
IESPNowW &ESPNow = esp8266;
#endif