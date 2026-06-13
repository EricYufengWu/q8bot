/*
 * "THE BEER-WARE LICENSE" (Revision 42):
 * regenbogencode@gmail.com wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return
 */
#ifndef ESPNOW_HPP
#define ESPNOW_HPP
#include <stdint.h>
#ifdef ESP32
#include <esp_now.h>
#elif ESP8266
#include <c_types.h>
#include <espnow.h>
#endif

class IESPNowW {
  public:
    virtual int add_peer(uint8_t *mac, int channel = 0) = 0;
    virtual int remove_peer(uint8_t *mac) = 0;
    virtual int send_message(uint8_t *mac, uint8_t *data, size_t datalen) = 0;
    virtual int set_mac(uint8_t *mac) = 0;
    virtual int init() { return esp_now_init(); }
    int reg_send_cb(esp_now_send_cb_t cb) {
        return esp_now_register_send_cb(cb);
    }
    int reg_recv_cb(esp_now_recv_cb_t cb) {
        return esp_now_register_recv_cb(cb);
    }
    int unreg_send_cb() { return esp_now_unregister_send_cb(); }
    int unreg_recv_cb() { return esp_now_unregister_recv_cb(); }
};

extern IESPNowW &ESPNow;
#endif
