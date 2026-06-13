/*
 * "THE BEER-WARE LICENSE" (Revision 42):
 * regenbogencode@gmail.com wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return
 */
#include "ESPNowW.h"
#ifdef ESP32
#include <cstring>
#include <list>
#include <esp_system.h>
class ESPNowW32 : public IESPNowW {
  public:
    virtual esp_err_t add_peer(uint8_t *mac, int channel);
    virtual esp_err_t remove_peer(uint8_t *mac);
    virtual esp_err_t send_message(uint8_t *mac, uint8_t *data, size_t datalen);
    virtual esp_err_t set_mac(uint8_t *mac);

  private:
    std::list<esp_now_peer_info_t> peers;
};

esp_err_t ESPNowW32::add_peer(uint8_t *mac, int channel) {
    peers.push_back(esp_now_peer_info_t{});
    esp_now_peer_info_t &peer = peers.back();
    std::memcpy(peer.peer_addr, mac, sizeof(peer.peer_addr));
    peer.channel = channel;
    peer.ifidx = WIFI_IF_STA;
    auto success = esp_now_add_peer(&peers.back());
    if (success != ESP_OK)
        remove_peer(mac);
    return success;
}
esp_err_t ESPNowW32::remove_peer(uint8_t *mac) {
    // find peer in peers
    for (auto it = peers.begin(); it != peers.end();) {
        if (0 == memcmp(mac, it->peer_addr, sizeof(it->peer_addr))) {
            it = peers.erase(it); // remove it
        }
    }
    return esp_now_del_peer(mac);
}
esp_err_t ESPNowW32::send_message(uint8_t *mac, uint8_t *data, size_t datalen) {
    return esp_now_send(mac, data, datalen);
}
esp_err_t ESPNowW32::set_mac(uint8_t *mac) {
    return esp_base_mac_addr_set(mac);
}
ESPNowW32 espnow = ESPNowW32();
IESPNowW &ESPNow = espnow;
#endif