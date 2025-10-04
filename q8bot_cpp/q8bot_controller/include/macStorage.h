#ifndef MACSTORAGE_H
#define MACSTORAGE_H

#include <Arduino.h>
#include <Preferences.h>

class macStorage {
private:
  Preferences _prefs;
  const char* _namespace = "q8bot";
  const char* _macKey = "peerMAC";

public:
  macStorage();

  // Load peer MAC address from NVS
  // Returns true if MAC found, false otherwise
  bool loadPeerMAC(uint8_t* mac);

  // Save peer MAC address to NVS
  void savePeerMAC(const uint8_t* mac);

  // Clear saved peer MAC from NVS
  void clearPeerMAC();
};

#endif
