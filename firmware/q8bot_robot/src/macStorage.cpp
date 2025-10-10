#include "macStorage.h"

macStorage::macStorage() {
  // Constructor
}

bool macStorage::loadPeerMAC(uint8_t* mac) {
  _prefs.begin(_namespace, false);
  size_t len = _prefs.getBytesLength(_macKey);
  if (len == 6) {
    _prefs.getBytes(_macKey, mac, 6);
    _prefs.end();
    return true;
  }
  _prefs.end();
  return false;
}

void macStorage::savePeerMAC(const uint8_t* mac) {
  _prefs.begin(_namespace, false);
  _prefs.putBytes(_macKey, mac, 6);
  _prefs.end();
}

void macStorage::clearPeerMAC() {
  _prefs.begin(_namespace, false);
  _prefs.remove(_macKey);
  _prefs.end();
}
