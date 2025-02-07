/*
 * Modified from the following author's creation by yufeng.wu0902@gmail.com.
 * "THE BEER-WARE LICENSE" (Revision 42):
 * regenbogencode@gmail.com wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy me a beer in return
 */
#include <Arduino.h>
#include <WiFi.h>
#include <ESPNowW.h>

// This Q8bot's MAC address. Change this to yours. 24:EC:4A:C9:91:7C
// uint8_t receiver_mac[] = {0x24, 0xEC, 0x4A, 0xC9, 0x91, 0x7C};
uint8_t receiver_mac[] = {0x54, 0x32, 0x04, 0x86, 0xF8, 0xC8};

// Char array for sending and receiving data
char myData[100];
// char theirData[100];
uint16_t theirData[100];
bool incoming = false;

// String for storing serial command
String cmd;

// PeerInfo
esp_now_peer_info_t peerInfo;

// Callback when data is received
void onRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
           mac_addr[5]);
  memcpy(&theirData, incomingData, len);
  // Serial.print("Received from ["); Serial.print(macStr); Serial.print("]: "); 
  // Serial.println(sizeof(theirData));
  for (int i = 0; i < 100; i++) {
    Serial.print(theirData[i]);
    Serial.print(" ");
  } Serial.println();
  memset(theirData, 0, sizeof(theirData));
}

// Callback when data is sent. Not used ATM
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_MODE_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Receive data callback function
  ESPNow.reg_recv_cb(onRecv);

  // Register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, receiver_mac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if(Serial.available()){
    uint8_t size_t = Serial.readBytesUntil(';', myData, sizeof(myData));
    myData[size_t] = '\0'; // Null-terminate the string
    // Serial.println(myData);

    esp_err_t result = esp_now_send(receiver_mac, (uint8_t *) &myData, 
                                    sizeof(myData));
  }
  delay(1);
}