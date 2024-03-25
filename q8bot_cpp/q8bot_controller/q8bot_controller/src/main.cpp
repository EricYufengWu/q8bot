/*
 * "THE BEER-WARE LICENSE" (Revision 42):
 * regenbogencode@gmail.com wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy me a beer in return
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float pos[8];
  uint16_t dur;
  bool torque = false;
} struct_message;

// Create a struct_message called myData
// struct_message myData;
char myData[100];

// Receiver MAC Address (Seeed XIAO) EC:DA:3B:36:10:F0
uint8_t receiver_mac[] = {0xEC, 0xDA, 0x3B, 0x36, 0x10, 0xF0};

// String for storing serial command
String cmd;

// peerInfo
esp_now_peer_info_t peerInfo;

// var for testing
uint8_t count = 0;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println(!status);
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  // Serial1.begin(115200);
  Serial.setTimeout(10);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
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
    // Motor position
    // for(int i = 0; i < 8; i++){
    //   cmd = Serial.readStringUntil(',');
    //   myData.pos[i] = cmd.toFloat();
    // }
    // // Move duration
    // cmd = Serial.readStringUntil(',');
    // myData.dur = cmd.toInt();
    // // Torque on/off
    // cmd = Serial.readStringUntil(',');
    // myData.torque = cmd.toInt();

    // cmd = Serial.readStringUntil(';');
    uint8_t size_t = Serial.readBytesUntil(';', myData, sizeof(myData));
    Serial.print(size_t);
    Serial.print(" ");
    Serial.println(myData);
    // cmd.toCharArray(myData, sizeof(cmd));
    // Serial.print(myData);
    // Serial.print(" Size:");
    // Serial.println(sizeof(myData));

    esp_err_t result = esp_now_send(receiver_mac, (uint8_t *) &myData, 
                                    sizeof(myData));
  } else{
    if (count < 100){
      count++;
    } else{
      count = 0;
    }
    myData[0] = count;
    // cmd.toCharArray(myData, sizeof(cmd));
    esp_err_t result = esp_now_send(receiver_mac, (uint8_t *) &myData, 
                                    sizeof(myData));
    delay(9);
  }
  delay(1);
}