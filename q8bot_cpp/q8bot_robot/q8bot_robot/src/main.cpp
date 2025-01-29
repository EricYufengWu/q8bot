#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Dynamixel2Arduino.h>
#include <ESPNowW.h>
#include <MAX17043.h>
#include "q8Dynamixel.h"

// Change this to your controller's MAC address if doing bi-directional ESPNow 24:EC:4A:C9:58:A4
uint8_t receiver_mac[] = {0xEC, 0xDA, 0x3B, 0x36, 0x10, 0xF0};

// ESPNow
char myData[100];
char theirData[100];
bool incoming = false;
esp_now_peer_info_t peerInfo;
// Dynamixel
HardwareSerial ser(0);
Dynamixel2Arduino q8dxl(ser, 8); // DIR pin on Q8bot is 8
q8Dynamixel q8(q8dxl);
uint32_t uint32Array[8];
// LED
const uint8_t LED_PIN = D0;
const uint8_t MODE_PIN = D3;
// MAX17043
float raw;
char intStr[10];

// Callback when data is received
void onRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
  //          mac_addr[5]);
  memcpy(&theirData, incomingData, sizeof(theirData));
  // Serial.print("Received from ["); Serial.print(macStr); Serial.print("]: "); 
  // Serial.println(theirData);
  q8.parseData(theirData);
}

// Callback when data is sent. Not used ATM
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void displayReading()
{
  // MAX17043 Demo code. To be cleaned up
  Serial.println("Device Reading:");
  // Serial.print("Address:       0x"); Serial.println(FuelGauge.address(), HEX);
  // Serial.print("Version:       "); Serial.println(FuelGauge.version());
  // Serial.print("ADC:           "); Serial.println(FuelGauge.adc());
  Serial.print("Voltage:       "); Serial.print(FuelGauge.voltage()); Serial.println(" mV");
  Serial.print("Percent:       "); Serial.print(FuelGauge.percent()); Serial.println("%");
  // Serial.print("Is Sleeping:   "); Serial.println(FuelGauge.isSleeping() ? "Yes" : "No");
  // Serial.print("Alert:         "); Serial.println(FuelGauge.alertIsActive() ? "Yes" : "No");
  // Serial.print("Threshold:     "); Serial.print(FuelGauge.getThreshold()); Serial.println("%");
  // Serial.print("Compensation:  0x"); Serial.println(FuelGauge.compensation(), HEX);
  Serial.println();
}

void addElementToArray(uint16_t*& array, size_t& currentSize, uint16_t newElement) {
    // Allocate a new array with one extra element
    uint16_t* newArray = new uint16_t[currentSize + 1];
    // Add new element to front
    newArray[0] = newElement;
    // Copy the existing elements to the new array
    memcpy(newArray + 1, array, currentSize * sizeof(uint16_t));
    // Delete the old array (free the memory)
    delete[] array;
    // Update the original pointer and size
    array = newArray;
    currentSize++;  // Increment the size of the array
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MODE_PIN, OUTPUT);

  // // Only needed to grab the MAC address. Disable and upload again.
  // while(!Serial){
  //   delay(100);
  // }
  // Serial.println("Serial port initialized.\n");
  // delay(5000);

  Serial.println("q8bot ESPNOW receiver:");
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();

  // ESPNow Init
  ESPNow.init();
  ESPNow.reg_recv_cb(onRecv);
  // Register for Send CB. Register pair.
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, receiver_mac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;      
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // MAX17043
  // Initialize the fuel gauge.
  if (FuelGauge.begin()){
    // Reset the device.
    FuelGauge.reset();
    delay(250);
    // Issue a quickstart command and wait for the device to be ready.
    FuelGauge.quickstart();
    delay(125);
    // Display an initial reading.
    displayReading();
  } else{
    Serial.println("MAX17043 NOT found.\n");
  }

  q8.begin();
  digitalWrite(MODE_PIN, HIGH);
}

void loop() {
  while (!q8.commStart()){
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Robot start!");

  while(1){
    // unsigned long startTime = micros(); // Get start time

    // uint16_t* posArray = q8.syncRead();
    // size_t currentSize = 16;
    // uint16_t battPercent = static_cast<uint16_t>(round(FuelGauge.percent()));
    // addElementToArray(posArray, currentSize, battPercent); // Add battery reading
    // myData[0] = '\0';
    // for (int i = 0; i < currentSize; i++){
    //   char temp[6];
    //   sprintf(temp, "%u", posArray[i]);
    //   if (i > 0){
    //     strcat(myData, ",");
    //   }
    //   strcat(myData, temp);
    // }
    // delete[] posArray;

    dtostrf(FuelGauge.percent(), 5, 2, myData);
    esp_err_t result = esp_now_send(receiver_mac, (uint8_t *) &myData, 
                                    sizeof(myData));

    delay(9800);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    
    // unsigned long endTime = micros(); // Get end time
    // unsigned long loopTime = endTime - startTime; // Calculate loop duration
    // Serial.println(loopTime); // Print loop time to serial monitor
  }
}

// Next step: Change 
