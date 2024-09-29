#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Dynamixel2Arduino.h>
#include <ESPNowW.h>
#include <MAX17043.h>
#include "q8Dynamixel.h"

// ESPNow
char myData[100];
bool incoming = false;
// Dynamixel
HardwareSerial ser(0);
Dynamixel2Arduino q8dxl(ser, 8);
q8Dynamixel q8(q8dxl);
// LED
const uint8_t LED_PIN = D0;
const uint8_t MODE_PIN = D3;

void onRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // ESPNow callback function to receive data
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
  //          mac_addr[5]);
  // Serial.print("Last Packet Recv from: ");
  // Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.println(myData);
  // Serial.print("Received a packet with size of: ");
  // Serial.println(len);
  q8.parseData(myData);
}

void displayReading()
{
  //
  // Get the voltage, battery percent
  // and other properties.
  //
  Serial.println("Device Reading:");
  Serial.print("Address:       0x"); Serial.println(FuelGauge.address(), HEX);
  Serial.print("Version:       "); Serial.println(FuelGauge.version());
  Serial.print("ADC:           "); Serial.println(FuelGauge.adc());
  Serial.print("Voltage:       "); Serial.print(FuelGauge.voltage()); Serial.println(" mV");
  Serial.print("Percent:       "); Serial.print(FuelGauge.percent()); Serial.println("%");
  Serial.print("Is Sleeping:   "); Serial.println(FuelGauge.isSleeping() ? "Yes" : "No");
  Serial.print("Alert:         "); Serial.println(FuelGauge.alertIsActive() ? "Yes" : "No");
  Serial.print("Threshold:     "); Serial.print(FuelGauge.getThreshold()); Serial.println("%");
  Serial.print("Compensation:  0x"); Serial.println(FuelGauge.compensation(), HEX);
  Serial.println();
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

  // MAX17043
  // Initialize the fuel gauge.
  if (FuelGauge.begin())
  {
    // Reset the device.
    Serial.println("Resetting device...");
    FuelGauge.reset();
    delay(250);

    // Issue a quickstart command and wait for the device to be ready.
    Serial.println("Initiating quickstart mode...");
    FuelGauge.quickstart();
    delay(125);

    // Display an initial reading.
    Serial.println("Reading device...");
    Serial.println();
    displayReading();
    Serial.println();
  }
  else
  {
    Serial.println("The MAX17043 device was NOT found.\n");
    while (true);
  }

  Serial.println("q8bot ESPNOW receiver:");
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();
  ESPNow.init();
  ESPNow.reg_recv_cb(onRecv);

  q8.begin();
  digitalWrite(MODE_PIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!q8.commStart()){
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Robot start!");

  while(1){
    displayReading();
    delay(9800);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }
}
