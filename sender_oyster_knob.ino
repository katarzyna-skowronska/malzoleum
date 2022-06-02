/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp8266-nodemcu/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <espnow.h>

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress1[] = {0x3C,0x61,0x05,0xFD,0x66,0x50};
uint8_t broadcastAddress2[] = {0x94,0xB9,0x7E,0x14,0x47,0xE2};

int state = 1;
bool change = 1;

// Data to send structure, Must match the receiver structure
typedef struct data_struct {
    int tau_m;
    int refraction_period;
    int threshold;
    int spike_duration;
} data_struct;

// Create a struct_message called test to store variables to be sent
data_struct parameters;

unsigned long lastTime = 0;  
unsigned long timerDelay = 200;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  char macStr[18];
  Serial.print("Packet to:");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  pinMode(5, INPUT_PULLUP);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(broadcastAddress2, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

}
 
void loop() {
  //read_knobs();
  if ((millis() - lastTime) > timerDelay) {
    // Set values to send
  
    read_knobs();
    // Send message via ESP-NOW
    esp_now_send(0, (uint8_t *) &parameters, sizeof(parameters));

    lastTime = millis();
  }
//  if (Serial.available() > 0) {
//        // read the incoming byte:
//        msg_number = Serial.parseInt();
//}
}


void read_knobs() {

  int sensorVal = digitalRead(5);
  int knob = analogRead(A0);
  if (sensorVal == LOW && change ==0) {
    state = state + 1;
    if (state > 4) {
      state =1;  
    }
    change = 1;
  }
  if (sensorVal == HIGH && change ==1) {
    change = 0;
  }
  if (state ==1){
    parameters.tau_m = knob;
  }
  else if (state ==2){
    parameters.refraction_period = knob;
  }

  else if (state ==3){
    parameters.threshold = knob;
  }
  else {
    parameters.spike_duration = knob;
  }

}
