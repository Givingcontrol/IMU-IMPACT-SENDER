
#include <ESP8266WiFi.h>
#include <espnow.h>

// REPLACE WITH RECEIVER MAC Address


// Variable to store if sending data was successful
String success;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    char datasending[32];
    float Impact_Value;
    float Raw_Impact_Value;
    float X_Accel;
    float Y_Accel;
    float Z_Accel;
  } struct_message;

// Create a struct_message called myData
struct_message ImpactData;

// Create a struct_message to hold incoming data
struct_message receivedData;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}


// Callback when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  /*
  incomingSwitch = receivedData.SwitchClosed;
  incomingRelay = receivedData.activateRelay;
  incomingLED = receivedData.LedOn;
  incomingOpenRelay = receivedData.deactivateRelay;
*/
}
