/* Basic demo for accelerometer readings from Adafruit MPU6050
// Comment out gyro
//Rui Santos
//  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp8266-nodemcu/
//Modified by Garth
//NODEMCU 
D1  GPIO5  SCL (I2C)
D2  GPIO4  SDA (I2C)
Vin 5V- usb
GND 0v
*/
#include <ESP8266WiFi.h>
#include <espnow.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x5C, 0xCF, 0x7F, 0xB8, 0x59, 0xD3};



// Define variables to store incoming readings
int IncomingAccel;

// Updates sent readings every 100 milliseconds
const long interval = 10; 
unsigned long previousMillis = 0;    // will store last time sensor was updated 


// Variable to store if sending data was successful
String success;

//Structure message that holds  data 
//Must match the receiver structure
typedef struct struct_message {
    int SumAccel;
} struct_message;

// Create a struct_message to hold sensor readings to send
struct_message Accel_Value_Reading;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

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
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
 // Serial.print("Bytes received: ");
  //Serial.println(len);
  IncomingAccel = incomingReadings.SumAccel;
  Serial.print(IncomingAccel);
  Serial.println(" %");
  
}


void printIncomingReadings(){
  // Display Readings in Serial Monitor
  //Serial.println("INCOMING READINGS");
 // Serial.print("Accelaration Vector: ");
  //Serial.print(IncomingAccel);
 //  Serial.println(" %");
}
 


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

 // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

 
  Serial.println("");
  delay(100);
}

void loop() {
/*
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;

   
    // Print incoming readings
    printIncomingReadings();
  }
*/
}
