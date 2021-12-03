// Basic demo for accelerometer readings from Adafruit MPU6050
// Comment out gyro
//Rui Santos
//  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp8266-nodemcu/
//Modified by Garth

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


Adafruit_MPU6050 mpu;

// Define variables to store for readings to be sent
int Accel_Vector ;

// Define variables to store incoming readings
int IncomingAccel;

// Updates sent readings every 100 milliseconds
const long interval = 100; 
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
  Serial.print("Bytes received: ");
  Serial.println(len);
  IncomingAccel = incomingReadings.SumAccel;
  
}

void getReadings(){
      /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
Accel_Vector = (sqrt((a.acceleration.x*a.acceleration.x)+(a.acceleration.y*a.acceleration.y)+(a.acceleration.z*a.acceleration.z)))/10;

// Serial.print("Acceleration Vector: ");
  Serial.print(Accel_Vector);

}

void printIncomingReadings(){
  // Display Readings in Serial Monitor
  Serial.println("INCOMING READINGS");
  Serial.print("Accelaration Vector: ");
  Serial.print(IncomingAccel);
   Serial.println(" %");
}
 


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  /*mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
*/
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
 // Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
  //  Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
  //  Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
  //  Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
 //   Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
 //   Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
  //  Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
  //  Serial.println("5 Hz");
    break;
  }


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

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;

    //Get sensor readings
    getReadings();

    //Set values to send
    Accel_Value_Reading.SumAccel = Accel_Vector;
    
    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &Accel_Value_Reading, sizeof(Accel_Value_Reading));

    // Print incoming readings
    printIncomingReadings();
  }

   /* Print out the values 
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  */

 /*
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(10);
  */
}
