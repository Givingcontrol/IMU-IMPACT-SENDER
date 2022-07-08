/*
 *  Mandatory includes
 */
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <TinyMPU6050.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ESPNOW.h"
#include "MAC.h"

/*
 *  Constructing MPU-6050
 */
MPU6050 mpu (Wire);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//MPU variables
float AccelX, AccelY, AccelZ, Accel_Vector, maxValue = 0;
float Raw_AccelX, Raw_AccelY, Raw_AccelZ, Raw_Accel_Vector, Raw_maxValue = 0;


/*
 *  Method that prints everything
 */
void PrintGets () {
  // Shows offsets
  Serial.println("--- Offsets:");
  Serial.print("GyroX Offset = ");
  Serial.println(mpu.GetGyroXOffset());
  Serial.print("GyroY Offset = ");
  Serial.println(mpu.GetGyroYOffset());
  Serial.print("GyroZ Offset = ");
  Serial.println(mpu.GetGyroZOffset());
  // Shows raw data
  Serial.println("--- Raw data:");
  Serial.print("Raw AccX = ");
  Serial.println(mpu.GetRawAccX());
  Serial.print("Raw AccY = ");
  Serial.println(mpu.GetRawAccY());
  Serial.print("Raw AccZ = ");
  Serial.println(mpu.GetRawAccZ());
  Serial.print("Raw GyroX = ");
  Serial.println(mpu.GetRawGyroX());
  Serial.print("Raw GyroY = ");
  Serial.println(mpu.GetRawGyroY());
  Serial.print("Raw GyroZ = ");
  Serial.println(mpu.GetRawGyroZ());
  // Show readable data
  Serial.println("--- Readable data:");
  Serial.print("AccX = ");
  Serial.print(mpu.GetAccX());
  Serial.println(" m/s²");
  Serial.print("AccY = ");
  Serial.print(mpu.GetAccY());
  Serial.println(" m/s²");
  Serial.print("AccZ = ");
  Serial.print(mpu.GetAccZ());
  Serial.println(" m/s²");
  Serial.print("GyroX = ");
  Serial.print(mpu.GetGyroX());
  Serial.println(" degrees/second");
  Serial.print("GyroY = ");
  Serial.print(mpu.GetGyroY());
  Serial.println(" degrees/second");
  Serial.print("GyroZ = ");
  Serial.print(mpu.GetGyroZ());
  Serial.println(" degrees/second");
  // Show angles based on accelerometer only
  Serial.println("--- Accel angles:");
  Serial.print("AccelAngX = ");
  Serial.println(mpu.GetAngAccX());
  Serial.print("AccelAngY = ");
  Serial.println(mpu.GetAngAccY());
  // Show angles based on gyroscope only
  Serial.println("--- Gyro angles:");
  Serial.print("GyroAngX = ");
  Serial.println(mpu.GetAngGyroX());
  Serial.print("GyroAngY = ");
  Serial.println(mpu.GetAngGyroY());
  Serial.print("GyroAngZ = ");
  Serial.println(mpu.GetAngGyroZ());
  // Show angles based on both gyroscope and accelerometer
  Serial.println("--- Filtered angles:");
  Serial.print("FilteredAngX = ");
  Serial.println(mpu.GetAngX());
  Serial.print("FilteredAngY = ");
  Serial.println(mpu.GetAngY());
  Serial.print("FilteredAngZ = ");
  Serial.println(mpu.GetAngZ());
  // Show filter coefficients
  Serial.println("--- Angle filter coefficients:");
  Serial.print("Accelerometer percentage = ");
  Serial.print(mpu.GetFilterAccCoeff());
  Serial.println('%');
  Serial.print("Gyroscope percentage = ");
  Serial.print(mpu.GetFilterGyroCoeff());
  Serial.println('%');
}

/*
 *  Setup
 */
void setup() {

  Serial.begin(115200);

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
  esp_now_add_peer(Hub, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

   Wire.begin();
 
  //Wire.pins(0, 2);
  Wire.pins(2, 0);   //This works        // set I2C pins (SDA = GPIO2, SCL = GPIO0), default clock is 100kHz
 
   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }


  // Initialization
  mpu.Initialize();

  // Calibration
  Serial.begin(9600);
  Serial.println("=====================================");
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");

  
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(10, 0);
  display.write("Calculating offsets,");
  display.setCursor(10,16 );
  display.write(" do not move MPU6050");
  display.display();
   mpu.Calibrate();
   mpu.Execute();
  //delay(2000); 
  
}

/*
 *  Loop
 */
void loop() {
  
  
  mpu.UpdateRawAccel();
  AccelX = mpu.GetAccX();
  AccelY = mpu.GetAccY();
  AccelZ = mpu.GetAccZ();
  Accel_Vector = sqrt((AccelX*AccelX)+(AccelY*AccelY)+(AccelZ*AccelZ));
  if (Accel_Vector > maxValue )
      {maxValue = Accel_Vector;}
  Raw_AccelX = mpu.GetRawAccX();
  Raw_AccelY = mpu.GetRawAccY();
  Raw_AccelZ = mpu.GetRawAccZ();
  Raw_Accel_Vector = sqrt((Raw_AccelX*Raw_AccelX)+(Raw_AccelY*Raw_AccelY)+(Raw_AccelZ*Raw_AccelZ));
  if (Raw_Accel_Vector > Raw_maxValue )
      {Raw_maxValue = Raw_Accel_Vector;}    
  PrintGets();
   display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);
  display.print("Accel_Vector:");
  display.println (Accel_Vector);
  display.print("maxValue:");
  display.println (maxValue);
  display.print("Raw_Ac_Vect:");
  display.println (Raw_Accel_Vector);
  display.print("Raw_maxValue:");
  display.println (Raw_maxValue);
  /*
  display.print(" GetRawAccX: ");
  display.println (mpu.GetAccX());
  display.print(" GetRawAccY:");
  display.println (mpu.GetAccY());
  display.print(" GetRawAccZ:");
  display.println (mpu.GetAccZ());
  */
  display.display();
    strcpy(ImpactData.datasending, "Sending Data");
    ImpactData.Raw_Impact_Value = Raw_maxValue;
    ImpactData.Impact_Value;
    //ImpactData.datasending;
  esp_now_send(Hub, (uint8_t *) &ImpactData, sizeof(ImpactData));
  delay(10); // 30 sec delay

}
