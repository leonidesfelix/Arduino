//#include <SparkFunMPU9250-DMP.h>
//#include "ESP32Sound.h"
#include "BluetoothSerial.h"
#include <FastLED.h>

//
// ARDUINO UPLOAD FIRMWARE PARAMETERS
//
// Placa: DOIT ESP32 DEVKIT V1
// Upload Speed: 115200
// Flash Frecuency: 80 MHZ
// Core Debug Level: None
// Port: Card Detected
//

// FIRMWARE OLED SCREEN
// MPU9250          OLED          BATTERY         ACTUATOR      TRIGGER1      TRIGGER2        LEDS          BUTTON
// -------------    ----------    -----------     ----------    -----------   -----------     -----------   -----------
// 3V3  ->  3V3     DO  ->  15    VCC ->  VIN     IN1 ->        D0  ->        D0  ->          DIN ->  32    D0  ->  
// GND  ->  GND     D1  ->  2     GND ->  GND     IN2 ->        GND ->  GND   GND ->  GND     5V  ->  3V3   GND ->  GND
// SCL  ->  22      RS  ->  4                                                                 GND ->  GND
// SDA  ->  21      DC  ->  5
//                  CS  ->  18

// FIRMWARE NO OLED
// MPU9250          BATTERY         ACTUATOR      TRIGGER1      TRIGGER2        LEDS          BUTTON
// -------------    -----------     ----------    -----------   -----------     -----------   -----------
// 3V3  ->  3V3     VCC ->  VIN     IN1 -> 3      D0  ->  12    D0  ->  14      DIN ->  2     D0  ->  4
// GND  ->  GND     GND ->  GND     IN2 -> 5      GND ->  GND   GND ->  GND     5V  ->  3V3   GND ->  GND
// SCL  ->  22                                                                  GND ->  GND
// SDA  ->  21      

// USABLE IOs
// (13),(12),(14),(27),(26),(25),33,(32)
// (15),(2),(4),16,17,(5),(18),19,(21),(22),23
// NOT USABLE IOs
// 34, 35 (RX1, TX1)
// 1, 3   (RX0, TX0)

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Define Bluetooth Configuration
#define BL_IDENTIFIER       "ZAPZAP_LeoCosplay"
#define IMU_VALUES_FORMAT   "TR|1|%f,%f,%f,%f,%i,%i,%i"  

#define VERBOSE
//#define VERBOSE_ROTATION

//Define FastLed Configuration
#define LED_PIN             2
#define NUM_LEDS            5     //(2 Leds para cada Indicador de Tag)
#define COLOR_ORDER         GRB
#define CHANNEL             0

// Define Buttons Configuration
#define FUSIL_TRIGGER     14
#define SHOTGUN_TRIGGER   12
#define LASER_TRIGGER     4

// Variables for triggers buttons state
byte fusilTriggerState = 0;
byte lastFusilTriggerState = 0;
byte shotgunTriggerState = 0;
byte lastShotgunTriggerState = 0;
byte laserTriggerState = 0;
byte lastLaserTriggerState = 0;

byte trigger1State = 0;
byte trigger2State = 0;
byte trigger3State = 0;

int ledColorChange = 1;

//BlueTooth Serial
BluetoothSerial SerialBT;

// Initialize Leds Array
CRGB leds[NUM_LEDS];

/*
// Initiaze IMU
MPU9250_DMP imu;
char rotation[44];

//Default IMU Orientation (Z to Up, Y Axis to the front, X to the right)
const signed char orientationDefault[9] = { 
  1, 0, 0, 
  0, 1, 0, 
  0, 0, 1 };
*/
void setup() {
  Serial.begin(500000);

  //Configure Tag Leds
  FastLED.addLeds<WS2812, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  
 
  // Configure triggers GPIOs
  pinMode(FUSIL_TRIGGER, INPUT_PULLUP);
  pinMode(SHOTGUN_TRIGGER, INPUT_PULLUP);
  pinMode(LASER_TRIGGER, INPUT_PULLUP);
  /*
  // MP9250
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      #ifdef VERBOSE
        Serial.println("Unable to communicate with MPU-9250");
        Serial.println("Check connections, and try again.");
        Serial.println();
      #endif
      
      delay(3000);
    }
  }

  // Initialize the digital motion processor (DMP)
  imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | // Send accelerometer data
                 DMP_FEATURE_GYRO_CAL       | // Calibrate the gyro data
                 DMP_FEATURE_SEND_CAL_GYRO  | // Send calibrated gyro data
                 DMP_FEATURE_6X_LP_QUAT     , // Calculate quat's with accel/gyro
                 60);                         // Set update rate to 10Hz.
              
  imu.dmpSetOrientation(orientationDefault);  
  */
  delay(3000);
  SerialBT.begin(BL_IDENTIFIER); //Bluetooth device name
  delay(1000);

  setLedsOn(ledColorChange);
  
  #ifdef VERBOSE
    Serial.println("The device started, now you can pair it with bluetooth!");    
    Serial.println("Leds On");
  #endif
}

void loop() {
  //IMURotation();  
    CheckForTriggers();
}
/*
void IMURotation()
{
  String output;
  unsigned short fifoCnt;
  inv_error_t result;

  fifoCnt = imu.fifoAvailable();

  if ( fifoCnt > 0) {
    result = imu.dmpUpdateFifo();

    if ( result == INV_SUCCESS) {
      imu.computeEulerAngles();
      output = "";

      float q0 = imu.calcQuat(imu.qx);
      float q1 = imu.calcQuat(imu.qy);
      float q2 = imu.calcQuat(imu.qz);
      float q3 = imu.calcQuat(imu.qw);

      sprintf(rotation, IMU_VALUES_FORMAT, q0, q1, q2, q3, trigger1State, trigger2State, trigger3State);
      SerialBT.println(rotation);

      #ifdef VERBOSE_ROTATION
        Serial.println(rotation);
      #endif
    }
    else{
      #ifdef VERBOSE
        Serial.println("Error getting IMU Values");
      #endif
    }
  }
}
*/

//Change Led Color
void setPixel(int Pixel, byte red, byte green, byte blue) {
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
}

//Turn On Tag Leds
void setLedsOn(int ledColor) {
  #ifdef VERBOSE
    Serial.println("SET LEDS ON");
  #endif
  switch(ledColor)
  {
    case 1: // Red & White (Player 1)
      setPixel(1, 255, 0, 0); //RED
      setPixel(2, 255, 0, 0); //RED
      setPixel(3, 255, 0, 0); //RED
      setPixel(4, 255, 0, 0); //RED
      #ifdef VERBOSE
        Serial.println("SET LEDS 1");
      #endif
      break;
    case 2: // Green & Yellow (Gun 1)
      setPixel(1, 0, 255, 0);
      setPixel(2, 0, 255, 0);
      setPixel(3, 0, 255, 0);
      setPixel(4, 0, 255, 0);
      #ifdef VERBOSE
        Serial.println("SET LEDS 2");
      #endif
      break;
    case 3: // Blue
      setPixel(1, 0, 0, 255);
      setPixel(2, 0, 0, 255);
      setPixel(3, 0, 0, 255);
      setPixel(4, 0, 0, 255);
      #ifdef VERBOSE
        Serial.println("SET LEDS 3");
      #endif     
      break;
    case 4: // Yellow
      setPixel(1, 255, 255, 0);
      setPixel(2, 255, 255, 0);
      setPixel(3, 255, 255, 0);
      setPixel(4, 255, 255, 0);
      #ifdef VERBOSE
        Serial.println("SET LEDS 4");
      #endif 
      break;
    case 5: // Cyan
      setPixel(1, 0, 255, 255);
      setPixel(2, 0, 255, 255);
      setPixel(3, 0, 255, 255);
      setPixel(4, 0, 255, 255);
      break;
    case 6: // Pink
      setPixel(1, 255, 0, 255);
      setPixel(2, 255, 0, 255);
      setPixel(3, 255, 0, 255);
      setPixel(4, 255, 0, 255);
      break;       
  }
  
  FastLED.show();
}

void CheckForTriggers()
{
  // Read trigger buttons state
  fusilTriggerState = digitalRead(FUSIL_TRIGGER);
  shotgunTriggerState = digitalRead(SHOTGUN_TRIGGER);
  laserTriggerState = digitalRead(LASER_TRIGGER);

  //If a trigger state changed
  if (fusilTriggerState != lastFusilTriggerState || shotgunTriggerState != lastShotgunTriggerState || laserTriggerState != lastLaserTriggerState) {        
    // compare the buttonState to its previous state
    if (fusilTriggerState != lastFusilTriggerState) {
      // if trigger pressed simulate recoil
      if (fusilTriggerState == LOW) {
                
        #ifdef VERBOSE
          Serial.println("TAG_TRIGGER1_ON");
        #endif

        trigger1State = 1;        
        
      }
      else{
          #ifdef VERBOSE
            Serial.println("TAG_TRIGGER1_OFF");
          #endif
          
          trigger1State = 0;
      }
    }
  
    if (shotgunTriggerState != lastShotgunTriggerState) {
      
      if (shotgunTriggerState == LOW) {
          #ifdef VERBOSE
            Serial.println("TAG_TRIGGER2_ON");
          #endif
          
          trigger2State = 1;          
      }
      else{
          #ifdef VERBOSE
            Serial.println("TAG_TRIGGER2_OFF");
          #endif
          
          trigger2State = 0;
      }
    }
    
    if (laserTriggerState != lastLaserTriggerState) {
      if (laserTriggerState == LOW) {
          #ifdef VERBOSE
            Serial.println("TAG_TRIGGER3_ON");
          #endif
          
          trigger3State = 1;

        #ifdef VERBOSE
          Serial.println("PASE SETLEDSON");
        #endif

        ledColorChange++;
        setLedsOn(ledColorChange); 
        if(ledColorChange >= 6) {
          ledColorChange = 1;
        }

      }
      else{
          #ifdef VERBOSE
            Serial.println("TAG_TRIGGER3_OFF");
          #endif
          
          trigger3State = 0;
      }
    }

    // Delay a little bit to avoid bouncing
    delay(5);
    
    // save the current state as the last state,
    //for next time through the loop
    lastFusilTriggerState = fusilTriggerState;
    lastShotgunTriggerState = shotgunTriggerState;
    lastLaserTriggerState = laserTriggerState;
    delay(5);
  }
}
