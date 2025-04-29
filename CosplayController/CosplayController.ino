#include <ESP32Servo.h>
#include "BluetoothSerial.h"

Servo myservo;  // crea el objeto servo

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Define Bluetooth Configuration
#define BL_IDENTIFIER       "GLAMED_LeoCosplay"
const char *btPin = "Leo234";
String btReceivedValue = "";

//BlueTooth Serial
BluetoothSerial SerialBT;


int pos = 0;    // posicion del servo

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 2;

void setup() {
  Serial.begin(115200);
  Serial.println("Now initialising Cosplay Controller");
  Serial.println("initialising Servos");
  
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 500, 2400);

  Serial.println("initialising Bluetooth..");
  delay(3000);
  SerialBT.begin(BL_IDENTIFIER); //Bluetooth device name
  SerialBT.setPin(btPin);
  delay(1000);

  Serial.println("Cosplay Controller ready..");
}

void loop() {
  CheckForBluetooth();
}

// Revisar comandos recibidos por Bluetooth
void CheckForBluetooth()
{
  if (SerialBT.available()) 
  {
    btReceivedValue = SerialBT.read();
    Serial.println(btReceivedValue);

    if(btReceivedValue == "49") //0
    {
        myservo.write(0);
    }
    else if(btReceivedValue == "50") //1
    {
        myservo.write(180); 
    }
    else if (btReceivedValue == "51") //2
    {
      
    }
    else if (btReceivedValue == "52") //3
    {
      
    }
  }
}
