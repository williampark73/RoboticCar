/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"


// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output
#define BLE_READPACKET_TIMEOUT         50   // Timeout in ms waiting to read a response


// SOFTWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins that will be used for 'SW' serial.
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SWUART_RXD_PIN       5    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       6   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         7   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         -1   // Optional, set to -1 if unused
#define BLUEFRUIT_UART_MODE_PIN        -1    // Set to -1 if unused
// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

Servo servoLeft, servoRight;
#define LEFTPIN                 13
#define RIGHTPIN                12

#define STOP                    90
#define LEFTWHEELFORWARD        180
#define LEFTWHEELBACKWARD       0
#define RIGHTWHEELFORWARD       0
#define RIGHTWHEELBACKWARD      180

#define trigPin                 2
#define echoPin                 3
#define LEDPin                  10
#define wallThreshold           6
#define beginLEDDistance        wallThreshold*5

bool wallInFront = false;
bool goingForward = false;
long oldDistance = 200;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

bool totalStop = false;

/**************************************************************************/
/*!
*/
/**************************************************************************/
void setup(void)
{
  
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(9600);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  servoLeft.attach(LEFTPIN);
  servoRight.attach(RIGHTPIN);
  servoLeft.write(90);
  servoRight.write(90);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LEDPin, OUTPUT);
  
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Wall Detection Functionality
  long duration, distance;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = ((duration * 34000) / 1000000) / 2;
  if (distance < wallThreshold) {
    wallInFront = true;
  } else {
    wallInFront = false;
  }
  if (wallInFront) {
    if (goingForward) {
      servoLeft.write(STOP);
      servoRight.write(STOP);
    }
  }

  // EXTENSION: Light Indicator
  if (distance > beginLEDDistance) {
    digitalWrite(LEDPin, LOW);
  } else if (distance < wallThreshold) {
    digitalWrite(LEDPin, HIGH);
  } else {
    int timeDelay = map(distance, wallThreshold, beginLEDDistance, 0, 1000);
    digitalWrite(LEDPin, LOW);
    delay(timeDelay);
    digitalWrite(LEDPin, HIGH);
    delay(timeDelay);
  }

  // Stoplight Functionality
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  if (red > 110) {
    if (green < 100) {
      totalStop = true;
      servoLeft.write(STOP);
      servoRight.write(STOP);
    }
  } else {
    totalStop = false;
  }
  
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;
  
    /* Got a packet! */
  // printHex(packetbuffer, len);
  
  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
    
    bool forward = (buttnum == 5);
    bool backward = (buttnum == 6);
    bool left = (buttnum == 7);

    if (!totalStop) {
      if (pressed) {
        if (forward) {
          if (!wallInFront) {
            servoLeft.write(LEFTWHEELFORWARD);
            servoRight.write(RIGHTWHEELFORWARD);
            goingForward = true;
          }
        } else if (backward) {
          servoLeft.write(LEFTWHEELBACKWARD);
          servoRight.write(RIGHTWHEELBACKWARD);
          goingForward = false;
        } else if (left) {
          servoLeft.write(LEFTWHEELBACKWARD);
          servoRight.write(RIGHTWHEELFORWARD);
          goingForward = false;
        } else {
          servoLeft.write(LEFTWHEELFORWARD);
          servoRight.write(RIGHTWHEELBACKWARD);
          goingForward = false;
        }
      } else {
        servoLeft.write(STOP);
        servoRight.write(STOP);
        goingForward = false;
      }
    }
  }

  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer+2);
    lon = parsefloat(packetbuffer+6);
    alt = parsefloat(packetbuffer+10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Accel\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Mag\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
 
}
