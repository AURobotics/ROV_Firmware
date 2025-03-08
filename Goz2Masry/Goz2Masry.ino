#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// Pin Definitions
const int thrusterPinsForward[5] = { 0 };  // Example forward pins
const int thrusterPinsReverse[5] = { 0 };  // Example reverse pins
const int ledPin = 0;                      //  pins for LEDs
const int valvePins[2] = { 0, 0 };         //  pins for valves

// Function to initialize pins
void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 6; i++) {
    pinMode(thrusterPins[i], OUTPUT);
  }
  pinMode(ledPin, OUTPUT);

  pinMode(valvePins[0], OUTPUT);
  pinMode(valvePins[1], OUTPUT);
}

// Function to parse and act on received data
void loop() {
  if (Serial.available() >= 9) {  // checks if there are at least 9 bytes available to read from the serial
    byte data[9];
    Serial.readBytes(data, 9);  // reads 9 bytes into the data array once they are available

    // Validate checksum (Byte 8 should be XOR of Byte 6 bits 1-7)
    byte xorCheck = 0;
    for (int i = 0; i < 7; i++) {
      xorCheck ^= (data[5] >> i) & 1;
    }
    if (data[7] != xorCheck) {
      Serial.println("Checksum error");
      return;
    }

    // Control LEDs and valves based on Byte 7
    digitalWrite(ledPin, (data[6] >> 1) & 1);

    for (int i = 0; i < 2; i++) {
      digitalWrite(valvePins[i], (data[6] >> (i)) & 1);
    }

    // Control thrusters based on Bytes 1-5 and directions in Byte 6
    for (int i = 0; i < 5; i++) {
      int speed = data[i];
      if ((data[5] >> i) & 1) {  // Direction bit , 1 = -ve , 0 = +ve 
        digitalWrite(thrusterPinsForward[i], LOW);
        digitalWrite(thrusterPinsReverse[i], HIGH);
        analogWrite(thrusterPinsReverse[i], abs(speed));  // Apply PWM to reverse pin.
      } else {
        digitalWrite(thrusterPinsForward[i], HIGH);
        digitalWrite(thrusterPinsReverse[i], LOW);
        analogWrite(thrusterPinsForward[i], abs(speed));  // Apply PWM to forward pin.
      }
    }
  }
}
