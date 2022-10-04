#include <DHT.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <Servo.h>

// Bluetooth
const unsigned int TXpin = 8;
const unsigned int RXpin = 9;
SoftwareSerial bluetooth(TXpin, RXpin); // (TX, RX)

// DHT-11
const unsigned int dht11pin = 5;
#define DHTTYPE DHT11
DHT dht(dht11pin, DHTTYPE);

// LDR
const unsigned int ldrInPin = A0;
const unsigned int ldrOutPin = A1;

// LM35
const unsigned int lm35pin = A2;

// buzzer
const unsigned int buzzerPin = 6;
unsigned int buzzerState = 0;

// interrupt
const unsigned int interruptPin = 2;

// servo motor
Servo waterValve;
const unsigned int waterValvePin = 7;