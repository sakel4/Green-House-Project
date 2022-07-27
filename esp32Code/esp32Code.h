#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Stepper.h>

// Bluetooth
const unsigned int TXpin = 22;
const unsigned int RXpin = 23;
SoftwareSerial bluetooth(TXpin, RXpin); // (TX, RX)

// Wifi
WiFiClient espClient;
PubSubClient client(espClient);
const char* ssid = "NILE";
const char* password = "n1l3LAB2.4GH";
// Add your MQtt Broker IP address, example (192.168.31.23):
const char* mQtt_server = "10.0.22.202";
const int mQtt_brokerPort = 1883;

// stepper
const unsigned int stepperPinIn1 = 5;
const unsigned int stepperPinIn2 = 19;
const unsigned int stepperPinIn3 = 18;
const unsigned int stepperPinIn4 = 21;
// Defines the number of steps per rotation
const unsigned int stepsPerRevolution = 2038;
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper stepper = Stepper(stepsPerRevolution, stepperPinIn1, stepperPinIn3, stepperPinIn2, stepperPinIn4);

// Led
const unsigned int redLedPin = 25;
const unsigned int yellowLedPin = 26;
const unsigned int greenLedPin = 27;
// AI led, TODO: Find and connect a led
const unsigned int aiLedPin = 2;

// Capacitive Sensors
// TODO: CHECK THE SENSORS AND THE PINS
// Water Level
const unsigned int waterLevelSensorPin = 4;
const unsigned int maxWaterTankCapacity = 450;
// Soil Moisture
const unsigned int soilMoistureSensorPin = 33;
// Touch sensors
const unsigned int startTouchSensorPin = 12;
const unsigned int middleTouchSensorPin = 13;
const unsigned int endTouchSensorPin = 14;

// DC MOTOR
const unsigned int dcMotorPinIn1 = 2;
const unsigned int dcMotorPinIn2 = 15;
const unsigned int dcMotorPinEN = 32;
const unsigned int freq = 30000;
const unsigned int pwmChannel = 0;
unsigned int resolution = 8;
const unsigned int dutyCycle = 200;

// global variables
enum requestType { temp, hum };
int defaultValue = 9999;
float temperatureIn = defaultValue;
float temperatureOut = defaultValue;
unsigned int lightIn = defaultValue;
unsigned int lightOut = defaultValue;
unsigned int humidity = defaultValue;
unsigned int timeOut = 5000;
unsigned int tempRequestCnt = 0;
unsigned int humidityRequestCnt = 0;
unsigned int numOfTempRequests = 4;
unsigned int numOfHumidityRequests = 1;
unsigned int lastTempRequest = millis();
unsigned int lastHumidityRequest = millis();
unsigned int doesHumidification = 0;
unsigned int doesDeHumidification = 0;
unsigned int doesCooling = 0;
unsigned int doesHeating = 0;
char shutterState = 'O';