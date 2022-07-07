#include <SoftwareSerial.h>
#include <Stepper.h>

// Bluetooth
const unsigned int TXpin = 12;
const unsigned int RXpin = 14;
SoftwareSerial bluetooth(TXpin, RXpin); // (TX, RX)

// Stepper
const unsigned int stepperPinIn1 = 5;
const unsigned int stepperPinIn2 = 19;
const unsigned int stepperPinIn3 = 18;
const unsigned int stepperPinIn4 = 21;
// Defines the number of steps per rotation
const unsigned int stepsPerRevolution = 2038;
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, stepperPinIn1, stepperPinIn3, stepperPinIn2, stepperPinIn4);

// Led
const unsigned int redLedPin = 25;
const unsigned int yellowLedPin = 26;
const unsigned int greenLedPin = 27;
// AI led, TODO: Find and connect a led
const unsigned int aiLedPin = 32;

// Capacitive Sensors
// TODO: CHECK THE SENSORS AND THE PINS
// Water Level
const unsigned int waterLevelSensorPin = 4;
const unsigned int maxWaterTankCapacity = 450;
// Soil Moisture
const unsigned int soilMoistureSensorPin = 33;
// Touch sensors
// TODO: Find touch sensors pins
const unsigned int startTouchSensor = 33;
const unsigned int middleTouchSensor = 33;
const unsigned int endTouchSensor = 33;

// DC MOTOR
const unsigned int dcMotorPin = 34;

// global variables
unsigned int metric = 0;

void setup()
{
    // initialize serial
    Serial.begin(9600);
    // Bluetooth setup
    bluetooth.begin(38400);
    // Led pin setup
    pinMode(redLedPin, OUTPUT);
    pinMode(yellowLedPin, OUTPUT);
    pinMode(greenLedPin, OUTPUT);
    pinMode(aiLedPin, OUTPUT);
    // dc motor setup
    pinMode(dcMotorPin, OUTPUT);
}

void loop()
{
    // TODO: Setup deep sleep
    // TODO: delay for sensors
    // TODO: Check if bluetooth received message
    //       Execute action
    // TODO: Request Metrics
}

// TODO: Request Metrics from Slave(Arduino)
void transmit(String message)
{
    bluetooth.println(message);
}

void getMetric(String metricType)
{
    transmit("_" + metricType);
    while (true)
    {
        if (bluetooth.available())
        {
            String message = bluetooth.readString();
            Serial.println(message);
            Serial.println(message.substring(0, 2));
            if (message.substring(0, 2) == metricType)
            {
                String stringValue = message.substring(3);
                unsigned int intValue = stringValue.toInt();
                if (stringValue != "0" and intValue != 0)
                {
                    metric = intValue;
                    break;
                }
                else
                {
                    // request again
                    getMetric(metricType);
                    break;
                }
            }
            else
            {
                // request again
                getMetric(metricType);
                break;
            }
        }
    }

    return;
}

// TODO: Water Level Detection
/*
    # below 25%

    - buzzer beeps periodically (until water level rises again)
    - Note: manual water fill
*/

double getWaterLevel()
{
    unsigned int sensorValue = touchRead(waterLevelSensorPin);
    // linear equation
    double result = 2.13 * pow(10, -4) * sensorValue - 1.27 * pow(10, -4);
    return (1 / result - 100);
}

void waterLevelCheck()
{
    double capacity = getWaterLevel();

    if (capacity < 0.25 * maxWaterTankCapacity)
    {
        // requests buzzer beeping
        transmit("_BZ");
        Serial.println("Buzzer Beeping!");
        // Filling Tank
        Serial.println("Filling Tank!");
        while (true)
        {
            if (capacity >= maxWaterTankCapacity)
            {
                // stop buzzer
                transmit("_BZ");
                Serial.println("Buzzer stops!");
                Serial.println("Tank Full!");
                break;
            }
        }
    }

    return;
}

// TODO: Soil Moisture Detection
/*
    # Below 40%

    - Irrigation starts (until 70% moisture level)

*/

double getSoilMoisture()
{
    unsigned int sensorValue = touchRead(soilMoistureSensorPin);
    // linear equation
    double result = 1.12 * pow(10, -3) * sensorValue - 0.0621;
    return (1 / result - 100);
}

void soilMoistureCheck()
{
    double moisture = getSoilMoisture();

    if (moisture < 40)
    {
        // open water tank valve
        transmit("_OV");
        Serial.println("Valve Open!");
        // irrigation
        Serial.println("Irrigation Started!");
        while (true)
        {
            if (getSoilMoisture() >= 70)
            {
                transmit("_CV");
                Serial.println("Valve Closed!");
                Serial.println("Irrigation Stopped!");
                break;
            }
        }
    }

    return;
}

// TODO: Temperature System
/*


    # Outside temperature Between 22C - 31C

    ## Shutter System

    - Open Shutter (when Inside temperature is 1C above Outside temperature)
    - Close Shutter (when Inside temperature is 1C below Outside temperature)

    ### Height Regulation (AI System)

    - The light intensity inside the greenhouse should remain between 40%-80% of the external one.


    # Outside temperature outside 22C - 31C

    ##AirCondition System

    ### Below 23C

    - blows HOT air (DC Motor) (until 28C)
    - Red Led ON (until 28C)

    ### Above 30C

    - blows cold air (DC Motor) (until 25C)
    - Green Led ON (until 25C)
*/

void regulateTemperature()
{
    getMetric("IT");
    unsigned int temperatureIn = metric;
    getMetric("OT");
    unsigned int temperatureOut = metric;
    getMetric("IL");
    unsigned int lightIn = metric;
    getMetric("OL");
    unsigned int lightOut = metric;

    if (temperatureOut >= 22 and temperatureOut <= 31)
        checkShutterState(temperatureIn, temperatureOut, lightIn, lightIn);
    else
    {
        // close shutter - activate air-conditioning
        // TODO: call the shutter position function
        airConditioning(temperatureIn, temperatureOut);
    }
}

void airConditioning(unsigned int tempIn, unsigned int tempOut)
{
    if (tempIn < 23)
    {
        Serial.println("Activating Hot Air-Conditioning!");
        digitalWrite(redLedPin, HIGH);
    }
    else if (tempIn >= 28)
    {
        Serial.println("Hot Air-Conditioning disabled!");
        digitalWrite(redLedPin, LOW);
    }

    if (tempIn > 30)
    {
        Serial.println("Activating Cold Air-Conditioning!");
        digitalWrite(greenLedPin, HIGH);
    }
    else if (tempIn < 25)
    {
        Serial.println("Cold Air-Conditioning disabled!");
        digitalWrite(greenLedPin, LOW);
    }
}

void checkShutterState(unsigned int tempIn, unsigned int tempOut, unsigned int lightIn, unsigned int lightOut)
{
    // TODO: logic
}

// TODO: Humidification System (DC Motor)
/*
    # Outside 50% - 70%

    - Active humidification/dehumidification (until humidity reaches 60%)
    - Yellow Led ON (until humidity reaches 60%)
*/

// TODO: Deep Sleep for power saving

// TODO: Server Communication (Node red)