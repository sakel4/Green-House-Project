#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Stepper.h>

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 7        /* Time ESP32 will go to sleep (in seconds) */

// Bluetooth
const unsigned int TXpin = 22;
const unsigned int RXpin = 23;
SoftwareSerial bluetooth(TXpin, RXpin); // (TX, RX)

// Wifi
WiFiClient espClient;
PubSubClient client(espClient);
const char *ssid = "NILE";
const char *password = "n1l3LAB2.4GH";
// Add your MQtt Broker IP address, example (192.168.31.23):
const char *mQtt_server = "10.0.22.202";
const int mQtt_brokerPort = 1883;

// Stepper
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
const unsigned int resolution = 8;
const unsigned int dutyCycle = 200;

// global variables
float metric = 0;
unsigned int doesHumidification = 0;
unsigned int doesDeHumidification = 0;
unsigned int doesCooling = 0;
unsigned int doesHeating = 0;
char shutterState = 'O';

void setup()
{
    // initialize serial
    Serial.begin(9600);
    Serial.println("Setup");
    // Setup time to sleep
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    // Bluetooth setup
    bluetooth.begin(38400);
    // Wifi
    setup_wifi();
    client.setServer(mQtt_server, mQtt_brokerPort);
    client.setCallback(callback);
    // Led pin setup
    pinMode(redLedPin, OUTPUT);
    pinMode(yellowLedPin, OUTPUT);
    pinMode(greenLedPin, OUTPUT);
    pinMode(aiLedPin, OUTPUT);
    // dc motor setup
    pinMode(dcMotorPinIn1, OUTPUT);
    pinMode(dcMotorPinIn2, OUTPUT);
    pinMode(dcMotorPinEN, OUTPUT);
    // configure PWM functionalities
    ledcSetup(pwmChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(dcMotorPinEN, pwmChannel);
    // detect shutter position
    detectShutterPosition();
    delay(2000);
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();
    regulateTemperature();
    humiditySystem();
    //    waterLevelCheck();
    //    soilMoistureCheck();
    Serial.println("Sleep for 10 seconds");
    Serial.flush();
    esp_deep_sleep_start();
}

void loop()
{
}
// TODO: Detect shutter state - position
void detectShutterPosition()
{

    if (touchRead(endTouchSensorPin) <= 40)
        shutterState = 'C';
    else if (touchRead(middleTouchSensorPin) <= 40)
        shutterState = 'M';
    else
        shutterState = 'O';
}

void triggerDC()
{
    digitalWrite(dcMotorPinIn1, HIGH);
    digitalWrite(dcMotorPinIn2, LOW);
    ledcWrite(pwmChannel, dutyCycle);
    delay(3000);
}

void disableDC()
{
    digitalWrite(dcMotorPinIn1, LOW);
    digitalWrite(dcMotorPinIn2, LOW);
}

// TODO: Request Metrics from Slave(Arduino)
void transmit(String message)
{
    Serial.println("Outgoing message");
    Serial.println(message);
    bluetooth.println(message);
}

void getMetric(String metricType)
{
    transmit("_" + metricType);
    long int messageTime = millis() + 15000;
    while (true)
    {
        if (millis() > messageTime)
        {
            Serial.println("Late");
            getMetric(metricType);
            break;
        }
        if (bluetooth.available())
        {
            String message = bluetooth.readString();
            Serial.println(message);
            Serial.println(message.substring(0, 2));
            if (message.substring(0, 2) == metricType)
            {
                String stringValue = message.substring(3);
                float intValue = stringValue.toFloat();
                if (stringValue != "0" and intValue != 0)
                {
                    metric = intValue;
                    break;
                }
                else
                {
                    // request again
                    Serial.println("Non integer");
                    getMetric(metricType);
                    break;
                }
            }
            else
            {
                // request again
                Serial.println("Wrong response");
                getMetric(metricType);
                break;
            }
        }
    }

    return;
}

// Wifi setup
void setup_wifi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32_Client"))
        {
            Serial.println("connected");
            // Subscribe to the subject
            // TODO: subscribe to the node red topics
            client.subscribe("esp32/temperature");
            client.subscribe("esp32/light");
            client.subscribe("esp32/humidity");
            client.subscribe("esp32/waterLevel");
            client.subscribe("esp32/soilMoisture");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void callback(char *topic, byte *message, unsigned int length)
{
}

void sendToSubject(String subject, String message)
{
    client.publish(subject.c_str(), message.c_str());
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
    double value = (1 / result - 100);
    String msg = String(value);
    sendToSubject("esp32/waterLevel", msg);
    return value;
}

void waterLevelCheck()
{
    double capacity = getWaterLevel();
    //    Serial.println(capacity);
    if (capacity < 0.25 * maxWaterTankCapacity)
    {
        // requests buzzer beeping
        transmit("_BZ");
        Serial.println("Buzzer Beeping!");
        // Filling Tank
        Serial.println("Filling Tank!");
        sendToSubject("esp32/events", "Tank_Filling");
        while (true)
        {
            capacity = getWaterLevel();
            //            Serial.println(capacity);
            delay(300);
            if (capacity >= maxWaterTankCapacity)
            {
                // stop buzzer
                transmit("_BZ");
                Serial.println("Buzzer stops!");
                Serial.println("Tank Full!");
                sendToSubject("esp32/events", "Tank_Stop_Filling");
                break;
            }
        }
    }

    return;
}

// TODO: Soil Moisture Detection
/*
 *
    # Below 40%

    - Irrigation starts (until 70% moisture level)

*/

double getSoilMoisture()
{
    unsigned int sensorValue = touchRead(soilMoistureSensorPin);
    // linear equation
    double result = 1.12 * pow(10, -3) * sensorValue - 0.0621;
    double value = (1 / result - 100);
    if (value > 100)
    {
        value = 100;
    }
    String msg = String(value);
    sendToSubject("esp32/soilMoisture", msg);
    return value;
}

void soilMoistureCheck()
{
    double moisture = getSoilMoisture();
    Serial.println(moisture);
    if (moisture < 40)
    {
        // open water tank valve
        transmit("_OV");
        Serial.println("Valve Open!");
        // irrigation
        Serial.println("Irrigation Started!");
        sendToSubject("esp32/events", "Irrigation");
        double soilM = 0;
        do
        {
            soilM = getSoilMoisture();
            Serial.println(soilM);
            delay(500);
        } while (soilM < 70);
        transmit("_CV");
        sendToSubject("esp32/events", "Tank_Stop_Filling");
        Serial.println("Valve Closed!");
        Serial.println("Irrigation Stopped!");
        sendToSubject("esp32/events", "No-Irrigation");
    }

    return;
}

// TODO: Temperature System
/*


    # Outside temperature Between 22C - 31C

    ## Shutter System

    - Open Shutter (when Inside temperature is at least 1C above Outside temperature)
    - Close Shutter (when Inside temperature is at least 1C below Outside temperature)

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
    String msg = String(metric);
    sendToSubject("esp32/temperatureIN", msg);
    unsigned int temperatureIn = metric;
    getMetric("OT");
    msg = String(metric);
    sendToSubject("esp32/temperatureOUT", msg);
    unsigned int temperatureOut = metric;
    getMetric("IL");
    msg = String(metric);
    sendToSubject("esp32/lightIN", msg);
    unsigned int lightIn = metric;
    getMetric("OL");
    msg = String(metric);
    sendToSubject("esp32/lightOUT", msg);
    unsigned int lightOut = metric;

    //    if (temperatureOut >= 22 and temperatureOut <= 31)
    if (temperatureOut >= 22 and temperatureOut <= 23)
    {
        checkShutterState(temperatureIn, temperatureOut, lightIn, lightIn);
    }
    else
    {
        // close shutter - activate air-conditioning
        // TODO: call the shutter position function
        Serial.println("AC");
        airConditioning(temperatureIn, temperatureOut);
    }
}

void airConditioning(unsigned int tempIn, unsigned int tempOut)
{

    //    if (tempIn < 23)
    if (tempIn < 29)
    {
        Serial.println("Activating Hot Air-Conditioning!");
        sendToSubject("esp32/events", "Heating");
        triggerDC();
        doesHeating = 1;
        digitalWrite(redLedPin, HIGH);
        delay(2000);
    }
    //    else if (tempIn >= 28)
    else if (tempIn >= 35)
    {
        Serial.println("Hot Air-Conditioning disabled!");
        doesHeating = 0;
        digitalWrite(redLedPin, LOW);
    }

    if (tempIn > 30)
    {
        Serial.println("Activating Cold Air-Conditioning!");
        sendToSubject("esp32/events", "Cooling");
        triggerDC();
        doesCooling = 1;
        digitalWrite(greenLedPin, HIGH);
        delay(2000);
    }
    else if (tempIn < 25)
    {
        Serial.println("Cold Air-Conditioning disabled!");
        doesCooling = 0;
        digitalWrite(greenLedPin, LOW);
    }

    if (doesCooling == 0 and doesHeating == 0 and doesHumidification == 0 and doesDeHumidification == 0)
    {
        sendToSubject("esp32/events", "No-AC");
        disableDC();
    }
}

void changeShutterState(char state)
{
    unsigned int touched = 0;
    if (state == 'C')
    {
        touched = touchRead(endTouchSensorPin);
        if (shutterState == 'C' or touched < 20)
            return;
        sendToSubject("esp32/events", "Moving_to_the_End");
        Serial.println("Moving_to_the_End");
        do
        {
            // CW movement
            stepper.setSpeed(15);
            stepper.step(stepsPerRevolution);
            delay(800);
            touched = touchRead(endTouchSensorPin);
        } while (touched < 20);
        shutterState = 'C';
        sendToSubject("esp32/events", "Still_at_End");
    }
    else if (state == 'O')
    {
        // TODO: call AI to decide the state (Middle or Fully Open)
        // The light intensity inside the greenhouse should remain between 40%-80% of the external one.
        touched = touchRead(startTouchSensorPin);
        if (shutterState == 'O' or touched < 20)
            return;
        sendToSubject("esp32/events", "Moving_to_the_Start");
        Serial.println("Moving_to_the_Start");
        do
        {
            // CCW movement
            stepper.setSpeed(15);
            stepper.step(-stepsPerRevolution);
            delay(800);
            touched = touchRead(startTouchSensorPin);
        } while (touched < 20);
        sendToSubject("esp32/events", "Still_at_Start");
        shutterState = 'O';
    }
}

void checkShutterState(unsigned int tempIn, unsigned int tempOut, unsigned int lightIn, unsigned int lightOut)
{
    // Open Shutter (when Inside temperature is at least 1C above Outside temperature)
    if (tempIn >= (tempOut + 1))
    {
        changeShutterState('O');
    }
    // Open Shutter (when Inside temperature is at least 1C above Outside temperature)
    else if (tempIn <= (tempOut - 1))
    {
        changeShutterState('C');
    }
}

// TODO: Humidification System (DC Motor)
/*
    # Outside 50% - 70%

    - Active humidification/dehumidification (until humidity reaches 60%)
    - Yellow Led ON (until humidity reaches 60%)
*/

void humiditySystem()
{
    getMetric("IH");
    String msg = String(metric);
    sendToSubject("esp32/humidity", msg);
    unsigned int humidity = metric;

    if (humidity < 50)
    {
        // humidification
        triggerDC();
        digitalWrite(yellowLedPin, HIGH);
        doesHumidification = 1;
    }
    else if (humidity > 70)
    {
        // de-humidification
        triggerDC();
        digitalWrite(yellowLedPin, HIGH);
        doesDeHumidification = 1;
    }

    if (doesHumidification == 0 and humidity >= 60)
    {
        if (doesCooling == 0 and doesHeating == 0 and doesDeHumidification == 0)
        {
            disableDC();
        }
        digitalWrite(yellowLedPin, LOW);
        doesHumidification = 0;
    }

    if (doesDeHumidification == 0 and humidity <= 60)
    {
        if (doesCooling == 0 and doesHeating == 0 and doesHumidification == 0)
        {
            disableDC();
        }
        digitalWrite(yellowLedPin, LOW);
        doesDeHumidification = 0;
    }
}

// TODO: Server Communication (Node red)