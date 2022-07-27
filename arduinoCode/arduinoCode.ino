#include "arduinoCode.h"

#pragma region Sleep related functions
void wakeUp()
{
    sleep_disable();
    detachInterrupt(0);
}

void sleepBoard()
{
    sleep_enable();
    attachInterrupt(0, wakeUp, LOW);
    Serial.println("Sleep forever");
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    delay(100);
    sleep_cpu();
}
#pragma endregion Sleep related functions

#pragma region Bluetooth communication functions
void receive()
{
    Serial.println("wakkk");
    // Receive
    if (bluetooth.available())
    {
        String message = bluetooth.readString();
        Serial.println(message);
        callAction(message);
        receive();
    }
}

void callAction(String message)
{
    message.trim();
    char firstLetter = message.charAt(0);
    if (firstLetter == 'B' or firstLetter == 'O' or firstLetter == 'C' or firstLetter == 'I')
    {
        char secondLetter = message.charAt(1);
        switch (secondLetter)
        {
        case 'Z':
            buzzer();
            break;
        case 'V':
            valveActuator(firstLetter);
            break;
        case 'L':
            lightSensor(firstLetter);
            break;
        case 'T':
            temperatureSensor(firstLetter);
            break;
        case 'H':
            humiditySensor();
            break;
        default:
            break;
        }
    }
}

// Bluetooth transmit
void transmit(String message)
{
    bluetooth.println(message);
}
#pragma endregion Bluetooth communication functions

#pragma region Sensors-Actuators functions
// TODO: make buzzer beeping periodically
void buzzer()
{
    String message = "";
    do
    {
        tone(buzzerPin, 1000);
        Serial.println("buzz");
        delay(1000);
        noTone(buzzerPin);
        delay(500);
        if (bluetooth.available())
        {
            message = bluetooth.readString();
            message.trim();
            if (message.compareTo("_BZ") != 0) {
                callAction(message);
            }
        }
    } while (message.compareTo("_BZ") != 0);
}

void valveActuator(char command)
{
    if (command == 'O')
    {
        waterValve.write(180);
    }
    else if (command == 'C')
    {
        waterValve.write(90);
    }
    else
        return;
}

void lightSensor(char command)
{
    int lightLevel = 0;
    if (command == 'I')
    {
        lightLevel = analogRead(ldrInPin);
        String msg = "IL_" + String(lightLevel);
        transmit(msg);
    }
    else if (command == 'O')
    {
        lightLevel = analogRead(ldrOutPin);
        String msg = "OL_" + String(lightLevel);
        transmit(msg);
    }
    else
        return;
}

void temperatureSensor(char command)
{
    float temperature = 0;
    if (command == 'I')
    {
        temperature = dht.readTemperature();
        String msg = "IT_" + String(temperature);
        transmit(msg);
    }
    else if (command == 'O')
    {
        temperature = analogRead(lm35pin);
        float mv = (temperature / 1024.0) * 5000;
        float cel = mv / 10;
        String msg = "OT_" + String(cel);
        transmit(msg);
    }
    else
    {
        return;
    }
}

void humiditySensor()
{
    int humidity = dht.readHumidity();
    String msg = "IH_" + String(humidity);
    transmit(msg);
}
#pragma endregion Sensors-Actuators functions

#pragma region Setup and loop
void setup()
{
    // initialize serial
    Serial.begin(9600);
    // start DHT-11
    dht.begin();
    // Bluetooth setup
    bluetooth.begin(38400);
    // buzzer setup
    pinMode(buzzerPin, OUTPUT);
    // servo setup
    waterValve.attach(waterValvePin);
    // interrupt
    // Setup interrupt pin to trigger interrupt on pullup.
    pinMode(interruptPin, INPUT_PULLUP);
    // delay for sensors initialization
    delay(2000);
}

void loop()
{
    sleepBoard();
    delay(2000);
    receive();
}
#pragma endregion Setup and loop