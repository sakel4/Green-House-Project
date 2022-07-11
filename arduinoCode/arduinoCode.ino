#include <dht.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <Servo.h>

// Bluetooth
const unsigned int TXpin = 8;
const unsigned int RXpin = 9;
SoftwareSerial bluetooth(TXpin, RXpin); // (TX, RX)

// DHT-11
dht DHT;
const unsigned int dht11pin = 5;

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

void setup()
{
    // initialize serial
    Serial.begin(9600);
    // start DHT-11
    DHT.read11(dht11pin);
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

void wakeUp()
{
    sleep_disable();
    detachInterrupt(0);
}

void sleepBoard()
{
    sleep_enable();
    attachInterrupt(0, wakeUp, LOW);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    delay(100);
    sleep_cpu();
}

void receive()
{
    Serial.println("wakkk");
    // Receive
    if (bluetooth.available())
    {
        String message = bluetooth.readString();
        Serial.println(message);
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
}

// Bluetooth transmit
void transmit(String message)
{
    bluetooth.println(message);
}
// TODO: make buzzer beeping periodically
void buzzer()
{
    buzzerState = !buzzerState;
    if (buzzerState == 1)
        tone(buzzerPin, 100);
    else
        noTone(buzzerPin);
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
    int temperature = 0;
    if (command == 'I')
    {
        temperature = DHT.temperature;
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
    int humidity = DHT.humidity;
    String msg = "IH_" + String(humidity);
    transmit(msg);
}
