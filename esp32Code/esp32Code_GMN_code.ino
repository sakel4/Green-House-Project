#include <SoftwareSerial.h>

SoftwareSerial SerialBT(18, 19); // ARDUNIO PIN 10 RX GOES TO BlueTooth TX AND
                                 // ARDUINO PIN 11 TX GOES TO BlueTooth RX

int water_level = 0;
String beeper;
String val;
int soil_level = 0;
int intemp = 0;
int outemp = 0;
int inldr = -1;
int outldr = -1;
int hum = -1;
bool cooling = false;
bool heating = false;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    SerialBT.begin(38400); // Baud Rate for command Mode.
    Serial.println("Bluetooth started! Ready to pair...");
}

void loop()
{
    // Feed any data from bluetooth to Terminal.
    if (Serial.available())
        SerialBT.write(Serial.read());

    // Feed all data from termial to bluetooth
    if (SerialBT.available())
    {
        String command = SerialBT.readString();
        char temp[50];
        command.toCharArray(temp, 50);
        Serial.write(temp);
        checkCommand(command);
    }
    delay(20);
    checkWater();
    checkSoil();
}

// POSSIBLE INPUTS FROM SLAVE
// TI23
// TO13
// IL0
// OL0
// HI0

void checkCommand(String command)
{

    String firstletter = command.substring(0, 2);
    String value = command.substring(3);
    int intvalue = value.toInt();

    if (firstletter == "TI")
    {
        checkInTemp(intvalue);
    }
    else if (firstletter == "TO")
    {
        checkOutTemp(intvalue);
    }
    else if (firstletter == "IL")
    {
        checkInLdr(intvalue);
    }
    else if (firstletter == "OL")
    {
        checkOutLdr(intvalue);
    }
    else
    {
        checkHumidity(intvalue);
    }
}

void checkWater()
{
    water_level = touchRead(4);
    Serial.println(water_level);

    if (water_level < 25)
    {
        beeper = "bz1";
    }

    else
    {
        beeper = "bz0";
    }
    // Wire.beginTransmission(2);
    // Wire.write(beeper);
    // Wire.endTransmission();
    delay(5000);
}

void checkSoil()
{
    soil_level = touchRead(5);
    Serial.println(soil_level);

    if (soil_level < 40)
    {
        val = "ev1";
    }

    else if (soil_level >= 70)
    {
        val = "ev0";
    }

    /// Wire.beginTransmission(2);
    // Wire.write(val);
    // Wire.endTransmission();
    delay(5000);
}

// if <23 air on if >28 air off
void checkInTemp(int intvalue)
{
    if (intvalue < 23)
    {
        heating = true;
        // hot air on and led on
    }
    else if (intvalue >= 28 && heating)
    {
        // hot air off and led off
        heating = false;
    }
    else if (intvalue > 30)
    {
        cooling = true;
        // cold air on and led on
    }
    else if (intvalue < 25 && cooling)
    {
        // cold air off and led off
        cooling = false;
    }
    intemp = intvalue;
}

void checkOutTemp(int intvalue)
{

    outemp = intvalue;
}

void checkInLdr(int intvalue)
{

    inldr = intvalue;
}

void checkOutLdr(int intvalue)
{

    outldr = intvalue;
}

void checkHumidity(int intvalue)
{
    if (intvalue < 50)
    {
        // start air cond. humidification and led on
    }
    else if (intvalue > 70)
    {
        // start air cond. dehumidificaation and led on
    }
    else if (intvalue == 60)
    {
        // stop air cond. and led off
    }

    hum = intvalue;
}
