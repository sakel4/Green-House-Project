#include "esp32Code.h"

#pragma region General use Functions
bool isLate(unsigned int timestamp) {
    return (millis() > timestamp + timeOut) ? true : false;
}

bool isDefaultValue(unsigned int value, bool isFullCheck, requestType type) {
    unsigned int request = (type == temp) ? tempRequestCnt : humidityRequestCnt;
    unsigned int numOfRequests = (type == temp) ? numOfTempRequests : numOfHumidityRequests;
    if (isFullCheck)
        return (value == defaultValue and request != numOfRequests) ? true : false;
    else
        return (value == defaultValue) ? true : false;
}

void resetValues() {
    // reset values to default
    temperatureIn = defaultValue;
    temperatureOut = defaultValue;
    lightIn = defaultValue;
    lightOut = defaultValue;
    tempRequestCnt = 0;
    //reset humidity value to default
    humidity = defaultValue;
    humidityRequestCnt = 0;
}
#pragma endregion General use Functions

#pragma region Shutter system
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
#pragma endregion Shutter system

#pragma region DC Motor Movement
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
#pragma endregion DC Motor Movement

#pragma region Bluetooth Communication
// TODO: Request Metrics from Slave(Arduino)
void transmit(String message)
{
    message = "_" + message;
    Serial.println("Outgoing message");
    Serial.println(message);
    bluetooth.println(message);
    delay(1000);
}

void bluetoothReceive()
{
    if (bluetooth.available())
    {
        //receive message
        String message = bluetooth.readString();
        Serial.println(message);
        String type = (message.substring(0, 2));
        //check if the first character is valid
        if (type.charAt(0) == 'O' or type.charAt(0) == 'I')
        {
            //get end convert the value to float
            float value = (message.substring(3)).toFloat();
            message = message.substring(3);
            // validate if the value is number 
            if (message != "0" and value == 0)
                return;
            //check if the second character is valid
            switch (type.charAt(1))
            {
            case 'L':
                if (type.charAt(0) == 'I')
                {
                    lightIn = value;
                    sendToSubject("esp32/lightIN", message);
                }
                else
                {
                    lightOut = value;
                    sendToSubject("esp32/lightOUT", message);
                }
                break;
            case 'T':
                if (type.charAt(0) == 'I')
                {
                    temperatureIn = value;
                    sendToSubject("esp32/temperatureIN", message);
                }
                else
                {
                    temperatureOut = value;
                    sendToSubject("esp32/temperatureOUT", message);
                }
                break;
            case 'H':
                humidity = value;
                sendToSubject("esp32/humidity", message);
                break;
            default:
                break;
            }
        }
    }
}
#pragma endregion Bluetooth Communication

#pragma region Wifi - NodeRed communication
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
            // Subscribe to the topics
            client.subscribe("esp32/automaticMode");
            client.subscribe("esp32/valve");
            client.subscribe("esp32/shutter");
            client.subscribe("esp32/fan_cold");
            client.subscribe("esp32/fan_hot");
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

void callback(char* topic, byte* message, unsigned int length)
{
    //Print that received message
    String messageTemp;
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }
    Serial.println();

    if (String(topic) == "esp32/automaticMode") {
        if (messageTemp == "true")
            automaticMode = true;
        else
            automaticMode = false;
    }
    if (automaticMode == true)
        return;
    // check if it is from a valid topic and execute the valid actions
    if (String(topic) == "esp32/valve")
    {
        Serial.println("Check");
        Serial.println(String(topic));
        Serial.println(messageTemp);
        if (messageTemp == "true" or messageTemp == "false")
            binaryDevicesControl("valve", messageTemp);
        else
            return;
    }
    if (String(topic) == "esp32/shutter")
    {
        Serial.println("enter");
        if (messageTemp == "0")
            changeShutterState('C');
        else if (messageTemp == "1")
            changeShutterState('M');
        else if (messageTemp == "2")
            changeShutterState('O');
        else
            return;
    }
    if (String(topic) == "esp32/fan_cold")
    {
        if (messageTemp == "true" or messageTemp == "false")
            binaryDevicesControl("fan_cold", messageTemp);
        else
            return;
    }
    if (String(topic) == "esp32/fan_hot")
    {
        if (messageTemp == "true" or messageTemp == "false")
            binaryDevicesControl("fan_hot", messageTemp);
        else
            return;
    }
}

void sendToSubject(String subject, String message)
{
    client.publish(subject.c_str(), message.c_str());
}

void binaryDevicesControl(String device, String state)
{
    // valve
    if (device == "valve")
    {
        Serial.println("Check");
        if (state == "true")
            transmit("OV");
        else
            transmit("CV");
    }
    unsigned int led;
    // fan
    if (device == "fan_hot")
    {
        led = redLedPin;
        Serial.print("Hot fan");
    }
    else if (device == "fan_cold")
    {
        led = greenLedPin;
        Serial.print("Cold fan");
    }
    else
        return;

    if (state == "true")
    {
        Serial.println(" ON");
        digitalWrite(led, HIGH);
        triggerDC();
    }
    else
    {
        Serial.println(" OFF");
        digitalWrite(led, LOW);
        disableDC();
    }

    return;
}
#pragma endregion Wifi - NodeRed communication

#pragma region DIY Sensors
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
    if (isDefaultValue(humidity, false, hum))
        return;

    double capacity = getWaterLevel();
    //    Serial.println(capacity);
    if (capacity < 0.25 * maxWaterTankCapacity and isFillingTank == false)
    {
        // requests buzzer beeping
        transmit("BZ");
        Serial.println("Buzzer Beeping!");
        // Filling Tank
        Serial.println("Filling Tank!");
        sendToSubject("esp32/events", "Tank_Filling");
        isFillingTank = true;
    }
    if (capacity >= maxWaterTankCapacity and isFillingTank == true) {

        // stop buzzer
        transmit("BZ");
        Serial.println("Buzzer stops!");
        Serial.println("Tank Full!");
        sendToSubject("esp32/events", "Tank_Stop_Filling");
        isFillingTank = false;
    }
    resetValues();

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
    if (moisture < 40 and isIrrigating == false)
    {
        // open water tank valve
        transmit("OV");
        Serial.println("Valve Open!");
        // irrigation
        Serial.println("Irrigation Started!");
        sendToSubject("esp32/events", "Irrigation");
        isIrrigating = true;
    }
    if (isIrrigating == true and moisture >= 70) {
        transmit("CV");
        sendToSubject("esp32/events", "Tank_Stop_Filling");
        Serial.println("Valve Closed!");
        Serial.println("Irrigation Stopped!");
        sendToSubject("esp32/events", "No-Irrigation");
        isIrrigating = false;
    }



    return;
}
#pragma endregion Custom Sensors

#pragma region Light and temperature control
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

    ### Below 23C (inside temperature)

    - blows HOT air (DC Motor) (until 28C)
    - Red Led ON (until 28C)

    ### Above 30C (inside temperature)

    - blows cold air (DC Motor) (until 25C)
    - Green Led ON (until 25C)
*/

void requestTemperature()
{
    // request values
    //tempIn
    if (isDefaultValue(temperatureIn, true, temp)) {
        tempRequestCnt++;
        transmit("IT");
        lastTempRequest = millis();
    }
    else if (isLate(lastTempRequest) and isDefaultValue(temperatureIn, false, temp))
        tempRequestCnt--;

    //tempOut
    if (isDefaultValue(temperatureOut, true, temp)) {
        tempRequestCnt++;
        transmit("OT");
        lastTempRequest = millis();
    }
    else if (isLate(lastTempRequest) and isDefaultValue(temperatureOut, false, temp))
        tempRequestCnt--;

    // LightIn
    if (isDefaultValue(lightIn, true, temp)) {
        tempRequestCnt++;
        transmit("IL");
        lastTempRequest = millis();
    }
    else if (isLate(lastTempRequest) and isDefaultValue(lightIn, false, temp))
        tempRequestCnt--;

    //LightOut
    if (isDefaultValue(lightOut, true, temp)) {
        tempRequestCnt++;
        transmit("OL");
        lastTempRequest = millis();
    }
    else if (isLate(lastTempRequest) and isDefaultValue(lightOut, false, temp))
        tempRequestCnt--;

    // exit if any of the value has not been received 
    if (isDefaultValue(lightIn, false, temp) or
        isDefaultValue(lightOut, false, temp) or
        isDefaultValue(temperatureOut, false, temp) or
        isDefaultValue(temperatureIn, false, temp))
        return;
    Serial.println("Got all values");
    //    if (temperatureOut >= 22 and temperatureOut <= 31)
    if (temperatureOut >= 22 and temperatureOut <= 31)
        checkShutterState();
    else
    {
        // close shutter - activate air-conditioning
        // TODO: call the shutter position function
        Serial.println("AC");
        airConditioning();
    }
}

void airConditioning()
{
    //    if (temperatureIn < 23)
    if (temperatureIn < 23)
    {
        Serial.println("Activating Hot Air-Conditioning!");
        sendToSubject("esp32/events", "Heating");
        digitalWrite(redLedPin, HIGH);
        triggerDC();
        doesHeating = 1;
        delay(2000);
    }
    //    else if (temperatureIn >= 28)
    else if (temperatureIn >= 28)
    {
        Serial.println("Hot Air-Conditioning disabled!");
        doesHeating = 0;
        digitalWrite(redLedPin, LOW);
    }

    if (temperatureIn > 30)
    {
        Serial.println("Activating Cold Air-Conditioning!");
        sendToSubject("esp32/events", "Cooling");
        digitalWrite(greenLedPin, HIGH);
        triggerDC();
        doesCooling = 1;
        delay(2000);
    }
    else if (temperatureIn < 25)
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

void moveShutter(int direction, unsigned int sensorPin)
{
    unsigned int step;
    unsigned int touched = 0;
    if (direction > 0)
        step = stepsPerRevolution;
    else
        step = -stepsPerRevolution;
    do
    {
        // CW movement
        stepper.setSpeed(10);
        stepper.step(step);
        delay(800);
        touched = touchRead(sensorPin);
        Serial.println(touched);
    } while (touched > 20);
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
        moveShutter(1, endTouchSensorPin);
        shutterState = 'C';
        sendToSubject("esp32/events", "Still_at_End");
    }
    else if (state == 'O')
    {
        touched = touchRead(startTouchSensorPin);
        if (shutterState == 'O' or touched < 20)
            return;
        sendToSubject("esp32/events", "Moving_to_the_Start");
        Serial.println("Moving_to_the_Start");
        moveShutter(-1, startTouchSensorPin);
        sendToSubject("esp32/events", "Still_at_Start");
        shutterState = 'O';
    }
    else if (state == 'M')
    {
        touched = touchRead(middleTouchSensorPin);
        if (shutterState == 'M' or touched < 20)
            return;
        sendToSubject("esp32/events", "Moving_to_the_Middle");
        Serial.println("Moving_to_the_Middle");
        if (shutterState == 'C')
            moveShutter(-1, middleTouchSensorPin);
        else
            moveShutter(1, middleTouchSensorPin);
        sendToSubject("esp32/events", "Still_at_Middle");
        shutterState = 'M';
    }
}

void checkShutterState()
{
    // Open Shutter (when Inside temperature is at least 1C above Outside temperature)
    if (temperatureIn >= (temperatureOut + 1))
    {
        // TODO: call AI to decide the state (Middle or Fully Open)
        // The light intensity inside the greenhouse should remain between 40%-80% of the external one.
        changeShutterState('O');
    }
    // Open Shutter (when Inside temperature is at least 1C above Outside temperature)
    else if (temperatureIn <= (temperatureOut - 1))
    {
        changeShutterState('C');
    }
}
#pragma endregion Light and temperature control

#pragma region Humidity
// TODO: Humidification System (DC Motor)
/*
    # Outside 50% - 70%
    - Active humidification/dehumidification (until humidity reaches 60%)
    - Yellow Led ON (until humidity reaches 60%)
*/

void humiditySystem()
{
    if (isDefaultValue(humidity, true, hum)) {
        humidityRequestCnt++;
        transmit("IH");
        lastHumidityRequest = millis();
    }
    else if (isLate(lastHumidityRequest) and isDefaultValue(humidity, false, hum))
        humidityRequestCnt--;

    if (isDefaultValue(humidity, false, hum))
        return;

    if (humidity < 50)
    {
        // humidification
        digitalWrite(yellowLedPin, HIGH);
        Serial.println("Start Humidification");
        sendToSubject("esp32/events", "Humidification");
        triggerDC();
        doesHumidification = 1;
    }
    else if (humidity > 70)
    {
        // de-humidification
        digitalWrite(yellowLedPin, HIGH);
        Serial.println("Start DeHumidification");
        sendToSubject("esp32/events", "DeHumidification");
        triggerDC();
        doesDeHumidification = 1;
    }

    if (doesHumidification == 1 and humidity >= 60)
    {
        Serial.println("Stop Humidification");
        sendToSubject("esp32/events", "No_Action");
        if (doesCooling == 0 and doesHeating == 0 and doesDeHumidification == 0)
        {
            disableDC();
        }
        if (doesDeHumidification == 0)
            digitalWrite(yellowLedPin, LOW);
        doesHumidification = 0;
    }

    if (doesDeHumidification == 1 and humidity <= 60)
    {
        Serial.println("Stop StepHumidification");
        sendToSubject("esp32/events", "No_Action");
        if (doesCooling == 0 and doesHeating == 0 and doesHumidification == 0)
        {
            disableDC();
        }
        if (doesHumidification == 0)
            digitalWrite(yellowLedPin, LOW);
        doesDeHumidification = 0;
    }
    resetValues();
}
#pragma endregion Humidity

#pragma region Setup and Loop

void setup()
{
    Serial.println("Setup");
    // initialize serial
    Serial.begin(9600);
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
    Serial.println("Green House Setup Completed");
    Serial.flush();
}

void loop()
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();
    if (automaticMode) {
        bluetoothReceive();
        requestTemperature();
        humiditySystem();
        waterLevelCheck();
        //    soilMoistureCheck();
    }
}
#pragma endregion region Setup and Loop