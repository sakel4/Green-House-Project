#include <SoftwareSerial.h>
#include <Stepper.h>

// Bluetooth
const unsigned int TXpin = 12;
const unsigned int RXpin = 14;
SoftwareSerial bluetooth(TXpin, RXpin); // (TX, RX)

// Stepper
const int stepperPinIn1 = 5;
const int stepperPinIn2 = 19;
const int stepperPinIn3 = 18;
const int stepperPinIn4 = 21;
// Defines the number of steps per rotation
const int stepsPerRevolution = 2038;
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, stepperPinIn1, stepperPinIn3, stepperPinIn2, stepperPinIn4);

// Leds
const int redLedPin = 25;
const int yellowLedPin = 26;
const int greenLedPin = 27;

// Capacitive Sensors
// TODO: CHECK THE SENSORS AND THE PINS
const int waterLevelSensorPin = 4;
const int soilMoistureSensorPin = 33;

//DC MOTOR
void setup(){
    // initialize serial
    Serial.begin(9600);
    // Bluetooth setup
    bluetooth.begin(38400);

}

void loop(){

}

//TODO: Water Level Detection
/*
    # below 25%
    
    - buzzer beeps periodically (until water level rises again)
    - Note: manual water fill
*/

// TODO: Soil Moisture Detection
/*
    # Below 40%

    - Irrigation starts (until 70% moisture level)

*/

// TODO: Request Metrics from Slave(Arduino)

// TODO: Cooling System
/*  


    # Outside temperature Between 22C - 31C

    ## Shutter 

    - Open Shutter (when Inside temperature is 1C above Outside temperature )
    - Close Shutter (when Inside temperature is 1C bellow Outside temperature )

    # Outside temperature outside 22C - 31C

    ##AirCodition
    
    ### Below 23C
    
    - blows HOT air (DC Motor) (until 28C)
    - Red Led ON (until 28C)

    ### Above 30C

    - blows cold air (DC Motor) (until 25C)
    - Green Led ON (until 25C)
*/

// TODO: Humidification System
/*
    # Outside 50% - 70%

    - Active humidification/dehumidification (until humidity reaches 60%)
    - Yellow Led ON (until humidity reaches 60%)
*/

// TODO: Shutter System
/*
    # Outside temperature

    ## 

    -  
*/
