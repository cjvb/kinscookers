
//include packages
#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <bits/stdc++.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <Arduino_APDS9960.h>


//color sensor pins
#define APDS9960_INT 2
#define I2C_SDA 21 //colorsensor data1
#define I2C_SCL 22 //colorsensor data2
#define I2C_FREQ 100000

//defined 
TwoWire I2C_0 = TwoWire(0);
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT);
ESP32SharpIR ir_left(ESP32SharpIR::GP2Y0A21YK0F, 32);
ESP32SharpIR ir_right(ESP32SharpIR::GP2Y0A21YK0F, 33);
QTRSensors qtr;
uint16_t sensors[4];
int minBlackVal = 40;
int maxWhiteVal = 40;
bool hasCalibarate = false;
int count = 0;
//turn var
int leftTurn = 0;
int rightTurn = 1;
bool superLeft = false;
bool superRight = false;
int supTurnDelay = 0;
//motor power
int power = 180;
int partialPower = 0;
int superPow = -power;
Servo servo;


//pins for other sensors
#define IN1 4
#define IN2 16
#define ENA 17
#define IN3 5
#define IN4 18
#define ENB 19
#define greenLED 13
#define redLED 12
#define blueLED 2
#define buzzer 14
#define lineReader 25
int joyX;
int joyY;
int currentMode;
std::vector<int> sample;
//Servo servo;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp)
{
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myGamepads[i] == nullptr)
        {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp)
{
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myGamepads[i] == gp)
        {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}

//motor controls ahead
// ENA
void rightMotor(int speed)
{
    if (speed >= 0)
    {
        if (speed > 255)
        {
            speed = 0;
        }
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    else if (speed < 255)
    {
        speed = speed * (-1);
        if (speed > 255)
        {
            speed = 255;
        }
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    analogWrite(ENA, speed);
    // Serial.print("Right Motor :: ");
    // Serial.println(speed);
}
// ENB

void leftMotor(int speed)
{

    if (speed >= 0)
    {
        if (speed > 255)
        {
            speed = 255;
        }
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
    else if (speed < 0)
    {
        speed = speed * (-1);
        if (speed > 255)
        {
            speed = 255;
        }
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    analogWrite(ENB, speed);
    // Serial.print("Left Motor :: ");
    // Serial.println(speed);
}


//lineSensor Controls
void forward(){
    Serial.println("Move forward");
    leftMotor((power*10)/10);
    rightMotor((power*10)/10);
}
void leftShift(){
    Serial.println("Left shift");
    leftMotor(power);
    rightMotor(partialPower);
}
void rightShift(){
    Serial.println("Right shift");
    leftMotor(partialPower);
    rightMotor(power);
}
void superTurn(int turnSide){
    if(turnSide == leftTurn) // left turn
    {
        Serial.println("Super Left");
        rightMotor((superPow*8)/10);
        leftMotor((power*8)/10);
        delay(supTurnDelay);
    }
    if(turnSide == rightTurn) // right turn
    {
        Serial.println("Super Right");
        leftMotor((superPow*8)/10);
        rightMotor((power*8)/10);
        delay(supTurnDelay);
    }

}

//debug code
void ledFeedback(int redL, int greenL, int blueL){
    if(redL == 1)
        digitalWrite(redLED, HIGH);
    else
        digitalWrite(redLED, LOW);
    if(greenL == 1)
        digitalWrite(greenLED, HIGH);
    else
        digitalWrite(greenLED, LOW);
    if(blueL == 1)
        digitalWrite(blueLED, HIGH);
    else
        digitalWrite(blueLED, LOW);
}
void buzz(int buz){
    if (buz == 1)
        digitalWrite(buzzer, HIGH);
    else
        digitalWrite(buzzer, LOW);
}

std::vector<int> initialColor()
{
    int r, g, b, a;
    // Wait until color is read from the sensor
    while (!apds.colorAvailable())
    {
        delay(5);
    }
    apds.readColor(r, g, b, a);

    Serial.print("RED: ");
    Serial.print(r);
    Serial.print(" GREEN: ");
    Serial.print(g);
    Serial.print(" BLUE: ");
    Serial.print(b);
    Serial.print(" AMBIENT: ");
    Serial.println(a);

    std::vector<int> rgb(3);
    rgb[0] = r;
    rgb[1] = g;
    rgb[2] = b;
    return rgb;
}

int totalDiff(std::vector<int> v1, std::vector<int> v2)
{
    int diff = 0;
    for (int i = 0; i < 3; i++)
    {
        diff += abs(v1[i] - v2[i]);
    }
    return diff;
}

void colorSensing(GamepadPtr controller)
{
    ledFeedback(1,0,0);
    if (controller->l1() == 1)
    {
        Serial.println("Sampling initial color...");
        sample = initialColor();
        for (int i = 0; i < 3; i++)
        {
            Serial.print(sample[i] + " ");
        }
        
        std::vector<int> newColor = initialColor();
        int diff = totalDiff(newColor, sample);
        // leftMotor((power*10)/10);
        // rightMotor((power*10)/10);
        while (diff < 50)
        {
            Serial.println("Waiting to begin search");
            delay(250);
            newColor = initialColor();
            diff = totalDiff(newColor, sample);
        }

        Serial.println("Beginning search..");
            Serial.println(diff);
        while (diff >= 50)
        {
            Serial.println("Not the same color!");
            newColor = initialColor();
            diff = totalDiff(newColor, sample);
            Serial.println(diff);
            delay(1000);
        }
        // rightMotor(0);
        // leftMotor(0);
        Serial.println("Found inital color!");
        digitalWrite(2, HIGH); // writes a digital high to pin 2
        delay(10000);          // waits for 10000 milliseconds (10 seconds)
        digitalWrite(2, LOW);
        delay(1000);
    }
    if (controller->r1() == 1)
    {
        int diff = 0;
        std::vector<int> color = initialColor();
        diff = totalDiff(color, sample);
        Serial.println(diff);
        while (diff > 50)
        {
            Serial.println("Not the same color!");
            diff = 0;
            color = initialColor();
            for (int i = 0; i < 3; i++)
            {
                diff += abs(color[i] - sample[i]);
            }
            Serial.println(diff);
            delay(1000);
        }
        Serial.println("Found inital color!");
        digitalWrite(2, HIGH); // writes a digital high to pin 2
        delay(10000);          // waits for 10000 milliseconds (10 seconds)
        digitalWrite(2, LOW);
        delay(1000);
    }
}

void driving(GamepadPtr controller)
{
    ledFeedback(0,1,0);
    if (controller && controller->isConnected()) {

            if (controller->a()) {
                digitalWrite(buzzer, HIGH);
            }
            if (!controller->a()) {
                digitalWrite(buzzer, LOW);
            }
            if (controller->l1() == 1) {
                //Serial.print("Servo move");
                servo.write(1600);
            }
            if (controller->r1() == 1) {
                //Serial.print("Servo move");
                servo.write(1400);
            }
            if (controller->l1() == 0 && controller->r1() == 0) {
                //Serial.print("Servo stop");
                servo.write(1500);
            }
            joyX = controller->axisX() / 3;
            joyY = controller->axisY() / 3;
            rightMotor(joyY+joyX);
            leftMotor(joyY-joyX);
        }
}

void lineSensor() {
        if(hasCalibarate){
        rightMotor(0);
        leftMotor(0);
        hasCalibarate= false;
    }
    qtr.readLineBlack(sensors); // Get calibrated sensor values returned in the sensors array
    Serial.print(sensors[0]);
    Serial.print(" | ");
    Serial.print(sensors[1]);
    Serial.print(" | ");
    Serial.print(sensors[2]);
    Serial.print(" | ");
    Serial.print(sensors[3]);
    Serial.print(" | ");
    Serial.print("SLeft: ");
    Serial.print(superLeft);
    Serial.print(" | ");
    Serial.print("SRight: ");
    Serial.print(superRight);
    Serial.print(" | ");
    Serial.print("liner: ");
    Serial.println(digitalRead(lineReader));

    //delay(250);
    //turnWhere = checkSide(sensors[0], sensors[3]);
    if(sensors[0] > minBlackVal && !superRight){
        superLeft = true;
    }
    if(sensors[3] > minBlackVal  && !superLeft){
        superRight = true;
    }
    if(superRight || superLeft)
    {
        ledFeedback(0, 0, 0);
        buzz(1);
        if(superRight){
            //stop(50);
            superTurn(rightTurn);
            if(digitalRead(lineReader) == 1)
                superRight = false;
        }
        if(superLeft){
            //stop(50);
            superTurn(leftTurn);
            if(digitalRead(lineReader) == 1)
                superLeft = false;
        }
    }
    if((sensors[1] < maxWhiteVal && sensors[2] < maxWhiteVal)&&(!superLeft && !superRight)){
        forward();
        ledFeedback(0, 0, 1);
        buzz(0);
    }
    else if((sensors[1] > minBlackVal) && (!superLeft && !superRight)){
        leftShift();
        ledFeedback(1, 0, 0);
        buzz(0);
    }
    else if((sensors[2] > minBlackVal) && (!superLeft && !superRight)){
        rightShift();
        ledFeedback(0, 1, 0);
        buzz(0);
    }
}

void maze(){
        ledFeedback(0,1,0);
        // check distances fom sensors
        float distLeft = float(ir_left.getDistanceFloat());
        float distRight = float(ir_right.getDistanceFloat());
        Serial.println("LEFT: ");
        Serial.print(distLeft);
        Serial.println("RIGHT: ");
        Serial.println(distRight);
        delay(50);
        

            if (30< distRight) {
                ledFeedback(1,0,1);
                rightMotor(150); 
                leftMotor(-150); 
                leftMotor(-120);
                rightMotor(-120);
                maze();
            } else if(25 <distLeft) {
                ledFeedback(0,1,1);
                rightMotor(-150); 
                leftMotor(150); 
                leftMotor(-120);
                rightMotor(-120);
                maze();
            }

        //if right dist is greater, slow down the right motor
        if (distRight < 11.00) {
            ledFeedback(1,0,0);
            Serial.println("adjust going left");
            rightMotor(-140);
            leftMotor(-130);
            delay(20);
        } 
        //if left left dist is greater, slow down the left motor
        else if (distLeft  < 11.0) {
            ledFeedback(0,1,0);
            Serial.println("adjust going right");
            rightMotor(-110);
            leftMotor(-150);
            delay(20);
        } 
        //if distances equal go straight (may not actually go straight)
        else {
            rightMotor(-166);
            leftMotor(-166);
            Serial.println("go straight");
            delay(100);
        }
        
        delay(1); //delay to prevent over-correction?
    }

void setup()
{
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
    ir_right.setFilterRate(1.0f);
    ir_left.setFilterRate(1.0f);


    pinMode(2, OUTPUT);

    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

     servo.attach(15);
    

    // motor controller outputs
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    apds.setInterruptPin(APDS9960_INT);
    apds.begin();
    Serial.begin(115200);

    
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.forgetBluetoothKeys();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

    joyX = 0;
    joyY = 0;
    currentMode = -1;
    power = 100;

    // set up Serial Communication and sensor pins
    Serial.begin(115200);
    //pin setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(blueLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(buzzer, OUTPUT);
    pinMode(lineReader, INPUT);
    qtr.setTypeAnalog(); // or setTypeRC()
    qtr.setSensorPins((const uint8_t[]) {36, 39, 34, 35}, 4); // pin numbers go in the curly brackets {}, and number of pins goes after
    //calibration sequence
    for (uint8_t i = 0; i < 250; i++) {
        delay(20); 
        Serial.println("calibrating");
        qtr.calibrate();
        superTurn(rightTurn);
        /*if(digitalRead(lineReader) == 1)
            count++;
        if(count%2 == 0)
            superTurn(leftTurn);
        else
            superTurn(rightTurn);*/
        //flash blue light 
        if(i%10 == 0){
            ledFeedback(0, 0, 1);
        }
        if(i%10==5){
            ledFeedback(0, 0, 0);
        }
    }
    hasCalibarate = true;
}


void loop()
{
    BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        GamepadPtr controller = myGamepads[i];
        if (controller && controller->isConnected())
        {
            if (controller->a() == 1)
            {
                currentMode = 0;
            }

            else if (controller->b() == 1)
            {
                currentMode = 1;
            }

            else if (controller->x() == 1)
            {
                currentMode = 2;
            }
            else if (controller->y() == 1)
            {
                currentMode = 3;
            }

            switch (currentMode)
            {
            case 0:
                colorSensing(controller);
                Serial.println("ColorSensingMode");
                break;
            case 1:
                driving(controller);
                Serial.println("Driving");
                break;
            case 2:
                lineSensor();
                Serial.println("Line Sensor");
                break;
            case 3:
                maze();
                Serial.println("Maze");
                break;
            default:
                driving(controller);
                Serial.println("Driving");
                break;
            }

            /*if (controller->l1() == 1) {
                 Serial.print("Servo move");
                 servo.write(1000);
             }
             if (controller->l1() == 0) {
                 Serial.print("Servo stop");
                 servo.write(1500);
             }

             if(controller->axisRY() > 0) { // negative y is upward on stick
                 Serial.println(" DC motor move");
                 digitalWrite(IN1, LOW);
                 digitalWrite(IN2, HIGH);
             }
             if(controller->axisRY() == 0) { // stop motor 1
                 Serial.println(" DC motor stop");
                 digitalWrite(IN1, LOW);
                 digitalWrite(IN2, LOW);
             }*/

            // PHYSICAL BUTTON A
        }
        vTaskDelay(1);
    }
}
