// Bare minimum code for spinning motors triggered by controller input

// Assumes servo is connected to pin 15

// Assumes motor controller IN1 and IN2 are connected to pins 14 and 12

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
#define APDS9960_INT 2
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

TwoWire I2C_0 = TwoWire(0);
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT);

#define IN1 4
#define IN2 16
#define ENA 17
#define IN3 5
#define IN4 18
#define ENB 19
int joyX;
int joyY;
int currentMode;
std::vector<int> sample;
// Servo servo;

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

void setup()
{
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    // servo.attach(15);

    // motor controller outputs
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    Serial.begin(115200);
    joyX = 0;
    joyY = 0;
    currentMode = 0;
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
    joyX = controller->axisX() / 3;
    joyY = controller->axisY() / 3;

    if (joyX > 50)
    {
        rightMotor(0);
        leftMotor(150);
    }
    if (joyX < -50)
    {
        rightMotor(150);
        leftMotor(0);
    }

    if (joyX < 50 && joyX > -50)
    {
        rightMotor(joyY);
        leftMotor(joyY);
    }
}

void loop()
{
    BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        GamepadPtr controller = myGamepads[i];
        if (controller && controller->isConnected())
        {
            if (controller->l1() == 1)
            {
                currentMode = 0;
            }

            else if (controller->r1() == 1)
            {
                currentMode = 1;
            }

            switch (currentMode)
            {
            case 0:
                colorSensing(controller);
                break;
            case 1:
                driving(controller);
                break;
            default:
                driving(controller);
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
