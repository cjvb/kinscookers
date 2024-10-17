// Bare minimum code for spinning motors triggered by controller input

// Assumes servo is connected to pin 15

// Assumes motor controller IN1 and IN2 are connected to pins 14 and 12

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <bits/stdc++.h>


#define IN1 4
#define IN2 16
#define ENA 17
#define IN3 5
#define IN4 18
#define ENB 19
int joyX;
int joyY;
//Servo servo;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}
//ENA
void rightMotor(int speed){
    if(speed >= 0){
        if (speed > 255){
        speed = 0;
        }
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    else if(speed < 255){
    speed = speed*(-1);
        if (speed > 255){
        speed = 255;
        }
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    analogWrite(ENA, speed);
    //Serial.print("Right Motor :: ");
    //Serial.println(speed);
}
//ENB
void leftMotor(int speed){
    
    if(speed >= 0){
        if (speed > 255){
        speed = 255;
        }
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
    else if(speed < 0){
        speed = speed*(-1);
        if (speed > 255){
        speed = 255;
        }
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    analogWrite(ENB, speed);
    //Serial.print("Left Motor :: ");
    //Serial.println(speed);
}

void setup() {
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
 
    //servo.attach(15);

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
}

void loop() {
    BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr controller = myGamepads[i];
        if (controller && controller->isConnected()) {
            if (controller->b()) {
                Serial.println("button a pressed");
            }
            joyX = controller->axisX() / 3;
            joyY = controller->axisY() / 3;
            
            if(joyX>50){
                rightMotor(0);
                leftMotor(150);
            }
            if(joyX<-50){
                rightMotor(150);
                leftMotor(0);
            }
            
            if(joyX<50 && joyX >-50){
                rightMotor(joyY);
                leftMotor(joyY);
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
