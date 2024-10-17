/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>

#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>

#include <Wire.h>
#include <Arduino_APDS9960.h>
#include <bits/stdc++.h>
#include <ColorConverterLib.h>

#define APDS9960_INT 2
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

TwoWire I2C_0 = TwoWire(0);
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT);

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



// Arduino setup function. Runs in CPU 1
void setup() {
    // Setup the Bluepad32 callbacks

    //sets up I2C protocol
    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    //sets up color sensor
    apds.setInterruptPin(APDS9960_INT);
    apds.begin();
    Serial.begin(115200);

    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

    // TODO: Write your setup code here
}

// Arduino loop function. Runs in CPU 1
void loop() {
    

    int r, g, b, a;
    // Wait until color is read from the sensor 
    while (!apds.colorAvailable()) { delay(5); }
    apds.readColor(r, g, b, a);
    // Read color from sensor apds.readColor(r, g, b, a);

    int sum = r + g + b;
    r /= sum; b /= sum; g /= sum;

    r *= 256; b *= 256; g *= 256; //convert to RGB relative values

    // import RGB to HSV library 

    double hue, saturation, value;
    RGBConverterLib::RgbToHsv(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b), hue, saturation, value);

    String Color = classifyToColor(hue);
    Serial.println(Color);

    vTaskDelay(1);
}

String classifyToColor(int hue) {
    if (hue < 15)
  {
   return "Red";
  }
  else if (hue < 45)
  {
    return "Orange";
  }
  else if (hue < 90)
  {
    return "Yellow";
  }
  else if (hue < 150)
  {
    return "Green";
  }
  else if (hue < 210)
  {
    return "Cyan";
  }
  else if (hue < 270)
  {
    return "Blue";
  }
  else if (hue < 330)
  {
    return "Magenta";
  }
  else
  {
    return "None";
  }
}
