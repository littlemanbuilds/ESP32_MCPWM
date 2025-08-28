/**
 * @file main.cpp
 */

#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  delay(50);
}

void loop()
{
  Serial.println("Hello from Little Man Builds...");
  delay(1000);
}