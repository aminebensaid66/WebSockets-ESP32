#include <Arduino.h>
int sensorPin = 34; // Use an ADC1 pin (e.g., GPIO 34)
int sensorValue = 0;

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    sensorValue = analogRead(sensorPin);          // Read sensor value
    float voltage = sensorValue * (3.3 / 4095.0); // Convert ADC value to voltage
    Serial.print("Voltage: ");
    Serial.println(voltage);
    delay(100);
}