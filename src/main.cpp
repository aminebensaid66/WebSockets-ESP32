#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Arduino.h>
#include <ESP32Servo.h>
const char *ssid = "AA";
const char *password = "insat2024";
int servo = 110;
int pwmValue = 50;
int servovariation = 10;
WebSocketsServer webSocket = WebSocketsServer(80);
// Motor pins
#define enA 12
#define MOTOR1_PIN1 27
#define MOTOR1_PIN2 14
#define MOTOR2_PIN2 26
#define MOTOR2_PIN1 25
#define enB 33
// Servo pin
#define SERVO_PIN 13
Servo myServo;
void turnLeft();
void turnRight();
void moveForward();
void moveBackward();
void stopCar();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_TEXT:
    // Process the command
    String command = (char *)payload;
    if (command.startsWith("PWM:"))
    {
      pwmValue = command.substring(4).toInt();
    }
    if (command.startsWith("SERVO:"))
    {
      servovariation = command.substring(6).toInt();
    }

    if (command == "FORWARD")
    {
      moveForward();
    }
    else if (command == "BACKWARD")
    {
      moveBackward();
    }
    else if (command == "LEFT")
    {
      turnLeft();
    }
    else if (command == "RIGHT")
    {
      turnRight();
    }
    else if (command == "STOP")
    {
      stopCar();
    }
    break;
  }
}

void moveForward()
{
  Serial.println("Moving forward");
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
  analogWrite(enA, pwmValue);
  analogWrite(enB, pwmValue);
}

void moveBackward()
{
  Serial.println("Moving backward");
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
  analogWrite(enA, pwmValue);
  analogWrite(enB, pwmValue);
}

void turnLeft()
{
  Serial.println("Turning left");
  servo = servo - servovariation;
  if (servo < 0)
  {
    servo = 0;
  }
  myServo.write(servo);
}

void turnRight()
{
  Serial.println("Turning right");
  servo = servo + servovariation;
  if (servo > 150)
  {
    servo = 150;
  }
  myServo.write(servo);
}

void stopCar()
{
  Serial.println("Stopping car");
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void setup()
{
  // Initialize motor pins
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  // Initialize servo pin
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  // Connect to Wi-Fi
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop()
{
  webSocket.loop();
}