#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Arduino.h>
#include <ESP32Servo.h>
const char *ssid = "globalnet";
const char *password = "gnet240819040812";
int servo = 90;
WebSocketsServer webSocket = WebSocketsServer(80);

// Motor pins
#define MOTOR1_PIN1 12
#define MOTOR1_PIN2 14
#define MOTOR2_PIN1 27
#define MOTOR2_PIN2 26
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
}

void moveBackward()
{
  Serial.println("Moving backward");
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
}

void turnLeft()
{
  Serial.println("Turning left");
  servo = servo - 10;
  if (servo < 0)
  {
    servo = 0;
  }
  myServo.write(servo);
}

void turnRight()
{
  Serial.println("Turning right");
  servo = servo + 10;
  if (servo > 180)
  {
    servo = 180;
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