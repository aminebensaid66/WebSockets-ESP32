#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Arduino.h>
#include <ESP32Servo.h>

const char *ssid = "AA";
const char *password = "insat2024";

int servo = 90; // Start at center position
int pwmValue = 50;
int servovariation = 2; // Reduce variation for smoother turning
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

bool turning = false;
int turnDirection = 0; // -1 for left, 1 for right, 0 for neutral
Servo myServo;

unsigned long lastTurnTime = 0; // Timer for smooth turning
const int turnInterval = 100;   // Delay between servo updates (in milliseconds)

void moveForward();
void moveBackward();
void stopCar();
void stopTurning();
void smoothTurn();

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_TEXT:
    String command = (char *)payload;
    Serial.println(command);

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
      turning = true;
      turnDirection = -1; // Left turn
    }
    else if (command == "RIGHT")
    {
      turning = true;
      turnDirection = 1; // Right turn
    }
    else if (command == "STOP")
    {
      stopCar();
    }
    else if (command == "stopTurn")
    {
      stopTurning();
    }

    break;
  }
}

void moveForward()
{
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
  analogWrite(enA, pwmValue);
  analogWrite(enB, pwmValue);
}

void moveBackward()
{
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
  analogWrite(enA, pwmValue);
  analogWrite(enB, pwmValue);
}

void smoothTurn()
{
  if (millis() - lastTurnTime >= turnInterval)
  {
    lastTurnTime = millis(); // Update timer

    if (turnDirection == -1 && servo > 0)
    {
      servo -= servovariation;
      if (servo < 30)
        servo = 30;
    }
    else if (turnDirection == 1 && servo < 150)
    {
      servo += servovariation;
      if (servo > 150)
        servo = 150;
    }

    myServo.write(servo); // Gradual update
  }
}

void stopTurning()
{
  turning = false;
  turnDirection = 0;
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
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(90); // Start at neutral position

  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop()
{
  webSocket.loop();

  if (turning)
  {
    smoothTurn(); // Gradual turning
  }
}