#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Arduino.h>
#include <ESP32Servo.h>

const char *ssid = "GALAXYA24";
const char *password = "225380747";

int servo = 90;
int backwardPWM = 120;
int pwmValue = 100;    // Start at 0 and gradually increase
int targetPWM = 255;   // Final target PWM
int pwmIncrement = 10; // Step size for gradual acceleration
int servovariation = 7;
WebSocketsServer webSocket = WebSocketsServer(80);

#define enA 12
#define MOTOR1_PIN1 27
#define MOTOR1_PIN2 14
#define MOTOR2_PIN2 26
#define MOTOR2_PIN1 25
#define enB 33

#define SERVO_PIN 13

bool turning = false;
bool accelerating = false;
int turnDirection = 0;
Servo myServo;

unsigned long lastTurnTime = 0;
const int turnInterval = 100;
unsigned long lastPWMUpdate = 0;
const int pwmUpdateInterval = 10;

void moveForward();
void moveBackward();
void stopCar();
void stopTurning();
void smoothTurn();
void updatePWM();
void setServo(int);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_TEXT:
    String command = (char *)payload;
    Serial.println(command);

    if (command.startsWith("PWM:"))
    {
      targetPWM = command.substring(4).toInt();
    }
    if (command.startsWith("SERVO:"))
    {
      servovariation = command.substring(6).toInt();
      setServo(servovariation);
    }

    if (command == "FORWARD")
    {
      accelerating = true; // Start acceleration
    }
    else if (command == "BACKWARD")
    {
      moveBackward();
    }
    else if (command == "LEFT")
    {
      turning = true;
      turnDirection = -1;
    }
    else if (command == "RIGHT")
    {
      turning = true;
      turnDirection = 1;
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
}

void moveBackward()
{
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
  analogWrite(enA, backwardPWM);
  analogWrite(enB, backwardPWM);
}

void updatePWM()
{
  if (accelerating && millis() - lastPWMUpdate >= pwmUpdateInterval)
  {
    lastPWMUpdate = millis();

    if (pwmValue < targetPWM)
    {
      pwmValue += pwmIncrement;
      if (pwmValue > targetPWM)
        pwmValue = targetPWM;
    }

    analogWrite(enA, pwmValue);
    analogWrite(enB, pwmValue);
  }
}

void smoothTurn()
{
  if (millis() - lastTurnTime >= turnInterval)
  {
    lastTurnTime = millis();

    if (turnDirection == -1 && servo > 50)
    {
      servo -= servovariation;
    }
    else if (turnDirection == 1 && servo < 150)
    {
      servo += servovariation;
    }

    myServo.write(servo);
  }
}
void setServo(int angle)
{
  myServo.write(angle);
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
  pwmValue = 50;
  accelerating = false;
  analogWrite(enA, 0);
  analogWrite(enB, 0);
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
  myServo.write(90);

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

  if (accelerating)
  {
    moveForward();
    updatePWM();
  }

  if (turning)
  {
    smoothTurn();
  }
}