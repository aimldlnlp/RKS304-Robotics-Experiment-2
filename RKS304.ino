#include <Servo.h>

Servo lifting1;
Servo lifting2;

// Left Motor (Motor A) pin definitions
#define pwmA 5  // PWM speed control pin for left motor
#define in1A 8  // Direction control pin 1 for left motor
#define in2A 7  // Direction control pin 2 for left motor

// Right Motor (Motor B) pin definitions
#define pwmB 3   // PWM speed control pin for right motor
#define in1B 12  // Direction control pin 1 for right motor
#define in2B 11  // Direction control pin 2 for right motor

// Motor speed variables
int MotorSpeed1 = 0;  // Current speed for motor 1
int MotorSpeed2 = 0;  // Current speed for motor 2

int leftSpeed = 0;
int rightSpeed = 0;

// Servo motor pin definition
#define servo1Pin 4  // Control pin for servo motor
#define servo2Pin 4  // COntrol pin for servo motor

// Ultrasonic sensor pins and variables
#define trigPin 6    // Trigger pin for ultrasonic sensor
#define echoPin 2    // Echo pin for ultrasonic sensor
float timing = 0.0;  // Store pulse duration
float distance;      // Store calculated distance

// Line following parameters
int baseSpeed = 60;  // Base motor speed for straight line movement

bool detect = false;  // Detection flag
bool detect2 = false;

// Error tracking
int errorValue = 0;  // Current error value for PD control

// Line sensor array setup
int sensor[6] = { A5, A4, A3, A2, A1, A0 };  // Analog pins for line sensors

// Calibration arrays for line sensors
int sensorMin[6] = { 1023, 1023, 1023, 1023, 1023, 1023 };  // Minimum values (initialized high)
int sensorMax[6] = { 0, 0, 0, 0, 0, 0 };                    // Maximum values (initialized low)
int averageSensor[6] = { 0, 0, 0, 0, 0, 0 };                // Average values for calibration
int riilSensor[6] = { 0, 0, 0, 0, 0, 0 };                   // Raw sensor readings
int outputSensor[6] = { 0, 0, 0, 0, 0, 0 };                 // Processed sensor outputs

//  Sensor Array   Error Value
//  0 0 0 0 0 1         -5
//  0 0 0 0 1 1         -4
//  0 0 0 0 1 0         -3
//  0 0 0 1 1 0         -2
//  0 0 0 1 0 0          1
//  0 0 1 1 0 0          0
//  0 0 1 0 0 0          1
//  0 1 1 0 0 0          2
//  0 1 0 0 0 0          3
//  1 1 0 0 0 0          4
//  1 0 0 0 0 0          5
//  0 0 0 0 0 0          Error

// Controls the speed of both motors using PWM
void Speed(int a, int b) {
  analogWrite(pwmA, a);
  analogWrite(pwmB, b);
}


// Makes the robot move forward at specified speeds
void Forward(int a, int b) {
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);

  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);

  Speed(a, b);
}

// Stops both motors
void Stop() {
  // Set all direction pins to LOW to stop motors
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);

  Speed(0, 0);  // Set speed to zero for both motors
}

void setup() {
  Serial.begin(9600);

  // Setup servo motor for lifting mechanism
  lifting1.attach(servo1Pin);
  lifting2.attach(servo2Pin);

  lifting1.write(180);
  lifting2.write(180);

  // Setup ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Setup dual motor control pins
  // Motor A
  pinMode(pwmA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);

  // Motor B
  pinMode(pwmB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Calibration process
  int sensorValue[6] = { 0, 0, 0, 0, 0, 0 };



  // /*

  // Calibration start sign
  Serial.println("Calibrating...");

  unsigned long timeStartCalibration = millis();
  while (millis() - timeStartCalibration < 7000) {
    for (int i = 0; i < 6; i++) {
      sensorValue[i] = analogRead(sensor[i]);

      // Find max value
      if (sensorValue[i] > sensorMax[i]) {
        sensorMax[i] = sensorValue[i];
      }

      // Find min value
      if (sensorValue[i] < sensorMin[i]) {
        sensorMin[i] = sensorValue[i];
      }
    }
  }

  // Calculate the sensor average
  for (int i = 0; i < 6; i++) {
    averageSensor[i] = (sensorMax[i] + sensorMin[i]) * 0.5;
  }
}

void loop() {
  while (!detect) {
    PDLineFollowing();
    distance = getDistance();

    if (distance <= 10 && distance != 0) {
      detect = true;
      break;
    }
  }

  unsigned long startTime = millis();

  while (detect) {
    unsigned long timeObjectLifting = millis();
    PDLineFollowing();
    unsigned long currentTime = millis();
    if (currentTime - startTime > 1100) {  // Check if 1.1 seconds have passed
      detect = false;
      break;
    }
  }

  Forward(0, 0);
  delay(2000);
  lifting1.write(90);
  lifting2.write(90);
  delay(1000);

  detect = true;

  while (detect) {
    PDLineFollowing();
    if ((outputSensor[0] == 0) && (outputSensor[1] == 0) && (outputSensor[2] == 0) && (outputSensor[3] == 0) && (outputSensor[4] == 0) && (outputSensor[5] == 0)) {
      detect = false;
      break;
    }
  }

  Forward(0, 0);
  delay(2000);
  lifting1.write(180);
  lifting2.write(180);
  delay(1000);

  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);

  Speed(70, 70);

  delay(1000);

  detect = true;

  while (detect) {
    Forward(0, 0);
  }
}

//  Sensor Array   Error Value
//  0 0 0 0 0 1         -5
//  0 0 0 0 1 1         -4
//  0 0 0 0 1 0         -3
//  0 0 0 1 1 0         -2
//  0 0 0 1 0 0         -1
//  0 0 1 1 0 0          0
//  0 1 1 0 0 0          1
//  0 0 1 0 0 0          2
//  0 1 0 0 0 0          3
//  1 1 0 0 0 0          4
//  1 0 0 0 0 0          5
//  0 0 0 0 0 0          Error

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(trigPin, LOW);
  timing = pulseIn(echoPin, HIGH);
  return timing * 0.034 * 0.5;
}

void ReadSensors() {
  delay(13);
  for (int i = 0; i < 6; i++) {
    riilSensor[i] = analogRead(sensor[i]);
  }

  for (int i = 0; i < 6; i++) {
    if (riilSensor[i] >= (averageSensor[i])) {
      outputSensor[i] = 0;
    }
    if (riilSensor[i] < (averageSensor[i])) {
      outputSensor[i] = 1;
    }
  }
}

int DetectLine() {
  if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 0) && (outputSensor[3] == 0) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 0;  //  1 1 0 0 1 1  Error Value = 0
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 0) && (outputSensor[5] == 1)) {
    return -3;  //  1 1 1 1 0 1   Error Value = -3
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 0) && (outputSensor[4] == 0) && (outputSensor[5] == 1)) {
    return -2;  //  1 1 1 0 0 1   Error Value = -2
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 0) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return -1;  //  1 1 1 0 1 1   Error Value = -1
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 0) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 1;  //  1 1 0 1 1 1          1
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 0) && (outputSensor[2] == 0) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 2;  //  1 0 0 1 1 1          2
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 0) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 3;  //  1 0 1 1 1 1          3
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 0) && (outputSensor[5] == 0)) {
    return -4;  //  1 1 1 1 0 0   Error Value = -4
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 0)) {
    return -5;  //  1 1 1 1 1 0   Error Value = -5
  } else if ((outputSensor[0] == 0) && (outputSensor[1] == 0) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 4;  //  0 0 1 1 1 1          4
  } else if ((outputSensor[0] == 0) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 5;  //  0 1 1 1 1 1          5
  }
}

void LineFollowing() {
  if (errorValue == 0) {
    Forward(baseSpeed, baseSpeed);
  } else if (errorValue == 1) {
    Forward(baseSpeed + 10, baseSpeed - 10);
  } else if (errorValue == 2) {
    Forward(baseSpeed + 15, baseSpeed - 15);
  } else if (errorValue == -1) {
    Forward(baseSpeed - 10, baseSpeed + 10);
  } else if (errorValue == -2) {
    Forward(baseSpeed - 15, baseSpeed + 15);
  }
  if (errorValue == 3) {
    Forward(7, 7);
    while (true) {
      ReadSensors();
      errorValue = DetectLine();
      Forward(150, 0);
      if (errorValue <= 0) {
        break;
      }
    }
  } else if (errorValue == 4) {
    Forward(7, 7);
    while (true) {
      ReadSensors();
      errorValue = DetectLine();
      Forward(200, 0);
      if (errorValue <= 0) {
        break;
      }
    }
  } else if (errorValue == 5) {
    Forward(7, 7);
    while (true) {
      ReadSensors();
      errorValue = DetectLine();
      Forward(255, 0);
      if (errorValue <= 0) {
        break;
      }
    }
  }
  if (errorValue == -3) {
    Forward(7, 7);
    while (true) {
      ReadSensors();
      errorValue = DetectLine();
      Forward(0, 150);
      if (errorValue >= 0) {
        break;
      }
    }
  } else if (errorValue == -4) {
    Forward(7, 7);
    while (true) {
      ReadSensors();
      errorValue = DetectLine();
      Forward(0, 200);
      if (errorValue >= 0) {
        break;
      }
    }
  } else if (errorValue == -5) {
    Forward(7, 7);
    while (true) {
      ReadSensors();
      errorValue = DetectLine();
      Forward(0, 255);
      if (errorValue >= 0) {
        break;
      }
    }
  }
}

void PDLineFollowing() {
  ReadSensors();
  static int lastError = 0;
  int currentError = DetectLine();

  const float Kp = 35.0;
  const float Kd = 1.0;

  int proportional = currentError;
  int derivative = currentError - lastError;

  float controlSignal = (Kp * proportional) + (Kd * derivative);

  leftSpeed = baseSpeed + controlSignal;
  rightSpeed = baseSpeed - controlSignal;

  if (leftSpeed < 0) {
    leftSpeed = 0;
  } else if (leftSpeed > 255) {
    leftSpeed = 255;
  }
  if (rightSpeed < 0) {
    rightSpeed = 0;
  } else if (rightSpeed > 255) {
    rightSpeed = 255;
  }

  Forward(leftSpeed, rightSpeed);
  lastError = currentError;
}
