#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

// hc-sr04 sensor
#define TRIGGER_PIN A1
#define ECHO_PIN A2
#define max_distance 50

// ir sensor
#define irLeft A4
#define irRight A5 
#define irCenter A3



// motor
#define MAX_SPEED 200
#define MAX_SPEED_OFFSET 20

// Motor Speeds
int M1_Speed = 70;
int M2_Speed = 70;
int LeftRotationSpeed = 100;
int RightRotationSpeed = 100;
Servo servo;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

int distance = 0;
int leftDistance;
int rightDistance;
boolean object;

void setup() {
  Serial.begin(9600);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  servo.attach(10);
  servo.write(90);
}

void loop() {
  if (digitalRead(irLeft) == 0 && digitalRead(irCenter) == 1 && digitalRead(irRight) == 0) {
    objectAvoid();
    // forward
  } else if (digitalRead(irLeft) == 0 && digitalRead(irCenter) == 1 && digitalRead(irRight) == 1) {
    objectAvoid();
    Stop();

    Serial.println("TR");
    
         moveRight();
      
      
      
    // RIGHT turn
   
  } else if (digitalRead(irLeft) == 0 && digitalRead(irCenter) == 0 && digitalRead(irRight) == 1) {
    objectAvoid();
    Stop();
    Serial.println("TR");
     // while (digitalRead(irLeft) == 0 && digitalRead(irCenter) == 1 && digitalRead(irRight) == 0) {
         moveRight();
      
    //  }
    
    // RIGHT turn
    
  } else if (digitalRead(irLeft) == 1 && digitalRead(irCenter) == 1 && digitalRead(irRight) == 0) {
    objectAvoid();
    Stop();
    Serial.println("TL");
   //  while (digitalRead(irLeft) == 0 && digitalRead(irCenter) == 1 && digitalRead(irRight) == 0){
        moveLeft();
      
    //  }
      
    // left turn
   
  } else if (digitalRead(irLeft) == 1 && digitalRead(irCenter) == 0 && digitalRead(irRight) == 0) {
    objectAvoid();
    Stop();
    Serial.println("TL");
    // while (digitalRead(irLeft) == 0 && digitalRead(irCenter) == 1 && digitalRead(irRight) == 0){
        moveLeft();
      
    //  }
        
   
  } else if (digitalRead(irLeft) == 1 && digitalRead(irCenter) == 1 && digitalRead(irRight) == 1) {
    int LeftSensorPinValue, RightSensorPinValue, MiddleSensorPinValue;
     /* Stop();
      delay(1000);*/

      Serial.println("Forward another 4 cm ");
        moveForward();
        delay(1000);
        Stop();
        delay(500);


    if (digitalRead(irLeft) == 0 && digitalRead(irCenter) == 1 && digitalRead(irRight) == 0) {
      moveForward();
    } else {
      turn180Degrees();
      delay(1000);
      moveForward();
      

    }
  } else if (digitalRead(irLeft) == 0 && digitalRead(irCenter) == 0 && digitalRead(irRight) == 0) {
    // Stop

  }
}

void objectAvoid() {
  distance = getDistance();
  if (distance <= 15) {
    // stop
    Stop();
    Serial.println("Stop");

    lookLeft();
    lookRight();
    delay(100);
    if (rightDistance <= leftDistance) {
      // left
      object = true;
      turn();
      Serial.println("moveLeft");
    } else {
      // right
      object = false;
      turn();
      Serial.println("moveRight");
    }
    delay(100);
  } else {
    // forward
    Serial.println("moveForward");
    moveForward();
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 100;
  }
  return cm;
}

int lookLeft() {
  // lock left
  servo.write(150);
  delay(500);
  leftDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("Left:");
  Serial.print(leftDistance);
  return leftDistance;
  delay(100);
}

int lookRight() {
  // lock right
  servo.write(30);
  delay(500);
  rightDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("   ");
  Serial.print("Right:");
  Serial.println(rightDistance);
  return rightDistance;
  delay(100);
}

void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(M1_Speed);
  motor2.setSpeed(M2_Speed);
  motor3.setSpeed(M1_Speed);
  motor4.setSpeed(M2_Speed);
}

void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(M1_Speed);
  motor2.setSpeed(M2_Speed);
  motor3.setSpeed(M1_Speed);
  motor4.setSpeed(M2_Speed);
  // Adjust the delay based on your robot's speed and the distance you want to move backward
  delay(200);
}

void turn() {
  if (object == false) {
    Serial.println("turn Right");
    moveLeft();
    delay(700);
    moveForward();
    delay(800);
    moveRight();
    delay(900);
    if (digitalRead(irRight) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
  else {
    Serial.println("turn left");
    moveRight();
    delay(700);
    moveForward();
    delay(800);
    moveLeft();
    delay(900);
    if (digitalRead(irLeft) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
}

void moveLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(LeftRotationSpeed);
  motor2.setSpeed(RightRotationSpeed);
  motor3.setSpeed(LeftRotationSpeed);
  motor4.setSpeed(RightRotationSpeed);
}

void moveRight() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(LeftRotationSpeed); 
  motor2.setSpeed(RightRotationSpeed);
  motor3.setSpeed(LeftRotationSpeed);
  motor4.setSpeed(RightRotationSpeed);
}

void turn180Degrees() {
  // Assuming your robot can turn left, adjust the motor speeds and directions accordingly
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(LeftRotationSpeed);
  motor2.setSpeed(RightRotationSpeed);
  motor3.setSpeed(LeftRotationSpeed);
  motor4.setSpeed(RightRotationSpeed);

  // Delay for a certain duration to complete the turn
  delay(1500); // Adjust the delay based on your robot's turning speed
  moveForward(); // You should have a stop function to release the motors
}
