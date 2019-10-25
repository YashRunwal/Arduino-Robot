// Servo Motor Setup
#include<Servo.h>
Servo ultrasonicServo; //create a Servo object named 'ultrasonicServo'


//Ultrasonic Sensor Setup
#include <NewPing.h>   // Include the NewPing Library
NewPing sonar(TRIGGER_PIN, ECHO_PIN, maxDistance_in_cms); // create a newping object named 'sonar'

const byte TRIGGER_PIN = A1;   // Declare trigger pin
const byte ECHO_PIN = A2;      // Declare the echo pin


// IR Sensor Setup
/*
 * SharpIR sensor(ir,25,93,model)
where:
    ir: the pin where your sensor is attached.
    25: the number of readings the library will make before calculating an average distance.
    93: the difference between two consecutive measurements to be taken as valid (in %)
    model: is an int that determines your sensor:  1080 for GP2Y0A21Y, 20150 for GP2Y0A02Y
*/
#include <SharpIR.h>
SharpIR sensor(ir, 25, 93, 1080);
#define ir = A0;
float calculateStepDistance;   // This variable stores the distances calculated using the IR Sensor
float limitedDistance = 10.00; // When the IR sensor reads the value which is greater than or equal to 'limitedDistance', the robot should stopMoving()


// Declaration of a few Variables that are used later
int maxDistance_in_cms = 200; // Declare a variable for maximum distance. Max Distance is always in cms
int currentDistance_in_cms = 0; // Declare a variable for the current measured distance.
int obstacleDistance_in_cms = 20; // Declare a varible for the obstacle detection distance.
byte blindMeasurement = 0; // When the ultrasonic sensor is detecting distance larger than the maxDistance_in_cms, it will output 0 cm
int leftDistance = 0; // Declare variable for the measured distance when the robot is looking left
int rightDistance = 0; // Declare variable for the measured distance when the robot is looking right



// Motor A Pins
const byte IN_1 = 10;
const byte IN_2 = 11;
const byte EN_LEFT = 5; 

//Motor 2 pins
const byte IN_3 = 12;
const byte IN_4 = 13;
const byte EN_RIGHT = 6;

// IR Sensor_1
#define sensor A0;

//void goForward();
//void goBackward();
//void goLeft();
//void goRight();
//void stop();

enum states {
    goForward_state,
    goBackward_state,
    goLeft_state,
    goRight_state,
    stopMoving_state,
    lookLeft_state,
    lookRight_state 
    };

enum states state = goForward_state;
boolean entry = HIGH;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
  
void setup() {
  
  // Setting all the motor control to outputs
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

   ultrasonicServo.attach(9);
   Serial.begin(9600);
}


void goForward(){

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  analogWrite(EN_LEFT, 150);
  
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(EN_RIGHT, 150);
  
  }

  

void goBackward(){

  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  analogWrite(EN_LEFT, 100);
  
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(EN_RIGHT, 70);
  }


void goLeft(){

  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  analogWrite(EN_LEFT, 100);
  
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(EN_RIGHT, 150);
  }

void goRight() {
  
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  analogWrite(EN_LEFT, 150);
  
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(EN_RIGHT, 100);
  }

void stopMoving(){
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
  analogWrite(EN_LEFT, 0);
  
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
  analogWrite(EN_RIGHT, 0);
  }

void lookForward(){
  ultrasonicServo.write(73);
}

int lookLeft(){
  ultrasonicServo.write(115);
  int distance = sonar.ping_cm();
  return distance;
}

int lookRight(){
  ultrasonicServo.write(25);
  int distance = sonar.ping_cm();
  return distance;
}

/*
int stepDistance_1(){
  calculateStepDistance = sensor.getDistance();
  return calculateStepDistance;
  }
*/
  
void loop() {
  switch(state) {
    
    case goForward_state:
    if (entry){
      lookForward();
      goForward();      
      entry = LOW; 
      }
      
    currentDistance_in_cms = sonar.ping_cm(); // Ultrasonic Sensor
    calculateStepDistance = sensor.getDistance(); // IR Sensor
    Serial.print("Distance = ");
    Serial.println(currentDistance_in_cms);
    
    if ((currentDistance_in_cms <= obstacleDistance_in_cms) && (currentDistance_in_cms != blindMeasurement) || calculateStepDistance > limitedDistance){
      stopMoving();
      state = goBackward_state;
      entry = HIGH; 
    }
    break;

    case lookLeft_state:
    if (entry){
    leftDistance = lookLeft();
    previousTime = millis();
    entry = LOW;
   }
   
    currentTime = millis();
    if ((currentTime - previousTime)>=500){
    stopMoving();
    state = lookRight_state;
    entry = HIGH;
   }
    break;

    case lookRight_state:
    if (entry){
    rightDistance = lookRight();
    previousTime = millis();
    entry = LOW;
   }
   
    currentTime = millis();
    if ((currentTime - previousTime)>=1000){
    stopMoving();

    if (leftDistance > rightDistance){
      state = goLeft_state;
    }
    else if (leftDistance < rightDistance){
      state = goRight_state;
    }
    /*
    else if (leftDistance == rightDistance){
      state = goBackward_state;
    }
    */
    entry = HIGH;
   }
   break;


  case goLeft_state:
   if (entry){
    goLeft();
    previousTime = millis();
    entry = LOW;
   }
   
   currentTime = millis();
   if ((currentTime - previousTime)>=500){
    stopMoving();
    state = goForward_state;
    entry = HIGH;
   }
   break;

  case goRight_state:
   if (entry){
    goRight();
    previousTime = millis();
    entry = LOW;
   }
   
   currentTime = millis();
   if ((currentTime - previousTime)>=500){
    stopMoving();
    state = goForward_state;
    entry = HIGH;
   }
   break;

  case goBackward_state:
   if (entry){
    goBackward();
    previousTime = millis();
    entry = LOW;
   }
   
   currentTime = millis();
   if ((currentTime - previousTime)>=1000){
    stopMoving();
    state = lookLeft_state;
    entry = HIGH;
   }
   break;
   
  }
}
