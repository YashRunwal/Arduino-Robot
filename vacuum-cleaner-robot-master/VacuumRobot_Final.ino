////Testing
//unsigned long auto_duration = 0;
//unsigned long auto_time = 0;
//unsigned long communication_begin = 0;
//unsigned long manual_time = 0;
//unsigned long manual_duration = 0;
//unsigned long obstacle_start = 0;
//unsigned long obstacle_end = 0;
//unsigned long obstacle_duration = 0;



//Include all necessary library
#include<Servo.h>
#include <SharpIR.h>
#include <NewPing.h>

//---------------- SERVO MOTOR SETUP -------------//

//Object declaration
Servo ultrasonicServo; //create a Servo object named 'ultrasonicServo'

//------------------------------------------------//


//-------------- ULTRASONIC SENSOR SETUP -------------//

//Ultrasonic sensor pins declaration
const byte TRIGGER_PIN = A1;   // Declare trigger pin
const byte ECHO_PIN = A2;      // Declare the echo pin

//Obstacle distance related variables
int maxDistance_in_cms = 200;       // Declare a variable for maximum distance. Max Distance is always in cms
int currentDistance_in_cms = 0;     // Declare a variable for the current measured distance.
const int OBSTACLE_DETECTION_DISTANCE_IN_CM = 20;   // Declare a varible for the obstacle detection distance.
const byte ULTRASONIC_BLIND_MEASUREMENT = 0;          // When the ultrasonic sensor is detecting distance larger than the maxDistance_in_cms, it will output 0 cm
int leftDistance = 0;               // Declare variable for the measured distance when the robot is looking left
int rightDistance = 0;              // Declare variable for the measured distance when the robot is looking right

//Object Declaration
NewPing sonar(TRIGGER_PIN, ECHO_PIN, maxDistance_in_cms); // create a newping object named 'sonar'

//---------------------------------------------------//


//-------------- INFRARED SENSOR SETUP --------------//

//Infrared sensor pin declaration
const byte IR_LEFT = A4; 
const byte IR_RIGHT = A3;

//Step distances related variables
float leftCliffDistance = 0;        // This variable stores the distances calculated using the IR Sensor
float rightCliffDistance = 0;
const float CLIFF_DISTANCE_LIMIT = 15.00;  // When the IR sensor reads the value which is greater than or equal to 'limitedDistance', the robot should stopMoving()

//Object Declaration
SharpIR IRSensor_Left(SharpIR::GP2Y0A21YK0F, IR_LEFT);
SharpIR IRSensor_Right(SharpIR::GP2Y0A21YK0F, IR_RIGHT);

//---------------------------------------------------//

//-----------------LIMIT SWITCH SETUP----------------//

const byte BUMPER_PIN = 8;
bool bumper;

//---------------------------------------------------//

//---------------- DC MOTOR SETUP -----------------//

//Right DC motor pins declaration
const byte IN_1 = 10;
const byte IN_2 = 11;
const byte EN_RIGHT = 5; 

//Left DC motor pins declaration
const byte IN_3 = 12;
const byte IN_4 = 13;
const byte EN_LEFT = 6;

//Encoder pins declaration
const byte LEFT_ENCODER = 3;
const byte RIGHT_ENCODER = 2;

//RPM calculation and driving Straight variables
unsigned int rpmLeft = 0;                 // left rpm reading
unsigned int rpmRight = 0;                // right rpm reading
volatile int pulsesLeft = 0;              // number of pulses of the left encoder
volatile int pulsesRight = 0;             // number of pulses of the right encoder
const unsigned int ENCODER_HOLES = 20;
unsigned long previousTimeEncoder = 0;
unsigned long currentTimeEncoder = 0;
const int ENCODER_UPDATE_DURATION = 100;
const byte MOTOR_OFFSET = 5;
byte leftMotorPower_Forward = 80;
byte rightMotorPower_Forward = 100;
byte leftMotorPower_Backward = 100;
byte rightMotorPower_Backward = 100;

//--------------------------------------------//


//---------------************************************************************************** BLUETOOTH SETUP*********************************************************************************** ---------------//



/*-------------------------------------------------------------------------------------------Variable declaration for Modes of the Robot----------------------------------------------*/

//-------------------------------------------This is Higher 1st level variable---------------//
char Main_Modes_Flag=0;                //Flag for the serial read in the of the 'modes' condition
//-------------------------------------------This is the 2nd level  'if'  variables---------------//
char Modes=0;                          //charecter to enter in the if clause
char Direction=0;                      //charecter to the change of direction and serial read to ´changes the modes
//------------------------------------------This is for 2nd level 'else' variable------------------//
char Mode_Flag_elseif=0;               //Flag for the serial read in the 'else if' of the 'modes entering' condition
char automode_modes_flag =0;
bool bluetooth_case_flag_off =LOW;
bool bluetooth_case_flag_on =LOW;

/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

//-----------------------------------Here the state machine for the bluetooth Auto mode is defined ---------//
enum bluetooth_states {
  on,off
};

enum bluetooth_states ble=on;
char M=0;             //Assign and check the serial read value and make the case 'on' and on_entry high
char J=0;             // Assign and check the serial read value and make the case 'off' and on_entry low
char K=0;             //Assign and check the serial read value and make the case 'on' and on_entry high
bool on_entry=LOW;    // bool for entering the bluetooth case 'on'
//-----------------------For the Time Releated and serial Read------------------------------------//
String inString = "";     // string to hold input
String check_string="";   //checking the value for the time_releated clenaing
unsigned long b_mill =0;
unsigned long  b_current=0;
bool Time_Flag=LOW;
int inString_now=0;     //length of dtring after serial read
int inString_before=0;   //length of dtring before serial read
char inChar=0;               //Charecter for string concatenation

//--------------------------*******************************************************************//--BlUETOOTH SETUP END--//******************************************************************************************---------------------//


//--------------- ROBOT'S STATE MACHINE ---------------//
enum states {
    GO_FORWARD_STATE,
    GO_BACKWARD_STATE,
    GO_LEFT_STATE,
    GO_RIGHT_STATE,
    STOP_MOVING_STATE,
    LOOK_LEFT_STATE,
    LOOK_RIGHT_STATE 
    };

enum states state = GO_FORWARD_STATE;
boolean entry = HIGH;

//State machine timer variables
unsigned long previousTimeState = 0;
unsigned long currentTimeState = 0;

//---------------------------------------------------//


//------------------- FUNCTION DECLARATION -----------------//
void LeftCounter()
{
   //Update count
   pulsesLeft++;
}

void RightCounter()
{
   //Update count
   pulsesRight++;
}

void goForward(){

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  analogWrite(EN_RIGHT,rightMotorPower_Forward) ;
  
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(EN_LEFT, leftMotorPower_Forward);

  }

void DriveStraight(){
  currentTimeEncoder = millis();
  if ((currentTimeEncoder - previousTimeEncoder) >= ENCODER_UPDATE_DURATION ){
    switch(state){
      case GO_FORWARD_STATE:
        analogWrite(EN_LEFT, leftMotorPower_Forward);
        analogWrite(EN_RIGHT, rightMotorPower_Forward);
        break;
    
      case GO_BACKWARD_STATE:
        analogWrite(EN_LEFT, leftMotorPower_Backward);
        analogWrite(EN_RIGHT, rightMotorPower_Backward);
        break;
    }
    
    //detachInterrupt(digitalPinToInterrupt(LEFT_ENCODER));
    //detachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER));
    //rpmLeft = (60 * 1000 / ENCODER_HOLES )/ (currentTimeEncoder - previousTimeEncoder)* pulsesLeft;
    //rpmRight = (60 * 1000 / ENCODER_HOLES )/ (currentTimeEncoder - previousTimeEncoder)* pulsesRight;
    previousTimeEncoder = currentTimeEncoder;

    
    /*
    Serial.print("RPM Left= ");
    Serial.print(rpmLeft,DEC);
    Serial.print("......... RPM Right= ");
    Serial.println(rpmRight,DEC);
    */
    
    //Master-Slave Algorithm
    //Master = Right Motor
    //Slave = Left Motor
    if (pulsesLeft > pulsesRight){
      switch(state){
        case GO_FORWARD_STATE:
          leftMotorPower_Forward -= MOTOR_OFFSET;
          break;
      
        case GO_BACKWARD_STATE:
          leftMotorPower_Backward -= MOTOR_OFFSET;
          break;
       }
    }
    
    if (pulsesLeft < pulsesRight){
      switch(state){
        case GO_FORWARD_STATE:
          leftMotorPower_Forward += MOTOR_OFFSET;
          break;
      
        case GO_BACKWARD_STATE:
          leftMotorPower_Backward += MOTOR_OFFSET;
          break;
       }  
    }
      //Restart the interrupt processing
      //attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), LeftCounter, FALLING);
      //attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), RightCounter, FALLING);
    pulsesLeft = 0;
    pulsesRight = 0;
  }
}

void goBackward(){

  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  analogWrite(EN_RIGHT, rightMotorPower_Backward);
  
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(EN_LEFT, leftMotorPower_Backward);
  }


void goRight(){

  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  analogWrite(EN_RIGHT, 100);
  
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(EN_LEFT, 150);
  }

void goLeft() {
  
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  analogWrite(EN_RIGHT, 150);
  
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(EN_LEFT, 100);
  }

void stopMoving(){
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
  analogWrite(EN_RIGHT, 0);
  
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
  analogWrite(EN_LEFT, 0);
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
  


/******************************************************----------------------------------------Setup-------------------------------------***********************************************/

void setup() {
  // Setting all the motor control pins to outputs
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  //Setting the encoder pins as inputs
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  //Initialize the servo motor
  ultrasonicServo.attach(9);
  lookForward();

  //Setting the limit switch to pullup-input
  pinMode(BUMPER_PIN, INPUT_PULLUP);
  
  //Interrupts for wheels encoder
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), LeftCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), RightCounter, FALLING);

  //Serial communication for debugging purpose and bluetooth communication
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}


/******************************************************----------------------------------------LOOP-------------------------------------***********************************************/

void loop() {
inString_before=inString.length();
      if(Time_Flag==HIGH){
        b_current = millis();
         Serial.print("b_mill_before_calc::");
         Serial.println(b_mill);
        Serial.print("b_mill::");
         Serial.println(b_mill);
         Serial.print("b_current::");
         Serial.print(b_current);
         
        if((b_current - b_mill)>(check_string.toInt()*1000)){
          Time_Flag=LOW;
          b_current=0;
          b_mill=0;
          check_string="";
          Modes='A';
          ble=on;
          M='1';
          state = GO_FORWARD_STATE;
          entry = HIGH;
          
        }
      }

     while(Serial.available() >0) {
      
        inChar = Serial.read();
        //communication_begin = millis();
        inString.concat(inChar);
        Serial.println("inString.length()");
        Serial.print(inString.length());
        inString_now=inString.length();
         if(inString.length() ==1){
          Main_Modes_Flag=inString[0];
          Serial.print("main_modes_flag::");
          Serial.println(Main_Modes_Flag);
          if(Main_Modes_Flag =='A'  || Main_Modes_Flag =='M' || Main_Modes_Flag =='F' || Main_Modes_Flag =='B' || Main_Modes_Flag =='R' || Main_Modes_Flag =='L'|| Main_Modes_Flag =='H'|| Main_Modes_Flag =='G'|| Main_Modes_Flag =='S'){
            inString="";
            }
         }
         if (inString.length()>1){
          char qw=inString[0];
          char we=inString[1];
          char er=inString[2];
          char rt=inString[3];
          if(we =='A'  || we =='M' || we =='F' || we =='B' || we =='R' || we =='L'|| we =='H'|| we =='G'|| we =='S'){
            inString="";
          }
          if(qw =='A'  || qw =='M' || qw =='F' || qw =='B' || qw =='R' || qw =='L'|| qw =='H'|| qw =='G'|| qw =='S'){
            inString="";
          }
          if(er =='A'  || er =='M' || er =='F' || er =='B' || er =='R' || er =='L'|| er =='H'|| er =='G'|| er =='S'){
            inString="";
          }
          if(rt =='A'  || rt =='M' || rt =='F' || rt =='B' || rt =='R' || rt =='L'|| rt =='H'|| rt =='G'|| rt =='S'){
            inString="";
          }
       }
    
     }
     
           if (inString_before > 1 && inString_now > 1 && inString_before==inString_now){
              Serial.print("lenght of main string:");
              Serial.println(inString.length());
              Serial.print("Value:");
              Serial.println(inString.toInt());
              Serial.print("String: ");
              Serial.println(inString);
             inString_before=0;
             inString_now=0;
             b_mill=millis();
             Serial.print("b_mill::");
             Serial.println(b_mill);
             b_current =b_mill;
             Time_Flag=HIGH;
             check_string=inString;
             inString = "";
             Modes=0;
             Direction=0;
             Main_Modes_Flag=0;
            
            }
     
     //-------------------------------------------------------Time releated and bluetooth read ending--------------------//
    
      
         if (Main_Modes_Flag=='M'){
           Modes='M';
           stopMoving();
           ble=on;
           M=0;
           Direction=0;
           Main_Modes_Flag=0;
         }
         else if(Main_Modes_Flag=='A'){
          Modes='A';
          stopMoving();
          ble=on;
          M=0;
          Direction=0;
          Main_Modes_Flag=0;
          }
         else if(Main_Modes_Flag== 'F' && Modes=='M'){
          Direction='F';
          M=0;
          Main_Modes_Flag=0;
         }
         else if(Main_Modes_Flag=='B' && Modes=='M'){
          Direction='B';
          M=0;
          Main_Modes_Flag=0;
         }
         else if (Main_Modes_Flag=='L' && Modes=='M'){
          Direction='L';
          M=0;
          Main_Modes_Flag=0;
         }
         else if(Main_Modes_Flag=='R' && Modes=='M'){
          Direction='R';
          M=0;
          Main_Modes_Flag=0;
         }
         else if(Main_Modes_Flag=='H' && Modes=='M'){
          Direction='H';
          Main_Modes_Flag=0;
          M=0;
         }
         else if(Main_Modes_Flag=='G' && Modes=='A'){
          //bluetooth_case_flag_off=HIGH;
          Modes='A';
          ble=on;
          M='1';
          state = GO_FORWARD_STATE;
          entry = HIGH;
          Main_Modes_Flag=0;
         }
         else if(Main_Modes_Flag=='S' && Modes=='A'){
          //bluetooth_case_flag_on=HIGH;
          Modes='A';
          ble=off;
          M=0;
          Main_Modes_Flag=0;
         }
           if (Modes=='M'){
//            manual_time = millis();
//            manual_duration = manual_time - communication_begin;
//            Serial.print("Manual Duration = ");
//            Serial.println(manual_duration);

            Serial.println("Manual-Mode");
                       if (Direction== 'F'){
                        goForward();
                        Serial.println("Forward");
                       }
                       else if (Direction=='B'){
                        goBackward();
                        Serial.println("Backward");
                       }
                       else if (Direction=='L'){
                        goLeft();
                        Serial.println("Left");
                       }
                       else if (Direction=='R'){
                        goRight();
                        Serial.println("Right");
                       }
                       else if (Direction=='H'){
                        stopMoving();
                        Serial.println("Stop");
                       }
         
            }
            else if (Modes=='A'){
//              auto_time = millis();
//              auto_duration = auto_time - communication_begin;
//              Serial.print("Auto Duration = ");
//              Serial.println(auto_duration);
              
              Serial.println("Auto-Mode");
                  switch(ble){
                          case(on):{
                                   Serial.print("in ble loop on");
    
                                  if (M=='1'){  
                                      Serial.println("in the M of on");
                                      currentDistance_in_cms = sonar.ping_cm(); // Ultrasonic Sensor                                     
                                      leftCliffDistance = IRSensor_Left.getDistance(); // Return IR Distance in cm
                                      rightCliffDistance = IRSensor_Right.getDistance();                                    
                                      bumper = digitalRead(BUMPER_PIN);
                                     //obstacle_start = millis(); 
                                      //-----------------------------------------------------------------------------//main control block of the code//-----------------------------//
                                      switch(state) {             
                                            case GO_FORWARD_STATE:
                                            if (entry){
                                              lookForward();
                                              goForward();      
                                              entry = LOW; 
                                              }
                                            /*
                                            Serial.print ("leftCliff = ");
                                            Serial.print (leftCliffDistance);
                                            Serial.print (".....rightClif = ");
                                            Serial.print( rightCliffDistance);
                                            Serial.print ("US Sensor = ");
                                            Serial.println (currentDistance_in_cms);
                                            */
                                            //Serial.println("Forward State");
                                            DriveStraight();
                                            if (((currentDistance_in_cms <= OBSTACLE_DETECTION_DISTANCE_IN_CM) && (currentDistance_in_cms != ULTRASONIC_BLIND_MEASUREMENT)) || ((leftCliffDistance > CLIFF_DISTANCE_LIMIT)||(rightCliffDistance > CLIFF_DISTANCE_LIMIT)) || (bumper == HIGH)){
                                              stopMoving();

                                              state = GO_BACKWARD_STATE;
                                              entry = HIGH; 
//                                              obstacle_end = millis();
//                                              obstacle_duration = obstacle_duration - obstacle_duration;
//                                              Serial.print("Obstacle Duration = ");
//                                              Serial.println(obstacle_duration);
                                            }
                                            break;
                                  
                                          case GO_BACKWARD_STATE:
                                            if (entry){
                                              goBackward();
                                              previousTimeState = millis();
                                              entry = LOW;
                                              }
                                          //Serial.println("Backward State");
                                          DriveStraight();   
                                           currentTimeState = millis();
                                           if ((currentTimeState - previousTimeState)>=800){
                                            stopMoving();
                                            state = LOOK_LEFT_STATE;
                                            entry = HIGH;
                                           }
                                           break; 
                                        
                                            case LOOK_LEFT_STATE:
                                            if (entry){
                                            leftDistance = lookLeft();
                                            previousTimeState = millis();
                                            entry = LOW;
                                           }
                                           //Serial.println("Look left State");
                                            currentTimeState = millis();
                                            if ((currentTimeState - previousTimeState)>=300){
                                            stopMoving();
                                            state = LOOK_RIGHT_STATE;
                                            entry = HIGH;
                                           }
                                            break;
                                        
                                            case LOOK_RIGHT_STATE:
                                            if (entry){
                                            rightDistance = lookRight();
                                            previousTimeState = millis();
                                            entry = LOW;
                                           }
                                           //Serial.println("Look right State");
                                            currentTimeState = millis();
                                            if ((currentTimeState - previousTimeState)>=300){
                                            stopMoving();
                                        
                                            if (leftDistance >+ rightDistance){
                                              state = GO_LEFT_STATE;
                                            }
                                            else if (leftDistance < rightDistance){
                                              state = GO_RIGHT_STATE;
                                            }
                                            /*
                                           else if (leftDistance == rightDistance){
                                              state = GO_BACKWARD_STATE;
                                            }
                                            */
                                            entry = HIGH;
                                           }
                                           break;
                                        
                                        
                                          case GO_LEFT_STATE:
                                           if (entry){
                                            goLeft();
                                            previousTimeState = millis();
                                            entry = LOW;
                                           }
                                           //Serial.println("Go left State");                                          
                                           if (((leftCliffDistance > CLIFF_DISTANCE_LIMIT)||(rightCliffDistance > CLIFF_DISTANCE_LIMIT)) || (bumper == HIGH)){
                                            stopMoving();
                                            state = GO_BACKWARD_STATE;
                                            entry = HIGH;
                                           }
                                           
                                           currentTimeState = millis();
                                           if ((currentTimeState - previousTimeState)>=500){
                                            stopMoving();
                                            state = GO_FORWARD_STATE;
                                            entry = HIGH;
                                           }
                                           break;
                                        
                                          case GO_RIGHT_STATE:
                                           if (entry){
                                            goRight();
                                            previousTimeState = millis();
                                            entry = LOW;
                                           }
                                           //Serial.println("Go right State");
                                           if (((leftCliffDistance > CLIFF_DISTANCE_LIMIT)||(rightCliffDistance > CLIFF_DISTANCE_LIMIT)) || (bumper == HIGH)){
                                            stopMoving();
                                            state = GO_BACKWARD_STATE;
                                            entry = HIGH;
                                           }
                                           
                                           currentTimeState = millis();
                                           if ((currentTimeState - previousTimeState)>=500){
                                            stopMoving();
                                            state = GO_FORWARD_STATE;
                                            entry = HIGH;
                                           }
                                           break;
                                        
                                   
                                          }
                                       //-----------------------------------------------------------------------------//main control block of the code//-----------------------------//
                                   }
                                  
                                  
                          break;
                          }
                
                          case(off):{
                            Serial.print("in ble loop off");
                            stopMoving();
                            lookForward();
                          break;
                          }
                  }
            }
 
  
}
