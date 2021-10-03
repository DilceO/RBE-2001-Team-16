#include <Arduino.h>
#include <RBE1001Lib.h>
//#include <ESP32Servo.h>

//for sensors
const int reflectancePin1 = 39;
const int reflectancePin2 = 36;
Rangefinder ultrasonic;
int roofApproachThreshold = 20;
int roofThreshold = 17;
int blockThreshold = 10;
int zoneThreshold = 3;
int distanceToB = 7;
double atStopPointLeft = 0;
double atStopPointRight = 0;
double facingCenterLeft = 0;
double facingCenterRight = 0;

//button
const int buttonPin = BOOT_FLAG_PIN;
bool upDown = false;

//for driving
Motor left_motor;
Motor right_motor;
double diam = 2.75;
double track = 5.875;
int defaultSpeed = 150;
double distanceCorrection = 0.95;
double bagDistance;
double zoneDistance;

//for line following (calling the linefollowing sensor functions)
int reflectance1;
int reflectance2;
int threshold = 1250;
int thresholdHigh = 1250;
double kp = 0.05;

//for servo arm
Servo lifter;
const int servoPin = 33;
int top = 0;
int bottom = 180;
int approachRoofHeight = 165;

int travelOutHeight = approachBagHeight;
int collectBagHeight = top;
int approachZoneHeight = collectBagHeight;
int deliverA = approachBagHeight;

bool safeToBeCollected = false;
bool safeToBeDeposited = false;
bool safeToPickUpNew = false;
bool safeToDropOffNew = false;
bool readyToPickUpNew = false;
bool depositingNew = false;

//state machines
enum ROBOT_STATES{
  START_SIDE,
  APPROACH_ROOF,
  PICKUP_OLD,
  TOWARD_BLOCK,
  DEPOSIT_OLD,
  WAIT_FOR_SIGNAL,
  PICKUP_NEW,
  BACK_TO_INTERSECTION,
  DEPOSIT_NEW,
  MOVE_OTHER_SIDE};

int robotState;

//functions
void lineFollow(int reflectance1, int reflectance2);
void turn(double angle, double diam3, double track2);
double ultrasonicRead ();
void pickUpOld();
void pickUpNew();
void depositOld();
void depositNew();
void straight(double distance, double wheelDiameter);

void setup() {
  Motor::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  Serial.begin(115200);
  // pin definitions https://wpiroboticsengineering.github.io/RBE1001Lib/RBE1001Lib_8h.html#define-members
  left_motor.attach(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB);
  right_motor.attach(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);
  ultrasonic.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  lifter.attach(servoPin);

  pinMode(reflectancePin1,INPUT);
  pinMode(reflectancePin2,INPUT);
  robotState = LINE_FOLLOW_OUT;
  bagState = START;
}

// line follower which uses the values from the two reflectance pins as arguments
void lineFollow(int reflectance_1, int reflectance_2){
  float error = reflectance_1 - reflectance_2;
  float effort = 0;
  effort = kp * error;
  right_motor.setSpeed(defaultSpeed+effort);
  left_motor.setSpeed(defaultSpeed-effort);

}

// turn function which has turn angle, diameter, and track as the arguments
void turn(double angle, double diam3, double track2){
    double degreeMove = (angle*track2)/diam3;
    left_motor.startMoveFor(degreeMove, 120);
    right_motor.moveFor(-degreeMove, 120);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
  }

// straight function which intakes the distance and the wheel diameter as arguments
void straight(double distance, double wheelDiameter) {
  double spin = (360*distance)/(wheelDiameter*PI);
  left_motor.startMoveFor(spin, 150);
  right_motor.moveFor(spin, 150);
}

// 
void deliverBag(void){ //for bag delivery
  if (ultrasonicRead() > zoneThreshold){
     lineFollow (reflectance1, reflectance2);
       if (bagState == A){
          lifter.write(deliverA); //to place on the ground
          //back up
          left_motor.startMoveFor(-90, 120);
          right_motor.moveFor(-90, 120);
        }
       if (bagState == B){
          delay(100);
          lifter.write(deliverB); //to place on 4mm
          //back up
          left_motor.startMoveFor(-90, 120);
          right_motor.moveFor(-90, 120);
        }
      if (bagState == C){
          lifter.write(deliverC); //to place on 8mm
          //back up
          left_motor.startMoveFor(-90, 120);
          right_motor.moveFor(-90, 120);
        }
  }
}

//read the ultrasonic
double ultrasonicRead(){
  delay(40);
  double distance = ultrasonic.getDistanceCM();
  //int maxDistance = 160;
  //int minDistance = .9;
  //if (distance > maxDistance) distance = maxDistance;
  //if(distance <= minDistance) distance = 120;
  //add something to take the average if the value is wildly inacurate
  return distance;

 }

// picking up old plate from roof
 void pickUpOld() {
   // code to actually grip the plate goes here
     // open gripper
     // move a little closer to fully grip plate with gripper open

   // code to wait for communication from IR remote
   if (safeToBeCollected) {
     // close gripper
     // raise arm a little to life plate off of pins
     // back away from roof
   }
   else {
     // wait for safeToBeCollected == true
   }
 }


// picking up new plate from block
 void pickUpNew() {

   if (readyToPickUpNew) {
     // code to actually grip the plate goes here
       // open gripper

     if (safeToPickUpNew) {
       // move a little closer to fully grip plate with gripper open
       // close gripper
     }
     else {
       printf("Swanbot is waiting to safely pick up new plate\n");
     }
   }

 }


// depositing old plate on block
 void depositOld() {
   if (ultrasonicRead() == blockThreshold) { // check again to see if we are the position we want to be in

     // code to wait for communication from IR remote

     if (safeToBeDeposited) {
       // open gripper
       delay(300);
       // close gripper
       // back away from block // back away enough to get back into original postion
     }
     else {
       // wait for safeToBeDeposited == true
       printf("Swanbot is waiting to safely deposit old plate\n");
     }
   }
   else {
     printf("Swanbot is not in the correct position\n");
   }
 }


// depoisiting new plate on roof
 void depositNew() {

   // code to actually ungrip the plate goes here
   // move a little closer to be in line with pins

   // code to wait for communication from IR remote
   if (safeToBeCollected) {
     // lower arm a little to put plate on of pins
     // open gripper to deposit plate
     // raise arm a little to avoid bumping
     // back away from roof
   }
   else {
     // wait for safeToBeCollected == true
   }

 }


void updateRobotState(void){

  switch (robotState){

    case LINE_FOLLOW_OUT:
        lifter.write(travelOutHeight); // default position
        if (ultrasonic.getDistanceCM() > roofApproachThreshold){
            lineFollow(reflectance1, reflectance2); // keep following line until we approach the roof
        }
        else {
          robotState = APPROACH_ROOF;
        }
        break;

    case APPROACH_ROOF:
        lifter.write (approachRoofHeight); // move arm to fortyFive roof height
         if (ultrasonicRead() > roofThreshold){
            lineFollow(reflectance1, reflectance2); //slowly follow line until closest position is reached
         }
         else { //when it reaches the pickup distance
            left_motor.setEffort(0);
            right_motor.setEffort(0);
            //bagDistance = ultrasonicRead(); //save position away from bag
            delay(100);
            if (!depositingNew) {
              robotState = PICKUP_OLD;
            }
            else {
              robotState = DEPOSIT_NEW; // if depositingNew is true, switch to deposit_new state
            }

         }
        break;

    case PICKUP_OLD: //already at pickup distance with arm raised

        pickUpOld();
        delay(300);
        turn(180, diam, track); // have to check if this works
        robotState = TOWARD_BLOCK;
        break;

    case TOWARD_BLOCK:
        safeToBeCollected = false; // setting this to false so we can reuse when we are depositing new plate
        if ((reflectance1 > threshold) && (reflectance2 > threshold)){ // reached intersection
          left_motor.setSpeed(0);
          right_motor.setSpeed(0);
          delay(150);
          turn(90, diam, track); // turn left to face block
          if (ultrasonicRead() > blockThreshold){
             lineFollow(reflectance1, reflectance2); // follow line until approach block to desired position
          }
          robotState = DEPOSIT_OLD;
        }
        else {
          lineFollow(reflectance1, reflectance2);
        }
        break;

    case DEPOSIT_OLD:

        depositOld();
        robotState = PICKUP_NEW;
        break;

    case PICKUP_NEW:
          // wait for IR communication that plate is ready to be picked up
          pickUpNew();
          robotState = BACK_TO_INTERSECTION;
          break;

    case BACK_TO_INTERSECTION:
          turn(180, diam, track);

          if ((reflectance1 > threshold) && (reflectance2 > threshold)){ // reached intersection
            left_motor.setSpeed(0);
            right_motor.setSpeed(0);
            delay(150);
            turn(-90, diam, track); // turn right to face roof
            depositingNew = true;
            robotState = APPROACH_ROOF;
          }
          else {
            lineFollow(reflectance1, reflectance2);
          }

          break;


    case DEPOSIT_NEW: // already at pickup distance with arm raised
        robotState = MOVE_OTHER_SIDE;
        break;


    case MOVE_OTHER_SIDE:
          // this should bring robot to the intersection facing the roof

          turn(-90, diam, track);
          straight(15);
          turn(90, diam, track);

          while((reflectance1 > threshold) && (reflectance2 > threshold)) {
            lineFollow(reflectance1, reflectance2);
          }

          turn(90, diam, track);

          while((reflectance1 > threshold) && (reflectance2 > threshold)) {
            lineFollow(reflectance1, reflectance2);
          }

          turn(90, diam, track);

          break;
  }

}

void loop() {
 while(digitalRead(buttonPin)) {} //wait for button press
  delay (500);

  while(true){
   reflectance1 = analogRead(reflectancePin1);
   reflectance2 = analogRead(reflectancePin2);
   updateRobotState();
  }
}
