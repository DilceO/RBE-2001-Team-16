#include <Arduino.h>
#include <RBE1001Lib.h>
//#include <ESP32Servo.h>

//for sensors
const int reflectancePin1=39;
const int reflectancePin2=36;
Rangefinder ultrasonic;
int bagApproachThreshold = 20;
int bagThreshold = 17;
int zoneApproachThreshold = 10;
int zoneThreshold = 3;
int distanceToB = 7;
double atStopPointLeft = 0;
double atStopPointRight = 0;
double facingCenterLeft = 0;
double facingCenterRight = 0;

//button
const int buttonPin = BOOT_FLAG_PIN;
bool upDown=false;

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

//for main control
int bagCount = 0;

//for servo arm
Servo lifter;
const int servoPin = 33;
int top = 0;
int bottom = 180;
int approachBagHeight = 165;
int travelOutHeight = approachBagHeight;
int collectBagHeight = top;
int approachZoneHeight = collectBagHeight;
int deliverA = approachBagHeight;
int deliverB = 135;
int deliverC = 100;

//state machines
enum ROBOT_STATES{LINE_FOLLOW_OUT, APPROACH_BAG, COLLECT_BAG, LINE_FOLLOW_BACK, INTERSECTION_BACK, APPROACH_ZONE, DELIVER_BAG, APPROACH_INTERSECTION, INTERSECTION_OUT, FREE_RANGE_COLLECT};
int robotState;

enum BAG_STATES{START, A, B, C};
int bagState;

//functions
void updateBagState(void);
void lineFollow(int reflectance1, int reflectance2);
void turn(double angle, double diam3, double track2);
double ultrasonicRead ();
void deliverBag(void);
void findFreeBag(double ultrasonic_distance);
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

void lineFollow(int reflectance_1, int reflectance_2){ //line following function
  float error = reflectance_1 - reflectance_2;
  float effort = 0;
  effort = kp * error;
  right_motor.setSpeed(defaultSpeed+effort);
  left_motor.setSpeed(defaultSpeed-effort);

}

void turn(double angle, double diam3, double track2){ //for navigation
    double degreeMove = (angle*track2)/diam3;
    left_motor.startMoveFor(degreeMove, 120);
    right_motor.moveFor(-degreeMove, 120);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
  }

void straight(double distance, double wheelDiameter) {
  double spin = (360*distance)/(wheelDiameter*PI);
  left_motor.startMoveFor(spin, 150);
  right_motor.moveFor(spin, 150);
}

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

 //find free range bag
 void findFreeBag(double ultrasonic_distance){
  double lastDistance = ultrasonicRead();
  double difference = 0;
  double bagRadius = 2.8575;
  int firstEdgeFound = 0;
  int secondEdgeFound = 0;
  double firstEdgeLeft = 0;
  double firstEdgeRight = 0;
  double FRZradius = 86.36;
  bool atCup = false;

  delay(100);

  while(atCup == false){
    ultrasonic_distance = ultrasonicRead();
    if(ultrasonic_distance > FRZradius){
      ultrasonic_distance = FRZradius;
    }
    difference = ultrasonic_distance - lastDistance;
    left_motor.setSpeed(-30);
    right_motor.setSpeed(30);
    delay(35);

    if ((difference > bagRadius)  && (lastDistance < FRZradius)
                                  && (ultrasonic_distance >= FRZradius)
                                  && (firstEdgeFound == 1)) {
      double secondEdgeLeft = left_motor.getCurrentDegrees();
      double secondEdgeRight = right_motor.getCurrentDegrees();
      double centerLeft = ((firstEdgeLeft + secondEdgeLeft) / 2);
      double centerRight = ((firstEdgeRight + secondEdgeRight) / 2);
      secondEdgeFound = 1;

      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      lifter.write(bottom);
      delay(25);

      left_motor.moveTo(centerLeft, 20);
      right_motor.moveTo(centerRight, 20);
      delay(3000);
      facingCenterLeft = left_motor.getCurrentDegrees();
      facingCenterRight = right_motor.getCurrentDegrees();
      delay(150);

      while(atCup == false){
        if (ultrasonic.getDistanceCM() < bagThreshold) {
          bagDistance = ultrasonic.getDistanceCM();
          straight(bagDistance - 9, diam);
          delay(100);
          left_motor.setSpeed(0);
          right_motor.setSpeed(0);
          lifter.write(top);
          bagCount++;
          delay(200);
          left_motor.moveTo(facingCenterLeft, 180);
          right_motor.moveTo(facingCenterRight, 180);
          delay(5000);
          left_motor.moveTo(atStopPointLeft, 60);
          right_motor.moveTo(atStopPointRight, 60);
          delay(1500);
          turn(-90, diam, track);
          delay(500);
          atCup = true;
          robotState = LINE_FOLLOW_BACK;
        }
        else {
          left_motor.setSpeed(178);
          right_motor.setSpeed(180);
          delay(35);
        }
      }

    }

    else if ((-difference > bagRadius)  && (ultrasonic_distance < FRZradius)
                                        && (lastDistance >= FRZradius)) {
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(150);

      firstEdgeFound = 1;
      firstEdgeLeft = left_motor.getCurrentDegrees();
      firstEdgeRight = right_motor.getCurrentDegrees();
      delay(100);
    }

    lastDistance = ultrasonic_distance;
  }
 }


void updateRobotState(void){
 
  switch (robotState){

  case LINE_FOLLOW_OUT:
      lifter.write(travelOutHeight); //default position
      if (bagState == START || bagState == A){
         if (ultrasonic.getDistanceCM() > bagApproachThreshold){
            lineFollow(reflectance1, reflectance2);
         }
         else {
         robotState = APPROACH_BAG;
         }
      }
      else if (bagState == B){ //looking for free range bag
        if ((reflectance1 > threshold) && (reflectance2 > threshold)){ //when it sees the tape where the first two cans were
            left_motor.setSpeed(0);
            right_motor.setSpeed(0);
            delay(200);
            atStopPointLeft = left_motor.getCurrentDegrees(); //save position for free-range finding
            atStopPointRight = right_motor.getCurrentDegrees();
            delay(200);
            turn(-80, diam, track);
            robotState = FREE_RANGE_COLLECT;
        }
        else {
          lineFollow(reflectance1, reflectance2);
          }
        }
        break;

  case APPROACH_BAG:
      lifter.write (approachBagHeight);
       if (ultrasonicRead() > bagThreshold){
          lineFollow(reflectance1, reflectance2);
      }
       else { //when it reaches the pickup distance
          left_motor.setEffort(0);
          right_motor.setEffort(0);
          bagDistance = ultrasonicRead(); //save position away from bag
          delay(100);
          robotState = COLLECT_BAG;        
       }
      break;

  case COLLECT_BAG: //already at pickup distance with lifter lowered
      straight ((bagDistance - 8), diam); //drive the known distance to the bag (minus a constant for arm length)
      delay(100);
      lifter.write(collectBagHeight);
      delay(300);
      turn(170, diam, track);
      bagCount++;
      robotState = LINE_FOLLOW_BACK;
      break;

  case LINE_FOLLOW_BACK:
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
        left_motor.setSpeed(0);
        right_motor.setSpeed(0);
        delay(150);
        robotState = INTERSECTION_BACK;
      }
      else {
        lineFollow(reflectance1, reflectance2);
      }
      break;

  case INTERSECTION_BACK:
      left_motor.startMoveFor(110, 120);
      right_motor.moveFor(110, 120);
      if (bagState == A){
        turn(-83, diam, track);//left
      }
      if (bagState == B){
        lineFollow(reflectance1, reflectance2);//straight
      }
      if (bagState == C){
        turn(83, diam, track);//right
      }

      robotState = APPROACH_ZONE;
      break;

  case APPROACH_ZONE:
        lifter.write(approachZoneHeight);
         //for delivering to a platform
        if (bagState == B || bagState == C){ //for delivering to a platform
          if (ultrasonicRead() > zoneApproachThreshold){
              lineFollow(reflectance1, reflectance2);
             }
          else { //when it reaches the pickup distance
              left_motor.setEffort(0);
              right_motor.setEffort(0);
              delay(100);
              zoneDistance = ultrasonicRead(); //save the distance to the zone
              delay(100);
              straight (zoneDistance-distanceToB, diam); //drive the known distance to the zone
              robotState = DELIVER_BAG;
            }
          }
        else { //for delivering to flat zone
          if ((reflectance1 > threshold) && (reflectance2 > threshold)){
            left_motor.setSpeed(0);
            right_motor.setSpeed(0);
            delay(100);
            robotState = DELIVER_BAG;

          }
          else {
            lineFollow(reflectance1, reflectance2);
          }
        }
        break;

  case DELIVER_BAG:
        delay(100);
        deliverBag();
        delay(700);
        turn(170, diam, track);
        robotState = APPROACH_INTERSECTION;
        break;

  case APPROACH_INTERSECTION:
    if (reflectance1 > thresholdHigh && reflectance2 > thresholdHigh){
          robotState = INTERSECTION_OUT;
    }
    else {
           lineFollow(reflectance1, reflectance2);
        break;


  case INTERSECTION_OUT:
        left_motor.startMoveFor(110, 120);
        right_motor.moveFor(110, 120);
      if (bagState == A){
        turn(80, diam, track);//right
      }
      if (bagState == B){
        lineFollow(reflectance1, reflectance2);//straight
      }
      if (bagState == C){
        turn(-80, diam, track);//left
      }
      robotState = LINE_FOLLOW_OUT;
      break;

     }

  case FREE_RANGE_COLLECT:
    findFreeBag(ultrasonicRead());
    break;
}
}

void updateBagState(void){ //for bag-based control
  switch(bagState){
    case START:
      if (bagCount == 1){
        bagState = A;
      }
      break;

    case A:
       if (bagCount == 2){
          bagState = B;
       }
       break;

    case B:
        if (bagCount == 3){
        bagState = C;
        }
        break;
    case C:
    break;
  }
}

void loop() { 
 while(digitalRead(buttonPin)) {} //wait for button press
  delay (500);

  while(true){
   reflectance1=analogRead(reflectancePin1);
   reflectance2=analogRead(reflectancePin2);

   updateRobotState();
   updateBagState();
  }
}