// THIS VERSION DOES NOT HAVE THE HORIZONTAL ANGLES ENABLED. ONLY THE VERTICAL SETUP HAS BEEN
// KEPT IN THIS VERSION. THE REASON IS THAT THE HORIZONTAL SETUP IS NOT PHYSICALLY SUSTAINABLE
// DUE TO ISSUES OF JOINT STRENGTH IN THE PROPER DIRECTION(CONSIDER A 2X12 LUMBER LOGIC HERE).
// THIS ALSO ALLOWS THE CODE TO BE SIMPLIFIED A FAIR AMOUNT, AND THE ULTRASONIC SENSOR CAN BE 
// UTILIZED MORE DIRECTLY. -JCH 9/3/18  
/////////////////////////////////////////////////////////////////////////////////////////////            
// Include libraries
#include <Servo.h>                 // Servo position control library
#include <Stepper.h>               // Stepper motor control 
#include <math.h>
#include <Wire.h>                       //library for I2C communication
#define STEPS 64     //32               //stepper motor step number
Servo servo1;                           //initialize a servo for the upper arm
Servo servo2;                           //initialize a servo for the lower arm
Servo servo3;                           //initialize a servo for the lower arm
Servo servo4;                           //initialize a servo for the lower arm
Servo servo5;                           //initialize a servo for the lower arm
Stepper stepper (STEPS, 8, 10, 9, 11);
/////////////////////////////////////////////////////////////////
// VARIABLES USED TO INITIALIZE EACH SECTION
/////////////////////////////////////////////////////////////////
const byte testing0    = 1;                   // prbytes all debug statements
const byte testing2    = 0;                   // testing stepper with ultrasonic output
const byte testing3    = 0;                   // testing can coordinate calculations output
const byte testing4    = 0;                   // testing shoulder stepper output                                     
const byte testing6    = 1;                   // testing vertical arm output(alternate)  
const byte testing7    = 0;                   // testing wrist servo output
const byte testing8    = 0;                   // testing wrist stepper output
const byte testing9    = 0;                   // testing tab opener: to tab from start output      
/////////////////////////////////////////////////////////////////
// VARIABLE SETUP FOR THE CODE
/////////////////////////////////////////////////////////////////
byte angle             = 0;                   // variable, initial angle for servo control
byte angle1            = 0;                   // variable, initial angle for servo control
byte angle2            = 0;                   // variable, initial angle for servo control
byte angle3            = 0;                   // variable, initial angle for servo control
byte angle4            = 0;                   // variable, initial angle for servo control
byte angle5            = 0;                   // variable, initial angle for servo control
double val             = 0;                   // 
byte iter, i, j;                              // variable, loop operator counter. i is the operator, iter is the max... (from i to iter)
byte dummy1 = 0, dummy2, dummy3, dummy4;      // dummy counting variable
double rotloc          = 0;                   // variable for determining step rotation 
double rottot          = 0;                   // variable for determining total rotation 
double distanceStorage[2][20];                // stores ultrasonic readings
byte distanceSum;                             // summed ultrasonic distance at each step
byte distanceAve;                             // computed average US distance at each step
const byte trigPin   = 4;                     // trig pin for the ultrasonic sensor
const byte echoPin   = 3;                     // echo pin for the ultrasonic sensor
long duration        = 0;                     // variable for sensor travel time
byte distance        = 0 ;                    // variable for calculated distance from "duration"
byte closestcan;
byte A;
double absoluteAngle;
double absoluteDist;
double maxrot        = M_PI/2;               //sets maximum rotation angle
byte angle4toTab     = 180;                  // sets final position for this step
byte angle5toTab     = 180;                  // sets final position for this step
double ArmAngle      = 0;                    // variable, initial angle for servo control
double dtr           = 3.1416 / 180;         //degrees to radians
double rtd           = 180/M_PI;             //radians to degrees
byte ArmLength = 9;                          // define upper arm length in inches



void setup() {                               // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);              // LED on board
  pinMode(trigPin, OUTPUT);                  // ultrasonic pin
  pinMode(echoPin, INPUT);                   // ultrasonic pin
  Serial.begin(9600);                        // communicates with serial monitor
  servo1.attach(6);                          // sets pin for servo1
  servo2.attach(7);                          // sets pin for servo2
  servo3.attach(13);                          // sets pin for servo3 
  servo4.attach(12);                          // sets pin for servo4
  servo5.attach(5);                          // sets pin for servo5
  stepper.setSpeed(200);                    // sets stepper motor speed, 1000 is highest
}

void loop(){                       
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Use ultrasonic sensor to survey the area and record distance readings to file
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  rottot = 0;                                                // sets up variable for use  
  rotloc = 0;                                                // sets up variable for use
  dummy1 = 0;                                                //sets index variable for distance matrix
  val = 0;                                                   //2048 is one whole rotation
  if (testing2 == 1){
    Serial.println("[I] STARTING STEPPER FOR ULTRASONIC IN 5 SECONDS");
    delay(1000);
    while (maxrot >= rottot){                                //allows rotation until angle maxrot is reached.
      val = 32;                                                //incremintally increases rotation by a fixed amount
      dummy1 = dummy1 + 1;                                     //indexes each loop
      rotloc = val/2048*2*M_PI;                                //determines radian angle traveled each loop
      rottot = rottot + rotloc;                                //determines total radian angle traveled
      if (testing0 == 1){
        Serial.print(F("maxrot = "));
        Serial.println(maxrot);
        Serial.print(F("rottot = "));
        Serial.println(rottot);
        Serial.print(F("rotloc = "));
        Serial.println(rotloc);
        Serial.print(F("dummy1 = "));
        Serial.println(dummy1);
        delay(2000);
      }//endif testing0
/////////////////////////////////////////////////////////////////////
//USES ULTRASONIC TO MEASURE 10 DISTANCES AT EACH TURN. AVERAGES THESE 10. PLACES THE AVERAGE VALUE INTO AN ARRAY FOR LATER USE.
/////////////////////////////////////////////////////////////////////
    iter = 5;                                // establishes number of times that the ultrasonic takes measurements per rotation step.
    distanceSum = 0;
    Serial.println("[I] STARTING ULTRASONIC READINGS IN 5 SECONDS");
    delay(2000);
    for (i = 1; i<iter+1; i++){              // the pobyte is to take "iter" readings and average them, then index the average as the distance for that position
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = duration*0.034/2;
      distanceSum = distanceSum + distance;
      if (testing0 == 1){
        Serial.print(F("i = "));
        Serial.println(i);
        Serial.print(F("iter = "));
        Serial.println(iter);
        Serial.print(F("duration = "));
        Serial.println(duration);
        Serial.print(F("distance(mm) = "));
        Serial.println(distance);
        Serial.print(F("distanceSum(mm) = "));
        Serial.println(distanceSum);
        delay(2000);
      }//endif testing0
    }//endfor
    distanceAve = distanceSum/iter;
    distanceStorage[1][dummy1] = distanceAve;                         //places distance as index 1 in a matrix
    distanceStorage[2][dummy1] = rottot;                              //places rottot   as index 2 in a matrix
    if (testing0 == 1){
      iter = dummy1;
      for (j = 1; j<iter; j++){  
        Serial.print(F("distanceStorage[1][j] = "));
        Serial.println(distanceStorage[1][j]);
        Serial.print(F("distanceStorage[2][j] = "));
        Serial.println(distanceStorage[2][j]);  
      }//endfor
      delay(2000);
    }//endif testing0
    stepper.step(val);                                     //rotates the shaft by "val"
    }//endwhile
    Serial.println("[I] RETURNING ULTRASONIC TO STARTING POSITION IN 5 SECONDS");
    delay(5000);
    stepper.step(-val*dummy1);                               //rotates the shaft back to starting position                  
  }//end testing2

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Use ultrasonic sensor readings to determine the closest can and it's coordinates.
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (testing3 == 1){
    Serial.println("[I] STARTING ULTRASONIC DISTANCE CALCULATIONS AND OPERATIONS IN 5 SECONDS");
      delay(5000);
      closestcan = 10000;                                                  // initialize with a very high value
      for (i = 1; i<dummy1; i++){
        A = distanceStorage[1][i];      
        closestcan = min(A,closestcan);
        if (testing0 == 1){
          Serial.print(F("i = "));
          Serial.println(i);
          Serial.print(F("dummy1 = "));
          Serial.println(dummy1);
          Serial.print(F("A = "));
          Serial.println(A);
          Serial.print(F("closestcan = "));
          Serial.println(closestcan);
          delay(2000);
        }//endif testing0
      }//endfor
      i=0;
      dummy2 = 0;
      while (closestcan > A){
        A = distanceStorage[1][i];
        i = i + 1;
        dummy2 = i - 1;
        if (testing0 == 1){
          Serial.print(F("A = "));
          Serial.println(A);
          Serial.print(F("closestcan = "));
          Serial.println(closestcan);
          delay(2000);
        }//endif testing0
      }//endwhile 
      absoluteDist  = distanceStorage[1][dummy2];                          //hypotenuse distance to can
      absoluteAngle = distanceStorage[2][dummy2];                          //x,y angle
      if (testing0 == 1){
        Serial.print(F("dummy2 = "));
        Serial.println(dummy2);
        Serial.print(F("absoluteDist(mm) = "));
        Serial.println(absoluteDist);
        Serial.print(F("absoluteAngle(deg) = "));
        Serial.println(absoluteAngle);
        delay(2000);
      }//endif testing0
  }//end testing3

//STEPPER TO MOVE SHOULDER
  if (testing4 == 1){
    Serial.println("[I] MOVING SHOULDER");
    val = 0;                                                              //2048 is one whole rotation
    dummy1 = 0;                                                           //sets index variable for distance matrix
    rottot = 0;                                                           // sets up variable for use  
    rotloc = 0;                                                           // sets up variable for use
    if ((maxrot >= rottot) && (dummy1 > 100)){                            //allows rotation until angle maxrot is reached.
      val = 8;                                                              //incremintally increases rotation by a fixed amount
      dummy1 = dummy1 + 1;                                                  //indexes each loop
      rotloc = val/2048*2*M_PI;                                               //determines radian angle traveled each loop
      rottot = rottot+rotloc;                                               //determines total radian angle traveled
      delay(200);                                                           //sets a delay between rotations
//    stepper.step(val);                                                  //rotates the shaft by "val"
    }//endif                                               
  }//end testing4

////////////////////////////////////////////////////////////////////////      
// SERVO TO MOVE UPPER AND LOWER ARM, THIS ROUTINE USES STRAIGHT CYLINDRICAL COORDIANTES
////////////////////////////////////////////////////////////////////////
  if (testing6 == 1){
  ArmAngle = 0;
  servo1.write(ArmAngle);
  servo2.write(ArmAngle); 
  Serial.println("[I] MOVING UPPER AND LOWER ARM SECTION");
  absoluteDist = 14;   
  if (testing0 == 1){
    Serial.print("absoluteDist = ");
    Serial.println(absoluteDist);
    Serial.print("ArmAngle = ");
    Serial.println(ArmAngle);
    delay(2000);
  }//endif
  ArmAngle = asin((absoluteDist/2)/ArmLength) * rtd;    
  servo1.write(ArmAngle);
  servo2.write(ArmAngle); 
  if (testing0 == 1){
    Serial.print("absoluteDist = ");
    Serial.println(absoluteDist);
    Serial.print("ArmAngle = ");
    Serial.println(ArmAngle);
    delay(2000);
  }//endif                
  }//end testing6


////////////////////////////////////////////////////////////////////////////////  
// SERVO TO MOVE WRIST, GET CORRECT ANGLE AND OPEN AND CLOSE IT.
////////////////////////////////////////////////////////////////////////////////
if (testing7 == 1){
    Serial.println("[I] MOVING WRIST, SERVO");
    byte wristLength = 5;                                       // define wrist length in inches
    byte wristMaxAngle = absoluteAngle;                         // defined in degrees
    byte wristAngle = 0;                                        // intialization of wrist angle in degrees
    servo3.write(wristAngle);//end while                       //command to rotate the wrist servo to the specified angle            
  }//end testing7

////////////////////////////////////////////////////////////////////////////////
// STEPPER TO MOVE WRIST, GET CORRECT ANGLE AND OPEN AND CLOSE IT.
////////////////////////////////////////////////////////////////////////////////
  if (testing8 == 1){
    Serial.println("[I] MOVING WRIST, STEPPER");
    val = 0;                                                   //2048 is one whole rotation
    dummy1 = 0;                                                //sets index variable for distance matrix
//    maxrot = M_PI/2;                                           //sets maximum rotation angle
    rottot = 0;                                                // sets up variable for use  
    rotloc = 0;                                                // sets up variable for use
    if ((maxrot >= rottot) && (dummy1 > 100)){                 //allows rotation until angle maxrot is reached.
    val = 8;                                                   //incremintally increases rotation by a fixed amount
    dummy1 = dummy1 + 1;                                       //indexes each loop
    rotloc = val/2048*2*M_PI;                                  //determines radian angle traveled each loop
    rottot = rottot+rotloc;                                    //determines total radian angle traveled
    delay(200);                                                //sets a delay between rotations
//    stepper.step(val);                                       //rotates the shaft by "val"
}//endif                
    delay(1000);                                               //delays one second
  }//end testing8  





  
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Determine if the object is a bottle or can, probably by the diameter or circumference.
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!






  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // If can, rotate until tab faces correct direction
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!





  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Pull and reseat tab
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
angle4 = 0;                                                         // sets current position(right now)
angle5 = 0;                                                         // sets current position(right now)
if (testing9 == 1){
  Serial.println("[I] MOVING SERVOS TO TAB");
  // OPERATE STRONG SERVO TO TAB                                   
     while((angle4 < angle4toTab) && (angle5 < angle5toTab)){
     if(angle4 < angle4toTab){                                      // stipulate max angle
     angle4 = angle4 + 1;}                                          // stipulate angle step
     servo4.write(angle4);                                          // command to rotate the servo to the specified angle
     delay(10);                                                     // minidelay between incrimental rotation                       
  // OPERATE WEAK SERVO TO TAB                                  
     if(angle5 < angle5toTab){                                      // stipulate angle
      angle5 = angle5 + 1;}                                         // stipulate angle step
     servo5.write(angle5);                                          // command to rotate the servo to the specified angle
     delay(10);}//endfor                                            // minidelay between incrimental rotation                       

  Serial.println("[I] PULLING TAB OPEN");
  // OPERATE STRONG SERVO TO PULL TAB OPEN                                   
     while((angle4 < angle4toTab) && (angle5 < angle5toTab)){
     if(angle4 < angle4toTab){                                      // stipulate max angle
     angle4 = angle4 + 1;}                                          // stipulate angle step
     servo4.write(angle4);                                          // command to rotate the servo to the specified angle
     delay(10);                                                     // minidelay between incrimental rotation                       
  // OPERATE WEAK SERVO TO PULL TAB OPEN                                  
     if(angle5 < angle5toTab){                                      // stipulate angle
      angle5 = angle5 + 1;}                                         // stipulate angle step
     servo5.write(angle5);                                          // command to rotate the servo to the specified angle
     delay(10);}//endfor                                            // minidelay between incrimental rotation                       

  Serial.println("[I] DEPRESSING TAB");
  // OPERATE STRONG SERVO TO DEPRESS TAB                                   
     while((angle4 < angle4toTab) && (angle5 < angle5toTab)){
     if(angle4 < angle4toTab){                                      // stipulate max angle
     angle4 = angle4 + 1;}                                          // stipulate angle step
     servo4.write(angle4);                                          // command to rotate the servo to the specified angle
     delay(10);                                                     // minidelay between incrimental rotation                       
  // OPERATE WEAK SERVO TO DEPRESS TAB                                  
     if(angle5 < angle5toTab){                                      // stipulate angle
      angle5 = angle5 + 1;}                                         // stipulate angle step
     servo5.write(angle5);                                          // command to rotate the servo to the specified angle
     delay(10);}//endfor                                            // minidelay between incrimental rotation                       


  Serial.println("[I] RETRACTING TAB OPENER TO REST POSITION");
  // OPERATE STRONG SERVO TO START POSITION                                   
     while((angle4 < angle4toTab) && (angle5 < angle5toTab)){
     if(angle4 < angle4toTab){                                      // stipulate max angle
     angle4 = angle4 + 1;}                                          // stipulate angle step
     servo4.write(angle4);                                          // command to rotate the servo to the specified angle
     delay(10);                                                     // minidelay between incrimental rotation                       
  // OPERATE WEAK SERVO TO START POSITION                                  
     if(angle5 < angle5toTab){                                      // stipulate angle
      angle5 = angle5 + 1;}                                         // stipulate angle step
     servo5.write(angle5);                                          // command to rotate the servo to the specified angle
     delay(10);}//endfor                                            // minidelay between incrimental rotation                       
}//end testing9



}  //ENDS VOID LOOP              
///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
