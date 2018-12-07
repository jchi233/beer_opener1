


            // Include libraries
#include <Servo.h>                 // Servo position control library
#include <Stepper.h>               // Stepper motor control 

#define STEPS 32     //32

Servo servo1;                           //initialize a servo for the upper arm
Servo servo2;                           //initialize a servo for the lower arm
Servo servo3;                           //initialize a servo for the wrist
Servo servo4;                           //initialize a servo for strong can opener
Servo servo5;                           //initialize a servo for weak can opener
Stepper stepper (STEPS, 8, 10, 9, 11);
int press1_pin     = 2;
int xToggle1_pin   = 0;
int yToggle1_pin   = 1;
//int press2_pin     = 
//int xToggle2_pin   =
//int yToggle2_pin   =
/////////////////////////////////////////////////////////////////
// IMPORTANT VARIABLES USED THROUGHOUT THE CODE ARE LISTED HERE
/////////////////////////////////////////////////////////////////
int testing0          = 1;                             // prints all debug statements
int testing1          = 0;                             // testing LED output and serial print
int testing2          = 0;                             // testing stepper with ultrasonic output
int testing3          = 0;                             // testing can coordinate calculations output
int testing4          = 0;                             // testing shoulder stepper output
int testing5          = 1;                             // testing upper arm servo output                    
    int testing5_1    = 1;                             // testing with toggle1                    
int testing6          = 1;                             // testing lower arm servo output                    
    int testing6_1    = 1;                             // testing with toggle1                    
int testing7          = 0;                             // testing wrist servo output
int testing8          = 0;                             // testing wrist stepper output
int testing9          = 0;                             // testing tab opener: to tab from start output      
    int testing9_1    = 0;                             // testing with toggle1                    
int testing10         = 0;                             // testing tab opener: open tab output               
    int testing10_1   = 0;                             // testing with toggle1                    
int testing11         = 0;                             // testing tab opener: push tab down output          
    int testing11_1   = 0;                             // testing with toggle1                    
int testing12         = 0;                             // testing tab opener: retreat to start output       
    int testing12_1   = 0;                             // testing with toggle1                    
int testing13         = 0;                             // testing LED output
int iter;                                      // variable loop operator
int i;                                        // variable, loop operator
int angle = 0;                                // variable, initial angle for servo control
int angle1 = 0;                               // variable, initial angle for servo control
int angle2 = 0;                               // variable, initial angle for servo control
int angle3 = 0;                               // variable, initial angle for servo control
int angle4 = 0;                               // variable, initial angle for servo control
int angle5 = 0;                               // variable, initial angle for servo control
int runn  = 0;                                // variable, debug mode setup
int val   = 0;                                // 
int x     = 0;                                // variable for x location
int y     = 0;                                // variable for y location
int z     = 0;                                // variable for z location
int xnow     = 0;                             // variable for current tip x location
int ynow     = 0;                             // variable for current tip y location
int znow     = 0;                             // variable for current tip z location
int xcomputed = 0;                            // position determined by ultrasonic
int ycomputed = 0;                            // position determined by ultrasonic
int zcomputed = 0;                            // position determined by ultrasonic
int step1 = 0;                                // variable that determines arm speed
int dummy1;                                   // dummy counting variable
int dummy2;                                   // dummy counting variable
int dummy3;                                   // dummy counting variable
int dummy4;                                   // dummy counting variable
int dummy5;                                   // dummy counting variable
int dummy6;                                   // dummy counting variable
int dummy7;                                   // dummy counting variable
int rotloc;                                   // variable for determining step rotation 
int rottot;                                   // variable for determining total rotation 
int maxrot;                                   // sets maximum rotation value for steppers and servos
int distanceStorage[3][100];                  // stores ultrasonic readings
int distanceSum;                              // summed US distance at each step
int distanceAve;                              // computed average US distance at each step
const int trigPin = 4;                        //trig pin for the ultrasonic sensor
const int echoPin = 3;                        //echo pin for the ultrasonic sensor
//long duration;                                //variable for sensor travel time
int duration;                                 //variable for sensor travel time
int distance;                                 //variable for calculated distance from "duration"
int gotox         = 0 ;
int gotoy         = 0 ;
int increments    = 100;                                //variable for incremental change of arm

void setup() {                                // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);               // LED on board
  pinMode(press1_pin, INPUT);                 // for toggle press
  digitalWrite(press1_pin, HIGH);             // for toggle press
  pinMode(trigPin, OUTPUT);                   // ultrasonic pin
  pinMode(echoPin, INPUT);                    // ultrasonic pin
  Serial.begin(9600);
  servo1.attach(1);
  servo2.attach(2);
  servo3.attach(3);
  servo4.attach(4);
  servo5.attach(5);
  stepper.setSpeed(1000);

}

void loop(){                                     // put your main code here, to run repeatedly:
  if (testing1 == 1){
    //one long, five quick
    digitalWrite(LED_BUILTIN, HIGH);             // turn the LED on (HIGH is the voltage level)
    delay(1000);                                 // wait for a second
    digitalWrite(LED_BUILTIN, LOW);              // turn the LED off by making the voltage LOW
    delay(100);                                  // wait for a second
    for (i = 0; i < 5; i++){
    digitalWrite(LED_BUILTIN, HIGH);  
   delay(50);                       
    digitalWrite(LED_BUILTIN, LOW);    
    delay(50);}//endfor
 Serial.print("i = ");
 Serial.println(i);
  }//end testing1
   

  
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Use ultrasonic sensor to survey the area and record distance readings to file
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  if (testing2 == 1){
    val = 0;                                                   //2048 is one whole rotation
    dummy1 = 0;                                                //sets index variable for distance matrix
    maxrot = M_PI/2;                                             //sets maximum rotation angle
    rottot = 0;                                                // sets up variable for use  
    rotloc = 0;                                                // sets up variable for use
    if ((maxrot >= rottot) && (dummy1 > 100)){                 //allows rotation until angle maxrot is reached.
    val = 8;                                                   //incremintally increases rotation by a fixed amount
    dummy1 = dummy1 + 1;                                       //indexes each loop
    rotloc = val/2048*2*M_PI;                                    //determines radian angle traveled each loop
    rottot = rottot+rotloc;                                    //determines total radian angle traveled
///////////////////////////////////////////////////////////////
    //USES ULTRASONIC TO MEASURE 10 DISTANCES AT EACH TURN. AVERAGES THESE 10. PLACES THE AVERAGE VALUE INTO AN ARRAY FOR LATER USE.
    iter = 10;                                                 // establishes number of times that the ultrasonic takes measurements per rotation step.
    distanceSum = 0;
    for (i = 1; i<iter; i++){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration*0.034/2;
    distanceSum = distanceSum + distance;
    }//endfor
    distanceAve = distanceSum/iter;
    distanceStorage[1][dummy1] = distanceAve;                         //places distance as index 1 in a matrix
//    PFunc("distanceAve",distanceAve);
    distanceStorage[2][dummy1] = rottot;                              //places rottot   as index 2 in a matrix
//    PFunc("rottot",rottot);
    distanceStorage[3][dummy1] = dummy1;                              //places dummy1   as index 3 in a matrix
//    PFunc("dummy1",dummy1);
//    distanceStorage(4,dummy1) = distance;                           //places distance as index 4 in a matrix
//    distanceStorage(5,dummy1) = distance;                           //places distance as index 5 in a matrix
///////////////////////////////////////////////////////////////    
    delay(200);                                                       //sets a delay between rotations
 stepper.step(val);}//endif                                           //rotates the shaft by "val"
 delay(1000);                                                         //delays one second
  }//end testing2


  

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Use ultrasonic sensor readings to determine the closest can and it's coordinates.
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // THIS ROUTINE SHOULD OUTPUT AN X, Y, Z VALUE TO BE CONSIDERED COMPLETE. 
  // A STEPPER SHOULD ROTATE THE SENSOR. THE STEPPER ANGLE AND SENSOR DISTANCE.....
  // .....SHOULD PROVIDE AN X, Y. FOR A FIRST VERSION, Z CAN BE ZERO.

  int closestcan;
  int A;
  int B;
  int absoluteAngle;
  int absoluteDist;
  int xcanpos = 0;
  int ycanpos = 0;
  if (testing3 == 1){
      closestcan = 10000;
      dummy2 = 0;
      if (i<100){
        A = distanceStorage[1][i-1];
        //B = distanceStorage[1][i+1];   
        //closestcan = min(distanceStorage[1][1],distanceStorage[1][2]);    
        closestcan = min(A,closestcan);
          if (A - closestcan >= 0){
            dummy2 = dummy2 + i;                                              //gives the index for the array that has the closest value
          }//endif  
      }//endif
      absoluteDist = distanceStorage[1][dummy2];                              //hypotenuse distance to can
      absoluteAngle = distanceStorage[2][dummy2];                             //x,y angle
      //absolutePhi = distanceStorage[3][dummy2];                             //z, 3d spherical angle
      xcanpos=sin(absoluteAngle)*absoluteDist;                                //find can x position
      ycanpos=cos(absoluteAngle)*absoluteDist;                                //find can y position 
  }//end testing3

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Actuating the arms to desired position to grasp the can/bottle goes here.
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//STEPPER TO MOVE SHOULDER
  if (testing4 == 1){
    val = 0;                                                              //2048 is one whole rotation
    dummy1 = 0;                                                           //sets index variable for distance matrix
    maxrot = M_PI/2;                                                        //sets maximum rotation angle
    rottot = 0;                                                           // sets up variable for use  
    rotloc = 0;                                                           // sets up variable for use
    if ((maxrot >= rottot) && (dummy1 > 100)){                            //allows rotation until angle maxrot is reached.
    val = 8;                                                              //incremintally increases rotation by a fixed amount
    dummy1 = dummy1 + 1;                                                  //indexes each loop
    rotloc = val/2048*2*M_PI;                                               //determines radian angle traveled each loop
    rottot = rottot+rotloc;                                               //determines total radian angle traveled
    delay(200);                                                           //sets a delay between rotations
    stepper.step(val);}                                                   //rotates the shaft by "val"
    delay(1000);}                                                         //delays one second
 
////////////////////////////////////////////////////////////////////////////////////////
//ARM VARIABLES/////////////////////////////////////////////////////////////////////////
  int upperArmLength = 5;                       // define upper arm length in inches
  int upperArmMaxAngle = absoluteAngle;         // sets the maximum angle that the arm will go to from start
  int upperArmAngle = 180;                      // the initial defined angle position of the upper arm
  int lowerArmLength = 5;                       // define lower arm length in inches
  int lowerArmMaxAngle = absoluteAngle;         // sets the maximum angle that the arm will go to from start
  int lowerArmAngle = 0;                        // the initial defined angle position of the lower arm
////////////////////////////////////////////////////////////////////////////////////////  
// SERVO TO MOVE UPPER ARM  
////////////////////////////////////////////////////////////////////////////////////////
  if (testing5 = 1){
      Serial.print("[I] MOVING UPPER ARM SECTION");
    if (testing5_1 = 1){
  xcanpos = analogRead(xToggle1_pin);
  xcanpos = map (xcanpos, 0, 1023, -10, 10);
  ycanpos = analogRead(yToggle1_pin);
  ycanpos = map (ycanpos, 0, 1023, 0, 10);
  delay(15);
    }  
    ynow  = upperArmLength * sin(upperArmAngle) + lowerArmLength * sin(lowerArmAngle);
    xnow  = upperArmLength * cos(upperArmAngle) + lowerArmLength * cos(lowerArmAngle);
    gotox = xcanpos / increments;
    if (testing0 = 1){
      Serial.print("xcanpos");
      Serial.println(xcanpos);
      Serial.print("ycanpos");
      Serial.println(ycanpos);
      Serial.print("xnow");
      Serial.println(xnow);
      Serial.print("ynow");
      Serial.println(ynow);
    }
    if(xnow >= xcanpos){
       if (testing0 = 1){
      Serial.print("xnow >= xcanpos");
       }
      while(xnow >= gotox){  
      upperArmAngle = upperArmAngle + 1;                                                                   //arbitrarily move upper arm
      servo1.write(upperArmAngle);                                                                         //command to rotate the upper arm servo to the specified angle                                  
      }//end while
    }//end if  
    else{
       if (testing0 = 1){
      Serial.print("xnow < xcanpos");
       }
      while(xnow < gotox){  
      upperArmAngle = upperArmAngle - 1;                                                                   //arbitrarily move upper arm
      servo1.write(upperArmAngle);                                                                         //command to rotate the upper arm servo to the specified angle                                  
      }//end while
    }//end else
  }//end testing5
      
// SERVO TO MOVE LOWER ARM
  if (testing6 = 1){
    if (testing6_1=1){
    }
      ynow = upperArmLength * sin(upperArmAngle) + lowerArmLength * sin(lowerArmAngle);                    //computing the y location that the tip is at
      xnow = upperArmLength * cos(upperArmAngle) + lowerArmLength * cos(lowerArmAngle);                    //computing the x location that the tip is at
      //y = tan(absoluteAngle) * xnow;                                                                       //computing the y location on the slope line corresponding to xnow.
      gotoy = ycanpos / increments;
      if(ynow <= gotoy){
        while(ynow <= gotoy){
        lowerArmAngle = lowerArmAngle + 1;
        servo2.write(lowerArmAngle);
        }//end while                                                                                       //command to rotate the lower arm servo to the specified angle            
      }//end if
      else{ 
        while(ynow > gotoy){
        lowerArmAngle = lowerArmAngle - 1;
        servo2.write(lowerArmAngle);
        }//end while 
      }//end else
    }//end testing6                  

// SERVO TO MOVE WRIST
  if (testing7 = 1){
    int wristLength = 5;                                                     // define wrist length in inches
    int wristMaxAngle = absoluteAngle;                                       // defined in degrees
    int wristAngle = 0;                                                      // intialization of wrist angle in degrees
    servo3.write(wristAngle);//end while                                //command to rotate the wrist servo to the specified angle            
      }//end testing7

// STEPPER TO MOVE WRIST
  if (testing8 = 1){
    val = 0;                                                   //2048 is one whole rotation
    dummy1 = 0;                                                //sets index variable for distance matrix
    maxrot = M_PI/2;                                             //sets maximum rotation angle
    rottot = 0;                                                // sets up variable for use  
    rotloc = 0;                                                // sets up variable for use
    if ((maxrot >= rottot) && (dummy1 > 100)){                 //allows rotation until angle maxrot is reached.
    val = 8;                                                   //incremintally increases rotation by a fixed amount
    dummy1 = dummy1 + 1;                                       //indexes each loop
    rotloc = val/2048*2*M_PI;                                    //determines radian angle traveled each loop
    rottot = rottot+rotloc;                                    //determines total radian angle traveled
    delay(200);                                                //sets a delay between rotations
    stepper.step(val);}//endif                                 //rotates the shaft by "val"
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
int angle4toTab = 180;                                              // sets final position for this step
int angle5toTab = 180;                                              // sets final position for this step
if (testing9 = 1){
  // OPERATE STRONG SERVO TO TAB                                   
     while((angle4 < angle4toTab) && (angle5 < angle5toTab)){
     if(angle4 < angle4toTab){                                      // stipulate max angle
     angle4 = angle4 + 1;}                                          // stipulate angle step
     servo4.write(angle4);                                     // command to rotate the servo to the specified angle
     delay(50);                                                     // minidelay between incrimental rotation                       
  // OPERATE WEAK SERVO TO TAB                                  
     if(angle5 < angle5toTab){                                      // stipulate angle
      angle5 = angle5 + 1;}                                         // stipulate angle step
     servo5.write(angle5);                                     // command to rotate the servo to the specified angle
     delay(50);}//endfor                                            // minidelay between incrimental rotation                       
}//end testing9 

angle4 = angle4toTab;                                               // sets current position at beginning of this step
angle5 = angle5toTab;                                               // sets current position at beginning of this step
angle4toTab = 0;                                                    // sets final position for this step
angle5toTab = 0;                                                    // sets final position for this step
if (testing10 = 1){
  // OPERATE STRONG SERVO TO PULL TAB OPEN                                   
     while((angle4 < angle4toTab) && (angle5 < angle5toTab)){
     if(angle4 < angle4toTab){                                      // stipulate max angle
     angle4 = angle4 + 1;}                                          // stipulate angle step
     servo4.write(angle4);                                     // command to rotate the servo to the specified angle
     delay(50);                                                     // minidelay between incrimental rotation                       
  // OPERATE WEAK SERVO TO PULL TAB OPEN                                  
     if(angle5 < angle5toTab){                                      // stipulate angle
      angle5 = angle5 + 1;}                                         // stipulate angle step
     servo5.write(angle5);                                     // command to rotate the servo to the specified angle
     delay(50);}//endfor                                            // minidelay between incrimental rotation                       
}//end testing10 

angle4 = angle4toTab;                                               // sets current position at beginning of this step
angle5 = angle5toTab;                                               // sets current position at beginning of this step
angle4toTab = 180;                                                  // sets final position for this step
angle5toTab = 180;                                                  // sets final position for this step
if (testing11 = 1){
  // OPERATE STRONG SERVO TO DEPRESS TAB                                   
     while((angle4 < angle4toTab) && (angle5 < angle5toTab)){
     if(angle4 < angle4toTab){                                      // stipulate max angle
     angle4 = angle4 + 1;}                                          // stipulate angle step
     servo4.write(angle4);                                     // command to rotate the servo to the specified angle
     delay(50);                                                     // minidelay between incrimental rotation                       
  // OPERATE WEAK SERVO TO DEPRESS TAB                                  
     if(angle5 < angle5toTab){                                      // stipulate angle
      angle5 = angle5 + 1;}                                         // stipulate angle step
     servo5.write(angle5);                                     // command to rotate the servo to the specified angle
     delay(50);}//endfor                                            // minidelay between incrimental rotation                       
}//end testing11 

angle4 = angle4toTab;                                               // sets current position at beginning of this step
angle5 = angle5toTab;                                               // sets current position at beginning of this step
angle4toTab = 0;                                                    // sets final position for this step
angle5toTab = 0;                                                    // sets final position for this step
if (testing12 = 1){
  // OPERATE STRONG SERVO TO START POSITION                                   
     while((angle4 < angle4toTab) && (angle5 < angle5toTab)){
     if(angle4 < angle4toTab){                                      // stipulate max angle
     angle4 = angle4 + 1;}                                          // stipulate angle step
     servo4.write(angle4);                                     // command to rotate the servo to the specified angle
     delay(50);                                                     // minidelay between incrimental rotation                       
  // OPERATE WEAK SERVO TO START POSITION                                  
     if(angle5 < angle5toTab){                                      // stipulate angle
      angle5 = angle5 + 1;}                                         // stipulate angle step
     servo5.write(angle5);                                     // command to rotate the servo to the specified angle
     delay(50);}//endfor                                            // minidelay between incrimental rotation                       
}//end testing12 








  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // If bottle, pop top and dispense.
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




}  //ENDS VOID LOOP              



//int A;
//int B;
//int PFunc(int A,int B){
//  Serial.print(A);
//  Serial.println(B);}



//////////////////////////////////////////////
//  SERVO ROUTINE THE GOES BACK AND FORTH 
//////////////////////////////////////////////
//      for(angle = 0; angle < 180; angle += 1){              // command to move from 0 degrees to 180 degrees                                   
//         servo_test.write(angle);                            //command to rotate the servo to the specified angle
//         delay(15);}                                         //minidelay between incrimental rotation
//      delay(1000);                                          //delay between switching directions
//      for(angle = 180; angle>=1; angle-=5){                 // command to move from 180 degrees to 0 degrees                                 
//         servo_test.write(angle);                            //command to rotate the servo to the specified angle
//         delay(5);}                                           //minidelay between incrimental rotation                       
//
//
//     basicServoFunction(0, 90, 10, 50);}
//
/////////////////////////////////////////////////
//  STEPPER MOTOR ROUTINE
//////////////////////////////////////////////////
//    val = 2048;                                            //sets up one rotation
//    delay(1000);                                           //sets a delay between rotations
// stepper.step(val);//                                     //rotates the shaft by "val"
//  
//////////////////////////////////////
// EXAMPLE OF A FUNCTION SETUP
//////////////////////////////////////
//int myMultiplyFunction(int x, int y){
//  int result;
//  result = x * y;
//  return result;}
//
//
//int i = 2;
//  int j = 3;
//  int k;
//
//  k = myMultiplyFunction(i, j); // k now contains 6
//  Serial.println(k);
//  delay(500)
//
//
//int basicServoFunction(int startPos1, int startPos2, int delayTime1, int delayTime2){
//  for(angle = startPos1; angle < 180; angle += 1)    // command to move from 0 degrees to 180 degrees 
//    {                                  
//      servo_test.write(angle);                 //command to rotate the servo to the specified angle
//      delay(delayTime1);                       
//   } 
//    delay(10);
//    for(angle = startPos2; angle>=1; angle-=5)     // command to move from 180 degrees to 0 degrees 
//    {                                
//      servo_test.write(angle);              //command to rotate the servo to the specified angle
//     delay(delayTime2);                       
//   } 
//}



///////////////////////////////////
//EXAMPLE OF ULTRASONIC SENSOR COMMANDS
///////////////////////////////////
//runn = 0;    
//if (runn == 1){
//    digitalWrite(trigPin, LOW);
//    delayMicroseconds(2);
//
//    digitalWrite(trigPin, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(trigPin, LOW);
//
//    duration = pulseIn(echoPin, HIGH);
//    distance = duration*0.034/2;
//
//    Serial.print("Distance: ");
//    Serial.println(distance);  
//      }

