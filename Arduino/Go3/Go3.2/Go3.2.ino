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
byte press1_pin     = 2;                 //defines pin for toggle1 press
byte xToggle1_pin   = 0;                 //defines pin for toggle1 x-dir
byte yToggle1_pin   = 1;                 //defines pin for toggle1 y-dir
/////////////////////////////////////////////////////////////////
// VARIABLES USED TO INITIALIZE EACH SECTION
/////////////////////////////////////////////////////////////////
const byte testing0          = 1;                   // prbytes all debug statements
const byte testing1          = 0;                   // testing LED output and serial print
const byte testing2          = 0;                   // testing stepper with ultrasonic output
const byte testing3          = 0;                   // testing can coordinate calculations output
const byte testing4          = 0;                   // testing shoulder stepper output
const byte testing5          = 0;                   // testing horizontal arm servo output                    
    const byte testing5_1    = 0;                   // testing lower arm motion with toggle1
    const byte testing5_2    = 0;                   // testing lower arm motion with manual input                                      
const byte testing6          = 1;                   // testing vertical arm output(alternate)  
const byte testing7          = 0;                   // testing wrist servo output
const byte testing8          = 0;                   // testing wrist stepper output
const byte testing9          = 0;                   // testing tab opener: to tab from start output      
/////////////////////////////////////////////////////////////////
// VARIABLE SETUP FOR THE CODE
/////////////////////////////////////////////////////////////////
byte angle             = 0;                   // variable, initial angle for servo control
byte angle1            = 0;                   // variable, initial angle for servo control
byte angle2            = 0;                   // variable, initial angle for servo control
byte angle3            = 0;                   // variable, initial angle for servo control
byte angle4            = 0;                   // variable, initial angle for servo control
byte angle5            = 0;                   // variable, initial angle for servo control
double val            = 0;                   // 
double xnow           = 0;                   // variable for current tip x location
double ynow           = 0;                   // variable for current tip y location
double znow           = 0;                   // variable for current tip z location
byte iter, i, j;                                 // variable, loop operator counter. i is the operator, iter is the max... (from i to iter)
byte dummy1 = 0, dummy2, dummy3, dummy4;          // dummy counting variable
double rotloc = 0;                                  // variable for determining step rotation 
double rottot = 0;                                  // variable for determining total rotation 
double distanceStorage[2][20];                 // stores ultrasonic readings
byte distanceSum;                             // summed ultrasonic distance at each step
byte distanceAve;                             // computed average US distance at each step
const byte trigPin     = 4;                   // trig pin for the ultrasonic sensor
const byte echoPin     = 3;                   // echo pin for the ultrasonic sensor
long duration         = 0;                   // variable for sensor travel time
byte distance          = 0 ;                  // variable for calculated distance from "duration"
double gotox          = 0;                   // variable for setting arm position
double gotoy          = 0;                   // variable for setting arm position
byte increments        = 10;                  // variable for incremental change of arm
byte rot_angle         = 0;
byte closestcan;
byte A;
double absoluteAngle;
double absoluteDist;
double xcanpos = 0;
double ycanpos = 0;
double maxrot = M_PI/2;        //sets maximum rotation angle
byte angle4toTab = 180;                                              // sets final position for this step
byte angle5toTab = 180;                                              // sets final position for this step
double ArmAngle             = 0;                   // variable, initial angle for servo control
double dtr           = 3.1416 / 180;           //degrees to radians
double rtd           = 180/M_PI;               //radians to degrees
byte ArmLength = 9;                       // define upper arm length in inches
byte upperArmLength = 5;                       // define upper arm length in inches
byte lowerArmLength = 5;                       // define lower arm length in inches
byte upperArmMaxAngle = 0;         // sets the maximum angle that the arm will go to from start
byte upperArmAngle = 0;                      // the initial defined angle position of the upper arm
byte lowerArmMaxAngle = 0;         // sets the maximum angle that the arm will go to from start
byte lowerArmAngle = 0;                        // the initial defined angle position of the lower arm


void setup() {                               // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);              // LED on board
  pinMode(press1_pin, INPUT);                // for toggle press
  digitalWrite(press1_pin, HIGH);            // for toggle press
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

void loop(){                                     // put your main code here, to run repeatedly:  
  if (testing1 == 1){
    Serial.println("[I] STARTING LED AND STEPPER TESTING IN 5 SECONDS");
    delay(2000);
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
    //val = 4096;                                            //sets up one rotation
    val = 512;                                            //sets up one rotation
    delay(1000);                                           //sets a delay between rotations
    stepper.step(val);                                     //rotates the shaft by "val"
    stepper.step(-val);                                     //rotates the shaft by "val"
  }//end testing1
   

  
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Use ultrasonic sensor to survey the area and record distance readings to file
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  rottot = 0;                                                // sets up variable for use  
  rotloc = 0;                                                // sets up variable for use
//  dummy1 = 0;                                                //sets index variable for distance matrix
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
        Serial.print("maxrot = ");
        delay(1000);
        Serial.println(maxrot);
        delay(1000);
        Serial.print("rottot = ");
        delay(1000);
        Serial.println(rottot);
        delay(1000);
        Serial.print("rotloc = ");
        delay(1000);
        Serial.println(rotloc);
        delay(1000);
        Serial.print("dummy1 = ");
        delay(1000);
        Serial.println(dummy1);
        delay(1000);
      }//endif testing0
/////////////////////////////////////////////////////////////////////
//USES ULTRASONIC TO MEASURE 10 DISTANCES AT EACH TURN. AVERAGES THESE 10. PLACES THE AVERAGE VALUE INTO AN ARRAY FOR LATER USE.
/////////////////////////////////////////////////////////////////////
    iter = 5;                                                // establishes number of times that the ultrasonic takes measurements per rotation step.
    distanceSum = 0;
    Serial.println("[I] STARTING ULTRASONIC READINGS IN 5 SECONDS");
    delay(2000);
    for (i = 1; i<iter+1; i++){                                  // the pobyte is to take 100 readings and average them, then index the average as the distance for that position
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = duration*0.034/2;
      distanceSum = distanceSum + distance;
      if (testing0 == 1){
        Serial.print("i = ");
        delay(1000);
        Serial.println(i);
        delay(1000);
        Serial.print("iter = ");
        delay(1000);
        Serial.println(iter);
        delay(1000);
        Serial.print("duration = ");
        delay(1000);
        Serial.println(duration);
        delay(1000);
        Serial.print("distance(mm) = ");
        delay(1000);
        Serial.println(distance);
        delay(1000);
        Serial.print("distanceSum(mm) = ");
        delay(1000);
        Serial.println(distanceSum);
        delay(1000);
      }//endif testing0
    }//endfor
    distanceAve = distanceSum/iter;
    distanceStorage[1][dummy1] = distanceAve;                         //places distance as index 1 in a matrix
    distanceStorage[2][dummy1] = rottot;                              //places rottot   as index 2 in a matrix
    //distanceStorage[3][dummy1] = dummy1;                            //places dummy1   as index 3 in a matrix
    //distanceStorage(4,dummy1) = distance;                           //places distance as index 4 in a matrix
    //distanceStorage(5,dummy1) = distance;                           //places distance as index 5 in a matrix
    if (testing0 == 1){
      iter = dummy1;
      for (j = 1; j<iter; j++){  
        Serial.print("distanceStorage[1][j] = ");
        Serial.println(distanceStorage[1][j]);
        Serial.print("distanceStorage[2][j] = ");
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
        //B = distanceStorage[1][i+1];       
        closestcan = min(A,closestcan);
        if (testing0 == 1){
          Serial.print("i = ");
          delay(1000);
          Serial.println(i);
          delay(1000);
          Serial.print("dummy1 = ");
          delay(1000);
          Serial.println(dummy1);
          delay(1000);
          Serial.print("A = ");
          delay(1000);
          Serial.println(A);
          delay(1000);
          Serial.print("closestcan = ");
          delay(1000);
          Serial.println(closestcan);
          delay(1000);
        }//endif testing0
      }//endfor
      i=0;
      dummy2 = 0;
      while (closestcan > A){
        A = distanceStorage[1][i];
        i = i + 1;
        dummy2 = i - 1;
        if (testing0 == 1){
          Serial.print("A = ");
          delay(1000);
          Serial.println(A);
          delay(1000);
          Serial.print("closestcan = ");
          delay(1000);
          Serial.println(closestcan);
          delay(1000);
        }//endif testing0
      }//endwhile 
      absoluteDist  = distanceStorage[1][dummy2];                          //hypotenuse distance to can
      absoluteAngle = distanceStorage[2][dummy2];                         //x,y angle
      //absolutePhi = distanceStorage[3][dummy2];                         //z, 3d spherical angle
      xcanpos=sin(absoluteAngle)*absoluteDist;                            //find can x position
      ycanpos=cos(absoluteAngle)*absoluteDist;                            //find can y position 
      if (testing0 == 1){
        Serial.print("dummy2 = ");
        delay(1000);
        Serial.println(dummy2);
        delay(1000);
        Serial.print("absoluteDist = ");
        delay(1000);
        Serial.println(absoluteDist);
        delay(1000);
        Serial.print("absoluteAngle = ");
        delay(1000);
        Serial.println(absoluteAngle);
        delay(1000);
        Serial.print("xcanpos(mm) = ");
        delay(1000);
        Serial.println(xcanpos);
        delay(1000);
        Serial.print("ycanpos(mm) = ");
        delay(1000);
        Serial.println(ycanpos);
        delay(1000);
      }//endif testing0
  }//end testing3

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Actuating the arms to desired position to grasp the can/bottle goes here.
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//STEPPER TO MOVE SHOULDER
  if (testing4 == 1){
    Serial.println("[I] MOVING SHOULDER");
    val = 0;                                                              //2048 is one whole rotation
    dummy1 = 0;                                                           //sets index variable for distance matrix
//    maxrot = M_PI/2;                                                        //sets maximum rotation angle
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
////////////////////////////////////////////////////////////////////////////////////////
//ARM VARIABLES/////////////////////////////////////////////////////////////////////////
  upperArmMaxAngle = absoluteAngle;         // sets the maximum angle that the arm will go to from start
  upperArmAngle = 0;                      // the initial defined angle position of the upper arm
  lowerArmMaxAngle = absoluteAngle;         // sets the maximum angle that the arm will go to from start
  lowerArmAngle = 0;                        // the initial defined angle position of the lower arm
  rot_angle = 5;
  gotoy = 0;
  gotox = 0;
////////////////////////////////////////////////////////////////////////      
// SERVO TO MOVE LOWER ARM
////////////////////////////////////////////////////////////////////////
  if (testing5 == 1){
    while (ynow <= ycanpos){
    while (xnow <= xcanpos){
    Serial.println("[I] MOVING LOWER ARM SECTION");
    Serial.println(1000);
    if (testing5_1 == 1){
      xcanpos = analogRead(xToggle1_pin);
      xcanpos = map (xcanpos, 0, 1023, 0, 10);
      ycanpos = analogRead(yToggle1_pin);
      ycanpos = map (ycanpos, 0, 1023, 0, 10);
      delay(15);
    }
    if (testing5_2 ==1){
      xcanpos = 7;
      ycanpos = 7;  
    }
      ynow = upperArmLength * sin(upperArmAngle * dtr) + lowerArmLength * sin(lowerArmAngle * dtr);                    //computing the y location that the tip is at
      xnow = upperArmLength * cos(upperArmAngle * dtr) + lowerArmLength * cos(lowerArmAngle * dtr);                    //computing the x location that the tip is at
      //gotoy needs to be fixed from incriments, to some other system...
      gotoy = ycanpos / increments + gotoy;
      if (testing0 == 1){
        Serial.print("xcanpos = ");
        Serial.print("\t");
        Serial.println("ycanpos = ");
        Serial.print(xcanpos);
        Serial.print("\t");
        Serial.println(ycanpos);
        Serial.print("xnow = ");
        Serial.print("\t");
        Serial.println("ynow = ");
        Serial.print(xnow);
        Serial.print("\t");
        Serial.println(ynow);
        delay(3000);
      }
      if(ynow < gotoy){
        Serial.println("[I] ynow < gotoy");
        while(ynow < gotoy){
        lowerArmAngle = lowerArmAngle + rot_angle;
        servo2.write(lowerArmAngle);                                                                                   //command to rotate the lower arm servo to the specified angle
        ynow = upperArmLength * sin(upperArmAngle * dtr) + lowerArmLength * sin(lowerArmAngle * dtr);                  //recalculate ynow
        if (testing0 == 1){
          Serial.print("upperArmAngle = ");
          Serial.print("\t");
          Serial.println("lowerArmAngle = ");
          Serial.print(upperArmAngle);
          Serial.print("\t");
          Serial.println(lowerArmAngle);
          Serial.print("ynow = ");
          Serial.print("\t");
          Serial.println("gotoy = ");
          Serial.print(ynow);
          Serial.print("\t");
          Serial.println(gotoy);
          delay(3000);
        }
        }//end while                                                                                                   
      }//end if
      else if (ynow > gotoy){ 
        Serial.println("[I] ynow > gotoy");
        //while(ynow > gotoy){
        //taken out for now because we cannot support them yet
        //lowerArmAngle = lowerArmAngle - rot_angle;
        //servo2.write(lowerArmAngle);                                                                                   //command to rotate the lower arm servo to the specified angle
        ynow = upperArmLength * sin(upperArmAngle * dtr) + lowerArmLength * sin(lowerArmAngle * dtr);                  //recalculate ynow
        if (testing0 == 1){
          Serial.print("upperArmAngle = ");
          Serial.print("\t");
          Serial.println("lowerArmAngle = ");
          Serial.print(upperArmAngle);
          Serial.print("\t");
          Serial.println(lowerArmAngle);
          Serial.print("ynow = ");
          Serial.print("\t");
          Serial.println("gotoy = ");
          Serial.print(ynow);
          Serial.print("\t");
          Serial.println(gotoy);
          delay(3000);
        }
        //}//end while 
      }//end else
      else {
        Serial.println("[I] ynow = gotoy");
      }                  
////////////////////////////////////////////////////////////////////////////////////////  
// SERVO TO MOVE UPPER ARM  
////////////////////////////////////////////////////////////////////////////////////////
    Serial.println("[I] MOVING UPPER ARM SECTION");
    ynow  = upperArmLength * sin(upperArmAngle * dtr) + lowerArmLength * sin(lowerArmAngle * dtr);
    xnow  = upperArmLength * cos(upperArmAngle * dtr) + lowerArmLength * cos(lowerArmAngle * dtr);
    //again, same as above, switch from increments to something more robust like delta
    gotox = xcanpos / increments;
    if (testing0 == 1){
        Serial.print("xcanpos = ");
        Serial.print("\t");
        Serial.println("ycanpos = ");
        Serial.print(xcanpos);
        Serial.print("\t");
        Serial.println(ycanpos);
        Serial.print("xnow = ");
        Serial.print("\t");
        Serial.println("ynow = ");
        Serial.print(xnow);
        Serial.print("\t");
        Serial.println(ynow);
        delay(3000);
      }
    if(xnow > gotox){//currently unsupported
       if (testing0 == 1){
        Serial.println("[I] xnow > gotox");
       }
       while(xnow > gotox){//currently unsupported  
       upperArmAngle = upperArmAngle - rot_angle;                                                                   //arbitrarily move upper arm
       servo1.write(upperArmAngle);                                                                         //command to rotate the upper arm servo to the specified angle
       xnow  = upperArmLength * cos(upperArmAngle * dtr) + lowerArmLength * cos(lowerArmAngle * dtr);       //recalculate xnow             
       if (testing0 == 1){
          Serial.print("upperArmAngle = ");
          Serial.print("\t");
          Serial.println("lowerArmAngle = ");
          Serial.print(upperArmAngle);
          Serial.print("\t");
          Serial.println(lowerArmAngle);
          Serial.print("xnow = ");
          Serial.print("\t");
          Serial.println("gotox = ");
          Serial.print(xnow);
          Serial.print("\t");
          Serial.println(gotox);
          delay(3000);
        }        
       }//end while
    }//end if  
    else if (xnow < gotox){
       if (testing0 == 1){
        Serial.println("[I] xnow < gotox");
       }
      while(xnow < gotox){  
      upperArmAngle = upperArmAngle - rot_angle;                                                                   //arbitrarily move upper arm
      servo1.write(upperArmAngle);                                                                         //command to rotate the upper arm servo to the specified angle
      xnow  = upperArmLength * cos(upperArmAngle * dtr) + lowerArmLength * cos(lowerArmAngle * dtr);       //recalculate xnow             
      if (testing0 == 1){
          Serial.print("upperArmAngle = ");
          Serial.print("\t");
          Serial.println("lowerArmAngle = ");
          Serial.print(upperArmAngle);
          Serial.print("\t");
          Serial.println(lowerArmAngle);
          Serial.print("xnow = ");
          Serial.print("\t");
          Serial.println("gotox = ");
          Serial.print(xnow);
          Serial.print("\t");
          Serial.println(gotox);
          delay(3000);
        }   
      }//end while
    }//end else
    else {
      Serial.println("[I] xnow = gotox");
    }
    }
  }
  }//end testing5



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
    delay(5000);
  }//endif
  ArmAngle = asin((absoluteDist/2)/ArmLength) * rtd;    
  servo1.write(ArmAngle);
  servo2.write(ArmAngle); 
  if (testing0 == 1){
    Serial.print("absoluteDist = ");
    Serial.println(absoluteDist);
    Serial.print("ArmAngle = ");
    Serial.println(ArmAngle);
    delay(5000);
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
