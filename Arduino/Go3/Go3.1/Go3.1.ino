            // Include libraries
#include <Stepper.h>               // Stepper motor control 
#include <math.h>
#define STEPS 64     //32               //stepper motor step number
Stepper stepper (STEPS, 8, 10, 9, 11);
/////////////////////////////////////////////////////////////////
// VARIABLES USED TO INITIALIZE EACH SECTION
/////////////////////////////////////////////////////////////////
const byte testing0          = 1;                   // prbytes all debug statements
const byte testing1          = 0;                   // testing LED output and serial print
const byte testing2          = 1;                   // testing stepper with ultrasonic output
const byte testing3          = 1;                   // testing can coordinate calculations output     
/////////////////////////////////////////////////////////////////
// VARIABLE SETUP FOR THE CODE
/////////////////////////////////////////////////////////////////
double val            = 0;                   // 
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
const byte delay1      = 100;
const byte delay2      = 2000;
byte closestcan;
byte A;
double absoluteAngle;
double absoluteDist;
double xcanpos = 0;
double ycanpos = 0;
const double maxrot = M_PI/2;        //sets maximum rotation angle

void setup() {                               // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);              // LED on board
  pinMode(trigPin, OUTPUT);                  // ultrasonic pin
  pinMode(echoPin, INPUT);                   // ultrasonic pin
  Serial.begin(9600);                        // communicates with serial monitor
  stepper.setSpeed(200);                    // sets stepper motor speed, 1000 is highest
}

void loop(){                                     // put your main code here, to run repeatedly:   
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Use ultrasonic sensor to survey the area and record distance readings to file
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  rottot = 0;                                                // sets up variable for use  
  rotloc = 0;                                                // sets up variable for use
  dummy1 = 0;                                                //sets index variable for distance matrix
  val = 0;                                                   //2048 is one whole rotation
  if (testing2 == 1){
    Serial.println(F("[I] STARTING STEPPER FOR ULTRASONIC IN 5 SECONDS"));
    delay(delay2);
    while (maxrot >= rottot){                                //allows rotation until angle maxrot is reached.
      val = 32;                                                //incremintally increases rotation by a fixed amount
      dummy1 = dummy1 + 1;                                     //indexes each loop
      rotloc = val/2048*2*M_PI;                                //determines radian angle traveled each loop
      rottot = rottot + rotloc;                                //determines total radian angle traveled
      if (testing0 == 1){
        Serial.print(F("maxrot = "));
        delay(delay1);
        Serial.println(maxrot);
        delay(delay1);
        Serial.print(F("rottot = "));
        delay(delay1);
        Serial.println(rottot);
        delay(delay1);
        Serial.print(F("rotloc = "));
        delay(delay1);
        Serial.println(rotloc);
        delay(delay1);
        Serial.print(F("dummy1 = "));
        delay(delay1);
        Serial.println(dummy1);
        delay(delay1);
      }//endif testing0
/////////////////////////////////////////////////////////////////////
//USES ULTRASONIC TO MEASURE 10 DISTANCES AT EACH TURN. AVERAGES THESE 10. PLACES THE AVERAGE VALUE INTO AN ARRAY FOR LATER USE.
/////////////////////////////////////////////////////////////////////
    iter = 1;                                                // establishes number of times that the ultrasonic takes measurements per rotation step.
    distanceSum = 0;
    Serial.println(F("[I] STARTING ULTRASONIC READINGS IN 5 SECONDS"));
    delay(delay2);
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
        Serial.print(F("i = "));
        delay(delay1);
        Serial.println(i);
        delay(delay1);
        Serial.print(F("iter = "));
        delay(delay1);
        Serial.println(iter);
        delay(delay1);
        Serial.print(F("duration = "));
        delay(delay1);
        Serial.println(duration);
        delay(delay1);
        Serial.print(F("distance(mm) = "));
        delay(delay1);
       Serial.println(distance);
        delay(delay1);
        Serial.print(F("distanceSum(mm) = "));
        delay(delay1);
        Serial.println(distanceSum);
        delay(delay1);
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
        Serial.print(F("distanceStorage[1][j] = "));
        Serial.println(distanceStorage[1][j]);
        Serial.print(F("distanceStorage[2][j] = "));
        Serial.println(distanceStorage[2][j]);  
      }//endfor
      delay(delay2);
    }//endif testing0
    stepper.step(val);                                     //rotates the shaft by "val"
    }//endwhile
    Serial.println(F("[I] RETURNING ULTRASONIC TO STARTING POSITION IN 5 SECONDS"));
    delay(5000);
    stepper.step(-val*dummy1);                               //rotates the shaft back to starting position                  
  }//end testing2

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Use ultrasonic sensor readings to determine the closest can and it's coordinates.
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (testing3 == 1){
    Serial.println(F("[I] STARTING ULTRASONIC DISTANCE CALCULATIONS AND OPERATIONS IN 5 SECONDS"));
      delay(5000);
      closestcan = 10000;                                                  // initialize with a very high value
      for (i = 1; i<dummy1; i++){
        A = distanceStorage[1][i];
        //B = distanceStorage[1][i+1];       
        closestcan = min(A,closestcan);
        if (testing0 == 1){
          Serial.print(F("i = "));
          delay(delay1);
          Serial.println(i);
          delay(delay1);
          Serial.print(F("dummy1 = "));
          delay(delay1);
          Serial.println(dummy1);
          delay(delay1);
          Serial.print(F("A = "));
          delay(delay1);
          Serial.println(A);
          delay(delay1);
          Serial.print(F("closestcan = "));
          delay(delay1);
          Serial.println(closestcan);
          delay(delay1);
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
          delay(delay1);
          Serial.println(A);
          delay(delay1);
          Serial.print(F("closestcan = "));
          delay(delay1);
          Serial.println(closestcan);
          delay(delay1);
        }//endif testing0
      }//endwhile 
      absoluteDist  = distanceStorage[1][dummy2];                          //hypotenuse distance to can
      absoluteAngle = distanceStorage[2][dummy2];                         //x,y angle
      //absolutePhi = distanceStorage[3][dummy2];                         //z, 3d spherical angle
      xcanpos=sin(absoluteAngle)*absoluteDist;                            //find can x position
      ycanpos=cos(absoluteAngle)*absoluteDist;                            //find can y position 
      if (testing0 == 1){
        Serial.print(F("dummy2 = "));
        delay(delay1);
        Serial.println(dummy2);
        delay(delay1);
        Serial.print(F("absoluteDist = "));
        delay(delay1);
        Serial.println(absoluteDist);
        delay(delay1);
        Serial.print(F("absoluteAngle = "));
        delay(delay1);
        Serial.println(absoluteAngle);
        delay(delay1);
        Serial.print(F("xcanpos(mm) = "));
        delay(delay1);
        Serial.println(xcanpos);
        delay(delay1);
        Serial.print(F("ycanpos(mm) = "));
        delay(delay1);
        Serial.println(ycanpos);
        delay(delay1);
      }//endif testing0
  }//end testing3
}  //ENDS VOID LOOP

