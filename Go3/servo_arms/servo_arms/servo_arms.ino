            // Include libraries
#include <Servo.h>                 // Servo position control library
Servo servo1;                           //initialize a servo for the upper arm
Servo servo2;                           //initialize a servo for the lower arm
/////////////////////////////////////////////////////////////////
// VARIABLES USED TO INITIALIZE EACH SECTION
/////////////////////////////////////////////////////////////////
byte testing0          = 1;                   // prbytes all debug statements
byte testing14         = 1;                   // testing for arm, cylindrical. This or 5 and 6, not all 3 together
/////////////////////////////////////////////////////////////////
// VARIABLE SETUP FOR THE CODE
/////////////////////////////////////////////////////////////////
double ArmAngle             = 0;                   // variable, initial angle for servo control
double absoluteAngle;
double absoluteDist;
double maxrot = M_PI/2;        //sets maximum rotation angle
double dtr           = 3.1416 / 180;           //degrees to radians
double rtd           = 180/M_PI;               //radians to degrees


void setup() {                               // put your setup code here, to run once:
  Serial.begin(9600);                        // communicates with serial monitor
  servo1.attach(6);                          // sets pin for servo1
  servo2.attach(7);                          // sets pin for servo2
}

void loop(){                                     // put your main code here, to run repeatedly:  

////////////////////////////////////////////////////////////////////////////////////////
//ARM VARIABLES/////////////////////////////////////////////////////////////////////////
byte ArmLength = 9;                       // define upper arm length in inches

////////////////////////////////////////////////////////////////////////      
// SERVO TO MOVE UPPER AND LOWER ARM, THIS ROUTINE USES STRAIGHT CYLINDRICAL COORDIANTES
////////////////////////////////////////////////////////////////////////
  if (testing14 == 1){
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
    }
    ArmAngle = asin((absoluteDist/2)/ArmLength) * rtd;    
    servo1.write(ArmAngle);
    servo2.write(ArmAngle); 
    if (testing0 == 1){
      Serial.print("absoluteDist = ");
      Serial.println(absoluteDist);
      Serial.print("ArmAngle = ");
      Serial.println(ArmAngle);
      delay(5000);
    }               
  }//end testing14



}  //ENDS VOID LOOP              
///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////


