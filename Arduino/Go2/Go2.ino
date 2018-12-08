            // Include libraries

//////////////////////////////////////////////
// VARIABLES USED TO INITIALIZE EACH SECTION
//////////////////////////////////////////////
int analogPin0 = A0;     //reads pin 0  
int analogPin1 = A1;     //reads pin 1  
int analogPin2 = A2;     //reads pin 2  
int analogPin3 = A3;     //reads pin 3  
int analogPin4 = A4;     //reads pin 4  
int analogPin5 = A5;     //reads pin 5 
int i = 0 ;              //dummy variable
const byte testing1 = 1;
const byte testing2 = 1;
const byte testing3 = 1;
const byte testing4 = 1;
const byte testing5 = 1;
double analogPin0Value, analogPin1Value, analogPin2Value, analogPin3Value, analogPin4Value, analogPin5Value;


void setup() {                               // put your setup code here, to run once:
 pinMode(LED_BUILTIN, OUTPUT);              // LED on board
 Serial.begin(9600);                        // communicates with serial monitor
}

void loop() {                                     // put your main code here, to run repeatedly:  
  if (testing1 == 1){
    Serial.println("[I] STARTING LED AND STEPPER TESTING IN 5 SECONDS");
    delay(5000);
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
   
///////////////////////////////////////////////
//////////////////////////////////////////////
///////////////////////////////////////////////

  if (testing2 == 1){
    Serial.println("[I] STARTING TO READ AND PRINT ANALOG VALUES IN 5 SECONDS");
    delay(5000);
    analogPin0Value = analogRead(analogPin0);
    analogPin1Value = analogRead(analogPin1);
    analogPin2Value = analogRead(analogPin2);
    analogPin3Value = analogRead(analogPin3);
    analogPin4Value = analogRead(analogPin4);
    analogPin5Value = analogRead(analogPin5);
    Serial.print("analogPin0Value = ");
    Serial.println(analogPin0Value);
    Serial.print("analogPin1Value = ");
    Serial.println(analogPin1Value);
    Serial.print("analogPin2Value = ");
    Serial.println(analogPin2Value);
    Serial.print("analogPin3Value = ");
    Serial.println(analogPin3Value);
    Serial.print("analogPin4Value = ");
    Serial.println(analogPin4Value);
    Serial.print("analogPin5Value = ");
    Serial.println(analogPin5Value);  
  }//end testing2


  if (testing3 == 1){
    Serial.println("[I] STARTING TO PROCESS ANALOG VALUES IN 5 SECONDS");
    delay(5000);

  }//end testing3


}//ENDVOIDLOOP
