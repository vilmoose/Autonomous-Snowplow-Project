/**
* Group 2 L2, SYSC4805
* Vilmos Feher
* R
*/

#include <Wire.h>
#include <VL53L1X.h>
#include <time.h>


// variables used for setting up the time of flight sensors
const uint8_t sensorCount = 2;
const uint8_t xshutPins[sensorCount] = { 4, 5};
VL53L1X sensors[sensorCount];
//variables used for motor output pins
const uint32_t m1 = 45;
const uint32_t m2 = 47;
const uint32_t m3 = 49;
const uint32_t m4 = 51;

const int trigPin = 2; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 7;

//variable used for debouncing the ultrasonic sensor
double previousDistance;
volatile uint32_t CaptureCountA;

/**
* Watchdog setup necessary for enabling watchdog timer
*/
void watchDogSetup(){}

void setup() {
  watchdogEnable(10000); //at 15s right now, system resets if after this time 
  //Serial.begin(115200); //used for testing

  /**
  * Setting up the signal for the motor driver to receive
  * Channel 1 is for the left side, channel 2 is for the right side
  */
  PMC->PMC_PCER1 |= PMC_PCER1_PID36; // Enable Clock to PWM module PIN 35
  PIOC->PIO_ABSR |= PIO_PC3B_PWMH0; // Assign C3 to PWM module (Periph_B)
  PIOC->PIO_PDR |= PIO_PDR_P3; // Release C3 from the PIO module
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(84);//Set PWM clock 1MHz (Mck/84)
  PWM->PWM_CH_NUM[0].PWM_CMR |= PWM_CMR_CPRE_CLKA // Set the clock source as CLKA
                              | PWM_CMR_CPOL; //Set output polarity be high.
  PWM->PWM_CH_NUM[0].PWM_CPRD = 1000-1; //Set PWM freq = 1MHz/(FREQ) 
  PWM->PWM_CH_NUM[0].PWM_CDTY = 1-1; // Set PWM duty cycle
  PWM->PWM_ENA = PWM_ENA_CHID0; // Enable the PWM channel

  PMC->PMC_PCER1 |= PMC_PCER1_PID38; // Enable Clock to PWM module PIN 37
  PIOC->PIO_ABSR |= PIO_PC5B_PWMH1; // Assign C5 to PWM module (Periph_B)
  PIOC->PIO_PDR |= PIO_PDR_P5; // Release C5 from the PIO module
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(84);//Set PWM clock 1MHz (Mck/84)
  PWM->PWM_CH_NUM[1].PWM_CMR |= PWM_CMR_CPRE_CLKA // Set the clock source as CLKA
                              | PWM_CMR_CPOL; //Set output polarity be high.
  PWM->PWM_CH_NUM[1].PWM_CPRD = 1000-1; //Set PWM freq = 1MHz/(FREQ) 
  PWM->PWM_CH_NUM[1].PWM_CDTY = 1-1; // Set PWM duty cycle
  PWM->PWM_ENA = PWM_ENA_CHID1; // Enable the PWM channel

  
  //Setting up the line follower sensor
  pinMode(A0, INPUT); //OUT1 (LEFT SIDE) d22
  pinMode(A1, INPUT); //OUT2 (RIGHT SIDE) d24
  pinMode(A2, INPUT); //OUT3 (MIDDLE) d26

  //assigning motor pins
  pinMode(m1, OUTPUT);  //Motor A(1) = front left
  pinMode(m2, OUTPUT); //Motor B(2) = back left
  pinMode(m3, OUTPUT); //Motor C(3) = front right
  pinMode(m4, OUTPUT); //Motor D(4) = back right

 
  // Setting up the Ultrasonic sensor 

  /**
  * Setting up both VL53L1X Time of Flight sensors
    TAKEN OUT FOR NOW BECAUSE ON DEC 1 STOPPPED WORKING AND HAVE NOT SOLVED ISSUE YET
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++){
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++){
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);
    sensors[i].setTimeout(300000);
    if (!sensors[i].init()){
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
    }
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);
    sensors[i].startContinuous(50);
  }
 */
  pinMode(13, OUTPUT); //builtin led for testing
  previousDistance = 0; //setting the previous distance to a default value
}

void loop() {
  watchdogReset();
  LFResponse();
  delay(200);
  USResponse();
  delay(200);
  //TOFResponse();
  //delay(100);
 }


// _________________________________________________________ //
//             Basic Movement Functions                      //
// _________________________________________________________ //

/**
* Function drives the motors so they move forward indefinetely 
*/
 void moveForward(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 900; 
  PWM->PWM_CH_NUM[1].PWM_CDTY = 900; 
  digitalWrite(m1, LOW); //LOW = FORWARD (LEFT SIDE)
  digitalWrite(m2, LOW); 
  digitalWrite(m3, HIGH); //(LOW) = BACKWARD (RIGHT SIDE)
  digitalWrite(m4, HIGH); 
 }

/**
* Function drives the motors so they move bakward for 1.5s
*/
void moveBackward(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 900;
  PWM->PWM_CH_NUM[1].PWM_CDTY = 900; 
  digitalWrite(m1, HIGH);
  digitalWrite(m2, HIGH); 
  digitalWrite(m3, LOW); 
  digitalWrite(m4, LOW); 
  delay(750);
}

/**
* Function drives the motors so they turn right for 1.5s
*/
void turnRight(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 400; 
  PWM->PWM_CH_NUM[1].PWM_CDTY = 900; 
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW); 
  digitalWrite(m3, LOW); 
  digitalWrite(m4, LOW); 
  delay(1500); 
}

/**
* Function drives the motors so they turn left for 1.5s
*/
void turnLeft(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 900; 
  PWM->PWM_CH_NUM[1].PWM_CDTY = 400; 
  digitalWrite(m1, HIGH);
  digitalWrite(m2, HIGH); 
  digitalWrite(m3, HIGH); 
  digitalWrite(m4, HIGH);
  delay(1500); 
}

void stop(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 0; // Set PWM duty cycle to 0 to turn power off to motors
  PWM->PWM_CH_NUM[1].PWM_CDTY = 0; // Set PWM duty cycle to 0 to turn power off to motors
  delay(1000);
}

 //___________________________________________________________//
 //                     Sensor Functions                      //
 // _________________________________________________________ //

/**
 * Function to cover line follower sensor activity
*/ 
 void LFResponse(){
  int leftIn = analogRead(A0); //corresponds to out1 on LF Sensor
  int rightIn = analogRead(A1); //corresponds to out2 on LF sensor
  int middleIn = analogRead(A2); //corresponds to out3 on LF sensor
  //read values as black surface if above 850 
  moveForward();
  if (leftIn > 750 & middleIn > 750 & rightIn < 750){
    stop();
    moveBackward();
    turnRight();
    //Serial.println("black line detected on left side, turning right");
  }else if (middleIn > 750 & rightIn > 750 & leftIn < 750){
    stop();
    moveBackward();
    turnLeft();
    //Serial.println("black line detected on right side, turning left");
  }else if(leftIn > 750 & middleIn > 750 & rightIn > 750){
    stop();
    moveBackward();
    turnLeft();
    turnLeft();
    //Serial.println("black line detected in front, turning left");
  }  
  //read values as white surface if under 600, disregard between 600-850 
  /** else if used for testing
  else if(leftIn < 850 | middleIn < 850 | rightIn < 850){
    Serial.println("white line detected");
  }  */
}


/**
* Function to send a trigger signal and calculate time in between response
*/
void USResponse(){
  long duration, dist;   // Variables to make the calculation for measured distance.
  //sending trigger signal
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //calculating duration of the signal and then converting it to cm
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  dist = microsecondsToCentimeters(duration);

  //if an object is 40 cm close, avoid the object detected
  if(dist < 40.00 & (abs(previousDistance - dist)  <  20)){
    //Serial.println("Obstacle detected, turning right to avoid object");
    stop();
    moveBackward();
    turnLeft();
  }
  previousDistance = dist; //set previous distance to latest distance 
}

/**
* Function to convert the trigger pulse into cm
*/
long microsecondsToCentimeters(long microseconds)
{
return microseconds / 29 / 2;
}

/*------------------------------------------------------------
SENSOR NOT WORKING - TAKEN OUT 
------------------------------------------------------------
void TOFResponse(){
  //testing
  int rightSensor = sensors[1].read();
  int leftSensor = sensors[0].read();
  float left1 = (float)leftSensor;
  float right1 = (float)rightSensor;
  Serial.print("Left side: (cm) ");
  Serial.println(left1);
  Serial.print("right side: (cm) ");
  Serial.println(right1);
  //delay(2000);
  //moveForward();
--------------------------------------------------------------------
(TESTING)
  Serial.println( sensors[0].read()); //left side
  if (sensors[0].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println("right side(mm): ");
  Serial.println(sensors[1].read()); //right side
  if (sensors[1].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print('\t');
  Serial.println(); 
--------------------------------------------------------------------
  if ( right1 < 200.0 & left1 > 200.0){
    Serial.println("I'm turning LEFT");
    turnLeft();
    moveForward();
  }else if(left1 < 200.0 & right1 > 200.0){
    Serial.println("I'm turning RIGHT");
    turnRight();
    moveForward();
  }else if(left1 < 200.0 & right1 < 200.0){
    turnRight();
    moveForward();
    Serial.println("both sense soeting");
  }
}
 **/