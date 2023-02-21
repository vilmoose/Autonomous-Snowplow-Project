/**
* Group 2 L2, SYSC4805
* Vilmos Feher
* R
*/

/**
* A struct to keep track of which Sensors are on/off
* An array of 9 sensors is created to keep track of system's sensors
*/ 
struct Sensor{
  int sensorID;
  bool state; //true = sensor on 
} listOfAllSensors[10];

void watchDogSetup(){}

void setup() {
  watchdogEnable(1000); //at 1s right now, will be changed later
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
  PWM->PWM_CH_NUM[0].PWM_CDTY = 500-1; // Set PWM duty cycle
  PWM->PWM_ENA = PWM_ENA_CHID0; // Enable the PWM channel
 
  PMC->PMC_PCER1 |= PMC_PCER1_PID38; // Enable Clock to PWM module PIN 37
  PIOC->PIO_ABSR |= PIO_PC5B_PWMH1; // Assign C5 to PWM module (Periph_B)
  PIOC->PIO_PDR |= PIO_PDR_P5; // Release C5 from the PIO module
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(84);//Set PWM clock 1MHz (Mck/84)
  PWM->PWM_CH_NUM[1].PWM_CMR |= PWM_CMR_CPRE_CLKA // Set the clock source as CLKA
                              | PWM_CMR_CPOL; //Set output polarity be high.
  PWM->PWM_CH_NUM[1].PWM_CPRD = 1000-1; //Set PWM freq = 1MHz/(FREQ) 
  PWM->PWM_CH_NUM[1].PWM_CDTY = 500-1; // Set PWM duty cycle
  PWM->PWM_ENA = PWM_ENA_CHID1; // Enable the PWM channel

  /**
  * Setting up the line follower sensor
  */
  pinMode(22, INPUT); //OUT1 (LEFT SIDE)
  pinMode(24, INPUT); //OUT2 (RIGHT SIDE)
  pinMode(26, INPUT); //OUT3 (MIDDLE)
  listOfAllSensors[0].sensorID = 1;
  listOfAllSensors[0].state = true; 

  /**
  * Setting up the Ultrasonic sensor
  */

  /**
  * Assigning motor direction to pins
  */ 
  pinMode(45, OUTPUT);  //Motor A(1) = front left
  pinMode(47, OUTPUT); //Motor B(2) = back left
  pinMode(49, OUTPUT); //Motor C(3) = front right
  pinMode(51, OUTPUT); //Motor D(4) = back right

  /**
  * Setting up the Ultrasonic sensor
  */
  listOfAllSensors[1].sensorID = 2;
  listOfAllSensors[1].state = false;
  //Setting the registers for the trigger signal
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;//TC0 power ON - Timer Counter 0 channel 0
  PIOB->PIO_PDR |= PIO_PDR_P25; // The pin is no more driven by GPIO
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0;// B Assign B25 to alternative periph_B (TIOA0):
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 //MCK/2 = 42 MHz,
  | TC_CMR_WAVE //Waveform mode
  | TC_CMR_WAVSEL_UP_RC //Count UP mode till RC
  | TC_CMR_ACPA_CLEAR //Clear TIOA0 on RA compare match
  | TC_CMR_ACPC_SET; // Set TIOA0 on RC compare match
  TC0->TC_CHANNEL[0].TC_RC = 2520000-1; //Set the frequency to 66.667Hz (Period 60 ms)
  TC0->TC_CHANNEL[0].TC_RA = 420-1; //Set the duty cycle (Pulse of 10 usec)
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;
  
  //Setting the registers for the echo signal 
  PMC->PMC_PCER0 |= PMC_PCER0_PID28; // Timer Counter 0 channel 1 IS TC1, TC1 power ON
  TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 // capture mode, MCK/2 = 42 MHz
  | TC_CMR_ABETRG // TIOA is used as the external trigger
  | TC_CMR_LDRA_FALLING// load RA on falling edge of TIOA
  | TC_CMR_ETRGEDG_RISING; // Trigger on rising edge
  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;


  pinMode(13, OUTPUT); //builtin led for testing
}

void loop() {
  WatchdogRest();
  analyzeData();
 }

/**
* Function checks which sensors are on and executes a response if the sensor is on
* Additional sensors will be added into this function and res
*/
void analyzeData(){
  
  if(listOfAllSensors[0].state == true){
    LFResponse();
  }
  if(listOfAllSensors[1].state == true){
    USResponse();
  }
}

// _________________________________________________________ //
//             Basic Movement Functions                      //
// _________________________________________________________ //

 void moveForward(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 500-1; // Set PWM duty cycle 50% (standard for now)
  PWM->PWM_CH_NUM[1].PWM_CDTY = 500-1; // Set PWM duty cycle 50% (standard for now)
  digitalWrite(45, LOW); 
  digitalWrite(47, LOW); 
  digitalWrite(49, HIGH); 
  digitalWrite(51, HIGH); 
 }

void moveBackward(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 400-1; // Set PWM duty cycle 40% to go slower backwards
  PWM->PWM_CH_NUM[1].PWM_CDTY = 400-1; // Set PWM duty cycle 50% (standard for now)
  digitalWrite(45, HIGH);
  digitalWrite(47, HIGH); 
  digitalWrite(49, LOW); 
  digitalWrite(51, LOW); 
 }
 
void turnRight(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 200-1; 
  PWM->PWM_CH_NUM[1].PWM_CDTY = 700-1; 
  digitalWrite(45, HIGH);
  digitalWrite(47, HIGH); 
  digitalWrite(49, LOW); 
  digitalWrite(51, LOW); 
}

void turnLeft(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 700-1; 
  PWM->PWM_CH_NUM[1].PWM_CDTY = 200-1; 
  digitalWrite(45, HIGH);
  digitalWrite(47, HIGH); 
  digitalWrite(49, LOW); 
  digitalWrite(51, LOW); 
}

void stop(){
  PWM->PWM_CH_NUM[0].PWM_CDTY = 1-1; // Set PWM duty cycle to 0 to turn power off to motors
  PWM->PWM_CH_NUM[1].PWM_CDTY = 1-1; // Set PWM duty cycle to 0 to turn power off to motors
}

 //___________________________________________________________//
 //                     Sensor Functions                      //
 // _________________________________________________________ //

/**
 * Function to cover line follower sensor activity
*/ 
 void LFResponse(){
  int leftIn = digitalRead(22); //corresponds to out1 on LF Sensor
  int rightIn = digitalRead(24); //corresponds to out2 on LF sensor
  int middleIn = digitalRead(26); //corresponds to out3 on LF sensor

  if((leftIn == LOW) || (rightIn == LOW) || (middleIn == LOW)){  //if any sensors detects a black surface stop the wheels
    stop();
    digitalWrite(13, LOW);
    /**
    if(leftIn == HIGH){
      moveBackward();
      delay(1500);
      turnRight();
      delay(2000);     
      }
    if(rightIn == HIGH){
      moveBackward();
      delay(1500);
      turnLeft();
      delay(2000);
      }
      */
  }else{
    moveForward();
    digitalWrite(13, HIGH);
  }

 }

void USResponse(){
  volatile uint32_t CaptureCountA;
  CaptureCountA = TC0->TC_CHANNEL[1].TC_RA;
  float dist = 340.0*CaptureCountA/(42000000.0)/2*100;// This is the distance between an obstale

  if (dist <= 30.0){
    stop();
  }
  
}
