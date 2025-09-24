//#include <PID_v1.h>
// 20 Hz PWM frequency using ATmega Timer1
//The digital pin for Trig and Echo for the ultrasonic on the right side
//purple and blue wire are triggers
//white wire are echo

const int TRIG_PIN_0 = 2; 
const int ECHO_PIN_0 = 3; 
const int TRIG_PIN_1 = 4; 
const int ECHO_PIN_1 = 5; 
const int TRIG_PIN_2 = 6; 
const int ECHO_PIN_2 = 7; 
const int TRIG_PIN_3 = 8; 
const int ECHO_PIN_3 = 9; 
const int TRIG_PIN_4 = 12; 
const int ECHO_PIN_4 = 13; 

const int PWM_PIN = 11;  //treadmill Speed Output pin using PWM signal
const int hall_pin = 30; //Hall Effect Sensor Pin

const byte EMERGENCY_PIN = 19; //Emergency stop signal from relay
const byte START_PIN = 20; //Start pin from GUI
const byte STOP_PIN = 21; //Stop pin from GUI


/* Treadmill sizing
 *    Given the lenght thr treadmill it will determine:
 *        0 accelration zone: the front side of the treadmill(ACEL_ZONE)
 *        0 deceleration zone: the back side of the treadmill (DECEL_ZONE)
 *        0 activite runing zone: middle part where the speed doesn't change 
 */
const int TREADMILL_LEN = 138;                //lenght of the treadmill (54.5in ~= 128cm)  
const float ACEL_ZONE = 0.4 * TREADMILL_LEN;  //Aceleration zone 
const float DECEL_ZONE = 0.6 * TREADMILL_LEN; //Decelration zone




// Value used by OCR1A to control the PWM of the treadmill motor
// Each index of the array is the differece of 0.5mph
const int TREADMILL_SPEEP_PWM[20] = {588, 696, 804, 912, 1021,  1129,  1237,  1345,  1454,  1562,  1670,  1778,  1887,  1995,  2103,  2212,  2320,  2428,  2536,  2645};
const float TREADMILL_SPEED_MPH[20] = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0};

const float TREADMILL_SETPOINT_RPM[20] = {87.54, 175.08, 262.62, 350.16, 437.7, 525.24, 612.72, 700.26, 787.8, 875.34, 962.88, 
                                      1050.42, 1137.96, 1225.5, 1313.04, 1400.58, 1488.12, 1575.66, 1663.14, 1750.68};

const float TREADMILL_SETPOINT_PWM[20] = {348, 466, 583, 701, 818, 936, 1053, 1171, 1288, 1406, 1523, 1641, 1758, 1876, 1993, 2111, 2228, 2346, 2463, 2581};
                                       
const float TREADMILL_SPEED_RPM[] = {87.54, 105.06, 122.52, 140.04, 157.56, 
                                      175.08, 192.6, 210.06, 227.58, 245.1, 262.62, 280.14, 297.6,
                                315.12, 332.64, 350.16, 367.62, 385.14, 402.66, 420.18, 437.7, 455.16, 472.68, 490.2, 507.72, 525.24,
                                542.7, 560.22, 577.74, 595.26, 612.72, 630.24, 647.76, 665.28, 682.8, 700.26, 717.78, 735.3, 752.82,
                                770.28, 787.8, 805.32, 822.84, 840.36, 857.82, 875.34, 892.86, 910.38, 927.9, 945.36, 962.88, 980.4,
                                997.92, 1015.38, 1032.9, 1050.42, 1067.94, 1085.46, 1102.92, 1120.44, 1137.96, 1155.48, 1173.0, 1190.46,
                                1207.98, 1225.5, 1243.02, 1260.48, 1278.0, 1295.52, 1313.04, 1330.56, 1348.02, 1365.54, 1383.06, 1400.58,
                                1418.1, 1435.56, 1453.08, 1470.6, 1488.12, 1505.58, 1523.1, 1540.62, 1558.14, 1575.66, 1593.12, 
                                1610.64, 1628.16, 1645.68, 1663.14, 1680.66, 1698.18, 1715.7, 1733.22, 1750.68, 1768.2, 1785.72, 1803.24, 1820.76, 1838.28};


const int TREADMILL_SPEED_PWM[] = {348, 372, 395, 419, 442, 
                                    466, 489, 513, 536, 560, 583, 607, 630, 654, 677, 
                                    701, 724, 748, 771, 795, 818, 842, 865, 889, 912, 
                                    936, 959, 983, 1006, 1030, 1053, 1077, 1100, 1124, 1147,
                                    1171, 1194, 1218, 1241, 1265, 1288, 1312, 1335, 1359, 1382, 
                                    1406, 1429, 1453, 1476, 1450, 1523, 1547, 1570, 1594, 1617,
                                    1641, 1664, 1688, 1711, 1735, 1758, 1782, 1805, 1829, 1852, 
                                    1876, 1899, 1923, 1946, 1970, 1993, 2017, 2040, 2064, 2087,
                                    2111, 2134, 2158, 2181, 2205, 2228, 2252, 2275, 2299, 2322, 
                                    2346, 2369, 2393, 2416, 2440, 2463, 2487, 2510, 2534, 2557, 
                                    2581, 2604, 2628, 2651, 2675, 2698};


//const float RPM_SETPOINT[20] = {+
const float ZONE_DISTANCE[2] = {DECEL_ZONE, ACEL_ZONE};



const int numUltSensor = 5; //the number of sensor used

/* 
 *  The sensors are weighted 
 *    Center sensor weight : 4
 *    2nd closest sensor weight: 2
 *    Out most sensor weight : 1
 *  
 *  The total weight of the sensor is 10:
 *    1 + 2 + 4 + 2 + 1 = 10 
 */
const int distanceWeight = 10; // totalWeight = 10 = 4 + 2 + 2 + 1 + 1



float duration, distance, total_distance; // array of duration and distance measured by the each ultra sonic sensor 
//float distance_data[DATA_POINTS] = {0.0};

int pwm_index = 0; // Index for the treadmill speed pwm array
int pwm_init = 348; // First value that represent 0.5 MPH


float hall_thresh = 1.0;
unsigned long MAX_SAMPLE_TIME = 500000; // the time (mircoseconds) the program has to sample the real RPM of the treadmill


volatile byte Start_sig = HIGH;
volatile byte Stop_sig = HIGH;
bool isRun = false;



//Variable used for the PID control https://www.youtube.com/watch?v=crw0Hcc67RY
double Setpoint = (double)TREADMILL_SETPOINT_PWM[0]; 
double Input = 0;
double Output;

//PID parameters
/*Ziegler-Nochols (no overshoot)
 * 
 * Ku
 * 
 * 
 * kp = 0.20Ku
 * ki = 0.40*Ku/Tu
 * kd = 0.066*Ku*Tu
 */
double Kp = 0.2; 
double Ki = 0.4/0.3;
double Kd = 0.000;

//create PID instance
//PID ST_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, AUTOMATIC);

int count_start = 0;
int count_Stop = 0;

void setup( void ){
  // all the set up need for the PWM
  //TIMER1 setup
  //TCCR1A=TCCR1B=0; 
  
  //set waveform generator mode WGM1 3..0: 1110 (mode 14; FAST PWM) clear OC1A on compare match
  //set prescaler CS12..0: 100 (clkio/256)
  //TCCR1A = (1 << COM1A1) | (1 << WGM11); 
  //TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
 
  //set top and compare register
 // ICR1 = 3124; //TOP for 20 Hz, 256 prescaler
  
  //default PWM compare
  // Sets the PWM of treadmill  at lowest speed 
  //OCR1A = 0;//TREADMILL_SPEED_PWM[0]; 

  //set up the PID
  //Setpoint = 100;
  //ST_PID.SetMode(AUTOMATIC);
  //ST_PID.SetTunings(Kp, Ki, Kd);
  //ST_PID.SetOutputLimits((double)0.0, (double)(TREADMILL_SPEEP_PWM[19]));  

  //on Mega2560, OC1A function is tied to pin 11 (9 on ATmega328)
  pinMode( PWM_PIN, OUTPUT );
 
  // set up needed for the ultra sonic sensor 
  pinMode(TRIG_PIN_0, OUTPUT);
  pinMode(ECHO_PIN_0, INPUT);

  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);

  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);

  pinMode(TRIG_PIN_4, OUTPUT);
  pinMode(ECHO_PIN_4, INPUT);

  pinMode(hall_pin, INPUT);

  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);

  //delay(10000);
  
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), GUI_Shutoff , HIGH);
  attachInterrupt(digitalPinToInterrupt(START_PIN), GUI_TurnOn , HIGH);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_PIN), Emergency_Shutoff, HIGH);
  Serial.begin(9600);

}//setup

void Emergency_Shutoff(){
  Setpoint = 0;
  Input = 0;
  Output = 0;
  isRun = false;
  TCCR1A=TCCR1B=0; 
  TCCR1A = (0 << COM1A1) & (0 << WGM11); 
  TCCR1B = (0 << WGM13) & (0 << WGM12) & (0 << CS12);
  //turn off timer to stop PWM signal
  //(1 << WGM13) | (1 << WGM12) | (1 << CS12);
  OCR1A = 0;
  //digitalWrite(OCR1A, LOW);
  //Serial.println("The Stop was pressed");
}

void GUI_Shutoff(){
  //disable everything to stop the treadmill
  //Serial.println("TOP");
  if (count_Stop == 0){
    count_Stop++;  
  }

  else{
      Setpoint = 0;
      Input = 0;
      Output = 0;
      isRun = false;
      TCCR1A=TCCR1B=0; 
      TCCR1A = (0 << COM1A1) & (0 << WGM11); 
      TCCR1B = (0 << WGM13) & (0 << WGM12) & (0 << CS12);
      //turn off timer to stop PWM signal
     //(1 << WGM13) | (1 << WGM12) | (1 << CS12);
      OCR1A = 0;
      //digitalWrite(OCR1A, LOW);
      //Serial.println("The Stop was pressed");
  }
}

void GUI_TurnOn(){
  /*//disable everything to stop the treadmill
  delay(500);
  Setpoint = TREADMILL_SPEED_RPM[0];*/
  if (count_start == 0){
    count_start++;
  }

  else{
    isRun = true;
    //Serial.println("The Start was pressed");
  
    TCCR1A=TCCR1B=0; 
    TCCR1A = (1 << COM1A1) | (1 << WGM11); 
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
  
  
    ICR1 = 3124;
    //turn off timer to stop PWM signal
     /*TCCR1A = (1 << COM1A1) | (1 << WGM11); 
     TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
     ICR1 = 3124; //TOP for 20 Hz, 256 prescaler
     OCR1A = TREADMILL_SPEED_PWM[0];*/
    //OCR1A = TREADMILL_SPEED_PWM[10];
  }
  //delay(1000);
  
}

/*Note:
 * The tach signal will go low when the magnet interact with the reed sensor accound for the frequency of the low pulse;
 * Must be able to sample the treadmill every 30 ms
 */

 int count = 0;
void loop( void ) {
  if (!isRun){
    Serial.println(0);
  }
  while(isRun){
    Serial.println(TREADMILL_SPEED_MPH[pwm_index]);
    OCR1A = TREADMILL_SPEED_PWM[pwm_index];
    pwm_index += 1;
    delay(1500);
    //Get runner zone
    //getRunnerDistance();

    //Change the setpoint
    //setTreadmillSpeed();

    //read the real speed of the treadmill
    //AvgHallEffect();
  
    //Adjust the real speed to Setpoint
    //setPID();
  }
  delay(500);
}
 
void monitorPrint(int sensorNum){
    Serial.print("Sensor ");
    Serial.print(sensorNum);
    Serial.print(" measures:");
    Serial.println(distance);
}

void getRunnerDistance(){

    //clear the distance and duration for use
    duration = 0;
    total_distance = 0;
    
    // check how close the runner is to the ultra sonic senors   

    // FOr sensor in the middle
    digitalWrite(TRIG_PIN_0, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_0, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_0, LOW);

    duration = pulseIn(ECHO_PIN_0, HIGH);
    distance = (duration*.0343)/2;
    total_distance += distance* 4;

    //monitorPrint(0);
  
    //sensor closes to center
    digitalWrite(TRIG_PIN_1, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_1, LOW);

    duration = pulseIn(ECHO_PIN_1, HIGH);
    distance = (duration*.0343)/2;
    total_distance += distance * 2;

    //monitorPrint(1);

    //sensor closet to center
    digitalWrite(TRIG_PIN_2, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_2, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_2, LOW);

    duration = pulseIn(ECHO_PIN_2, HIGH);
    distance = (duration*.0343)/2;
    total_distance += distance * 2;

   // monitorPrint(2);

    //sensor furthest from center
    digitalWrite(TRIG_PIN_3, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_3, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_3, LOW);

    duration = pulseIn(ECHO_PIN_3, HIGH);
    distance = (duration*.0343)/2;
    total_distance += distance;

    //monitorPrint(3);

    //sensor furthest from center
    digitalWrite(TRIG_PIN_4, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_4, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_4, LOW);


    duration = pulseIn(ECHO_PIN_4, HIGH);
    distance = (duration*.0343)/2;
    total_distance += distance;

    //monitorPrint(4);
    
    delay(10); // maniplate this to affect the sample rate

    total_distance = total_distance / distanceWeight;

    /*delay(50);
    Serial.print("Average distance measures:");
    Serial.println(total_distance);
    Serial.println("\n");*/
}

void setTreadmillSpeed(){
    // Serial.println(total_distance);
     //check if the treadmill is at max speed and if the runner is in the acceleration zone
    if ((pwm_index != 19) && (total_distance <= ZONE_DISTANCE[1])){
      pwm_index += 1; 
      //Setpoint
      Setpoint = TREADMILL_SETPOINT_PWM[pwm_index];
    }

    // check if the treadmill is at the lowest speed and if the runner is in the decelration zone 
    else if ((pwm_index != 0) && (total_distance >= ZONE_DISTANCE[0])){
      pwm_index -= 1;
      //Setpoint 
      Setpoint = TREADMILL_SETPOINT_PWM[pwm_index];
    }

    //Serial.print("The setpoint is: " );
   //Serial.println(TREADMILL_SPEED_MPH[pwm_index]);
}
float sampleHallEffect(){
      float hallCount = -1;            // start at -1 start the timer it completed a revolution first
      unsigned long start = 0;      // the start time for counting the revolution
      unsigned long endTime = 0;   // end timer for counting the revolution
      unsigned long timePassed = 0;  // the time laspse from the begin to end of the time for the revolution 
      bool firstIter = false;           // see if this is the first iteration of shaft
      bool onState = false;          // true if the magnetic is still on magnetic 
      float rpsCalc = 0.0;           // the ratation per ssecond (rps) calculatedfrom rhe elapsed time
      unsigned long currTime = 0;
      //counting number of times the hall sensor is passed by the magnet demonstrating a complete revolution
      while (true){
        if (digitalRead(hall_pin)== 0){  
          if (onState==false){
            
            //Start timer as soon as it complete a revolution
            if (firstIter == false){
              start = micros();
              firstIter = true;
            }
            
            onState = true;
            hallCount+=1.0;
            currTime = micros();
            
           /* //Stop   
            if ((currTime - start) >= 500000){
                currTime -= start;
                Serial.print("It took :");
                Serial.print(currTime);
                Serial.print(" mircoseconds and ");
                Serial.print(hallCount);
                Serial.println(" Counts");
              break;
              
            }*/
            
          }
        }
        
        else{
          onState = false;
        }
        if (hallCount>=hall_thresh){
          break;
        }
      }
      endTime = micros();


      //Account if the the timer reset
      if (endTime < start){
        
      }
    

      //calculate the time elpase and the rotation per second(rps)
      timePassed = (endTime-start);
      rpsCalc = (float)(1000000.0/(timePassed))* hallCount;

      /*Serial.print("time elpased: ");
      Serial.print(timePassed);
      Serial.println(" mirco Seconds");
      
      Serial.print("Hall Effect Reading is: ");
      Serial.print(rpsCalc);
      Serial.println(" RPS");  */    
      return (rpsCalc * 60);
}


void AvgHallEffect(){
  float numSample = 10;          //Number of time the RMP is going to be sampled
  float rpmSample[20]; //Array of sampled RMP 
  float rpmCalc = 0;          //The calculated RPM              

  int numCount = 0;
  unsigned long start = micros();
  float elapsedTime = 0;
  unsigned long endTime = 0;
  
  const float MAX_RPM = 1850 ;//the max rpm that would be consider for averaging 

  
  // Time restricted 
  while(elapsedTime < MAX_SAMPLE_TIME){
    rpmSample[numCount] = sampleHallEffect();

    //check how long it took to sample
    endTime = micros();
    elapsedTime = (endTime - start);

    //Sum all the sample except the first since it garabge 
    if (numCount != 0 && rpmSample[numCount] < MAX_RPM){
      rpmCalc += rpmSample[numCount];
      numCount++;
      
    }

    if (numCount == 0){
      numCount++;
    }

        
  }
  
  
  /*
  //sample the motor numSample number of time and sum all the Samples
  for (int i = 0; i < numSample + 1; i++){
    rpmSample[i] = sampleHallEffect();
    
    
    if (i != 0 && rpmSample[i] < MAX_RPM){
      rpmCalc += rpmSample[i];
      numCount++;
    }
  }
  
  */
  
   //Find the average
   endTime = micros();
   elapsedTime = (endTime - start) / 1000000.0;
  


  
   rpmCalc = rpmCalc / (numCount - 1);
 /*
   Serial.print("Num of samples: ");
   Serial.println(numSample);
   */
   
   //Serial.print("THIS IS RPM: ");
   //Serial.println(rpmCalc);

   //set the PID INPUT as the current PRM of the treadmill
   Input = rpmCalc;
   
   //Serial.print("Elapsed Time: ");
   //Serial.print(elapsedTime);
   //Serial.println(" s");
   
   delay(10);
}


void setPID(){
  int PwmRpmIndex = 0;

  //while (Output < (Setpoint - 9) || Output > (Setpoint + 9) ){
    //print input RPM
    //Serial.print("The curent PWM Signal is: ");
    //Serial.println(Input);

    // Does PID calculation (RPM)
    //ST_PID.Compute();

    //Round the RPM to the nearest 0.1 increment
    PwmRpmIndex = FindIndex();

    //adjust the rounded RPM value to the input
    Output = TREADMILL_SPEED_RPM[PwmRpmIndex];
  
    Input = Output; //update input
 
    //change the speed of the treadmill using PWM signal
    OCR1A = TREADMILL_SPEED_PWM[PwmRpmIndex];
  
  //}

}

int FindIndex(){
  int currIndex = 0;
  int pidValue = Output;
  
  for(int currIndex=0;currIndex<96; currIndex++){
    int diff = TREADMILL_SPEED_RPM[currIndex] - pidValue;
    if(diff<0){diff=diff*(-1);} //absolute value
    if(diff<=9){ //if in range
      return currIndex;
    }
  }
}