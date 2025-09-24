// 20 Hz PWM frequency using ATmega Timer1

const int TRIG_PIN = 2; //digital pin on the right side
const int ECHO_PIN = 3; //digital pin on the right side
const int PWM_PIN = 11; //digital pin for the pwm signal
const int DATA_POINTS = 100;

//The lenght of the treadmill in cm its 45.5 inch == 115.57cm
const int TREADMILL_LEN = 116;

// The acceleration and deceleration zone of the treadmill  
const float ACEL_ZONE = 0.2 * TREADMILL_LEN;
const float DECEL_ZONE = 0.8 * TREADMILL_LEN;

// Value used by OCR1A to control the PWM of the treadmill motor
// Each index of the array is the differece of 0.5mph
const int TREADMILL_SPEEP_PWM[20] = {588, 696, 804, 912, 1021,  1129,  1237,  1345,  1454,  1562,  1670,  1778,  1887,  1995,  2103,  2212,  2320,  2428,  2536,  2645};
const float TREADMILL_SPEED_MPH[20] = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0};
const float ZONE_DISTANCE[2] = {DECEL_ZONE, ACEL_ZONE};

float duration, distance;
float distance_data[DATA_POINTS] = {0.0};

int pwm_index = 0; // Index for the treadmill speed pwm array

void setup( void ){
  // all the set up need for the PWM
  //TIMER1 setup
  TCCR1A=TCCR1B=0;
  
  //set waveform generator mode WGM1 3..0: 1110 (mode 14; FAST PWM) clear OC1A on compare match
  //set prescaler CS12..0: 100 (clkio/256)
  TCCR1A = (1 << COM1A1) | (1 << WGM11); 
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
 
  //set top and compare register
  ICR1 = 3124; //TOP for 20 Hz, 256 prescaler
  
  //default PWM compare
  // Sets the PWM of treadmill  at lowest speed 
  OCR1A = TREADMILL_SPEEP_PWM[0]; 

  //on Mega2560, OC1A function is tied to pin 11 (9 on ATmega328)
  pinMode( PWM_PIN, OUTPUT );
 
  // set up needed for the ultra sonic sensor 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.begin(9600);

}//setup


void loop( void ) {
  getRunnerDistance();
  Serial.println(TREADMILL_SPEED_MPH[pwm_index]);
  setTreadmillSpeed();
    
  // wait for 0.4 second to complete the 0.5 sample rate requirement
  delay(400);
  }
  
void populateArray(float dataArray[], float dataPoint) {
  /*
  This function populates the data array   
  */
    // check if index value is zero and adds new value for initial populating values
    for (int i = 0; i < DATA_POINTS; ++i) {
        if (dataArray[i] == 0.0) {
            // vacant index found and new value added
            dataArray[i] = dataPoint;
            //printf("Distance data point %f added at index %f\n", dataPoint, i); //for debugging purposes
            Serial.print("Distance populating: ");
            Serial.println(distance);
        }
    }

    // array is fully populated
    //perform a left shift to get index zero value out
    for (int i = 0; i < DATA_POINTS - 1; ++i) {
        dataArray[i] = dataArray[i + 1];
    }

    // Add the new value to the end
    //new values at the back of the queue need to be shifted in
    dataArray[DATA_POINTS - 1] = dataPoint;
    //printf("Distace point %f added to the end of the array\n", dataPoint); //for debugging purposes
    Serial.print("Distance at end of array: ");
    Serial.println(distance);
    return;

}
  void getRunnerDistance(){
    // check how close the runner is to the ultra sonic senors   
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration*.0343)/2;

    //This display the distance that was gather by the 
    //populateArray(distance_data, distance);
    //Serial.println(TREADMILL_SPEED_MPH[pwm_index]);
    delay(100); // maniplate this to affect the sample rate
  }

  void setTreadmillSpeed(){
     //check if the treadmill is at max speed and if the runner is in the acceleration zone
    if ((pwm_index != 19) && (distance <= ZONE_DISTANCE[1])){
      pwm_index += 1; 
      OCR1A = TREADMILL_SPEEP_PWM[pwm_index];
    }

    // check if the treadmill is at the lowest speed and if the runner is in the decelration zone 
    else if ((pwm_index != 0) && (distance >= ZONE_DISTANCE[0])){
      pwm_index -= 1;
      OCR1A = TREADMILL_SPEEP_PWM[pwm_index];
    }
  }
