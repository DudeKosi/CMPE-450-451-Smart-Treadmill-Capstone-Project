/*
 * HC-SR04 example sketch
 *
 * https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380
 *
 * by Isaac100
 */


/*
 * pin 9 (A0) is echo (input)
 * pin 10 (A1) is trig (output)
 * 
 */
const int trigPin = 2; //digital pin on the right side
const int echoPin = 3; //digital pin on the right side
const int DATA_POINTS = 100;

float duration, distance;
float distance_data[DATA_POINTS] = {0.0};

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

//need to make a shift data function for aray
//need to make an array to hold the data
//think about how to incorporate this into pid loop

void loop() {
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  populateArray(distance_data, distance);
  //Serial.print("Distance: ");
  //Serial.println(distance);
  delay(100); // maniplate this to affect the sample rate
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
