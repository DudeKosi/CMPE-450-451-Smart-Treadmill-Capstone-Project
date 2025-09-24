const byte startPin = 2;
const byte PWM = 8;
volatile byte start = LOW;

void setup() {
  pinMode(startPin, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(startPin), controlTreadmill, CHANGE);
}

void controlTreadmill(){
  start = !start;
}

void loop() {

  digitalWrite(PWM, start);

}