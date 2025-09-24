int NUMGRADE = 10;

void setup() {
  // put your setup code here, to run once:
    pinMode(7, OUTPUT);
    pinMode(6, OUTPUT);
    Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //raise the grade
  for (int i = 0; i < NUMGRADE; i++){
    digitalWrite(7, HIGH);
    delay(1500);
    digitalWrite(7, LOW);
    Serial.println("UPPPITY");
  }

  //lower the grade
  for (int j = 0; j < NUMGRADE; j++){
    digitalWrite(6, HIGH);
    delay(1500);
    digitalWrite(6, LOW);
    Serial.println("DOWNITY");
  }
}

/*void setup() {
  // put your setup code here, to run once:
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  delay(1500);
  digitalWrite(7, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}*/
