#define LEDA 4
#define LEDK 5
#define touchR 2
#define touchL 3

void setup() {
  pinMode(LEDK,OUTPUT);
  pinMode(LEDA,OUTPUT);
  pinMode(touchR,INPUT_PULLDOWN);
  pinMode(touchL,INPUT_PULLDOWN);
  Serial.begin(9600);
  Serial.println("HEY");
}

void LED_ON(){
  digitalWrite(LEDA,HIGH);
  digitalWrite(LEDK,LOW);
}

void LED_OFF(){
  digitalWrite(LEDA,LOW);
  digitalWrite(LEDK,HIGH);
}

void CheckTouch(){
  digitalRead(touchR);
  Serial.print(digitalRead(touchR));
  digitalRead(touchL);
  Serial.print(" ");
  Serial.println(digitalRead(touchL));  
}

void loop() {
  LED_OFF();
  delay(100);
  LED_ON();
  delay(100);
  CheckTouch();
}
