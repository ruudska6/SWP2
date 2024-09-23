#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
  
}

void loop() {
  int i = 0;
  digitalWrite(PIN_LED, 0);
  delay(1000);


  for(i=0;i<=4;i++) {
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);
  }

  digitalWrite(PIN_LED, 1);

  while(true) {
    
  }
}
