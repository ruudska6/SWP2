// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 200.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() { 
  float distance;

  // wait until next sampling time. // polling
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance
  
  int brightness;

  // 거리 기반으로 LED 밝기 조절
  if (distance <= 100.0 || distance >= 300.0) {
    // 100mm 이하 및 300mm 이상에서 LED 꺼짐
    brightness = 255;  // LED 최소 (꺼짐)
  } else if (distance == 200.0) {
    // 200mm에서 LED 최대 밝기
    brightness = 0; // LED 가장 밝게
  } else if ((distance == 150.0 || distance == 250.0)) {
    // 150mm 및 250mm에서 LED 50% 밝기
    brightness = 128; // 50% 밝기 반올림 했음.
  } else {
    // 그 외 거리에서는 선형적으로 밝기 조절
    if (distance > 100.0 && distance < 200.0) {
      brightness = map(distance, 100, 200, 255, 0);
    } else if (distance > 200.0 && distance < 300.0) {
      brightness = map(distance, 200, 300, 0, 255);
    }
  }

  analogWrite(PIN_LED, brightness);

  // 출력 디버깅을 위해 Serial 모니터에 정보 출력
  Serial.print("Distance: ");  
  Serial.print(distance);
  Serial.print(" mm, LED Brightness: ");
  Serial.println(brightness);

  // update last sampling time
  last_sampling_time += INTERVAL;
}


// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm

  // Pulse duration to distance conversion example (target distance = 17.3m)
  // - pulseIn(ECHO, HIGH, timeout) returns microseconds (음파의 왕복 시간)
  // - 편도 거리 = (pulseIn() / 1,000,000) * SND_VEL / 2 (미터 단위)
  //   mm 단위로 하려면 * 1,000이 필요 ==>  SCALE = 0.001 * 0.5 * SND_VEL
  //
  // - 예, pusseIn()이 100,000 이면 (= 0.1초, 왕복 거리 34.6m)
  //        = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
  //        = 100,000 * 0.001 * 0.5 * 346
  //        = 17,300 mm  ==> 17.3m
}
