#include <Servo.h>

// Arduino pin assignment
#define PIN_IR    A0         // IR sensor at Pin A0
#define PIN_LED   9          // LED pin
#define PIN_SERVO 10         // Servo pin

// Servo duty cycle boundaries (microseconds)
#define _DUTY_MIN 1000       // Servo position at 0 degrees
#define _DUTY_NEU 1500       // Servo neutral position (horizontal)
#define _DUTY_MAX 2000       // Servo position at 180 degrees

// Distance boundaries for control (in mm)
#define _DIST_CLOSE 100.0    // Close distance threshold (100mm or 10cm)
#define _DIST_FAR   250.0    // Far distance threshold (250mm or 25cm)

// Configurable parameters
#define EMA_ALPHA  0.7       // EMA weight for new sample (range: 0 to 1)
#define LOOP_INTERVAL 20     // Loop interval in milliseconds

Servo myservo;
unsigned long last_loop_time = 0;  // Last loop timestamp initialized to 0

float dist_prev = _DIST_CLOSE;
float dist_ema = _DIST_CLOSE;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);  // Set servo to neutral position (horizontal)
  
  Serial.begin(1000000);    // Set Serial to maximum speed
}

void loop()
{
  unsigned long time_curr = millis();
  int duty;
  float a_value, dist_raw;

  // Wait until the next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  // Read IR sensor and calculate distance in mm
  a_value = analogRead(PIN_IR);
  dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0;

  // Apply range filter (valid distances only)
  if (dist_raw >= _DIST_CLOSE && dist_raw <= _DIST_FAR) {
    digitalWrite(PIN_LED, HIGH); // Turn on LED if within valid range
  } else {
    digitalWrite(PIN_LED, LOW);  // Turn off LED if out of range
    dist_raw = dist_prev;        // Use previous distance if out of range
  }

  // Apply EMA filter for distance
  dist_ema = EMA_ALPHA * dist_ema + (1.0 - EMA_ALPHA) * dist_raw;
  dist_prev = dist_raw;

  // Determine servo position based on distance
  if (dist_ema <= _DIST_CLOSE) {
    // Ball is very close: lower the servo (e.g., 0 degrees)
    duty = _DUTY_MIN;
  } else if (dist_ema >= _DIST_FAR) {
    // Ball is far: raise the servo (e.g., 180 degrees)
    duty = _DUTY_MAX;
  } else {
    // Ball is in between: map distance to servo position
    duty = _DUTY_MIN + (dist_ema - _DIST_CLOSE) * (_DUTY_MAX - _DUTY_MIN) / (_DIST_FAR - _DIST_CLOSE);
  }

  // Move servo to calculated position
  myservo.writeMicroseconds(duty);

  // Debugging: Serial output
  Serial.print("IR:");        Serial.print(a_value);
  Serial.print(", dist_raw:"); Serial.print(dist_raw);
  Serial.print(", EMA Distance:"); Serial.print(dist_ema);
  Serial.print(", Servo Duty:"); Serial.println(duty);
}
