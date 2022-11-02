#include <Servo.h>

// Arduino pin assignment

#define PIN_POTENTIOMETER 3 // Potentiometer at Pin A3
#define PIN_SENSOR 0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 553  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counter-clockwise position (180 degree)

#define LOOP_INTERVAL 50   // Loop Interval (unit: msec)

#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 250

#define _EMA_ALPHA 0.2

float dist_raw;              // distance measured unit: mm
float dist_prev = _DIST_MIN;
float dist_ema;  // distance measured unit: mm


Servo myservo;
unsigned long last_loop_time;   // unit: msec

void setup()
{
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  pinMode(PIN_LED, OUTPUT);
  
  Serial.begin(1000000);
}

void loop()
{
  unsigned long time_curr = millis();
  int a_value, duty;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  // Remove Next line !!!
  a_value = analogRead(PIN_SENSOR);
  dist_raw = (6762.0/(a_value-9)-4.0)*10.0;

  if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;    
    digitalWrite(PIN_LED, 1);
  } else if (dist_raw > _DIST_MAX) {
    dist_raw = dist_prev;    
    digitalWrite(PIN_LED, 1);   
    
  } else {    // In desired Range    
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, 0);
  }

  // Read IR Sensor value !!!
  // Convert IR sensor value into distance !!!
  // we need distance range filter here !!!
  // we need EMA filter here !!!
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  // map distance into duty
  duty = 923.0 / 75.0 * dist_ema - 2033.0 / 3.0;
  myservo.writeMicroseconds(duty);

  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",IR:");   Serial.print(a_value);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",ema:");  Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(duty);  
  Serial.print(",Max:");   Serial.print(_DIST_MAX);

  Serial.println("");
}
