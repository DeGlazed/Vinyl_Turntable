#include <SoftwareSerial.h>
#include <util/atomic.h>
#define ENCA 2
#define PWM_PIN 5

const byte rxpin = 8;
const byte txpin = 9;
SoftwareSerial myserial (rxpin, txpin);
char w;
int pwm;

int cnt = 0;

long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;

float vFilt = 0;
float vPrev = 0;

float eintegral = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myserial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENCA, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
/* 
// put your main code here, to run repeatedly:
//for some reason 255 = 0; ~85 = full power
 if(myserial.available()) {
   w = myserial.read();
   switch(w) {
    case '0': 
      pwm = 0;
      break;
    case '1': pwm = 20; 
      break;
    case '2': pwm = 40; 
      break;
    case '3': pwm = 60; 
      break;
   }
   Serial.write(w);
 }
//
// if(Serial.available()) {
//   w = Serial.read();
//   myserial.write(w);
// }
 */
int pos = 0;

ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
  pos = pos_i;
}

long currT = micros();
float deltaT = ((float) (currT- prevT))/1.0e6;
float velocity = (pos - posPrev)/deltaT;

//switch to RPM
float v = velocity/360.0*60.0;

//low pass filter to filter out noise
vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
vPrev = v;

//target
float vt = 10.0;

//control signal
float kp = 3;
float ki = 1;
float e = vt - vFilt;
eintegral = eintegral + e * deltaT;
float u = kp * e + ki * eintegral;

if(u < 0)
  pwm = 0;
pwm = (int) fabs(u);
if(pwm > 255){
  pwm = 255;
}

setMotor(255 - pwm, PWM_PIN);

Serial.println(v);
//for consistent sampling
delay(1);
}

void setMotor(int pwmVal, int pwmPin)
{
  analogWrite(pwmPin, pwmVal);  
}

void readEncoder()
{
   pos_i++;
}
