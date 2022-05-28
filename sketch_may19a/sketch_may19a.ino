#include <SoftwareSerial.h>
#include <util/atomic.h>
#define ENCA 2
#define ENCB 3
#define PWM_PINA 5
#define PWM_PINB 6

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

float vt = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myserial.begin(9600);
  pinMode(PWM_PINA, OUTPUT);
  pinMode(PWM_PINB, OUTPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {

if(myserial.available()){
  char w = myserial.read();
  switch(w) {
    case '0': vt = 0.0; break;
    case '1': vt = 33.0; break;
    case '2': vt = 45.0; break;
    case '3': vt = 78.0; break;
  }
  Serial.println(w);
  Serial.println(vt);
}
  
int pos = 0;
ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
  pos = pos_i;
}

long currT = micros();
float deltaT = ((float) (currT- prevT))/1.0e6;
float velocity = (pos - posPrev)/deltaT;
posPrev = pos;
prevT = currT;

//switch to RPM
float v = velocity/360.0*60.0;

//low pass filter to filter out noise
vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
vPrev = v;

//control signal
float kp = 5;
float ki = 10;
float e = vt - vFilt;
eintegral = eintegral + e * deltaT;
float u = kp * e + ki * eintegral;

int dir = 1;
if(u < 0)
  dir = -1;
pwm = (int) fabs(u);
if(pwm > 255)
  pwm = 255;

setMotor(dir, pwm);

//Serial.println(v);
//for consistent sampling
delay(1);
}

void setMotor(int dir, int pwmVal)
{
  if(dir == 1)
    analogWrite(PWM_PINB, pwmVal); 
  if(dir == -1)
    analogWrite(PWM_PINA, pwmVal);
}

void readEncoder()
{
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b > 0)
    increment = 1;
  else
    increment = -1;
   pos_i = pos_i + increment;
}
