
#include <Encoder.h>

Encoder encoder_fleft(43, 42);
Encoder encoder_fright(44, 45); //pin is changed from original (35, 34)
Encoder encoder_bleft(47, 46);
Encoder encoder_bright(48, 49); //pin is changed from original (31, 30)

const uint8_t LF_PWM = 3;
const uint8_t RF_PWM = 2;
const uint8_t LB_PWM = 5;
const uint8_t RB_PWM = 4;

const uint8_t LF_BACK = 25;
const uint8_t LF_FORW = 24;

const uint8_t RF_BACK = 23;
const uint8_t RF_FORW = 22;

const uint8_t LB_BACK = 29;
const uint8_t LB_FORW = 28;

const uint8_t RB_BACK = 26; //pin was interchanged RB_BACK = 28// RB_FORW = 29
const uint8_t RB_FORW = 27;

void setpins()
{
  pinMode(LF_FORW,OUTPUT);
  pinMode(LF_BACK,OUTPUT);
  pinMode(RF_FORW,OUTPUT);
  pinMode(RF_BACK,OUTPUT);
  pinMode(LF_PWM,OUTPUT);
  pinMode(RF_PWM,OUTPUT);
  pinMode(LB_FORW,OUTPUT);
  pinMode(LB_BACK,OUTPUT);
  pinMode(RB_FORW,OUTPUT);
  pinMode(RB_BACK,OUTPUT);
  pinMode(LB_PWM,OUTPUT);
  pinMode(RB_PWM,OUTPUT);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);
}


void Move_motor(int speed_pwm,const uint8_t pwm,const uint8_t forw,const uint8_t back)
{
  if(speed_pwm >= 0)
  {
    digitalWrite(forw, HIGH);
    digitalWrite(back, LOW);
    analogWrite(pwm, abs(speed_pwm));
  }
  else if(speed_pwm < 0)
  {
    digitalWrite(forw, LOW);
    digitalWrite(back, HIGH);
    analogWrite(pwm, abs(speed_pwm));
  }
}

void setup() {
  // put your setup code here, to run once:
  setpins();
  Serial.begin(9600);

}

void loop() {
  int speedy = -200;
  // put your main code here, to run repeatedly:
  long fleft = encoder_fleft.read();
  long fright = encoder_fright.read();
  long bleft = encoder_bleft.read();
  long bright = encoder_bright.read();
  Serial.print("fleft: ");
  Serial.print(fleft);
  Serial.print("  ");
  Move_motor(speedy,LF_PWM,LF_FORW,LF_BACK);
  Serial.print("fright: ");
  Serial.print(fright);
  Serial.print("  ");
  Move_motor(speedy,RF_PWM,RF_FORW,RF_BACK);
  Serial.print("bleft: ");
  Serial.print(bleft);
  Serial.print("  ");
  Move_motor(speedy,LB_PWM,LB_FORW,LB_BACK);
  Serial.print("bright: ");
  Serial.print(bright);
  Serial.println("  ");
  Move_motor(speedy,RB_PWM,RB_FORW,RB_BACK);
  delay(50);

}
