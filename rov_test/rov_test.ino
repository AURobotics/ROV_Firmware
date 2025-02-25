#define dir1 16
#define pwm1 4

#define dir2 0
#define pwm2 2

#define dir3 15
#define pwm3 23

#define dir4 17
#define pwm4 13

#define pwm5_1 33
#define pwm5_2 25

#define pwm6_1 27
#define pwm6_2 14




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(dir1,OUTPUT);
  pinMode(pwm1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(pwm2,OUTPUT);
  pinMode(dir3,OUTPUT);
  pinMode(pwm3,OUTPUT);
  pinMode(dir4,OUTPUT);
  pinMode(pwm4,OUTPUT);

  pinMode(pwm5_1,OUTPUT);
  pinMode(pwm5_2,OUTPUT);       
  pinMode(pwm6_1,OUTPUT);
  pinMode(pwm6_2,OUTPUT);

  pinMode(32,OUTPUT);
  digitalWrite(32,HIGH); // enable bts

  pinMode(26,OUTPUT);
  digitalWrite(26,HIGH); // enable bts


}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("i am working loop1");
  // digitalWrite(dir1,HIGH);
  // digitalWrite(pwm1,HIGH);

  // digitalWrite(dir2,LOW);
  // digitalWrite(pwm2,HIGH);

  // digitalWrite(dir3,HIGH);
  // digitalWrite(pwm3,HIGH);
  // digitalWrite(dir4,LOW);
  // digitalWrite(pwm4,HIGH);

  digitalWrite(pwm5_1,HIGH);
  digitalWrite(pwm5_2,LOW);
  digitalWrite(pwm6_1,HIGH);
  digitalWrite(pwm6_2,LOW);
  

  delay(1000);
  Serial.println("i am working loop2");

  // digitalWrite(dir2,HIGH);
  // digitalWrite(pwm2,HIGH);

  // digitalWrite(dir1,LOW);
  // digitalWrite(pwm1,HIGH);

  // digitalWrite(dir3,LOW);
  // digitalWrite(pwm3,HIGH);
  // digitalWrite(dir4,HIGH);
  // digitalWrite(pwm4,HIGH);

  digitalWrite(pwm5_1,LOW);
  digitalWrite(pwm5_2,HIGH);
  digitalWrite(pwm6_1,LOW);
  digitalWrite(pwm6_2,HIGH);

  delay(1000);

}
