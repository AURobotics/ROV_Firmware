// A
#define dirH1   17
#define pwmH1   13

// B
#define dirH2   15
#define pwmH2   23

// C
#define dirH3   16
#define pwmH3   24

// D
#define dirH4   0
#define pwmH4   2

// E
#define enV1    32
#define pwmV1_1 33
#define pwmV1_2 25

// F
#define enV2    26
#define pwmV2_1 27
#define pwmV2_2 14

#define light 19 

void turnLight(bool state ){
  digitalWrite(light , state ? HIGH : LOW ) ;
}

int HorizontalThrusterPinsDir[4]={dirH1,dirH2,dirH3,dirH4} ;
int HorizontalThrusterPinsSpeed[4]={pwmH1 , pwmH2 ,pwmH3 ,pwmH4} ;

float outputHorizontalThrusters[4] = {255, 255, 255, 255};    // values for the horizontal thrusters  
float outputVerticalThrusters[2] = {127, 127};                // values for the vertical thrusters 

int VerticalThrusterSp1[2] = {pwmV1_1 , pwmV2_1} ;
int VerticalThrusterSp2[2] = {pwmV1_2 , pwmV2_2} ;


void setup_H_motors(){

  pinMode(dirH1,OUTPUT);
  pinMode(pwmH1,OUTPUT);
  
  pinMode(dirH2,OUTPUT);
  pinMode(pwmH2,OUTPUT);
  
  pinMode(dirH3,OUTPUT);
  pinMode(pwmH3,OUTPUT);
  
  pinMode(dirH4,OUTPUT);
  pinMode(pwmH4,OUTPUT);

}

void setup_V_motors(){

  pinMode(pwmV1_1,OUTPUT);
  pinMode(pwmV1_2,OUTPUT);
  pinMode(enV1,OUTPUT);

  digitalWrite(enV1,HIGH);

  pinMode(pwmV2_1,OUTPUT);
  pinMode(pwmV2_2,OUTPUT);
  pinMode(enV2,OUTPUT);

  digitalWrite(enV2,HIGH);

}
void controlHmotors(){
  for (int num = 0 ; num <=3 ; num++ ){
     digitalWrite(HorizontalThrusterPinsDir[num], (outputHorizontalThrusters[num] >= 0) ? HIGH : LOW );
     analogWrite(HorizontalThrusterPinsSpeed[num] , abs(outputHorizontalThrusters[num]) ) ; 
  }
 
}

void controlVmotors(){
  for (int num = 0 ; num <=1 ; num++ ){
    (outputVerticalThrusters[num]>=0 )? analogWrite(VerticalThrusterSp1[num], abs(outputVerticalThrusters[num]) ) : analogWrite(VerticalThrusterSp2[num] , abs(outputVerticalThrusters[num]) ) ;  
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setup_H_motors() ;
  setup_V_motors() ;
for (int num = 0 ; num <=3 ; num++ ){
     digitalWrite(HorizontalThrusterPinsDir[num], LOW );
     digitalWrite(HorizontalThrusterPinsSpeed[num] , LOW) ; 
  }
  delay(1000);

   pinMode(light , OUTPUT) ; // set light pin

   controlHmotors();
   controlVmotors();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  turnLight(1); 
  delay(1000);
  turnLight(0); 
  delay(1000);


}
