#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "ClosedCube_TCA9548A.h"
#include "pio_encoder.h"
#include "Adafruit_TCS34725.h"

#define UART_BAUD 9600
#define TCA9548A_I2C_ADDRESS  0x70
#define LEDA 4
#define LEDK 5
#define touchR 2
#define touchL 3
#define S11059_ADDR 0x2A

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true


// our RGB -> eye-recognized gamma color
byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
VL53L0X Tof ;
ClosedCube::Wired::TCA9548A tca9548a;

int distFL;
int distFR;
int distRF;
int distRR;
int distL ;

float red, green, blue;

// 使うPINの宣言
const int Rmotor1 = 6;
const int Rmotor2 = 7;
const int REN_A = 8;
const int REN_B = 9;
const int Lmotor1   = 10;
const int Lmotor2  = 11;
const int LEN_A = 12;
const int LEN_B = 13;

//エンコーダーで読んだ角度の値
long Rotation_R = 0L;
long Rotation_L = 0L;

long PID_Rotation_R = 0L;
long PID_Rotation_L = 0L;

//Motor_straight用のPID制御の調整用定数
const double N_Weight = 0.7;
const double P_Weight = 0.3;
const double D_Weight = 0.1;
const float RproK = 7;
const float LproK = 7;

//プロトタイプ宣言（関数の動作説明は下へ）
void Motor_stop(int Motor_LR);
void Motor_move(int Motor_LR, float power);
void Motor_straight(float power);


//エンコーダーインスタンス作成
PioEncoder encR(REN_A);
PioEncoder encL(LEN_A);

enum distance {           //上から0 ~ 4で変数に割り振られる。
  Leftfront,              //0
  Rightfront,             //1
  Rightsidefront,         //2
  Rightsideback,          //3
  leftside,               //4
};
//-------------------------------------------あとで消すゾーンはじめ
void motorrun(int LR,int power){
  Serial.println("motor running");
}


int L;
int R;
//-------------------------------------------あとで消すゾーンおわり


void checkhole(){
  getcolor();
  if (red <=76 && green <= 76 && blue <= 76){
    int beforeencorder = encL.getCount() ;
    while(encL.getCount() <= beforeencorder + 400){
    Motor_move(0,-150);
    Motor_move(1,-150);
    //Serial.println(encL.getCount());
  }
  Motor_stop();
  } 
}

void Init_distance(int connectors){
  // tca9548a.address(0x70);
   
  //センサの初期化
  int i;
  for(i=0; i<=connectors; i++){
    tca9548a.selectChannel(i);   
    Tof.setTimeout(500);
    if (!Tof.init()){    
      Serial.print("Failed to detect and initialize sensor-");
      Serial.println(i);
      //while (1) {}
    }
    Serial.println(i);
    Tof.startContinuous();
  }
}

void init_color(){
  tca9548a.address(0x70);
    tca9548a.selectChannel(5);   
    if (tcs.begin()) {
      Serial.println("Color is good");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1){
        Serial.println("COLOR IS NOT GOOD.");
        delay(100);
        }; // halt!
    }
}

int Get_distance(int location){
  tca9548a.selectChannel(location);
  int Distance = Tof.readRangeSingleMillimeters();    //距離センサ値取得+表示も
  if (Tof.timeoutOccurred()) { Serial.print("Distance TIMEOUT");}   //timeout用
  return Distance;
}

void GetdistanceAll(){
  //getcolor
  distFL = Get_distance(0);
  distFR = Get_distance(1);  
  distRF = Get_distance(2) + 25; //位置の問題を修正
  distRR = Get_distance(3);
  distL = Get_distance(4);

  Serial.print("distFL");
  Serial.println(distFL);
  Serial.print("distFR");
  Serial.println(distFR);
  Serial.print("distRF");
  Serial.println(distRF);
  Serial.print("distRR");  
  Serial.println(distRR);  
  Serial.print("distL");  
  Serial.println(distL);  
}

void getcolor() {
  tca9548a.selectChannel(5);  
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));

  Serial.print("\n");
 }

void checkobject(){
  digitalRead(touchR);
  Serial.print(digitalRead(touchR));
  if (digitalRead(touchR) == 1){
    Motor_move(0,-100);
    Motor_move(1,-250);
    delay(700);
    Motor_move(0,-200);
    Motor_move(1,200);
    delay(400);
    Motor_move(0,200);
    Motor_move(1,200);
    delay(700);
  }
  digitalRead(touchL);
  Serial.print(" ");
  Serial.println(digitalRead(touchL));  
  if (digitalRead(touchL) == 1){
    Motor_move(0,-250);
    Motor_move(1,-100);
    delay(700);
    Motor_move(0,200);
    Motor_move(1,-200);
    delay(400);
    Motor_move(0,200);
    Motor_move(1,200);
    delay(700);
  }
}

void led_on(){
  digitalWrite(LEDA,HIGH);
  digitalWrite(LEDK,LOW);
}

void led_off(){
  digitalWrite(LEDA,LOW);
  digitalWrite(LEDK,HIGH);
}

void turnleft(){
  int beforeencorder = encL.getCount() ;
  while(encL.getCount() >= beforeencorder - 1700){
    Motor_move(0,-200);
    Motor_move(1,200);
    //Serial.println(encL.getCount());
  }
}

void turnright(){
  int beforeencorder = encL.getCount() ;
  while(encL.getCount() >= beforeencorder + 1700){
    Motor_move(1,-200);
    Motor_move(0,200);
    //Serial.println(encL.getCount());
  }
}

void zenshin(){
  int beforeencorder = encL.getCount(); 
  while(encL.getCount() >= beforeencorder - 1700){
    //Motor_straight(200);
    Motor_move(1,200);
    Motor_move(0,200);
    //Serial.println(encL.getCount());
    checkobject();
    checkhole();
  }
}

void checkvic(){
  for (int i ;i <= 4 ; i++){
  led_on();
  delay(500);
  led_off();
  delay(500);
  }
}

// void Motor_stop(int Motor_LR){     //片方のモータをブレーキさせる。Motor_LR=0のときR、=1のときL
//   if(Motor_LR == 0){
//     digitalWrite(Rmotor1, HIGH);
//     digitalWrite(Rmotor2, HIGH);
//   }else if(Motor_LR == 1){
//     digitalWrite(Lmotor1, HIGH);
//     digitalWrite(Lmotor2, HIGH);
//   }else{
//     Serial.println("Choose Lmotor(0) or Rmotor(1)");
//   }
// }



void Motor_move(int Motor_LR, float power){    //motor_LRはMotor_stopと同じ。power = -255~255

  int motor1, motor2;

  if(Motor_LR ==  0){
    motor1 = Rmotor1;
    motor2 = Rmotor2;
  }else if(Motor_LR == 1){
    motor1 = Lmotor1;
    motor2 = Lmotor2;
  }else{
    Serial.println("Choose Lmotor(0) or Rmotor(1)");
  }

  if(power >= 0.0 && power <= 255.0){ 
    analogWrite(motor1, power);
    digitalWrite(motor2, LOW);
  }else if(power <= 0.0 && power >= -255.0){
    digitalWrite(motor1, LOW);
    analogWrite(motor2, -power);
  }else if(power >= 255.0 || power <= -255.0){
    Serial.println("関数Motr_moveの値powerは-255~255の範囲のみ");
  }

}


void Motor_straight(float power){         //まっすぐ進むよう(P制御採用)  引数：-255 ~ 255

  int Rfreq = int(PID_Rotation_R - encR.getCount());
  int Lfreq = int(PID_Rotation_L - encL.getCount());

  PID_Rotation_R = encR.getCount();
  PID_Rotation_L = encL.getCount();

  float Rproportion = RproK * (float)(Lfreq - Rfreq);
  float Lproportion = LproK * (float)(Rfreq - Lfreq);

  float Rvalue = power * N_Weight + Rproportion * P_Weight;
  float Lvalue = power * N_Weight + Lproportion * P_Weight;


  Motor_move(0, Rvalue);
  Motor_move(1, Lvalue);

  Serial.print("Rfreq: ");      Serial.print(Rfreq);
  Serial.print(",  Lfreq: ");   Serial.print(Lfreq);
  Serial.print(",  Lf-Rf: ");   Serial.print(Lfreq - Rfreq);
  // Serial.print(", RproK: ");    Serial.print(RproK);
  Serial.print(",  Rpro: ");    Serial.print(Rproportion);
  Serial.print(",  Lpro: ");    Serial.print(Lproportion);
  Serial.print(",  Rvalue: ");  Serial.print(Rvalue);
  Serial.print(",  Lvalue: ");  Serial.println(Lvalue);

}

void Motor_stop(){
  Motor_move(0,0);
  Motor_move(1,0);
}

///*
void front_correct()
  {
    int count = 0;
    GetdistanceAll();

    if (distFR < 130  && distFL < 130){
         while ((distFR < 70  || distFL < 70) and count < 20)
         {
          Motor_move(0,20);
          Motor_move(1,20);
          GetdistanceAll();
          count = count + 1;
         }
          Motor_move(0,0);
          Motor_move(1,0);
    }
    else{
          if (distFR > 90  && distFL > 90 && distFR < 250  && distFL < 250)
          {
              while ((distFR > 70  || distFL > 70) and count < 20)
            {
              Motor_move(0,-20);
              Motor_move(1,-20);
              GetdistanceAll();
                        count = count + 1;
            }
          }
          Motor_move(0,0);
          Motor_move(1,0);
    }
    }


void right_correct(){
  GetdistanceAll();
  int count = 0;

  if(distRF < 130 &&  distRR < 130){ //壁がないのに右壁で修正するのを防止する

    if (distRF > distRR){
      while (distRF > 5 + distRR and count < 60 )
         {
          Motor_move(0,-30);
          Motor_move(1,30);
          GetdistanceAll();
          count = count + 1 ;
         }
    }

    else{
          while (10 + distRF < distRR and count < 60)
         {
          Motor_move(0,30);
          Motor_move(1,-30);
          GetdistanceAll();
          count = count + 1 ;
         }
    }
  }

   }

void front_turn_correct(){

  GetdistanceAll();
  int count = 0;

  if(5 +distFL < 130 &&  distFR < 130){ //壁がないのに前壁で修正するのを防止する



    if (distFL > distFR){
      while (distFL > 5 +distFR and count <60 )
         {
          Motor_move(0,-30);
          Motor_move(1,30);
          GetdistanceAll();
          count = count + 1 ;
         }
    }

    else{
          while (distFL < 10 +distFR and count < 60)
         {
          Motor_move(0,30);
          Motor_move(1,-30);
          GetdistanceAll();
          count = count + 1 ;
         }
    }
  }
 }
//*/

void setup() {
  delay(2000);
  led_on();
  // put your setup code here, to run once:
  //pinMode (25,OUTPUT);
  //Serial.begin(UART_BAUD);
  Serial.println("START");

  pinMode(LEDK,OUTPUT);
  pinMode(LEDA,OUTPUT);
  pinMode(touchR,INPUT_PULLDOWN);
  pinMode(touchL,INPUT_PULLDOWN);

  //Tof.startContinuous();

  Serial.begin(UART_BAUD);
  //while(!Serial){};

  pinMode(Lmotor1, OUTPUT);//モータ系
  pinMode(Lmotor2, OUTPUT);
  pinMode(Rmotor1, OUTPUT);
  pinMode(Rmotor2, OUTPUT); 
  encL.begin();
  encR.begin();
  //モータ系終

  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();
  init_color();
  Init_distance(4);
}

void loop() {
  //Serial.println("HEY");

  //getcolor();
  GetdistanceAll();

  delay(1000);
  if (distRF < 200 &&  distRR < 200)
  {
     if (distFR < 250  && distFL < 250)
     {
      //右壁前壁
      turnleft();
     }else {
      //右壁前なし
      // turnright();
      zenshin();
    }
  }
  else {
    //右なし前調べてない
    turnright();
    zenshin();     
    }
  Motor_stop();
  delay(1000);
  checkobject();
  GetdistanceAll();
}