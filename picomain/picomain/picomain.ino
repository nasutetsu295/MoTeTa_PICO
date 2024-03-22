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
#define touchR 3
#define touchL 2//かえたよ
#define S11059_ADDR 0x2A

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground
int beforeencorder = 0;
// set to false if using a common cathode LED
#define commonAnode true

#include <SoftwareSerial.h>

int get_data(int size);       //ラスパイ4Bからのデータを取得 引数:通信で送られうる最大の値

const int UART_SIZE = 17;     //コピペで

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

int exitflag;

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
  if (green >= 91 && blue >= 88){
    int beforeencorder = encL.getCount() ;
    while(encL.getCount() <= beforeencorder + 400){
    Motor_move(0,-150);
    Motor_move(1,-150);
    //Serial.println(encL.getCount());
  }
  Motor_stop(0);
  exitflag =1;
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
  distRF = Get_distance(2) - 25; //位置の問題を修正
  distRR = Get_distance(3);
  distL = Get_distance(4);

  Serial.println("");
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
    Motor_stop(0);
    Motor_move(0,-200);
    Motor_move(1,-200);
    delay(400);
    Motor_move(0,-200);
    Motor_move(1,200);
    delay(400);
    Motor_move(0,200);
    Motor_move(1,200);
    delay(300);
    Motor_move(0,200);
    Motor_move(1,-200);
    delay(400);
    //exitflag = 1;
  }
  digitalRead(touchL);
  Serial.print(" ");
  Serial.println(digitalRead(touchL));  
  if (digitalRead(touchL) == 1){
    Motor_stop(0);
    Motor_move(1,-200);
    Motor_move(0,-200);
    delay(400);
    Motor_move(1,-200);
    Motor_move(0,200);
    delay(400);
    Motor_move(1,200);
    Motor_move(0,200);
    delay(300);
    Motor_move(1,200);
    Motor_move(0,-200);
    delay(400);
    //exitflag = 1;
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
  while(encL.getCount() >= beforeencorder - 1200){
    Motor_move(0,-200);
    Motor_move(1,200);
    //Serial.println(encL.getCount());

  }
}

void turnright(){
  int beforeencorder = encL.getCount() ;
  while(encL.getCount() <= beforeencorder + 1200){
    Motor_move(1,-200);
    Motor_move(0,200); //左
    //Serial.println(encL.getCount());

  }
}

void zenshin(){
  beforeencorder = encL.getCount(); 
  while(encL.getCount() >= beforeencorder - 1800){
    //Motor_straight(200);
    Motor_move(1,200);
    Motor_move(0,200);
    //Serial.println(encL.getCount());
    //checkhole();
    if (exitflag == 1){exitflag = 0; return;}
    checkobject();
  }
}

void checkvic(){
  if (get_data(17) == 1){
    for (int i ;i <= 4 ; i++){
    led_on();
    delay(500);
    led_off();
    delay(500);
   }
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

void Motor_stop(int Motor_LR){     //片方のモータをブレーキさせる。Motor_LR=0のときR、=1のときL

    digitalWrite(Rmotor1, HIGH);
    digitalWrite(Rmotor2, HIGH);

    digitalWrite(Lmotor1, HIGH);
    digitalWrite(Lmotor2, HIGH);

    //Serial.println("Choose Lmotor(0) or Rmotor(1)");
  
}

int get_data(int size){
  int data = 0;
  for (int i = 0; i < size; i++){
    if ( Serial1.available() > 0){//dataが来た時にtureになる。
      data++;
      Serial1.read();
      // Serial.print(Serial1.read());//読み取り、表示
    }
  }
  return data;
}

///*
void front_correct()
  {
    
    int count = 0;
    GetdistanceAll();
    GetdistanceAll();

    if (distFR < 130  && distFL < 130){
         while ((distFR < 70  || distFL < 70) && (count < 20))
         {
          Motor_move(0,-120);
          Motor_move(1,-120);
          GetdistanceAll();
          GetdistanceAll();
          count = count + 1;
         }
          Motor_stop(0);
          Motor_stop(1);

          return;
    }
    else{
          if ((distFR > 90  && distFL > 90) && (distFR < 250  && distFL < 250))
          {
              while ((distFR > 130  || distFL > 130) && (count < 20))
            {
              Motor_move(0,120);
              Motor_move(1,120);
              GetdistanceAll();
              GetdistanceAll();
              count = count + 1;
            }
          }
          Motor_stop(0);
          Motor_stop(1);
          return;
    }
    }


void right_correct(){
  GetdistanceAll();
  GetdistanceAll();
  int count = 0;

  if(distRF < 150 &&  distRR < 150 ){ //壁がないのに右壁で修正するのを防止する

    if (distRF > distRR){
      while ((distRF + 15 < distRR) && (count < 30) )
         {
          Motor_move(0,100);
          Motor_move(1,-100);
          GetdistanceAll();
          GetdistanceAll();
          count = count + 1 ;
         }
    }

    else{
          while ((distRF > 15 + distRR) && (count < 30))//なんかおかしいなあ
         {
          Motor_move(0,-100);
          Motor_move(1,100);
          GetdistanceAll();
          GetdistanceAll();
          count = count + 1 ;
         }
    }
  }
  Motor_stop(6);
   }

void front_turn_correct(){
  GetdistanceAll();
  GetdistanceAll();
  int count = 0;

  if(distFL < 180 &&  distFR < 180 ){ //壁がないのに前壁で修正するのを防止する

    if (distFL > distFR){
      while ((distFL >= distFR) && (count < 30 ))
         {
          Motor_move(0,120);
          Motor_move(1,-120);
          Serial.println("");
          Serial.println(count);
          GetdistanceAll();
          GetdistanceAll();
          count = count + 1 ;
         }
    }

    else{
          while ((distFL <= distFR) && (count < 30))//なんかおかしいなあ
         {
          Motor_move(0,-120);
          Motor_move(1,120);
          Serial.println("");
          Serial.println(count);
          GetdistanceAll();
          GetdistanceAll();
          count = count + 1 ;
         }
    }
  }
  Motor_stop(6);
   }
//*/

void checkswamp(){
  getcolor();
  Motor_stop(0);
  if (blue >= 100){
  delay(580);
  }
}

void setup() {
  delay(2000);
  //led_on();
  // put your setup code here, to run once:
  //pinMode (25,OUTPUT);
  //Serial.begin(UART_BAUD);


  pinMode(LEDK,OUTPUT);
  pinMode(LEDA,OUTPUT);
  pinMode(touchR,INPUT_PULLDOWN);
  pinMode(touchL,INPUT_PULLDOWN);

  //Tof.startContinuous();

  Serial.begin(UART_BAUD);
  Serial.println("START");
  Serial1.begin(9600);   // UART0初期化 ラズパイ4Bとの通信用
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
  if (distRF < 250 &&  distRR < 250)
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
    front_turn_correct();
    right_correct();
    zenshin();     
    }
  Motor_stop(0);
  delay(1000);
  //checkobject();
  GetdistanceAll();

  front_turn_correct();
  right_correct();
  front_correct();
  checkswamp();

}