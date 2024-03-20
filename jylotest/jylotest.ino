#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <pico/stdlib.h>

// BNO055センサーオブジェクトの作成

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
void setup(void) {
   Wire.setSDA(21); // SDAピンの設定
   Wire.setSCL(20); // SCLピンの設定
  Serial.begin(9600);
  // センサーオブジェクトの初期化
  if (!bno.begin())
  {
    Serial.println("初期化に失敗");
    while (1){Serial.println("HELP!");};
  }

  // センサーキャリブレーションの初期値を設定
  uint8_t calibrationData[] = {12, 0, 0, 0, 62, 3, 0, 0, 25, 95, 0, 16, 32, 33, 0, 32, 1, 0, 0, 0, 12, 0};
  bno.setSensorOffsets(calibrationData);
}

void loop(void) {
  sensors_event_t event;
  bno.getEvent(&event);

  // ヨーの値を表示
  Serial.print("Yaw: ");
  Serial.println(event.orientation.x);
}