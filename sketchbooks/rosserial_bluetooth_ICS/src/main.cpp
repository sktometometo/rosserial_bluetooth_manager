//
//  @file KrsServo1.ino
//  @brief KrsServoSample1
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2017/12/26
//
//  ID:0のサーボをポジション指定で動かす
//  範囲は、左5500 - 中央7500 - 右9500
//  0.5秒ごとに指定数値まで動く
//  ICSの通信にはHardwareSerialを使います。
//

#include <M5Stack.h>
#include <IcsHardSerialClass.h>
// #include <IcsSoftSerialClass.h>

const byte H_RX_PIN = 16;
const byte H_TX_PIN = 17;

const byte S_RX_PIN = 22;
const byte S_TX_PIN = 21;

const byte EN_PIN = 21;

const long BAUDRATE = 115200;
const int TIMEOUT = 200;
const int SERVO_ID = 4;

IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT);  //インスタンス＋ENピン(2番ピン)およびUARTの指定
// IcsSoftSerialClass krs(S_RX_PIN, S_TX_PIN, EN_PIN, BAUDRATE, TIMEOUT);
// //インスタンス＋ENピン(2番ピン)およびUARTの設定、softSerial版

void setup()
{
  // put your setup code here, to run once:
  M5.begin();
  krs.begin();  //サーボモータの通信初期設定
  Serial1.begin(115200, SERIAL_8E1, H_RX_PIN, H_TX_PIN);
  Serial.println("Initialized");
}

void loop()
{
  int id = krs.getID();
  Serial.printf("ID: %d\n", id);
  /*
  krs.setPos(SERVO_ID, 7500);  //位置指令　ID:0サーボを7500へ 中央
  Serial.println("Move to center");
  delay(500);                  // 0.5秒待つ
  krs.setPos(SERVO_ID, 9500);  //位置指令　ID:0サーボを9500へ 右
  Serial.println("Move to right");
  delay(500);                  // 0.5秒待つ
  krs.setPos(SERVO_ID, 7500);  //位置指令　ID:0サーボを7500へ 中央
  Serial.println("Move to Center");
  delay(500);                  // 0.5秒待つ
  krs.setPos(SERVO_ID, 5500);  //位置指令　ID:0サーボを5500へ 左
  Serial.println("Move to Left");
  delay(500);
  */
  /*
  delay(1000);
  Serial1.println("Hello");
  delay(1000);
  Serial1.println("World.");
  */
}
