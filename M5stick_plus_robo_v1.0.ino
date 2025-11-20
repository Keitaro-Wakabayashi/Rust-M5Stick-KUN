#define Ver "v1.0_p2"
// Interfaceのサポートページに載せたV1.0ベースでＭ5StickC_plus２テスト用


#include "EEPROM.h"
#include <M5Unified.h> 
//#include <M5StickCPlus2.h> // M5Unified どちらでも行ける感じ
#include <Kalman.h>

#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#include <RemoteXY.h>
#define REMOTEXY_BLUETOOTH_NAME "M5Stick2_IP2"

#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 714 bytes
  { 255,4,0,217,0,195,2,19,0,0,0,0,31,2,126,200,200,80,1,1,
  5,0,67,0,251,127,10,0,253,202,6,64,8,201,68,30,53,25,100,85,
  3,115,70,36,8,94,178,1,16,80,0,73,0,68,0,83,112,100,0,4,
  22,115,9,90,70,76,130,7,160,2,26,12,7,158,25,25,0,2,25,7,
  255,30,26,78,111,46,48,0,49,32,58,32,83,112,101,101,100,95,99,97,
  108,0,50,32,58,32,107,80,0,51,32,58,32,107,73,0,52,32,58,32,
  107,68,0,53,32,58,32,107,115,112,100,0,54,32,58,32,107,100,115,116,
  0,78,111,46,55,0,78,111,46,56,0,78,111,46,57,0,49,48,32,58,
  32,80,105,116,99,104,95,111,102,102,115,101,116,50,0,49,49,32,58,32,
  109,111,116,111,114,95,111,102,102,115,101,116,76,0,49,50,32,58,32,109,
  111,116,111,114,95,111,102,102,115,101,116,82,0,78,111,46,49,51,0,78,
  111,46,49,52,0,78,111,46,49,53,0,78,111,46,49,54,0,78,111,46,
  49,55,0,78,111,46,49,56,0,78,111,46,49,57,0,78,111,46,50,48,
  0,111,112,116,105,111,110,32,50,49,0,111,112,116,105,111,110,32,50,50,
  0,111,112,116,105,111,110,32,50,51,0,111,112,116,105,111,110,32,50,52,
  0,111,112,116,105,111,110,32,50,53,0,111,112,116,105,111,110,32,50,54,
  0,111,112,116,105,111,110,32,50,55,0,111,112,116,105,111,110,32,50,56,
  0,111,112,116,105,111,110,32,50,57,0,111,112,116,105,111,110,32,51,48,
  0,111,112,116,105,111,110,32,51,49,0,111,112,116,105,111,110,32,51,50,
  0,111,112,116,105,111,110,32,51,51,0,111,112,116,105,111,110,32,51,52,
  0,111,112,116,105,111,110,32,51,53,0,111,112,116,105,111,110,32,51,54,
  0,111,112,116,105,111,110,32,51,55,0,111,112,116,105,111,110,32,51,56,
  0,111,112,116,105,111,110,32,51,57,0,111,112,116,105,111,110,32,52,48,
  0,111,112,116,105,111,110,32,52,49,0,111,112,116,105,111,110,32,52,50,
  0,111,112,116,105,111,110,32,52,51,0,111,112,116,105,111,110,32,52,52,
  0,111,112,116,105,111,110,32,52,53,0,111,112,116,105,111,110,32,52,54,
  0,111,112,116,105,111,110,32,52,55,0,111,112,116,105,111,110,32,52,56,
  0,111,112,116,105,111,110,32,52,57,0,111,112,116,105,111,110,32,53,48,
  0,111,112,116,105,111,110,32,53,49,0,111,112,116,105,111,110,32,53,50,
  0,111,112,116,105,111,110,32,53,51,0,111,112,116,105,111,110,32,53,52,
  0,111,112,116,105,111,110,32,53,53,0,111,112,116,105,111,110,32,53,54,
  0,111,112,116,105,111,110,32,53,55,0,111,112,116,105,111,110,32,53,56,
  0,111,112,116,105,111,110,32,53,57,0,111,112,116,105,111,110,32,54,48,
  0,111,112,116,105,111,110,32,54,49,0,111,112,116,105,111,110,32,54,50,
  0,111,112,116,105,111,110,32,54,51,0,111,112,116,105,111,110,32,54,52,
  0,5,201,30,150,150,255,2,86,86,3,2,26,31 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t slider_01; // from -100 to 100
  uint8_t Select_01; // from 0 to 65
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100

    // output variables
  char text_1[201]; // string UTF8 end zero
  float onlineGraph_01_var1;
  float onlineGraph_01_var2;
  float onlineGraph_01_var3;
  float onlineGraph_01_var4;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)

// IMU定数の変更（M5Unifiedの最新仕様に合わせる）
#define MPU6886 IMU // 互換性のために定義を維持
#define MPU6886_AFS m5::mpu6886::ACCEL_RANGE_2G // M5Unifiedの加速度レンジ定数
#define MPU6886_GFS m5::mpu6886::GYRO_RANGE_250DPS // M5Unifiedのジャイロレンジ定数

// Motor and hardware definitions
#define MOTOR_PIN_L 0
#define MOTOR_PIN_R 26
#define BTN_A 37
#define BTN_B 39
#define M5_LED 19
#define INIT_FLAG_ADDR 0     
#define INIT_MAGIC     0xA5A5

// Control parameters
int motor_offsetL = 0, motor_offsetR = 0;
int16_t motor_init_L = 1500, motor_init_R = 1500;
float kpower = 0.003;
float kp = 6.3;
float ki = 1.4;
float kd = 0.48;
float kspd = 5.0;
float kdst = 0.14;
float gain[10];
float Kyaw = 10.0;
float Kyawtodeg = 0.069;
float Kspin = 0.0;
float Pitch_offset = 81, Pitch_offset2 = 0.0, Pitch_power = 0.0;
int Pitch_offset2_address = 4, motor_offsetL_address = 8, motor_offsetR_address = 12;
int fil_N = 5;

// control variables
Kalman kalman;
long lastMs = 0;
float acc[3], accOffset[3];
float gyro[3], gyroOffset[3];
float Pitch, yaw, Pitch_filter;
int wait_count, sec_count;
unsigned char motor_sw, servo_offset_sw;
int16_t power, powerL = motor_init_L, powerR = motor_init_R;
unsigned long ms10, ms100, ms1000, ms2000;
char strPtr[10];
float Speed, yawAng, yawAngx10;
float Angle, dAngle, k_speed, P_Angle, I_Angle, D_Angle;
int joy_x, joy_y;
int8_t selector_No1, select_val, slider_val;
float tmp_gain;
int L_wink_F, R_wink_F;
float batt;

void setup() {
  M5.begin();  
  M5.Lcd.setTextFont(4);
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 50);
    
  if (!EEPROM.begin(100))  M5.Lcd.printf("EEPROM init failed!");
  int flag = EEPROM.readUShort(INIT_FLAG_ADDR);
  if (flag != INIT_MAGIC) {
    M5.Lcd.printf("No_init!");
    delay(2000);
    EEPROM.writeInt(INIT_FLAG_ADDR, INIT_MAGIC);
    EEPROM.writeInt(Pitch_offset2_address, 0);
    EEPROM.writeInt(motor_offsetL_address, 0);
    EEPROM.writeInt(motor_offsetR_address, 0);
    EEPROM.commit();  
  }
  Pitch_offset2 = EEPROM.readInt(Pitch_offset2_address) * 0.01; 
  motor_offsetL = EEPROM.readInt(motor_offsetL_address);  
  motor_offsetR = EEPROM.readInt(motor_offsetR_address);
  
  RemoteXY_Init();
  
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(M5_LED, OUTPUT);
  pinMode(MOTOR_PIN_L, OUTPUT);
  pinMode(MOTOR_PIN_R, OUTPUT);

  // IMU初期化（レンジ設定をM5Unifiedのデフォルトで処理）
  M5.Imu.begin(); // デフォルトで2G, 250DPSを使用（必要に応じて調整）

  M5.Lcd.setCursor(0, 60);  
  digitalWrite(M5_LED, HIGH); // Inverted logic for M5StickCPlus2 LED
  M5.Lcd.println("Calibrating");
  delay(500);

  calibration();

  readGyro();
  kalman.setAngle(getPitch());
  lastMs = micros();
  digitalWrite(M5_LED, LOW); // Inverted logic: HIGH to turn off
  
  M5.Lcd.setRotation(2);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf(Ver);

  gain[1] = 1.0;
  gain[2] = 1.4;
  gain[3] = 1.0;
  gain[4] = 1.4;
  gain[5] = 1.0;
  gain[6] = 1.0;
  ms10 = ms100 = ms1000 = millis();
  Stick_kun();
  servo_stop();  
}

void loop() {
  get_Angle();

  if (millis() > ms10) {
    if (servo_offset_sw == 0) { 
      if (-30 < Pitch_filter && Pitch_filter < 30) {
        wait_count += 1;
        if (wait_count > 200) {
          PID_ctrl();
        }
      } 
      else {
        PID_reset();
      }
    }
    else {
      servo_stop();
    }
    ms10 += 10;
  }

  if (millis() > ms100) {
    RemoteXY_ctrl();
    display_ctrl();
    face_ctrl();
    ms100 += 100;
  }

  if (millis() > ms1000) {
    sec_count++;
    batt = M5.Power.getBatteryVoltage() / 1000.0;
    if (digitalRead(BTN_A) == 0) motor_sw = !motor_sw;
    if (digitalRead(BTN_B) == 0) servo_offset_sw = !servo_offset_sw;   
    ms1000 += 1000;
  }
}// loop_end

// ボディピッチ角度の取得
void get_Angle() {
  readGyro();
  applyCalibration();
  float kalman_dt = (micros() - lastMs) / 1000000.0;
  lastMs = micros();
  Pitch = kalman.getAngle(getPitch(), gyro[0], kalman_dt) + Pitch_offset + Pitch_offset2 + Pitch_power;
  Pitch_filter = (Pitch + Pitch_filter * (fil_N - 1)) / fil_N; // LPF  
  Angle = Pitch_filter;
}

// サーボ駆動停止パルス出力
void servo_stop() {
  powerL = motor_init_L + motor_offsetL;
  powerR = motor_init_R + motor_offsetR;
  pulse_drive(powerL, powerR);
}

// PID変数リセット
void PID_reset() {
  Pitch_power = wait_count = power = Speed = yawAng = yawAngx10 = I_Angle = 0;
}

// 倒立振子ロボ PID制御
void PID_ctrl() {
  Speed += kpower * gain[1] * power;
  P_Angle = kp * gain[2] * Angle;
  I_Angle += ki * gain[3] * Angle + kdst * gain[6] * Speed;
  D_Angle = kd * gain[4] * dAngle;
  k_speed = kspd * gain[5] * Speed;

  power = P_Angle + I_Angle + D_Angle + k_speed;
  
  if (I_Angle > 300) { power = Speed = I_Angle = Pitch_power = 0; }
  if (I_Angle < -300) { power = Speed = I_Angle = Pitch_power = 0; }
  
  if (motor_sw == 1) {
    if (Kspin != 0) { yawAng = 1; yawAngx10 = 0; } 
    
    powerL = -power + motor_offsetL + motor_init_L - int16_t(Kyaw * yawAng) + Kspin;
    powerR =  power + motor_offsetR + motor_init_R - int16_t(Kyaw * yawAng) + Kspin;

    pulse_drive(powerL, powerR);
  } 
  else {
    digitalWrite(MOTOR_PIN_L, LOW);
    digitalWrite(MOTOR_PIN_R, LOW);
  }
}

 

// スマホRemoteXY操作
void RemoteXY_ctrl() {
  RemoteXY_Handler();
  joy_x = RemoteXY.joystick_01_x;
  joy_y = RemoteXY.joystick_01_y;
  
  Kspin = 0;
  if (joy_x > 20) Kspin = (joy_x - 20) * 0.5;
  if (joy_x < -20) Kspin = (joy_x + 20) * 0.5;

  if (joy_y > 20) Pitch_power = Pitch_power + (joy_y - 20) * 0.002;
  if (joy_y < -20) Pitch_power = Pitch_power + (joy_y + 20) * 0.002;
  
  slider_val = RemoteXY.slider_01;
  select_val = RemoteXY.Select_01;

  if (select_val < 10) {
    if (slider_val < -5) tmp_gain = ((float)slider_val + 100) / 95.0;
    if (slider_val > -5) tmp_gain = 1.0;
    if (slider_val > 5) tmp_gain = (slider_val) * 0.09 + 1.0;
    gain[select_val] = tmp_gain;
  } 
  else {
    if (select_val == 10) {
      Pitch_offset2 = (float)(slider_val * 0.05);
    }
    if (select_val == 11) {
      motor_offsetL = (int)slider_val;
    }
    if (select_val == 12) {
      motor_offsetR = (int)slider_val;
    }
 
  }

  snprintf(RemoteXY.text_1, 200, Ver
    " %3d  Scal:%3.1f     P:%3.1f    I:%3.1f    D:%3.1f     Spd;%3.1f     dst:%3.1f     Pof:%3.1f    L:%3d   R:%3d",
    sec_count, gain[1], gain[2], gain[3], gain[4], gain[5], gain[6], Pitch_offset2, motor_offsetL, motor_offsetR);

  RemoteXY.onlineGraph_01_var1 = constrain(P_Angle, -300, 300);
  RemoteXY.onlineGraph_01_var2 = constrain(I_Angle, -300, 300);
  RemoteXY.onlineGraph_01_var3 = constrain(D_Angle, -300, 300);
  RemoteXY.onlineGraph_01_var4 = constrain(k_speed, -300, 300);
}

// ディスプレイ表示の制御
void display_ctrl() {
  M5.Lcd.setCursor(0, 25);
  if (servo_offset_sw == 0) {
    if (motor_sw == 1) M5.Lcd.printf("on    ");
    else M5.Lcd.printf("off   ");
    M5.Lcd.printf(" %5.1f   ", Angle);      
  }
  else {
    M5.Lcd.printf("offset     ");
    EEPROM.writeInt(Pitch_offset2_address, (int)(Pitch_offset2 * 100));
    EEPROM.writeInt(motor_offsetL_address, motor_offsetL);
    EEPROM.writeInt(motor_offsetR_address, motor_offsetR);
    EEPROM.commit();       
  }

  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("%3.1fv ", batt);
  M5.Lcd.printf("  %4.1f    ", yawAng);

  M5.Lcd.setCursor(0, 75);
  M5.Lcd.printf("x %1d   ", (int8_t)(joy_x * 0.1));
  M5.Lcd.setCursor(65, 75);
  M5.Lcd.printf("y %1d   ", (int8_t)(joy_y * 0.1));
}

// 顔の表情制御
void face_ctrl() {
  if ((abs(Kspin) > 5) || (abs((int16_t)yawAng) > 5)) {
    if (Kspin > 0 || yawAng > 5) { draw_L_wink(); L_wink_F = 1; }
    else { draw_R_wink(); R_wink_F = 1; }
  } 
  else {
    if (R_wink_F == 1) { draw_R_wink_reset(); R_wink_F = 0; }
    if (L_wink_F == 1) { draw_L_wink_reset(); L_wink_F = 0; }
  }

  if ((Angle > -3) && (Angle < 3)) draw_smile();
  else draw_henoji();

}

 

// servo pulse drive  
void pulse_drive(int16_t powerL, int16_t powerR) {
  powerL = constrain(powerL, 500, 2500);
  powerR = constrain(powerR, 500, 2500);
  bool doneR = false;
  bool doneL = false;
  uint32_t usec = micros();
  digitalWrite(MOTOR_PIN_L, HIGH);
  digitalWrite(MOTOR_PIN_R, HIGH);

  while (!doneR || !doneL) {
    uint32_t width = micros() - usec;
    if (width >= powerL) { digitalWrite(MOTOR_PIN_L, LOW); doneL = true; }
    if (width >= powerR) { digitalWrite(MOTOR_PIN_R, LOW); doneR = true; }
  }
}

// Calibrate IMU sensors
void calibration() {
  float gyroSum[3] = {0};
  float accSum[3] = {0};
  for (int i = 0; i < 500; i++) {
    readGyro();
    gyroSum[0] += gyro[0];
    gyroSum[1] += gyro[1];
    gyroSum[2] += gyro[2];
    accSum[0] += acc[0];
    accSum[1] += acc[1];
    accSum[2] += acc[2];
    delay(2);
  }
  gyroOffset[0] = gyroSum[0] / 500;
  gyroOffset[1] = gyroSum[1] / 500;
  gyroOffset[2] = gyroSum[2] / 500;
  accOffset[0] = accSum[0] / 500;
  accOffset[1] = accSum[1] / 500;
  accOffset[2] = accSum[2] / 500 - 1.0;
}

// Read gyro and accelerometer data
void readGyro() {
  float gx, gy, gz, ax, ay, az;
  M5.Imu.getGyro(&gx, &gy, &gz);
  M5.Imu.getAccel(&ax, &ay, &az);
  gyro[0] = gx;
  gyro[1] = gy;
  gyro[2] = gz;
  acc[0] = ax;
  acc[1] = ay;
  acc[2] = az;
  dAngle = (gyro[0] - gyroOffset[0]);
  yawAngx10 += (gyro[1] - gyroOffset[1]) * 0.001; //M5_plusでは0.01
  yawAng = yawAngx10 * Kyawtodeg;
}

// Apply calibration to sensor data
void applyCalibration() {
  gyro[0] -= gyroOffset[0];
  gyro[1] -= gyroOffset[1];
  gyro[2] -= gyroOffset[2];
  acc[0] -= accOffset[0];
  acc[1] -= accOffset[1];
  acc[2] -= accOffset[2];
}

// Calculate Pitch angle
float getPitch() {
  return atan2(acc[1], acc[2]) * RAD_TO_DEG;
}

// スティックくん描画
void Stick_kun() {
#define face_center 65
  M5.Lcd.fillRect(0, 105, 134, 239, WHITE);
  M5.Lcd.fillEllipse(face_center - 27, 160, 10, 20, BLACK);
  M5.Lcd.fillEllipse(face_center + 27, 160, 10, 20, BLACK);
  for (int i = 0; i < 5; i++) {
    M5.Lcd.drawLine(face_center - 15, 200 + i, face_center - 10, 204 + i, BLACK);
    M5.Lcd.drawLine(face_center - 10, 204 + i, face_center, 206 + i, BLACK);
    M5.Lcd.drawLine(face_center, 206 + i, face_center + 10, 204 + i, BLACK);
    M5.Lcd.drawLine(face_center + 10, 204 + i, face_center + 15, 200 + i, BLACK);
  }
  for (int i = 0; i < 5; i++) {
    M5.Lcd.drawLine(face_center - 25, 130 + i, face_center - 45, 125 + i, BLACK);
    M5.Lcd.drawLine(face_center + 25, 130 + i, face_center + 45, 125 + i, BLACK);
  }
}

void draw_L_wink() {
  M5.Lcd.fillEllipse(face_center - 27, 160, 10, 20, WHITE);
  draw_wink_sub(face_center - 27, 160, BLACK);
}

void draw_L_wink_reset() {
  draw_wink_sub(face_center - 27, 160, WHITE);
  M5.Lcd.fillEllipse(face_center - 27, 160, 10, 20, BLACK);
}

void draw_R_wink() {
  M5.Lcd.fillEllipse(face_center + 27, 160, 10, 20, WHITE);
  draw_wink_sub(face_center + 27, 160, BLACK);
}

void draw_R_wink_reset() {
  draw_wink_sub(face_center + 27, 160, WHITE);
  M5.Lcd.fillEllipse(face_center + 27, 160, 10, 20, BLACK);
}

void draw_wink_sub(unsigned char x, unsigned char y, unsigned short color) {
  for (int i = 0; i < 5; i++) {
    M5.Lcd.drawLine(x - 15, y + 6 + i, x - 10, y + 4 + i, color);
    M5.Lcd.drawLine(x - 10, y + 4 + i, x, y + i, color);
    M5.Lcd.drawLine(x, y + i, x + 10, y + 4 + i, color);
    M5.Lcd.drawLine(x + 10, y + 4 + i, x + 15, y + 6 + i, color);
  }  
}

void draw_henoji(){
//　白スマイルで消す
  for(int i = 0; i < 5; i++){
    M5.Lcd.drawLine(face_center-15, 200+i,   face_center-10,  204+i, WHITE);
    M5.Lcd.drawLine(face_center-10, 204+i,   face_center,     206+i, WHITE);
    M5.Lcd.drawLine(face_center,    206+i,   face_center+10,  204+i, WHITE);
    M5.Lcd.drawLine(face_center+10, 204+i,   face_center+15,  200+i, WHITE);
  }

//　黒への字
  for(int i = 0; i < 5; i++){
    M5.Lcd.drawLine(face_center-15,  206+i, face_center-10,   204+i, BLACK);
    M5.Lcd.drawLine(face_center-10,  204+i, face_center,      200+i, BLACK);
    M5.Lcd.drawLine(face_center,     200+i, face_center+10,   204+i, BLACK);
    M5.Lcd.drawLine(face_center+10,  204+i, face_center+15,   206+i, BLACK);
  }
}
void draw_smile(){
//　白への字で消す
  for(int i = 0; i < 5; i++){
    M5.Lcd.drawLine(face_center-15,  206+i, face_center-10,   204+i, WHITE);
    M5.Lcd.drawLine(face_center-10,  204+i, face_center,      200+i, WHITE);
    M5.Lcd.drawLine(face_center,     200+i, face_center+10,   204+i, WHITE);
    M5.Lcd.drawLine(face_center+10,  204+i, face_center+15,   206+i, WHITE);
  }

//　黒でスマイル
  for(int i = 0; i < 5; i++){
    M5.Lcd.drawLine(face_center-15, 200+i,   face_center-10,  204+i, BLACK);
    M5.Lcd.drawLine(face_center-10, 204+i,   face_center,     206+i, BLACK);
    M5.Lcd.drawLine(face_center,    206+i,   face_center+10,  204+i, BLACK);
    M5.Lcd.drawLine(face_center+10, 204+i,   face_center+15,  200+i, BLACK);
  }
}
