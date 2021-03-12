//MPU6050による加速度andジャイロの取得と離床判定
//離床判定…加速度センサから取得した3軸合成加速度の絶対値の5回移動平均が5回連続で2.5[G]を超える

#include <Wire.h>

#define MPU6050_ACCEL_XOUT_H 0x3B 
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_I2C_ADDRESS  0x68

#define A_FLIGHT 2.50 //しきい値
boolean BURNING = false; //離床判定によってフェーズがBURNINGに移行．フラグにしてます...
int SAMPLING_RATE = 400000; //MPU6050のサンプリングレートは400kHz

// 構造体定義
typedef union accel_union {
  struct {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
  }
  reg;
  struct {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
  }
  value;
};

//MPU6050読み出し用の関数
int MPU6050_read(int start, uint8_t *buffer, int size) {
  int i, n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1) {
    return (-10);
  }
  n = Wire.endTransmission(false);// hold the I2C-bus
  if (n != 0) {
    return (n);
  }
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while (Wire.available() && i < size) {
    buffer[i++] = Wire.read();
  }
  if ( i != size) {
    return (-11);
  }
  return (0);
}

//MPU6050動作開始用の関数用の関数
int MPU6050_write(int start, const uint8_t *pData, int size) {
  int n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);// write the start address
  if (n != 1) {
    return (-20);
  }
  n = Wire.write(pData, size);// write data bytes
  if (n != size) {
    return (-21);
  }
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0) {
    return (error);
  }

  return (0);
}
//MPU6050動作開始用の関数
int MPU6050_write_reg(int reg, uint8_t data) {
  int error;
  error = MPU6050_write(reg, &data, 1);
  Serial.print("error = ");
  Serial.println(error);
  return (error);
}

//タイマ割込みのための変数
volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//タイマ割込み用の関数
void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  int error;
  uint8_t c;

  Wire.begin(21, 22);
  Serial.begin(115200);
  
  //加速度レンジ±8[G]に設定
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
  error = MPU6050_read(MPU6050_PWR_MGMT_1, &c, 1);
  MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0);
  
  //タイマ割込み初期化
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / SAMPLING_RATE, true);
  timerAlarmEnable(timer1);
}

void loop() {
  //離床判定
  int ai = 0; //移動平均カウントのための変数
  int acnt = 0; //連続回数カウントのための変数
  
  while(BURNING = false){
    //タイマ割込み処理
    if(timeCounter1 > 0){
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);
  int error;
  accel_union accel;
  error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *)&accel, sizeof(accel));
  uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap
  SWAP (accel.reg.x_accel_h, accel.reg.x_accel_l);
  SWAP (accel.reg.y_accel_h, accel.reg.y_accel_l);
  SWAP (accel.reg.z_accel_h, accel.reg.z_accel_l);
  
  float ax = accel.value.x_accel / 4096.0; //FS_SEL_2 4,096 LSB / g
  float ay = accel.value.y_accel / 4096.0;
  float az = accel.value.z_accel / 4096.0;
  Serial.print(ax, 2);
  Serial.print("\t");
  Serial.print(ay, 2);
  Serial.print("\t");
  Serial.print(az, 2);
  Serial.print("\t");
  Serial.print("");
  
  //移動平均をとる
  float asqrt[6];
      for(ai = 0; ai < 5; ai++){
    asqrt[ai+1] = asqrt[ai];
  }
  asqrt[0] = sqrt(pow(accel.value.x_accel, 2)+pow(accel.value.y_accel, 2)+pow(accel.value.z_accel, 2)) / 4096.0; //３軸合成加速度
  float aave = (asqrt[0]+asqrt[1]+asqrt[2]+asqrt[3]+asqrt[4]) / 5;

  //連続回数を調べる
  if(aave > A_FLIGHT){
    acnt++;
  } else{
    acnt = 0;
  }          
  if(acnt > 4){
    BURNING = true;
    Serial.print("PHASE:BURNING");
  }
  }
  }
}
