/*
參考文章 Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs
使用library Arduino_LSM9DS1.h: https://github.com/arduino-libraries/Arduino_LSM9DS1 
作者寫定的條件如下
Accelerometer range is set at ±4 g with a resolution of 0.122 mg.
Gyroscope range is set at ±2000 dps with a resolution of 70 mdps.
Magnetometer range is set at ±400 uT with a resolution of 0.014 uT.
Accelerometer and gyrospcope output data rate is fixed at 104 Hz.
Magnetometer output data rate is fixed at 20 Hz.

可視情況自行修改，例如gryo用2000可能過頭了，在sensor datasheet中有提到±245/±500/±2000 dps angular rate full scale，個人認為用500 degree per second可能比較合適
datasheet請參考:https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
自行撰寫、設定暫存器請參考:

//////////////////////////////////////////////////////
2022/05/07: 
1. 開始撰寫程式、確認acc, gyro, mag三個sensor的連接無問題
2. 寫子程式: (a)quaternion_pruduct.  (b)quaternion2Euler
3. 測試

*/


#include <Arduino_LSM9DS1.h>

float x, y, z;
int degreesX = 0;
int degreesY = 0;
int static_variable = 1;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
}

void loop() {

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }

  Serial.print("X:");
  Serial.print(x/sqrt(x*x+y*y+z*z));

  Serial.print(",");
  Serial.print("Y:");
  Serial.print(y/sqrt(x*x+y*y+z*z));

  Serial.print(",");
  Serial.print("Z:");
  Serial.print(z/sqrt(x*x+y*y+z*z));
  
  Serial.print(",");
  Serial.print("sum:");
  Serial.println(sq(x/sqrt(x*x+y*y+z*z)) + sq(y/sqrt(x*x+y*y+z*z)) + sq(z/sqrt(x*x+y*y+z*z)));
}
