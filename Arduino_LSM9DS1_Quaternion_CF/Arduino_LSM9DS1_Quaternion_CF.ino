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
3. 測試文章section 3的做法，未來將作為CF的initial condition

*/


#include <Arduino_LSM9DS1.h>

float ax, ay, az;  // acc value
float gx, gy, gz;  // gyro value
float mx, my, mz;  // mag value
float a_sq, m_sq;  // acc and mag square root

float axn, ayn, azn;  // normalized acc
float mxn, myn, mzn;  // normalized mag

float qacc[4], qmag[4];  // quaternion of acc and mag

float R_acc[3][3];

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

}

void loop() {

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (IMU.gyroscopeAvailable()) {
  IMU.readGyroscope(gx, gy, gz);
  }

  if (IMU.magneticFieldAvailable()) {
  IMU.readMagneticField(mx, my, mz);
  }

// normalize
a_sq = sqrt(sq(ax)+sq(ay)+sq(az));
axn = ax/a_sq;
ayn = ay/a_sq;
azn = az/a_sq;

m_sq = sqrt(sq(mx)+sq(my)+sq(mz));
mxn = mx/m_sq;
myn = my/m_sq;
mzn = mz/m_sq;

// acc quaternion from eq.(25)
  if (azn >= 0){
    qacc[0] = sqrt((azn+1)/2);
    qacc[1] = -ayn/sqrt(2*(azn+1));
    qacc[2] = axn/sqrt(2*(azn+1));
    qacc[3] = 0;
    }
  else {
    qacc[0] = -ayn/sqrt(2*(1-azn));
    qacc[1] = sqrt((1-azn)/2);
    qacc[2] = 0;
    qacc[3] = axn/sqrt(2*(1-azn));
  }

Serial.println(57.29577*quaternion2Theta(qacc[0],qacc[1],qacc[2],qacc[3]));
// acc rotation matrix from quaternion, see eq.(9)

/*  
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
*/
}

// Quaternion to Euler angle Phi
float quaternion2Phi(float q0, float q1, float q2, float q3){
  float phi;
  phi = atan2( 2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2) );
  return phi;
}

// Quaternion to Euler angle Theta
float quaternion2Theta(float q0, float q1, float q2, float q3){
  float theta;
  theta = asin( 2*(q0*q2-q3*q1) );
  return theta;
}

// Quaternion to Euler angle Psi
float quaternion2Psi(float q0, float q1, float q2, float q3){
  float psi;
  psi = atan2( 2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3) );
  return psi;
}



/*
基本的資料讀取
<Acc>
  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }

<Gyro>
  float x, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
  }

<Mag>
  float x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
  }
    
*/
