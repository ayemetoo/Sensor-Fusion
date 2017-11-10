#include <sensor_fusion.h>
// DEFINE INT_STATUS IN HEADER

bool getData(int *acc, int *gyro);

struct vector bias_a, bias_g, acc, gyro;
int max_Samples = 75;

void setup() {
  //configure device
  //set PWR_MGMT_1 register to take the IMU out of sleep mode
  //set GYRO_CONFIG register to the largest possible full-scale range to enable the
  //detection of high-velocity rotations
  //set CONFIG register to the largest possible bandwidth

  bias_a.x = bias_a.y = bias_a.z = 0;
  bias_g.x = bias_g.y = bias_g.z = 0;

  // Get samples
  for (int i = 0; i < max_Samples; i++) {
    getData();

    bias_a.x += acc.x - 0;
    bias_a.y += acc.y - 0;
    bias_a.z += acc.z - 9.81;

    bias_g.x += gyro.x - 0;
    bias_g.y += gyro.y - 0;
    bias_g.z += gyro.z - 0;
  }

  bias_a.x /= max_Samples;
  bias_a.y /= max_Samples;
  bias_z.z /= max_Samples;

  bias_g.x /= max_Samples;
  bias_g.y /= max_Samples;
  bias_g.z /= max_Samples;

  Serial.begin(9600);

  Serial.println("Bias_xA= " + bias_a.x);
  Serial.println("Bias_yA= " + bias_a.y);
  Serial.println("Bias_zA= " + bias_a.z);

  Serial.println("Bias_xG= " + bias_g.x);
  Serial.println("Bias_yG= " + bias_g.y);
  Serial.println("Bias_zG= " + bias_g.z);

}

void loop() {
  if (getData()){
    printVector(acc);
    printVector(gyro);
  }
}

void printVector(struct vector v){
  Serial.print(v.x);
  Serial.print(" ");
  Serial.print(v.y);
  Serial.print(" ");
  Serial.print(v.z);
  Serial.println();
}

//change comment
/* Get data about x y z coordinates from accelerometer and gyroscope.
   Takes in two arrays, acc and gyro. If data is available, acc and gyro
   will be filled with the x y z coordinates and the function returns true.
   Otherwise it returns false and arrays are not touched.*/
bool getData(int *acc, int *gyro) {
  uint8_t *buf;
  readReg(INT_STATUS, buf, 1);
  if (buf[7] == 1) {
    byte* ptr = &(acc);
    byte reg;
    
    for(reg=0x3B;reg<=0x40;reg++){
      readReg(reg, ptr, 1);
      ptr++;
    }
    
    ptr = &(gyro);
    for(reg=0x43;reg<=0x48;reg++){
      readReg(reg, ptr, 1);
      ptr++;
    }

    return 1;
  }
  return 0;
}
