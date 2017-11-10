#include <sensor_fusion.h>
// DEFINE INT_STATUS IN HEADER

bool getData(int *acc, int *gyro);


  int max_Samples = 75;
  
  int bias_xa = 0;
  int bias_ya = 0;
  int bias_za = 0;

  int bias_xg = 0;
  int bias_yg = 0;
  int bias_zg = 0;
  
void setup() {
  //configure device
  //set PWR_MGMT_1 register to take the IMU out of sleep mode
  //set GYRO_CONFIG register to the largest possible full-scale range to enable the 
  //detection of high-velocity rotations
  //set CONFIG register to the largest possible bandwidth
  
  int *acc, *gyro;
  // Get samples
  for (int i = 0; i < max_Samples; i++) {
    getData(acc, gyro);

    bias_xa += acc[0] - 0;
    bias_ya += acc[1] - 0;
    bias_za += acc[2] - 9.81;

    bias_xg += gyro[0] - 0;
    bias_yg += gyro[1] - 0;
    bias_zg += gyro[2] - 0;
  }

  bias_xa /= max_Samples;
  bias_ya /= max_Samples;
  bias_za /= max_Samples;

  bias_xg /= max_Samples;
  bias_yg /= max_Samples;
  bias_zg /= max_Samples;
}

void loop() {
  // put your main code here, to run repeatedly:

}

/* Get data about x y z coordinates from accelerometer and gyroscope.
   Takes in two arrays, acc and gyro. If data is available, acc and gyro
   will be filled with the x y z coordinates and the function returns true.
   Otherwise it returns false and arrays are not touched.*/
bool getData(int *acc, int *gyro) {
  uint8_t *buf;
  readReg(INT_STATUS, buf, 1);
  if (buf[7] == 1) {
    uint8_t *buffMccree;
    int x_a, y_a, z_a,
             x_g, y_g, z_g;

    // 59 & 60
    readReg(0x3B, buffMccree, 1);
    x_a = *buffMccree;
    x_a = x_a << 8;

    readReg(0x3C, buffMccree, 1);
    x_a = x_a | *buffMccree;

    // 61 & 62
    readReg(0x3D, buffMccree, 1);
    y_a = *buffMccree;
    y_a = y_a << 8;

    readReg(0x3E, buffMccree, 1);
    y_a = y_a | *buffMccree;

    // 63 & 64
    readReg(0x3F, buffMccree, 1);
    z_a = *buffMccree;
    z_a = z_a << 8;

    readReg(0x40, buffMccree, 1);
    z_a = z_a | *buffMccree;

    // 67 & 68
    readReg(0x43, buffMccree, 1);
    x_g = *buffMccree;
    x_g = x_g << 8;

    readReg(0x44, buffMccree, 1);
    x_g = x_g | *buffMccree;

    // 69 & 70
    readReg(0x45, buffMccree, 1);
    y_g = *buffMccree;
    y_g = y_g << 8;

    readReg(0x46, buffMccree, 1);
    y_g = y_g | *buffMccree;

    // 71 & 72
    readReg(0x47, buffMccree, 1);
    z_g = *buffMccree;
    z_g = z_g << 8;

    readReg(0x48, buffMccree, 1);
    z_g = z_g | *buffMccree;

    acc[0] = x_a;
    acc[1] = y_a;
    acc[2] = z_a;

    gyro[0] = x_g;
    gyro[1] = y_g;
    gyro[2] = z_g;

    return 1;
  }
  return 0;
}
