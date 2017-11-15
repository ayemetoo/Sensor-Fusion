#include "sensor_fusion.h"
// DEFINE INT_STATUS IN HEADER

struct vector bias_a, bias_g, acc, gyro;
int max_Samples = 75;

void setup() {
  //configure device
  //set PWR_MGMT_1 register to take the IMU out of sleep mode
  uint8_t *pwr_mgmt_1;
  readReg(0x6B, pwr_mgmt_1, 1);
  *pwr_mgmt_1 = 0xBF & *pwr_mgmt_1; //disable SLEEP
  writeReg(0x6B, pwr_mgmt_1, 1);
  //set GYRO_CONFIG register to the largest possible full-scale range to enable the
  //detection of high-velocity rotations
  uint8_t *gyro_config;
  readReg(0x1B, gyro_config, 1);
  *gyro_config = 0x18 | *gyro_config; //highest thingy
  writeReg(0x1B, gyro_config, 1);
  //set CONFIG register to the largest possible bandwidth
  uint8_t *configy;
  readReg(0x1A, configy, 1);
  *configy = 0xF8 & *configy;
  writeReg(0x1A, configy, 1);
  //enable DATA_RDY
  uint8_t *RDY;
  readReg(0x38, RDY, 1);
  *RDY = *RDY | 0x01;
  writeReg(0x38, RDY, 1);

  Serial.begin(9600);

  uint8_t *WHOAMI;
  readReg(0x75,WHOAMI,1);
  Serial.println(String(*WHOAMI));

  bias_a.x = bias_a.y = bias_a.z = 0;
  bias_g.x = bias_g.y = bias_g.z = 0;

  // Get samples
  for (int i = 1; i <= max_Samples; i++) {

    if (getData()) {

      bias_a.x += acc.x - 0;
      bias_a.y += acc.y - 0;
      bias_a.z += acc.z - 9.81;

      bias_g.x += gyro.x - 0;
      bias_g.y += gyro.y - 0;
      bias_g.z += gyro.z - 0;

      Serial.println("Loop" + String(i));
      printVector(bias_a);
      printVector(bias_g);
    }
    else {
      Serial.println("No data");
      i--;
    }
  }

  bias_a.x /= max_Samples;
  bias_a.y /= max_Samples;
  bias_a.z /= max_Samples;

  bias_g.x /= max_Samples;
  bias_g.y /= max_Samples;
  bias_g.z /= max_Samples;

  printVector(bias_a);
  printVector(bias_g);


}

void loop() {
  if (getData()) {
    vector unit_a;
    vector_normalize(&acc, &unit_a);
    printVector(unit_a);
    //vector_normalize(&gyro);
    //printVector(gyro);
  }
  else
    Serial.println("No data");
}

void printVector(struct vector v) {
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
bool getData() {
  uint8_t *buf;
  readReg(0x3A, buf, 1);
  Serial.println(String(*buf));
  if ((*buf)&0x01) {
    byte* ptr = (byte*) & (acc);
    byte reg;

    for (reg = 0x3B; reg <= 0x40; reg++) {
      readReg(reg, ptr, 1);
      ptr++;
    }

    ptr = (byte*) & (gyro);
    for (reg = 0x43; reg <= 0x48; reg++) {
      readReg(reg, ptr, 1);
      ptr++;
    }

    return true;
  }
  return false;
}
