
#include "MPU_6050.h"
#include "lib_attitude.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire.h"

#define DEBUG_INERTIAL

  unsigned long last_read_time;
  float         last_x_angle;  // These are the filtered angles
  float         last_y_angle;
  float         last_z_angle;
  float         last_gyro_x_angle;  // Store the gyro angles to compare drift
  float         last_gyro_y_angle;
  float         last_gyro_z_angle;
  float    base_x_accel;
  float    base_y_accel;
  float    base_z_accel;
  float    base_x_gyro;
  float    base_y_gyro;
  float    base_z_gyro;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
  // Read the raw values.
  // Read 14 bytes at once,
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.  Returns the error value

  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;

  int error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));

  // Swap all high and low bytes.
  // After this, the registers values are swapped,
  // so the structure name like x_accel_l does no
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
  SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
  SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
  SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
  SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
  SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
  SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

  return error;
}


void calibrate_sensors() {
  int num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;

  Serial.println("Starting Calibration");

  // Discard the first ten set of values read from the IMU
  for(int i=0;i<10;i++){
  read_gyro_accel_vals((uint8_t *) &accel_t_gyro);}

  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;

  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;

  Serial.println("Finishing Calibration");
}


int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n;
  //int error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}



int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}


int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}



void initInertial()
{
#ifdef DEBUG_INERTIAL
	Serial.println("start init inertial");
#endif
  last_read_time=0;
  last_x_angle=0;  // These are the filtered angles
  last_y_angle=0;
  last_z_angle=0;
  last_gyro_x_angle=0;  // Store the gyro angles to compare drift
  last_gyro_y_angle=0;
  last_gyro_z_angle=0;
  base_x_accel=0;
  base_y_accel=0;
  base_z_accel=0;
  base_x_gyro=0;
  base_y_gyro=0;
  base_z_gyro=0;
  int error;
  uint8_t c;

  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  // According to the datasheet, the 'sleep' bit
  // should read a '1'. But I read a '0'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. Even if the
  // bit reads '0'.
  error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);

  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

  //Initialize the angles
  calibrate_sensors();
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
#ifdef DEBUG_INERTIAL
	Serial.println("end init inertial");
#endif
}





int readInertial(int value)
{
  //int error;
//  double dT;
  accel_t_gyro_union accel_t_gyro;

  // Read the raw values.
  int error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);

  // Get the time of reading for rotation computations
  unsigned long t_now = millis();

  // Convert gyro values to degrees/sec
  float FS_SEL = 131;

  float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
  float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
  float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;


  // Get raw acceleration values
  //float G_CONVERT = 16384;
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;

  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180/3.14159;
//  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_z = atan(sqrt(pow(accel_x,2) + pow(accel_y,2))/accel_y)*RADIANS_TO_DEGREES;
  //float accel_angle_z = 0;

  // Compute the (filtered) gyro angles
  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();

  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();

  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  //float angle_z = alpha*gyro_angle_z + (1.0 - alpha)*accel_angle_z;  //Accelerometer doesn't give z-angle
  float angle_z = gyro_angle_z;
  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
  switch(value){
    case X_ANGLE:
	    return(angle_x);
    case Y_ANGLE:
  	    return(angle_y);
    case Z_ANGLE:
	    return(angle_z);
    case X_ACCEL:
	    return(accel_x);
    case Y_ACCEL:
	    return(accel_y);
    case Z_ACCEL:
	    return(accel_z);
    case X_ANGLE_ACCEL:
	    return(accel_angle_x);
    case Y_ANGLE_ACCEL:
	    return(accel_angle_y);
    case Z_ANGLE_ACCEL:
	    return(accel_angle_z);
    default:
	    return(0);
  }


}
