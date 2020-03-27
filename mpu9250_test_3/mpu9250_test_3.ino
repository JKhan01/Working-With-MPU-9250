#include "MPU9250.h"
#include "math.h"

double pitch, yaw, roll;                   // Variables for final filter output angles.
double roll_g, pitch_g, yaw_g;             // Variables for pitch, yaw, roll values obtained from gyrometer.
double roll_in, pitch_in, yaw_in;          // Variables for initial values obtained without filters
double ax, ay, az, gx, gy, gz, mx, my, mz; // Variables for raw readings from the IMU MPU9250
double hp=0.98,lp = (1- hp);                    // Variables for high frequency and low frequency multipliers.

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status, count = 0;
double delta_t = 0.005; //end loop delay will be delta_t*1000
void setup()
{
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
}

void loop() {
  // read the sensor
  
  
  IMU.readSensor();
  // display the data
  Serial.print("AccelX: ");
  ax = IMU.getAccelX_mss();
  Serial.print(ax, 6);
  Serial.print("  ");
  Serial.print("AccelY: ");
  ay = IMU.getAccelY_mss();
  Serial.print(ay, 6);
  Serial.print("  ");
  Serial.print("AccelZ: ");
  az = IMU.getAccelZ_mss();
  Serial.println(az, 6);

  Serial.print("GyroX: ");
  gx = IMU.getGyroX_rads();
  Serial.print(gx, 6);
  Serial.print("  ");
  Serial.print("GyroY: ");
  gy = IMU.getGyroY_rads();
  Serial.print(gy, 6);
  Serial.print("  ");
  Serial.print("GyroZ: ");
  gz = IMU.getGyroZ_rads();
  Serial.println(gz, 6);

  Serial.print("MagX: ");
  mx = IMU.getMagX_uT();
  Serial.print(mx, 6);
  Serial.print("  ");
  Serial.print("MagY: ");
  my = IMU.getMagY_uT();
  Serial.print(my, 6);
  Serial.print("  ");
  Serial.print("MagZ: ");
  mz = IMU.getMagZ_uT();
  Serial.println(mz, 6);

  Serial.print("Temperature in C: ");
  Serial.println(IMU.getTemperature_C(), 6);
  Serial.println();

  compute_angles();
  Serial.println("Angles without noise filtering:");

  //Converting the global variable values to local degree values for printing.

  double roll_t = ((180 * (roll)) / 3.14);
  double pitch_t = ((180 * (pitch)) / 3.14);
  double yaw_t = ((180 * (yaw)) / 3.14);
  Serial.print("ROLL ANGLE: ");
  Serial.println(roll_t);
  Serial.print("PITCH ANGLE: ");
  Serial.println(pitch_t);
  Serial.print("YAW ANGLE: ");
  Serial.println(yaw_t);
  Serial.println(" ");
  //delay(1000);
  delay(delta_t * 1000);         //delta_t = 0.2s for filtering

}



void compute_angles()
{
  // Basic mathematical computation without handling of inconsisitent noise and primitive interferences.
  //bias_andScale();        //Temporary function for error recalibration
  roll_in = atan(ay / az);
  pitch_in = atan((-ax) / sqrt(((ay) * (ay)) + ((az) * (az))));
  double MX = (mx * cos(pitch_in)) + (mz * sin(pitch_in));
  double MY = (mx * sin(roll_in) * sin(pitch_in)) + (my * cos(roll_in)) - (mz * sin(roll_in) * cos(pitch_in));
  yaw_in = atan(MY / MX);
  // Calculating gyroscopic values
  roll_g = gy * delta_t;
  pitch_g = gx * delta_t;
  yaw_g = gz * delta_t;

  //Applying the filters
  complementary_filter();


}


void complementary_filter()
{
  // Complementary filtered angle CF = ((HighPass_Constant*(CF+GyroData about the axis))+((LowPass_Constant*(Othe sensor [Accelero or Magneto] data for the axis))))
  if (count == 0)
  {
    Serial.println("first iteration!!");
    roll = roll_in;
    pitch = pitch_in;
    yaw = yaw_in;
  }
  roll = (hp * (roll + roll_g)) + (lp * roll_in);
  pitch = (hp * (pitch + pitch_g)) + (lp * pitch_in);
  yaw = (hp * (yaw + yaw_g)) + (lp * yaw_in);
  count += 1;
}


// Code for recallibration
//void bias_andScale()
//{
//  // For Accelerometer
//  IMU.calibrateAccel();
//  double accXBias= IMU.getAccelBiasX_mss();
//  double accXScale= IMU.getAccelScaleFactorX();
//  double accYBias= IMU.getAccelBiasY_mss();
//  double accYScale= IMU.getAccelScaleFactorY();
//  double accZBias= IMU.getAccelBiasZ_mss();
//  double accZScale= IMU.getAccelScaleFactorZ();
//  ax = ((ax- accXBias)/accXScale);
//  ay = ((ay- accYBias)/accYScale);
//  az = ((az- accZBias)/accZScale);
//
//  // For Magnetometer
//  IMU.calibrateMag();
//  double magXBias = IMU.getMagBiasX_uT();
//  double magXScale = IMU.getMagScaleFactorX();  
//  double magYBias = IMU.getMagBiasY_uT();
//  double magYScale = IMU.getMagScaleFactorY();
//  double magZBias = IMU.getMagBiasZ_uT();
//  double magZScale = IMU.getMagScaleFactorZ();
//  mx= ((mx-magXBias)/magXScale);
//  my= ((my-magYBias)/magYScale);
//  mz= ((mz-magZBias)/magZScale);
//
//  // For Gyrometer
//  IMU.calibrateGyro();
//  double gyroXBias = IMU.getGyroBiasX_rads();
//  double gyroYBias = IMU.getGyroBiasY_rads();
//  double gyroZBias = IMU.getGyroBiasZ_rads();
//  gx = (gx - gyroXBias);
//  gy = (gy - gyroYBias);
//  gz = (gz - gyroZBias);
//
//
//  Serial.println("Values after Calibration: ");
//  Serial.print ("AccelX: ");
//  Serial.print (ax,6);
//  Serial.print ("  ");
//  Serial.print ("AccelY: ");
//  Serial.print (ay,6);
//  Serial.print ("  ");
//  Serial.print ("AccelZ: ");
//  Serial.println (az,6);
//  Serial.print ("MagX: ");
//  Serial.print (mx,6);
//  Serial.print ("  ");
//  Serial.print ("MagY: ");
//  Serial.print (my,6);
//  Serial.print ("  ");
//  Serial.print ("MagZ: ");
//  Serial.println (mz,6);
//  Serial.print ("GyroX: ");
//  Serial.print (gx,6);
//  Serial.print ("  ");
//  Serial.print ("GyroY: ");
//  Serial.print (gy,6);
//  Serial.print ("  ");
//  Serial.print ("GyroZ: ");
//  Serial.println (gz,6);
//  Serial.println ("  ");
//  
//}
