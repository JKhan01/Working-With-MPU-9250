#include "MPU9250.h"
#include "math.h"

double pitch, yaw, roll;                   // Variables for final filter output angles.
double roll_g, pitch_g, yaw_g;             // Variables for pitch, yaw, roll values obtained from gyrometer.
double roll_in, pitch_in, yaw_in;          // Variables for initial values obtained without filters
double ax, ay, az, gx, gy, gz, mx, my, mz; // Variables for raw readings from the IMU MPU9250
double hp = 0.98, lp = (1 - hp);            // Variables for high frequency and low frequency multipliers.

// All data variables and matrices below handle values in DEGREES.
double mRoll_error = 1.50, mPitch_error = 1.25, mYaw_error = 4.5;     // Variables for constant measured errors.
double eRoll = 0, ePitch = 1.0, eYaw = 30.0;
double eRoll_error = 1.0, ePitch_error = 2.0, eYaw_error = 6.0;
double measured_value[][1] = {{roll}, {pitch}, {yaw}}, measured_error[][1] = {{mRoll_error}, {mPitch_error}, {mYaw_error}}; // Measured value and error matrices for ROll, Pitch and Yaw.
double estimate_value[][1] = {{eRoll}, {ePitch}, {eYaw}} , estimate_error[][1] = {{eRoll_error}, {ePitch_error}, {eYaw_error}};      // Estimate matrices for Roll, Pitch and Yaw.
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

void loop()
{
  // read the sensor
  int i, j;



  //compute_angles();
  kalman_filter();
  Serial.println("Angles without noise filtering:");

  //Converting the global variable values to local degree values for printing.

  double roll_t =  estimate_value[0][0];
  double pitch_t = estimate_value[1][0];
  double yaw_t = estimate_value[2][0];
  Serial.print("ROLL ANGLE: ");
  Serial.println(roll_t, 6);
  Serial.print("PITCH ANGLE: ");
  Serial.println(pitch_t, 6);
  Serial.print("YAW ANGLE: ");
  Serial.println(yaw_t, 6);
  Serial.println(" ");
  //delay(1000);
  delay(1000);         //delta_t = 0.2s for filtering

}



void compute_angles()
{
  // Basic mathematical computation without handling of inconsisitent noise and primitive interferences.
  //bias_andScale();        //Temporary function for error recalibration
  IMU.readSensor();
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  double raw_data[][3] = {{ax, ay, az}, {mx, my, mz}, {gx, gy, gz}};
  Serial.println("X           Y          Z");
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      Serial.print ((raw_data[i][j]), 6);
      Serial.print (" ");
    }
    Serial.println("");
  }
  roll_in = atan(ay / az);
  pitch_in = atan((-ax) / sqrt(((ay) * (ay)) + ((az) * (az))));
  double MX = (mx * cos(pitch_in)) + (mz * sin(pitch_in));
  double MY = (mx * sin(roll_in) * sin(pitch_in)) + (my * cos(roll_in)) - (mz * sin(roll_in) * cos(pitch_in));
  yaw_in = atan(MY / MX);
  // Calculating gyroscopic values
  roll_g = gy * delta_t;
  pitch_g = gx * delta_t;
  yaw_g = gz * delta_t;

  // Applying the filters
  complementary_filter();
  //kalman_filter();

}

void kalman_filter()
{
  // indexes 0->Roll 1->Pitch 2->Yaw
  int i, j;


  double kGain[3][1];
  compute_angles();
  for ( i = 0; i < 3; i++)
  {
    for ( j = 0; j < 1; j++)
    {
      kGain[i][j] = (estimate_error[i][j]) / ((estimate_error[i][j]) + (measured_error[i][j]));
    }
  }

  while (((kGain[0][0] + kGain[1][0] + kGain[2][0]) / 3) > 0.1)
  {

    for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 1; j++)
      {
        estimate_value[i][j] = estimate_value[i][j] + ((kGain[i][j]) * ((measured_value[i][j]) - (estimate_value[i][j])));
      }
    }

    for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 1; j++)
      {
        estimate_error[i][j] = (1 - kGain[i][j]) * (estimate_error[i][j]);
      }
    }
    delay(delta_t * 1000);
    compute_angles();

    for ( i = 0; i < 3; i++)
    {
      for ( j = 0; j < 1; j++)
      {
        kGain[i][j] = (estimate_error[i][j]) / ((estimate_error[i][j]) + (measured_error[i][j]));
      }
    }

  }
  Serial.println("X         Y         Z");
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 1; j++)
    {
      Serial.print(kGain[i][j]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 1; j++)
    {
      Serial.print(estimate_value[i][j]);
      Serial.print(" ");
    }
    Serial.println("");
  }

}
void complementary_filter()
{
  // Complementary filtered angle CF = ((HighPass_Constant*(CF+GyroData about the axis))+((LowPass_Constant*(Othe sensor [Accelero or Magneto] data for the axis))))
  int i, j;
  if (count == 0)
  {
    Serial.println("first iteration!!");
    roll = roll_in;
    pitch = pitch_in;
    yaw = yaw_in;
  }
  roll = ((hp * (roll + roll_g)) + (lp * roll_in));
  pitch =  ((hp * (pitch + pitch_g)) + (lp * pitch_in));
  yaw =  ((hp * (yaw + yaw_g)) + (lp * yaw_in));
  count += 1;
  measured_value[0][0] = roll * (180/3.14);
  measured_value[1][0] = pitch * (180/3.14);
  measured_value[2][0] = yaw * (180/3.14);
  Serial.println("Measured Values");
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 1; j++)
    {
      Serial.print(measured_value[i][j]);
      Serial.print(" ");
    }
    Serial.println("");
  }
}
