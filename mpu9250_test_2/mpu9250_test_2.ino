#include "MPU9250.h"
#include "math.h"

double pitch,yaw,roll,MX,MY,roll_in,pitch_in,yaw_in,ax,ay,az,gx,gy,gz,mx,my,mz;
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  Serial.print("AccelX: ");
  ax=IMU.getAccelX_mss();
  Serial.print(ax,6);
  Serial.print("  ");
  Serial.print("AccelY: ");  
  ay=IMU.getAccelY_mss();
  Serial.print(ay,6);
  Serial.print("  ");
  Serial.print("AccelZ: ");  
  az=IMU.getAccelZ_mss();
  Serial.println(az,6);
  
  Serial.print("GyroX: ");
  gx=IMU.getGyroX_rads();
  Serial.print(gx,6);
  Serial.print("  ");
  Serial.print("GyroY: ");  
  gy=IMU.getGyroY_rads();
  Serial.print(gy,6);
  Serial.print("  ");
  Serial.print("GyroZ: ");  
  gz=IMU.getGyroZ_rads();
  Serial.println(gz,6);

  Serial.print("MagX: ");  
  mx=IMU.getMagX_uT();
  Serial.print(mx,6);
  Serial.print("  ");  
  Serial.print("MagY: ");
  my=IMU.getMagY_uT();
  Serial.print(my,6);
  Serial.print("  ");
  Serial.print("MagZ: ");  
  mz=IMU.getMagZ_uT();
  Serial.println(mz,6);
  
  Serial.print("Temperature in C: ");
  Serial.println(IMU.getTemperature_C(),6);
  Serial.println();
  delay(200);

  compute_angles();
  Serial.println("ANgles without noise filtering:");
  //Converting the global variable values to local degree values for printing.
  double roll_t=((180*(roll_in))/3.14);
  double pitch_t=((180*(pitch_in))/3.14);
  double yaw_t=((180*(yaw_in))/3.14);
  Serial.print("ROLL ANGLE: ");
  Serial.println(roll_t);
  Serial.print("PITCH ANGLE: ");
  Serial.println(pitch_t);
  Serial.print("YAW ANGLE: ");
  Serial.println(yaw_t);
  delay(1000);
}



void compute_angles()
{
  // Basic mathematical computation without handling of inconsisitent noise and primitive interferences.
  roll_in=atan(ay/az);
  pitch_in=atan((-ax)/sqrt(((ay)*(ay))+((az)*(az))));
  MX=(mx*cos(pitch_in))+(mz*sin(pitch_in));
  MY=(mx*sin(roll_in)*sin(pitch_in))+(my*cos(roll_in))-(mz*sin(roll_in)*cos(pitch_in));
  yaw_in=atan(MY/MX);
}
