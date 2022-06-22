#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <MadgwickAHRS.h>

LSM9DS1 imu;
Madgwick filter;

unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
float increment = 0.0f;
int increment_cnt = 0;
int initial = 1;
float initialvalue;


void setup() {
  Serial.begin(115200);
  Wire.begin(16,17);
  filter.begin(119);
  imu.begin();

  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);


  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 119;
  microsPrevious = micros();

  
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  int mix, miy, miz;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;
  unsigned long microsNow;
  microsNow = micros();
  if ((microsNow - microsPrevious) < microsPerReading) {
    return;
  }
  float samplingFreq = 1000000.0f / (microsNow - microsPrevious);

  microsPrevious = microsNow;

  if ( imu.gyroAvailable() )
  {
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    imu.readMag();
  }
  
    // read raw data from CurieIMU
    aix= imu.ax;
    aiy = imu.ay;
    aiz = imu.az;
    gix = imu.gx;
    giy = imu.gy;
    giz = imu.gz;
    mix = imu.mx;
    miy = imu.my;
    miz = imu.mz;
    
    ax = imu.calcAccel(aix);
    ay = imu.calcAccel(aiy);
    az = imu.calcAccel(aiz);
    gx = imu.calcGyro(gix);
    gy = imu.calcGyro(giy);
    gz = imu.calcGyro(giz);
    mx = imu.calcMag(mix);
    my = imu.calcMag(miy);
    mz = imu.calcMag(miz);
    // convert from raw data to gravity and degrees/second units
    filter.updateIMU(gx, gy, -gz, ax, ay, -az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw() - increment;
    increment = increment + 0.0030;

      if (initial){
    initialvalue = heading;
  }
  if ((heading - initialvalue) > 20 && (heading - initialvalue) < 40 ){
    digitalWrite(26, HIGH);
    digitalWrite(25, LOW);
    digitalWrite(33, LOW);
    digitalWrite(32, LOW);
  }
  if ((heading - initialvalue) > 40 && (heading - initialvalue) < 60 ){
    digitalWrite(26, LOW);
    digitalWrite(25, HIGH);
    digitalWrite(33, LOW);
    digitalWrite(32, LOW);
  }
  if ((heading - initialvalue) > 60 && (heading - initialvalue) < 80 ){
    digitalWrite(26, LOW);
    digitalWrite(25, LOW);
    digitalWrite(33, HIGH);
    digitalWrite(32, LOW);
  }
  if ((heading - initialvalue) > 80 && (heading - initialvalue) < 100){
    digitalWrite(26, LOW);
    digitalWrite(25, LOW);
    digitalWrite(33, LOW);
    digitalWrite(32, HIGH);
  }
  initial = 0;

//    Serial.print(ax);
//    Serial.print('\t');
//    Serial.print(ay);
//    Serial.print('\t');
//    Serial.print(az);
//    Serial.print('\t');
//    Serial.print(gx);
//    Serial.print('\t');
//    Serial.print(gy);
//    Serial.print('\t');
//    Serial.print(gz);
//    Serial.print('\t');
    Serial.print(increment);
    Serial.print('\t');
    Serial.print(roll);
    Serial.print('\t');
    Serial.print(pitch);
    Serial.print('\t');               
    Serial.print(heading);
    Serial.println('\t');
    
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}